/**
 * @file    teleop.c
 * @brief   键盘遥控底盘（前后左右 + 原地旋转）
 * @author  Ltttttts
 *
 * 用法:
 *   ./teleop [串口设备]
 *
 * 键位说明:
 *   W / ↑    前进
 *   S / ↓    后退
 *   A / ←    左平移
 *   D / →    右平移
 *   Q        原地左转（逆时针）
 *   E        原地右转（顺时针）
 *   空格      急停
 *   X / Esc  退出
 *
 *   +/-      调速（增减 10%）
 *
 * ================================================================
 * TODO: 连接硬件后将下面的 0 改为 1
 * ================================================================
 */
#define HARDWARE_ENABLED 0

#include "serial_port.h"
#include "emm_v5_driver.h"
#include "kinematics.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <signal.h>
#include <math.h>

/* ---- 底盘参数 ---- */
#define WHEEL_RADIUS_M      (0.05)
#define HALF_LX_M           (0.12)
#define HALF_LY_M           (0.10)
#define ENCODER_PPR         (4000)

/* ---- 速度参数 ---- */
#define TELEOP_LINEAR_SPEED     (0.30)   /* 线速度 m/s    */
#define TELEOP_ANGULAR_SPEED    (0.80)   /* 角速度 rad/s  */
#define TELEOP_SPEED_STEP       (0.03)   /* 调速步进      */
#define TELEOP_LOOP_INTERVAL_US (20000U) /* 控制循环 20ms */

/* ---- 各轮方向修正因子（连接硬件后按实际方向调整） ---- */
#define DIR_FACTOR_FL  (1.0)
#define DIR_FACTOR_FR  (1.0)
#define DIR_FACTOR_RL  (1.0)
#define DIR_FACTOR_RR  (1.0)

/* ---- 全局状态 ---- */
static SerialPort_t  *s_port    = NULL;
static EmmMotor_t    *s_motor[4] = { NULL };
static Kinematics_t   s_kin;
static int            s_running  = 1;

/* ---- 原始终端属性（退出时恢复） ---- */
static struct termios s_old_tio;
static int            s_tio_saved = 0;

/* ---- 信号处理：Ctrl+C 安全退出 ---- */

static void sigint_handler(int sig)
{
    (void)sig;
    s_running = 0;
}

/* ---- 终端设置：原始模式 + 非阻塞 ---- */

static int terminal_set_raw(void)
{
    struct termios tio;

    if (tcgetattr(STDIN_FILENO, &tio) != 0) {
        perror("tcgetattr");
        return -1;
    }

    s_old_tio   = tio;
    s_tio_saved = 1;

    /* 原始模式：不回显、不缓冲、不处理信号键 */
    tio.c_lflag &= (tcflag_t)(~(ICANON | ECHO));
    tio.c_cc[VMIN]  = 0U;
    tio.c_cc[VTIME] = 0U;

    if (tcsetattr(STDIN_FILENO, TCSANOW, &tio) != 0) {
        perror("tcsetattr");
        return -1;
    }

    /* 非阻塞读取 */
    fcntl(STDIN_FILENO, F_SETFL,
          fcntl(STDIN_FILENO, F_GETFL) | O_NONBLOCK);

    return 0;
}

static void terminal_restore(void)
{
    if (s_tio_saved) {
        tcsetattr(STDIN_FILENO, TCSANOW, &s_old_tio);
    }
}

/* ---- 读取键盘 ---- */

static int read_key(void)
{
    char c;

    if (read(STDIN_FILENO, &c, 1U) == 1) {
        return (int)(unsigned char)c;
    }
    return -1;  /* 无输入 */
}

/* ---- 初始化 / 清理 ---- */

static int init_all(const char *device)
{
#if HARDWARE_ENABLED
    size_t   i;
    int      ret;
    uint8_t addrs[MOTOR_COUNT] = {
        MOTOR_ADDR_FL, MOTOR_ADDR_FR,
        MOTOR_ADDR_RL, MOTOR_ADDR_RR
    };

    s_port = serial_port_create(device, EMM_DEFAULT_BAUDRATE);
    if (s_port == NULL) { return ERR_NULL_PTR; }

    ret = serial_port_open(s_port);
    if (ret != ERR_OK) {
        fprintf(stderr, "[错误] 无法打开 %s\n", device);
        return ret;
    }
    serial_port_flush(s_port);

    for (i = 0; i < MOTOR_COUNT; i++) {
        s_motor[i] = emm_motor_create(addrs[i], s_port);
        if (s_motor[i] == NULL) { return ERR_NULL_PTR; }
    }

    for (i = 0; i < MOTOR_COUNT; i++) {
        ret = emm_motor_enable(s_motor[i]);
        if (ret != ERR_OK) {
            fprintf(stderr, "[错误] 电机 %d 使能失败\n",
                    addrs[i]);
            return ret;
        }
    }
    printf("硬件已初始化，电机已使能\n");
#else
    (void)device;
    printf("[模拟] 键盘遥控模式（不访问硬件）\n");
    printf("[模拟] 按 X 或 Esc 退出\n\n");
#endif

    kinematics_init(&s_kin, WHEEL_RADIUS_M, HALF_LX_M,
                    HALF_LY_M, ENCODER_PPR);
    return ERR_OK;
}

static void deinit_all(void)
{
#if HARDWARE_ENABLED
    size_t i;
    for (i = 0; i < MOTOR_COUNT; i++) {
        if (s_motor[i] != NULL) {
            emm_motor_stop(s_motor[i], false);
            emm_motor_disable(s_motor[i]);
            emm_motor_destroy(s_motor[i]);
            s_motor[i] = NULL;
        }
    }
    if (s_port != NULL) {
        serial_port_close(s_port);
        serial_port_destroy(s_port);
        s_port = NULL;
    }
#else
    printf("\n[模拟] 已停止，再见\n");
#endif
}

/* ---- 发送速度指令 ---- */

static void send_velocity(const double rpm[4])
{
#if HARDWARE_ENABLED
    size_t i;
    for (i = 0; i < MOTOR_COUNT; i++) {
        (void)emm_motor_set_velocity(s_motor[i],
                     (int16_t)rpm[i],
                     EMM_DEFAULT_ACC, true);
    }
    (void)emm_motor_sync_trigger(s_motor, MOTOR_COUNT);
#else
    printf("\r[%s]  FL=%6.0f  FR=%6.0f  RL=%6.0f  RR=%6.0f",
           "模拟", rpm[0], rpm[1], rpm[2], rpm[3]);
    fflush(stdout);
#endif
}

static void send_stop(void)
{
#if HARDWARE_ENABLED
    size_t i;
    for (i = 0; i < MOTOR_COUNT; i++) {
        (void)emm_motor_stop(s_motor[i], false);
    }
#endif
}

/* ---- 打印当前状态 ---- */

static void print_status(double linear, double angular,
                         const double rpm[4])
{
#if HARDWARE_ENABLED
    printf("[硬件] v=%.2f  ω=%.2f  "
           "FL=%.0f FR=%.0f RL=%.0f RR=%.0f\n",
           linear, angular,
           rpm[0], rpm[1], rpm[2], rpm[3]);
#else
    (void)linear; (void)angular; (void)rpm;
#endif
}

/* ---- 主控制循环 ---- */

int main(int argc, char *argv[])
{
    const char      *device;
    RobotVelocity_t  cmd;
    WheelVelocity_t  wheels;
    double           rpm[4];
    double           linear_spd;
    double           angular_spd;
    int              key;

    device     = (argc >= 2) ? argv[1] : EMM_DEFAULT_DEVICE;
    linear_spd  = TELEOP_LINEAR_SPEED;
    angular_spd = TELEOP_ANGULAR_SPEED;

    /* 信号处理 */
    signal(SIGINT,  sigint_handler);
    signal(SIGTERM, sigint_handler);

    /* 终端设置 */
    if (terminal_set_raw() != 0) {
        return EXIT_FAILURE;
    }

    /* 硬件初始化 */
    if (init_all(device) != ERR_OK) {
        terminal_restore();
        deinit_all();
        return EXIT_FAILURE;
    }

    printf("\n=== 键盘遥控 ===\n");
    printf("  W=前进  S=后退  A=左移  D=右移\n");
    printf("  Q=左转  E=右转  空格=停止  X=退出\n");
    printf("  +/-=调速\n");
    printf("  当前: 线速度=%.2f m/s  角速度=%.2f rad/s\n\n",
           linear_spd, angular_spd);

    /* 控制循环 */
    while (s_running) {
        key = read_key();

        if (key < 0) {
            /* 无按键 —— 保持不变，继续循环 */
            usleep(TELEOP_LOOP_INTERVAL_US);
            continue;
        }

        /* 解析按键 */
        cmd.vx    = 0.0;
        cmd.vy    = 0.0;
        cmd.omega = 0.0;

        switch (key) {
        case 'w': case 'W': case 65:  /* ↑ 方向键上 */
            cmd.vx = linear_spd;
            break;

        case 's': case 'S': case 66:  /* ↓ 方向键下 */
            cmd.vx = -linear_spd;
            break;

        case 'a': case 'A': case 68:  /* ← 方向键左 */
            cmd.vy = linear_spd;       /* +vy = 左平移 */
            break;

        case 'd': case 'D': case 67:  /* → 方向键右 */
            cmd.vy = -linear_spd;
            break;

        case 'q': case 'Q':
            cmd.omega = angular_spd;   /* 左转 CCW */
            break;

        case 'e': case 'E':
            cmd.omega = -angular_spd;  /* 右转 CW */
            break;

        case ' ':
            /* 急停 */
            cmd.vx    = 0.0;
            cmd.vy    = 0.0;
            cmd.omega = 0.0;
            printf("\n[急停]\n");
            break;

        case '+': case '=':
            linear_spd  += TELEOP_SPEED_STEP;
            angular_spd += TELEOP_SPEED_STEP;
            printf("\r速度: v=%.2f  ω=%.2f\n",
                   linear_spd, angular_spd);
            continue;  /* 不改变运动状态 */

        case '-': case '_':
            linear_spd  -= TELEOP_SPEED_STEP;
            angular_spd -= TELEOP_SPEED_STEP;
            if (linear_spd  < 0.0)  { linear_spd  = 0.0;  }
            if (angular_spd < 0.0)  { angular_spd = 0.0;  }
            printf("\r速度: v=%.2f  ω=%.2f\n",
                   linear_spd, angular_spd);
            continue;

        case 'x': case 'X': case 27:  /* Esc */
            s_running = 0;
            continue;

        default:
            /* 未知按键，忽略 */
            continue;
        }

        /* 计算各轮转速 */
        kinematics_inverse(&s_kin, &cmd, &wheels);
        rpm[0] = wheels.rpm[0] * DIR_FACTOR_FL;
        rpm[1] = wheels.rpm[1] * DIR_FACTOR_FR;
        rpm[2] = wheels.rpm[2] * DIR_FACTOR_RL;
        rpm[3] = wheels.rpm[3] * DIR_FACTOR_RR;

        /* 发送指令 */
        send_velocity(rpm);

        print_status(cmd.vx, cmd.omega, rpm);

        usleep(TELEOP_LOOP_INTERVAL_US);
    }

    /* 停止并清理 */
    send_stop();
    terminal_restore();
    deinit_all();

    return EXIT_SUCCESS;
}
