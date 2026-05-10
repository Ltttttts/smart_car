/**
 * @file    teleop.c
 * @brief   键盘遥控底盘 + 仪表盘
 * @author  Ltttttts
 *
 * 键位:
 *   W/S    前进/后退    空格  急停
 *   A/D    左移/右移    X    退出
 *   Q/E    左转/右转    +/-  调速
 *
 * ================================================================
 * TODO: 连接硬件后将下面的 0 改为 1
 * ================================================================
 */
#define HARDWARE_ENABLED 0
#define _DEFAULT_SOURCE

#include "serial_port.h"
#include "emm_v5_driver.h"
#include "kinematics.h"
#include "dashboard.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <signal.h>
#include <math.h>

#define WHEEL_RADIUS_M      (0.05)
#define HALF_LX_M           (0.12)
#define HALF_LY_M           (0.10)
#define ENCODER_PPR         (4000)

#define TELEOP_LINEAR_SPEED    (0.30)
#define TELEOP_ANGULAR_SPEED   (0.80)
#define TELEOP_SPEED_STEP      (0.03)

#define DIR_FACTOR_FL  (1.0)
#define DIR_FACTOR_FR  (1.0)
#define DIR_FACTOR_RL  (1.0)
#define DIR_FACTOR_RR  (1.0)

#define CMD_HINT "W前进  S后退  A左移  D右移  Q左转  E右转  空格急停  X退出"

#if HARDWARE_ENABLED
static SerialPort_t  *s_port    = NULL;
static EmmMotor_t    *s_motor[4] = { NULL };
#endif
static Kinematics_t   s_kin;
static Dashboard_t    s_dash;
static int            s_running = 1;
static struct termios s_old_tio;
static int            s_tio_saved = 0;

static void sigint_handler(int sig) { (void)sig; s_running = 0; }

/* ---- 终端原始模式 ---- */

static int terminal_set_raw(void)
{
    struct termios tio;
    if (tcgetattr(STDIN_FILENO, &tio) != 0) return -1;
    s_old_tio = tio; s_tio_saved = 1;
    tio.c_lflag &= (tcflag_t)(~(ICANON | ECHO));
    tio.c_cc[VMIN]  = 0U;
    tio.c_cc[VTIME] = 0U;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &tio) != 0) return -1;
    fcntl(STDIN_FILENO, F_SETFL,
          fcntl(STDIN_FILENO, F_GETFL) | O_NONBLOCK);
    return 0;
}

static void terminal_restore(void)
{
    if (s_tio_saved)
        tcsetattr(STDIN_FILENO, TCSANOW, &s_old_tio);
}

static int read_key(void)
{
    char c;
    if (read(STDIN_FILENO, &c, 1U) == 1) return (int)(unsigned char)c;
    return -1;
}

/* ---- 初始化 / 清理 ---- */

static int init_all(const char *device)
{
#if HARDWARE_ENABLED
    size_t i; int ret;
    uint8_t addrs[MOTOR_COUNT] = {
        MOTOR_ADDR_FL, MOTOR_ADDR_FR,
        MOTOR_ADDR_RL, MOTOR_ADDR_RR
    };
    s_port = serial_port_create(device, EMM_DEFAULT_BAUDRATE);
    if (s_port == NULL) return ERR_NULL_PTR;
    ret = serial_port_open(s_port);
    if (ret != ERR_OK) return ret;
    serial_port_flush(s_port);
    for (i = 0; i < MOTOR_COUNT; i++) {
        s_motor[i] = emm_motor_create(addrs[i], s_port);
        if (s_motor[i] == NULL) return ERR_NULL_PTR;
    }
    for (i = 0; i < MOTOR_COUNT; i++) emm_motor_enable(s_motor[i]);
#else
    (void)device;
#endif
    kinematics_init(&s_kin, WHEEL_RADIUS_M, HALF_LX_M,
                    HALF_LY_M, ENCODER_PPR);
    dash_init(&s_dash, WHEEL_RADIUS_M, "键盘遥控");
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
        }
    }
    if (s_port != NULL) {
        serial_port_close(s_port);
        serial_port_destroy(s_port);
    }
#endif
    dash_cleanup();
}

/* ---- 发送速度 ---- */

static void send_velocity(const double rpm[4])
{
#if HARDWARE_ENABLED
    size_t i;
    for (i = 0; i < MOTOR_COUNT; i++)
        (void)emm_motor_set_velocity(s_motor[i],
                     (int16_t)rpm[i],
                     EMM_DEFAULT_ACC, true);
    (void)emm_motor_sync_trigger(s_motor, MOTOR_COUNT);
#endif
}

static void send_stop(void)
{
#if HARDWARE_ENABLED
    size_t i;
    for (i = 0; i < MOTOR_COUNT; i++)
        (void)emm_motor_stop(s_motor[i], false);
#endif
}

/* ============ 主函数 ============ */

int main(int argc, char *argv[])
{
    const char *device;
    double      linear_spd, angular_spd;
    int         key;
    const char *mode_str;

    device      = (argc >= 2) ? argv[1] : EMM_DEFAULT_DEVICE;
    linear_spd  = TELEOP_LINEAR_SPEED;
    angular_spd = TELEOP_ANGULAR_SPEED;
    mode_str    = "模拟";

    signal(SIGINT, sigint_handler);
    signal(SIGTERM, sigint_handler);
    terminal_set_raw();

    if (init_all(device) != ERR_OK) {
        terminal_restore();
        deinit_all();
        return EXIT_FAILURE;
    }

    /* 首帧 dash_update（绘制框架） */
    dash_update(&s_dash, 0, 0, 0, (double[]){0,0,0,0},
                0.0, mode_str, CMD_HINT);

    while (s_running) {
        RobotVelocity_t cmd = { 0, 0, 0 };
        WheelVelocity_t wheels;
        double          rpm[4];
        double          zero[4] = { 0, 0, 0, 0 };

        key = read_key();
        if (key < 0) {
            usleep(20000U);
            continue;
        }

        switch (key) {
        case 'w': case 'W': cmd.vx =  linear_spd; break;
        case 's': case 'S': cmd.vx = -linear_spd; break;
        case 'a': case 'A': cmd.vy =  linear_spd; break;
        case 'd': case 'D': cmd.vy = -linear_spd; break;
        case 'q': case 'Q': cmd.omega =  angular_spd; break;
        case 'e': case 'E': cmd.omega = -angular_spd; break;

        case ' ':
            send_stop();
            dash_update(&s_dash, 0, 0, 0, zero,
                        0.0, mode_str, "已急停");
            usleep(20000U);
            continue;

        case '+': case '=':
            linear_spd  += TELEOP_SPEED_STEP;
            angular_spd += TELEOP_SPEED_STEP;
            dash_update(&s_dash, cmd.vx, cmd.vy, cmd.omega,
                        zero, 0.0, mode_str, CMD_HINT);
            continue;

        case '-': case '_':
            linear_spd  -= TELEOP_SPEED_STEP;
            angular_spd -= TELEOP_SPEED_STEP;
            if (linear_spd  < 0.0) linear_spd  = 0.0;
            if (angular_spd < 0.0) angular_spd = 0.0;
            dash_update(&s_dash, cmd.vx, cmd.vy, cmd.omega,
                        zero, 0.0, mode_str, CMD_HINT);
            continue;

        case 'x': case 'X': case 27:
            s_running = 0; continue;

        default: continue;
        }

        /* 运动学解算 */
        kinematics_inverse(&s_kin, &cmd, &wheels);
        rpm[0] = wheels.rpm[0] * DIR_FACTOR_FL;
        rpm[1] = wheels.rpm[1] * DIR_FACTOR_FR;
        rpm[2] = wheels.rpm[2] * DIR_FACTOR_RL;
        rpm[3] = wheels.rpm[3] * DIR_FACTOR_RR;

        send_velocity(rpm);
        dash_update(&s_dash, cmd.vx, cmd.vy, cmd.omega,
                    rpm, 0.0, mode_str, CMD_HINT);

        usleep(20000U);
    }

    send_stop();
    terminal_restore();
    deinit_all();
    return EXIT_SUCCESS;
}
