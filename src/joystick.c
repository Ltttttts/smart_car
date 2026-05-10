/**
 * @file    joystick.c
 * @brief   手柄遥控底盘 + 仪表盘 (evdev 协议，自动检测)
 * @author  Ltttttts
 *
 * 用法:
 *   ./joystick [串口设备]                     # 正常控制
 *   ./joystick /dev/ttyUSB0 --calibrate        # 查看 evdev 码
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
#include <signal.h>
#include <math.h>
#include <time.h>
#include <dirent.h>
#include <linux/input.h>

#define WHEEL_RADIUS_M      (0.05)
#define HALF_LX_M           (0.12)
#define HALF_LY_M           (0.10)
#define ENCODER_PPR         (4000)

#define JOY_MAX_LINEAR_SPEED   (0.50)
#define JOY_MAX_ANGULAR_SPEED  (1.20)
#define JOY_DEADZONE           (8000)

#define DIR_FACTOR_FL  (1.0)
#define DIR_FACTOR_FR  (1.0)
#define DIR_FACTOR_RL  (1.0)
#define DIR_FACTOR_RR  (1.0)

#define CMD_HINT "左摇杆:前进/后退+平移  右摇杆X:旋转  A急停  B退出  右扳机加速"

#if HARDWARE_ENABLED
static SerialPort_t  *s_port    = NULL;
static EmmMotor_t    *s_motor[4] = { NULL };
#endif
static Kinematics_t   s_kin;
static Dashboard_t    s_dash;
static int            s_joy_fd   = -1;
static int            s_running  = 1;
static int            s_axis[64] = { 0 };
static int            s_axis_center[64] = { 0 };
static uint8_t        s_btn[512] = { 0 };

static void sigint_handler(int sig) { (void)sig; s_running = 0; }

static double now_sec(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1.0e9;
}

/* ============ evdev 设备检测 ============ */

static int has_abs_capability(int fd)
{
    unsigned long bits[(ABS_MAX + 1U) /
                      (sizeof(unsigned long) * 8U) + 1U];
    unsigned int i;
    int found;
    memset(bits, 0, sizeof(bits));
    if (ioctl(fd, EVIOCGBIT(EV_ABS, sizeof(bits)), bits) < 0)
        return 0;
    found = 0;
    for (i = (unsigned int)ABS_X; i <= (unsigned int)ABS_RZ; i++) {
        if (bits[i / (sizeof(unsigned long) * 8U)] &
            (1UL << (i % (sizeof(unsigned long) * 8U))))
            found++;
    }
    return (found >= 4);
}

static int auto_detect_joy(char *path, size_t path_size)
{
    DIR *dir = opendir("/dev/input");
    struct dirent *entry;
    char full[256];
    if (!dir) return -1;

    while ((entry = readdir(dir)) != NULL) {
        int fd;
        if (strncmp(entry->d_name, "event", 5U) != 0) continue;
        snprintf(full, sizeof(full), "/dev/input/%s", entry->d_name);
        fd = open(full, O_RDONLY | O_NONBLOCK);
        if (fd < 0) continue;
        if (has_abs_capability(fd)) {
            char name[256] = {0};
            ioctl(fd, EVIOCGNAME(sizeof(name) - 1U), name);
            printf("检测到手柄: %s (%s)\n", name, full);
            close(fd);
            snprintf(path, path_size, "%s", full);
            closedir(dir); return 0;
        }
        close(fd);
    }
    closedir(dir); return -1;
}

/* ============ 校准模式 ============ */

static const char *ev_code_str(int type, int code)
{
    if (type == EV_ABS) {
        if ((unsigned)code <= ABS_RZ) {
            static const char *n[] = {
                [ABS_X]="ABS_X", [ABS_Y]="ABS_Y", [ABS_Z]="ABS_Z",
                [ABS_RX]="ABS_RX", [ABS_RY]="ABS_RY", [ABS_RZ]="ABS_RZ"
            };
            if (n[code]) return n[code];
        }
        if (code == ABS_HAT0X) return "HAT_X";
        if (code == ABS_HAT0Y) return "HAT_Y";
    }
    if (type == EV_KEY) {
        switch (code) {
        case BTN_A: return "BTN_A"; case BTN_B: return "BTN_B";
        case BTN_X: return "BTN_X"; case BTN_Y: return "BTN_Y";
        case BTN_TL: return "BTN_TL"; case BTN_TR: return "BTN_TR";
        case BTN_SELECT: return "BTN_SELECT";
        case BTN_START: return "BTN_START";
        default: break;
        }
    }
    return NULL;
}

static int calibrate_mode(void)
{
    struct input_event ev;
    printf("=== 手柄校准模式 ===\n");
    printf("操作各摇杆和按键，查看下方码值。按 Ctrl+C 退出。\n\n");
    printf("  ABS_X=0  ABS_Y=1  ABS_Z=2  ABS_RX=3  ABS_RY=4  ABS_RZ=5\n\n");

    while (s_running) {
        while (read(s_joy_fd, &ev, sizeof(ev)) == (ssize_t)sizeof(ev)) {
            if (ev.type == EV_ABS)
                printf("  ABS[%s (%d)] = %d\n",
                       ev_code_str(EV_ABS, ev.code) ?: "?", ev.code, ev.value);
            else if (ev.type == EV_KEY && ev.value)
                printf("  [%s (%d)] = %d\n",
                       ev_code_str(EV_KEY, ev.code) ?: "?", ev.code, ev.value);
        }
        usleep(50000U);
    }
    return 0;
}

/* ---- 映射 evdev 轴值（带自动中心校准） ---- */

static double map_evdev_axis(int raw, int center, double max_val)
{
    int c = raw - center;
    if (abs(c) < JOY_DEADZONE) return 0.0;
    return ((double)c / 32767.0) * max_val;
}

/* ---- 初始化 / 清理 ---- */

static int init_all(const char *device)
{
    char joy_path[128];

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
        emm_motor_enable(s_motor[i]);
    }
#else
    (void)device;
#endif

    if (auto_detect_joy(joy_path, sizeof(joy_path)) != 0) {
        fprintf(stderr, "未检测到手柄\n");
        return ERR_SERIAL_OPEN;
    }
    s_joy_fd = open(joy_path, O_RDONLY | O_NONBLOCK);
    if (s_joy_fd < 0) return ERR_SERIAL_OPEN;

    /* 自动校准：采样 200ms 的静止轴值作为中心 */
    {
        struct input_event ev;
        long long sum[64] = {0};
        int cnt[64] = {0};
        double deadline = now_sec() + 0.2;
        while (now_sec() < deadline) {
            while (read(s_joy_fd, &ev, sizeof(ev)) == (ssize_t)sizeof(ev)) {
                if (ev.type == EV_ABS && (unsigned)ev.code < 64) {
                    sum[ev.code] += ev.value;
                    cnt[ev.code]++;
                }
            }
            usleep(10000U);
        }
        for (int i = 0; i < 64; i++)
            if (cnt[i] > 0) s_axis_center[i] = (int)(sum[i] / cnt[i]);
    }

    kinematics_init(&s_kin, WHEEL_RADIUS_M, HALF_LX_M,
                    HALF_LY_M, ENCODER_PPR);
    dash_init(&s_dash, WHEEL_RADIUS_M, "手柄遥控");
    return ERR_OK;
}

static void deinit_all(void)
{
    if (s_joy_fd >= 0) { close(s_joy_fd); s_joy_fd = -1; }
#if HARDWARE_ENABLED
    size_t i;
    for (i = 0; i < MOTOR_COUNT; i++) {
        if (s_motor[i] != NULL) {
            emm_motor_stop(s_motor[i], false);
            emm_motor_disable(s_motor[i]);
            emm_motor_destroy(s_motor[i]);
        }
    }
    if (s_port != NULL) { serial_port_close(s_port); serial_port_destroy(s_port); }
#endif
    dash_cleanup();
}

/* ---- 发送速度 ---- */

static void send_velocity(const double rpm[4])
{
#if HARDWARE_ENABLED
    size_t i;
    for (i = 0; i < MOTOR_COUNT; i++)
        (void)emm_motor_set_velocity(s_motor[i], (int16_t)rpm[i],
                                     EMM_DEFAULT_ACC, true);
    (void)emm_motor_sync_trigger(s_motor, MOTOR_COUNT);
#else
    (void)rpm;
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
    const char     *device;
    const char     *mode_str;
    struct input_event ev;

    device  = (argc >= 2) ? argv[1] : EMM_DEFAULT_DEVICE;
    mode_str = "模拟";

    signal(SIGINT, sigint_handler);
    signal(SIGTERM, sigint_handler);

    if (init_all(device) != ERR_OK) {
        deinit_all();
        return EXIT_FAILURE;
    }

    if (argc >= 3 && strcmp(argv[2], "--calibrate") == 0)
        return calibrate_mode();

    dash_update(&s_dash, 0, 0, 0, (double[]){0,0,0,0},
                0.0, mode_str, CMD_HINT);

    while (s_running) {
        double          boost, sp_vx, sp_vy, sp_om;
        RobotVelocity_t cmd = { 0, 0, 0 };
        WheelVelocity_t wheels;
        double          rpm[4];
        double          zero[4] = { 0, 0, 0, 0 };

        while (read(s_joy_fd, &ev, sizeof(ev)) == (ssize_t)sizeof(ev)) {
            if (ev.type == EV_ABS && (unsigned)ev.code < 64)
                s_axis[ev.code] = ev.value;
            else if (ev.type == EV_KEY && (unsigned)ev.code < 512)
                s_btn[ev.code] = (uint8_t)ev.value;
        }

        if (s_btn[BTN_B]) break;

        if (s_btn[BTN_A]) {
            send_stop();
            dash_update(&s_dash, 0, 0, 0, zero,
                        0.0, mode_str, "已急停");
        } else {
            boost = (s_axis[ABS_Z] > 50000) ? 2.0 : 1.0;

            sp_vx = map_evdev_axis(s_axis[ABS_Y],
                                   s_axis_center[ABS_Y],
                                   JOY_MAX_LINEAR_SPEED * boost);
            sp_vy = map_evdev_axis(s_axis[ABS_X],
                                   s_axis_center[ABS_X],
                                   JOY_MAX_LINEAR_SPEED * boost);
            sp_om = map_evdev_axis(s_axis[ABS_RX],
                                   s_axis_center[ABS_RX],
                                   JOY_MAX_ANGULAR_SPEED * boost);

            cmd.vx    = -sp_vx;
            cmd.vy    = sp_vy;
            cmd.omega = -sp_om;

            kinematics_inverse(&s_kin, &cmd, &wheels);
            rpm[0] = wheels.rpm[0] * DIR_FACTOR_FL;
            rpm[1] = wheels.rpm[1] * DIR_FACTOR_FR;
            rpm[2] = wheels.rpm[2] * DIR_FACTOR_RL;
            rpm[3] = wheels.rpm[3] * DIR_FACTOR_RR;

            send_velocity(rpm);
            dash_update(&s_dash, cmd.vx, cmd.vy, cmd.omega,
                        rpm, 0.0, mode_str, CMD_HINT);
        }

        usleep(20000U);
    }

    send_stop();
    deinit_all();
    return EXIT_SUCCESS;
}
