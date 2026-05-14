/**
 * @file    app/joystick/main.c
 * @brief   手柄遥控底盘程序
 * @author  Ltttttts
 *
 * 功能：
 *   1. 自动检测 USB 手柄（遍历 /dev/input/event*，找有摇杆能力设备）
 *   2. 读取摇杆和按键状态
 *   3. 换算成底盘速度（vx/vy/omega）
 *   4. 运动学逆解算 → 四个轮子的 RPM
 *   5. 通过串口发给 EMM_V5 驱动器
 *
 * 手柄映射：
 *   左摇杆 Y = 前进/后退   左摇杆 X = 左右平移
 *   右摇杆 X = 旋转        右扳机(Z轴) = 2倍加速
 *   A键 = 急停              B键 = 退出
 *
 * 命令行参数：
 *   ./joystick [串口设备]         正常模式
 *   ./joystick [串口设备] --calibrate   手柄校准
 *   ./joystick [串口设备] --test-motors 电机测试
 *   ./joystick [串口设备] --camera      同时启动视频推流
 *   可配合 --camera-device/res/fps/quality/port/dir/www 修改推流参数
 */

#define _DEFAULT_SOURCE

#include "hal/serial_port.h"
#include "driver/emm_v5.h"
#include "control/kinematics.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <time.h>
#include <dirent.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <linux/input.h>

/* ========== 底盘物理参数 ========== */
#define WHEEL_RADIUS_M      (0.05)   /* 轮子半径（米） */
#define HALF_LX_M           (0.12)   /* 前后半轴距（米） */
#define HALF_LY_M           (0.10)   /* 左右半轴距（米） */
#define ENCODER_PPR         (4000)   /* 编码器每转脉冲数 */

/* ========== 手柄控制参数 ========== */
#define JOY_MAX_LINEAR_SPEED   (1.00)   /* 最大线速度（m/s） */
#define JOY_MAX_ANGULAR_SPEED  (2.00)   /* 最大角速度（rad/s） */
#define JOY_DEADZONE           (8000)   /* 摇杆死区（-8000~8000 视为零） */

/* ========== 轮子方向修正系数 ========== */
/* 如果某个电机装反了，把对应系数改成 -1.0 即可反转 */
#define DIR_FACTOR_FL  (1.0)
#define DIR_FACTOR_FR  (1.0)
#define DIR_FACTOR_RL  (1.0)
#define DIR_FACTOR_RR  (1.0)

/* ========== 摄像头推流默认参数 ========== */
#define CAMERA_DEFAULT_DIR      \
    "/home/orangepi/Desktop/mjpg/mjpg-streamer/mjpg-streamer-experimental"
#define CAMERA_DEFAULT_DEVICE   "/dev/video0"
#define CAMERA_DEFAULT_RES      "640x480"
#define CAMERA_DEFAULT_FPS      "30"
#define CAMERA_DEFAULT_QUALITY  "60"
#define CAMERA_DEFAULT_PORT     "8080"
#define CAMERA_DEFAULT_WWW      "./www"
#define CAMERA_ARG_BUF_SIZE     (256U)

/* ========== 全局变量 ========== */
static SerialPort_t  *s_port    = NULL;
static EmmMotor_t    *s_motor[4] = { NULL };
static Kinematics_t   s_kin;
static int            s_joy_fd   = -1;          /* 手柄设备的文件描述符 */
static int            s_running  = 1;           /* 控制循环是否继续 */
static int            s_axis[64] = { 0 };       /* 各摇杆轴的当前值 */
static int            s_axis_center[64] = { 0 };/* 各摇杆轴的中心校准值 */
static uint8_t        s_btn[512] = { 0 };       /* 各按键的状态 */
static pid_t          s_camera_pid = -1;        /* 摄像头推流子进程 PID */

typedef struct {
    bool        enabled;
    const char *dir;
    const char *device;
    const char *resolution;
    const char *www_dir;
    const char *fps;
    const char *quality;
    const char *port;
} CameraOptions_t;

/* 四个电机的总线地址，顺序 [FL, FR, RL, RR] */
static const uint8_t s_motor_addrs[MOTOR_COUNT] = {
    MOTOR_ADDR_FL, MOTOR_ADDR_FR,
    MOTOR_ADDR_RL, MOTOR_ADDR_RR
};

/* Ctrl+C 信号处理函数：设置退出标志 */
static void sigint_handler(int sig) { (void)sig; s_running = 0; }

static void print_usage(const char *prog)
{
    printf("用法: %s [串口设备] [选项]\n\n", prog);
    printf("串口设备:\n");
    printf("  电机串口路径，默认 %s\n\n", EMM_DEFAULT_DEVICE);
    printf("模式选项 (互斥):\n");
    printf("  --test-motors       电机单独测试模式\n");
    printf("  --calibrate          手柄校准模式\n\n");
    printf("摄像头选项:\n");
    printf("  --camera             启动视频推流\n");
    printf("  --camera-device <设备>   摄像头设备，默认 %s\n",
           CAMERA_DEFAULT_DEVICE);
    printf("  --camera-res <宽x高>     分辨率，默认 %s\n",
           CAMERA_DEFAULT_RES);
    printf("  --camera-fps <帧率>      默认 %s\n",
           CAMERA_DEFAULT_FPS);
    printf("  --camera-quality <质量>  JPEG 质量，默认 %s\n",
           CAMERA_DEFAULT_QUALITY);
    printf("  --camera-port <端口>     HTTP 端口，默认 %s\n",
           CAMERA_DEFAULT_PORT);
    printf("  --camera-dir <路径>      mjpg_streamer 目录，默认\n"
           "                          %s\n",
           CAMERA_DEFAULT_DIR);
    printf("  --camera-www <路径>      Web 目录，默认 %s\n",
           CAMERA_DEFAULT_WWW);
    printf("\n");
    printf("其他:\n");
    printf("  --help, -h           显示本帮助\n\n");
    printf("示例:\n");
    printf("  %s /dev/ttyS1\n", prog);
    printf("  %s /dev/ttyS1 --calibrate\n", prog);
    printf("  %s /dev/ttyS1 --test-motors\n", prog);
    printf("  %s /dev/ttyS1 --camera\n", prog);
    printf("  %s /dev/ttyS1 --camera --camera-device /dev/video0"
           " --camera-res 640x480 --camera-port 8080\n", prog);
    printf("\n手柄操作:\n");
    printf("  左摇杆 Y  = 前进/后退    左摇杆 X  = 左右平移\n");
    printf("  右摇杆 X  = 旋转         右扳机    = 2倍加速\n");
    printf("  A键 = 急停               B键 = 退出\n");
}

static int is_help_arg(const char *s)
{
    return (strcmp(s, "--help") == 0 || strcmp(s, "-h") == 0);
}

static int require_option_value(int argc, char *argv[], int index)
{
    if ((index + 1) >= argc) {
        fprintf(stderr, "[错误] %s 需要一个参数\n", argv[index]);
        return -1;
    }
    return 0;
}

static int parse_main_options(int argc, char *argv[], bool *test_mode,
                              bool *calibrate_enabled,
                              CameraOptions_t *camera)
{
    int i;

    if (test_mode == NULL || calibrate_enabled == NULL || camera == NULL) {
        return -1;
    }

    *test_mode = false;
    *calibrate_enabled = false;
    memset(camera, 0, sizeof(*camera));

    for (i = 2; i < argc; i++) {
        if (is_help_arg(argv[i])) {
            print_usage(argv[0]);
            exit(EXIT_SUCCESS);
        } else if (strcmp(argv[i], "--test-motors") == 0) {
            *test_mode = true;
        } else if (strcmp(argv[i], "--calibrate") == 0) {
            *calibrate_enabled = true;
        } else if (strcmp(argv[i], "--camera") == 0) {
            camera->enabled = true;
        } else if (strcmp(argv[i], "--camera-dir") == 0) {
            if (require_option_value(argc, argv, i) != 0)
                { print_usage(argv[0]); return -1; }
            camera->dir = argv[++i];
        } else if (strcmp(argv[i], "--camera-device") == 0) {
            if (require_option_value(argc, argv, i) != 0)
                { print_usage(argv[0]); return -1; }
            camera->device = argv[++i];
        } else if (strcmp(argv[i], "--camera-res") == 0) {
            if (require_option_value(argc, argv, i) != 0)
                { print_usage(argv[0]); return -1; }
            camera->resolution = argv[++i];
        } else if (strcmp(argv[i], "--camera-www") == 0) {
            if (require_option_value(argc, argv, i) != 0)
                { print_usage(argv[0]); return -1; }
            camera->www_dir = argv[++i];
        } else if (strcmp(argv[i], "--camera-fps") == 0) {
            if (require_option_value(argc, argv, i) != 0)
                { print_usage(argv[0]); return -1; }
            camera->fps = argv[++i];
        } else if (strcmp(argv[i], "--camera-quality") == 0) {
            if (require_option_value(argc, argv, i) != 0)
                { print_usage(argv[0]); return -1; }
            camera->quality = argv[++i];
        } else if (strcmp(argv[i], "--camera-port") == 0) {
            if (require_option_value(argc, argv, i) != 0)
                { print_usage(argv[0]); return -1; }
            camera->port = argv[++i];
        } else {
            fprintf(stderr, "[错误] 未知选项: %s\n\n", argv[i]);
            print_usage(argv[0]);
            return -1;
        }
    }

    return 0;
}

static int start_camera_stream(const CameraOptions_t *camera)
{
    const char *dir;
    const char *dev;
    const char *res;
    const char *fps;
    const char *qual;
    const char *port;
    const char *www;
    char input_arg[CAMERA_ARG_BUF_SIZE];
    char output_arg[CAMERA_ARG_BUF_SIZE];
    int  ret;
    pid_t pid;

    if (camera == NULL || !camera->enabled) {
        return 0;
    }

    dir  = camera->dir        ? camera->dir        : CAMERA_DEFAULT_DIR;
    dev  = camera->device     ? camera->device     : CAMERA_DEFAULT_DEVICE;
    res  = camera->resolution ? camera->resolution : CAMERA_DEFAULT_RES;
    fps  = camera->fps        ? camera->fps        : CAMERA_DEFAULT_FPS;
    qual = camera->quality    ? camera->quality    : CAMERA_DEFAULT_QUALITY;
    port = camera->port       ? camera->port       : CAMERA_DEFAULT_PORT;
    www  = camera->www_dir    ? camera->www_dir    : CAMERA_DEFAULT_WWW;

    ret = snprintf(input_arg, sizeof(input_arg),
                   "./input_uvc.so -d %s -r %s -f %s -q %s",
                   dev, res, fps, qual);
    if (ret < 0 || (size_t)ret >= sizeof(input_arg)) {
        fprintf(stderr, "[错误] 摄像头输入参数过长\n");
        return -1;
    }

    ret = snprintf(output_arg, sizeof(output_arg),
                   "./output_http.so -w %s -p %s",
                   www, port);
    if (ret < 0 || (size_t)ret >= sizeof(output_arg)) {
        fprintf(stderr, "[错误] 摄像头输出参数过长\n");
        return -1;
    }

    pid = fork();
    if (pid < 0) {
        perror("[错误] fork 摄像头子进程失败");
        return -1;
    }
    if (pid == 0) {
        if (chdir(dir) != 0) {
            perror("[错误] 无法进入 mjpg_streamer 目录");
            _exit(EXIT_FAILURE);
        }
        execlp("./mjpg_streamer", "./mjpg_streamer",
               "-i", input_arg,
               "-o", output_arg,
               (char *)NULL);
        perror("[错误] 启动 mjpg_streamer 失败");
        _exit(EXIT_FAILURE);
    }

    s_camera_pid = pid;
    printf("[摄像头] 已启动推流 PID=%ld\n", (long)pid);
    fflush(stdout);
    return 0;
}

static void stop_camera_stream(void)
{
    int status;
    int ret;

    if (s_camera_pid <= 0) {
        return;
    }

    (void)kill(s_camera_pid, SIGTERM);
    do {
        ret = waitpid(s_camera_pid, &status, 0);
    } while (ret < 0 && errno == EINTR);
    if (ret < 0) {
        perror("[警告] 等待摄像头子进程退出失败");
    }
    s_camera_pid = -1;
}

/**
 * @brief  获取单调时钟的当前时间（秒）
 */
static double now_sec(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1.0e9;
}

/* ============ evdev 手柄设备检测 ============ */

/**
 * @brief  检查一个输入设备是否有摇杆能力（ABS 轴）
 *
 * 通过 ioctl 读取设备的 ABS 功能位图，
 * 如果 ABS_X, ABS_Y, ABS_Z, ABS_RX 等都有，就认为是手柄。
 *
 * @param  fd  已打开的输入设备文件描述符
 * @return 1 = 是手柄，0 = 不是
 */
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

/**
 * @brief  自动检测第一个可用的手柄设备
 *
 * 遍历 /dev/input 目录下所有 event* 设备，
 * 找到第一个具有摇杆能力的设备。
 *
 * @param  path       输出缓冲区：存放检测到的设备路径
 * @param  path_size  缓冲区大小
 * @return 0 成功，-1 没有找到手柄
 */
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

/**
 * @brief  把 evdev 的事件码转换成可读的字符串（用于校准显示）
 */
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

/**
 * @brief  手柄校准模式：显示所有摇杆和按键的原始数值
 *
 * 用户通过观察这个输出来确认手柄的映射关系。
 * 校准模式下小车不动，只读数据、显示。
 */
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

/* ========== 摇杆值映射 ========== */

/**
 * @brief  把 evdev 的原始摇杆值映射到 [-max_val, +max_val] 范围
 *
 * 原始值范围：-32768 ~ 32767
 * 映射过程：
 *   1. 减去中心校准值
 *   2. 死区过滤（小范围波动视为 0）
 *   3. 归一化到 [0, 1] 再乘以 max_val
 *
 * @param  raw       evdev 原始值
 * @param  center    中心校准值（静止时的读数）
 * @param  max_val   输出范围的最大值
 * @return 映射后的速度值
 */
static double map_evdev_axis(int raw, int center, double max_val)
{
    int c = raw - center;
    if (abs(c) < JOY_DEADZONE) return 0.0;
    return ((double)c / 32767.0) * max_val;
}

/* ============ 初始化 / 清理 ============ */

/**
 * @brief  初始化电机（创建 + 配置 + 使能）
 */
static int init_motors(const char *device)
{
    size_t i; int ret;
    s_port = serial_port_create(device, EMM_DEFAULT_BAUDRATE);
    if (s_port == NULL) return ERR_NULL_PTR;
    ret = serial_port_open(s_port);
    if (ret != ERR_OK) return ret;
    serial_port_flush(s_port);

    for (i = 0; i < MOTOR_COUNT; i++) {
        s_motor[i] = emm_motor_create(s_motor_addrs[i], s_port);
        if (s_motor[i] == NULL) return ERR_NULL_PTR;
        /* 设为闭环模式（2），让编码器参与位置/速度控制 */
        (void)emm_motor_set_ctrl_mode(s_motor[i], 2U, false);
        usleep(10000U);
        (void)emm_motor_enable(s_motor[i]);
        usleep(10000U);
    }
    usleep(100000U);  /* 等待所有电机稳定 */
    return ERR_OK;
}

/**
 * @brief  初始化所有子系统（电机 + 手柄 + 运动学）
 */
static int init_all(const char *device)
{
    char joy_path[128];
    int ret;

    ret = init_motors(device);
    if (ret != ERR_OK) return ret;

    if (auto_detect_joy(joy_path, sizeof(joy_path)) != 0) {
        fprintf(stderr, "未检测到手柄\n");
        return ERR_SERIAL_OPEN;
    }
    s_joy_fd = open(joy_path, O_RDONLY | O_NONBLOCK);
    if (s_joy_fd < 0) return ERR_SERIAL_OPEN;

    /* 自动校准：采样 200ms 的静止摇杆值作为中心基准 */
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
    return ERR_OK;
}

/**
 * @brief  反初始化：停止所有电机、关闭串口和手柄
 */
static void deinit_all(void)
{
    if (s_joy_fd >= 0) { close(s_joy_fd); s_joy_fd = -1; }
    size_t i;
    for (i = 0; i < MOTOR_COUNT; i++) {
        if (s_motor[i] != NULL) {
            emm_motor_stop(s_motor[i], false);
            emm_motor_disable(s_motor[i]);
            emm_motor_destroy(s_motor[i]);
        }
    }
    if (s_port != NULL) { serial_port_close(s_port); serial_port_destroy(s_port); }
}

/* ========== 发送速度指令 ========== */

/**
 * @brief  把四个轮子的 RPM 发给电机驱动器
 *
 * 用同步模式：先让每个电机缓冲指令，最后一次性触发所有电机同时启动。
 * 这样四个轮子一起开始转，运动更平滑。
 *
 * @param  rpm  四个轮子的目标转速 [FL, FR, RL, RR]
 */
static void send_velocity(const double rpm[4])
{
    size_t i;
    for (i = 0; i < MOTOR_COUNT; i++) {
        (void)emm_motor_set_velocity(s_motor[i], (int16_t)rpm[i],
                                     200U, true);
    }
    (void)emm_motor_sync_trigger(s_motor, MOTOR_COUNT);
}

/**
 * @brief  让所有电机立即停止
 */
static void send_stop(void)
{
    size_t i;
    for (i = 0; i < MOTOR_COUNT; i++)
        (void)emm_motor_stop(s_motor[i], false);
}

/* ============ 电机单独测试模式 ============ */

static const char *s_motor_names[4] = {
    "FL (前左)", "FR (前右)", "RL (后左)", "RR (后右)"
};

/**
 * @brief  测试单个电机：正转或反转指定时间
 */
static void test_single_motor(EmmMotor_t *m, const char *name,
                              int16_t rpm, int duration_ms)
{
    printf("  ▶ %s : RPM=%+d (%d ms)\n", name, (int)rpm, duration_ms);
    (void)emm_motor_set_velocity(m, rpm, EMM_DEFAULT_ACC, false);
    /* 分小段睡眠，期间可以 Ctrl+C 中断 */
    for (int i = 0; i < duration_ms / 200 && s_running; i++)
        usleep(200000U);
    (void)emm_motor_stop(m, false);
    usleep(300000U);
}

/**
 * @brief  电机测试模式：逐一检测每个电机正反转
 */
static int test_motors_mode(void)
{
    const int16_t test_rpm = 200;

    printf("\n========== 电机单独测试模式 ==========\n");
    printf("测试流程：每个电机正转 2s → 停 0.5s → 反转 2s → 停 1s\n");
    printf("请观察车轮旋转方向是否符合预期。\n");
    printf("按 Ctrl+C 中止测试。\n\n");

    for (int round = 0; round < 2 && s_running; round++) {
        for (size_t i = 0; i < MOTOR_COUNT && s_running; i++) {
            printf("[%d/%d] 测试 %s (地址 %d)\n",
                   (int)i + 1, MOTOR_COUNT,
                   s_motor_names[i], s_motor_addrs[i]);
            test_single_motor(s_motor[i], s_motor_names[i],
                              test_rpm, 2000);
            if (!s_running) break;
            test_single_motor(s_motor[i], s_motor_names[i],
                              -test_rpm, 2000);
            printf("\n");
        }
    }

    printf("========== 测试结束 ==========\n\n");
    return 0;
}

/* ============ 主函数 ============ */

int main(int argc, char *argv[])
{
    const char     *device;
    struct input_event ev;
    bool            test_mode = false;
    bool            calibrate = false;
    CameraOptions_t camera;

    /* --help / -h 直接显示帮助 */
    if (argc >= 2 && is_help_arg(argv[1])) {
        print_usage(argv[0]);
        return EXIT_SUCCESS;
    }

    /* 第一个参数 = 串口设备路径，没传就用默认 */
    device  = (argc >= 2) ? argv[1] : EMM_DEFAULT_DEVICE;

    signal(SIGINT, sigint_handler);
    signal(SIGTERM, sigint_handler);

    if (parse_main_options(argc, argv, &test_mode, &calibrate,
                           &camera) != 0) {
        return EXIT_FAILURE;
    }

    if (test_mode) {
        if (init_motors(device) != ERR_OK) { deinit_all(); return EXIT_FAILURE; }
        int ret = test_motors_mode();
        deinit_all();
        return ret;
    }

    if (init_all(device) != ERR_OK) {
        deinit_all();
        return EXIT_FAILURE;
    }

    /* 校准模式：--calibrate 参数 */
    if (calibrate) {
        int ret = calibrate_mode();
        deinit_all();
        return ret;
    }

    if (start_camera_stream(&camera) != 0) {
        deinit_all();
        return EXIT_FAILURE;
    }

    /* ---- 主控制循环 ---- */
    while (s_running) {
        double          boost, sp_vx, sp_vy, sp_om;
        RobotVelocity_t cmd = { 0, 0, 0 };
        WheelVelocity_t wheels;
        double          rpm[4];

        /* 读取所有待处理的 evdev 事件（非阻塞） */
        while (read(s_joy_fd, &ev, sizeof(ev)) == (ssize_t)sizeof(ev)) {
            if (ev.type == EV_ABS && (unsigned)ev.code < 64)
                s_axis[ev.code] = ev.value;
            else if (ev.type == EV_KEY && (unsigned)ev.code < 512)
                s_btn[ev.code] = (uint8_t)ev.value;
        }

        if (s_btn[BTN_B]) break;   /* B 键退出 */

        if (s_btn[BTN_A]) {
            /* A 键急停 */
            send_stop();
        } else {
            /* ---- 正常控制流程 ---- */
            /* 右扳机（ABS_Z）按下时速度翻倍 */
            boost = (s_axis[ABS_Z] > 50000) ? 2.0 : 1.0;

            /* 映射摇杆值到速度 */
            sp_vx = map_evdev_axis(s_axis[ABS_Y],
                                   s_axis_center[ABS_Y],
                                   JOY_MAX_LINEAR_SPEED * boost);
            sp_vy = map_evdev_axis(s_axis[ABS_X],
                                   s_axis_center[ABS_X],
                                   JOY_MAX_LINEAR_SPEED * boost);
            sp_om = map_evdev_axis(s_axis[ABS_RX],
                                   s_axis_center[ABS_RX],
                                   JOY_MAX_ANGULAR_SPEED * boost);

            /* 摇杆方向与底盘方向可能相反，取反 */
            cmd.vx    = -sp_vx;
            cmd.vy    = -sp_vy;
            cmd.omega = -sp_om;

            /* 逆运动学：机器人速度 → 各轮 RPM */
            kinematics_inverse(&s_kin, &cmd, &wheels);
            rpm[0] = wheels.rpm[0] * DIR_FACTOR_FL;
            rpm[1] = wheels.rpm[1] * DIR_FACTOR_FR;
            rpm[2] = wheels.rpm[2] * DIR_FACTOR_RL;
            rpm[3] = wheels.rpm[3] * DIR_FACTOR_RR;

            send_velocity(rpm);
        }

        usleep(10000U);  /* 10ms 延迟 ≈ 100Hz 控制频率 */
    }

    send_stop();
    deinit_all();
    stop_camera_stream();
    return EXIT_SUCCESS;
}
