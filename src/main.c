/**
 * @file    main.c
 * @brief   智能小车演示程序：麦克纳姆轮运动 + 电机状态回读
 * @author  Ltttttts
 *
 * 用法:
 *   ./smart_car [串口设备]
 *
 * 默认串口设备: /dev/ttyUSB0
 *
 * 安全提示：首次运行前请将小车架起（车轮悬空），确认方向正确后再落地。
 * 各轮方向修正因子见下方 DIR_FACTOR_xx 宏。
 */

#include "serial_port.h"
#include "emm_v5_driver.h"
#include "kinematics.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/* ---- 底盘几何参数（按你的实际小车测量填入） ---- */
#define WHEEL_RADIUS_M      (0.05)    /* 轮半径 50mm           */
#define HALF_LX_M           (0.12)    /* 前后半轴距             */
#define HALF_LY_M           (0.10)    /* 左右半轴距             */
#define ENCODER_PPR         (4000)    /* 编码器每转脉冲（需对照手册确认） */

/* ---- 演示时序 ---- */
#define DEMO_MOVE_DURATION_S  (2U)
#define DEMO_PAUSE_DURATION_S (1U)

/* ---- 各轮方向修正因子 ----
 *
 * 运动学公式假定特定旋转方向为正向。如果某个电机的
 * 实际安装方向与假定相反，将对应因子改为 -1.0 即可。
 *
 * 默认：全为 1.0（CCW=正转 FL/RL，CW=正转 FR/RR）
 */
#define DIR_FACTOR_FL  (1.0)
#define DIR_FACTOR_FR  (1.0)
#define DIR_FACTOR_RL  (1.0)
#define DIR_FACTOR_RR  (1.0)

/* ---- 模块级全局变量 ---- */
static SerialPort_t  *s_port    = NULL;
static EmmMotor_t    *s_motor[4] = { NULL };
static Kinematics_t   s_kin;

/* ---- 辅助：打印错误 ---- */

static void print_error(const char *ctx, int err)
{
    fprintf(stderr, "[错误] %s: %d\n", ctx, err);
}

/* ---- 辅助：安全延时 ---- */

static void demo_sleep(uint32_t seconds)
{
    if (seconds > 60U) {
        return;
    }
    sleep(seconds);
}

/* ============ 初始化 / 反初始化 ============ */

static int init_all(const char *device)
{
    size_t   i;
    int      ret;
    uint8_t addrs[MOTOR_COUNT] = {
        MOTOR_ADDR_FL,
        MOTOR_ADDR_FR,
        MOTOR_ADDR_RL,
        MOTOR_ADDR_RR
    };

    /* 创建并打开串口 */
    s_port = serial_port_create(device, EMM_DEFAULT_BAUDRATE);
    if (s_port == NULL) {
        return ERR_NULL_PTR;
    }

    ret = serial_port_open(s_port);
    if (ret != ERR_OK) {
        fprintf(stderr,
                "[错误] 无法打开 %s (是否使用了 sudo?)\n",
                device);
        return ret;
    }

    serial_port_flush(s_port);

    /* 创建 4 个电机对象 */
    for (i = 0; i < MOTOR_COUNT; i++) {
        s_motor[i] = emm_motor_create(addrs[i], s_port);
        if (s_motor[i] == NULL) {
            return ERR_NULL_PTR;
        }
    }

    /* 初始化运动学 */
    kinematics_init(&s_kin, WHEEL_RADIUS_M, HALF_LX_M,
                    HALF_LY_M, ENCODER_PPR);

    return ERR_OK;
}

static void deinit_all(void)
{
    size_t i;

    for (i = 0; i < MOTOR_COUNT; i++) {
        if (s_motor[i] != NULL) {
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
}

/* ============ 电机使能 / 失能 ============ */

static int enable_all_motors(void)
{
    size_t i;
    int    ret;

    for (i = 0; i < MOTOR_COUNT; i++) {
        ret = emm_motor_enable(s_motor[i]);
        if (ret != ERR_OK) {
            print_error("使能电机", ret);
            return ret;
        }
    }

    return ERR_OK;
}

static void disable_all_motors(void)
{
    size_t i;

    for (i = 0; i < MOTOR_COUNT; i++) {
        (void)emm_motor_disable(s_motor[i]);
    }
}

/* ============ 读取编码器 ============ */

static void read_and_print_encoders(void)
{
    int32_t    enc[4];
    int        ret;
    size_t     i;
    const char *names[MOTOR_COUNT] = { "FL", "FR", "RL", "RR" };

    printf("---- 电机编码器读数 ----\n");
    for (i = 0; i < MOTOR_COUNT; i++) {
        ret = emm_motor_read_encoder(s_motor[i], &enc[i]);
        if (ret == ERR_OK) {
            printf("  [%s] 编码器: %d\n", names[i], enc[i]);
        } else {
            printf("  [%s] 读取失败: %d\n", names[i], ret);
        }
    }
}

/* ============ 小车移动（速度模式 + 同步触发） ============ */

/**
 * @brief  以给定 RPM 驱动四轮，利用同步触发同时启动
 */
static int move_wheels_sync(const double target_rpm[4])
{
    size_t i;
    int    ret;

    for (i = 0; i < MOTOR_COUNT; i++) {
        ret = emm_motor_set_velocity(s_motor[i],
                                     (int16_t)target_rpm[i],
                                     EMM_DEFAULT_ACC, true);
        if (ret != ERR_OK) {
            print_error("速度指令下发", ret);
            return ret;
        }
    }

    /* 同步触发：4 个电机同时启动 */
    ret = emm_motor_sync_trigger(s_motor, MOTOR_COUNT);
    if (ret != ERR_OK) {
        print_error("同步触发", ret);
    }

    return ret;
}

/**
 * @brief  急停所有电机
 */
static void stop_all_motors(void)
{
    size_t i;

    for (i = 0; i < MOTOR_COUNT; i++) {
        (void)emm_motor_stop(s_motor[i], false);
    }
}

/* ============ 演示动作序列 ============ */

static void demo_forward(void)
{
    RobotVelocity_t robot;
    WheelVelocity_t wheels;
    double          rpm[4];

    robot.vx    = 0.2;   /* 0.2 m/s 前进 */
    robot.vy    = 0.0;
    robot.omega = 0.0;

    kinematics_inverse(&s_kin, &robot, &wheels);

    rpm[0] = wheels.rpm[0] * DIR_FACTOR_FL;
    rpm[1] = wheels.rpm[1] * DIR_FACTOR_FR;
    rpm[2] = wheels.rpm[2] * DIR_FACTOR_RL;
    rpm[3] = wheels.rpm[3] * DIR_FACTOR_RR;

    printf("[演示] 前进  (%.1f m/s, %d s)\n",
           robot.vx, DEMO_MOVE_DURATION_S);

    if (move_wheels_sync(rpm) == ERR_OK) {
        demo_sleep(DEMO_MOVE_DURATION_S);
        stop_all_motors();
    }
}

static void demo_backward(void)
{
    RobotVelocity_t robot;
    WheelVelocity_t wheels;
    double          rpm[4];

    robot.vx    = -0.2;
    robot.vy    = 0.0;
    robot.omega = 0.0;

    kinematics_inverse(&s_kin, &robot, &wheels);

    rpm[0] = wheels.rpm[0] * DIR_FACTOR_FL;
    rpm[1] = wheels.rpm[1] * DIR_FACTOR_FR;
    rpm[2] = wheels.rpm[2] * DIR_FACTOR_RL;
    rpm[3] = wheels.rpm[3] * DIR_FACTOR_RR;

    printf("[演示] 后退  (%.1f m/s, %d s)\n",
           -robot.vx, DEMO_MOVE_DURATION_S);

    if (move_wheels_sync(rpm) == ERR_OK) {
        demo_sleep(DEMO_MOVE_DURATION_S);
        stop_all_motors();
    }
}

static void demo_strafe_right(void)
{
    RobotVelocity_t robot;
    WheelVelocity_t wheels;
    double          rpm[4];

    robot.vx    = 0.0;
    robot.vy    = -0.2;  /* vy 负值 = 向右平移 */
    robot.omega = 0.0;

    kinematics_inverse(&s_kin, &robot, &wheels);

    rpm[0] = wheels.rpm[0] * DIR_FACTOR_FL;
    rpm[1] = wheels.rpm[1] * DIR_FACTOR_FR;
    rpm[2] = wheels.rpm[2] * DIR_FACTOR_RL;
    rpm[3] = wheels.rpm[3] * DIR_FACTOR_RR;

    printf("[演示] 右平移 (%.1f m/s, %d s)\n",
           -robot.vy, DEMO_MOVE_DURATION_S);

    if (move_wheels_sync(rpm) == ERR_OK) {
        demo_sleep(DEMO_MOVE_DURATION_S);
        stop_all_motors();
    }
}

static void demo_rotate_cw(void)
{
    RobotVelocity_t robot;
    WheelVelocity_t wheels;
    double          rpm[4];

    robot.vx    = 0.0;
    robot.vy    = 0.0;
    robot.omega = -0.5;  /* 负值 = 顺时针（俯视） */

    kinematics_inverse(&s_kin, &robot, &wheels);

    rpm[0] = wheels.rpm[0] * DIR_FACTOR_FL;
    rpm[1] = wheels.rpm[1] * DIR_FACTOR_FR;
    rpm[2] = wheels.rpm[2] * DIR_FACTOR_RL;
    rpm[3] = wheels.rpm[3] * DIR_FACTOR_RR;

    printf("[演示] 顺时针旋转 (%.1f rad/s, %d s)\n",
           -robot.omega, DEMO_MOVE_DURATION_S);

    if (move_wheels_sync(rpm) == ERR_OK) {
        demo_sleep(DEMO_MOVE_DURATION_S);
        stop_all_motors();
    }
}

/* ============ 主函数 ============ */

int main(int argc, char *argv[])
{
    const char *device;
    int         ret;

    device = (argc >= 2) ? argv[1] : EMM_DEFAULT_DEVICE;

    printf("=== 智能小车 麦克纳姆轮演示 ===\n");
    printf("串口设备: %s\n", device);

    /* 初始化 */
    ret = init_all(device);
    if (ret != ERR_OK) {
        fprintf(stderr, "初始化失败: %d\n", ret);
        deinit_all();
        return EXIT_FAILURE;
    }

    /* 使能全部电机 */
    ret = enable_all_motors();
    if (ret != ERR_OK) {
        fprintf(stderr, "电机使能失败: %d\n", ret);
        deinit_all();
        return EXIT_FAILURE;
    }

    printf("电机已使能。\n");
    demo_sleep(1U);

    /* ---- 运动演示 ---- */
    demo_forward();
    demo_sleep(DEMO_PAUSE_DURATION_S);

    demo_backward();
    demo_sleep(DEMO_PAUSE_DURATION_S);

    demo_strafe_right();
    demo_sleep(DEMO_PAUSE_DURATION_S);

    demo_rotate_cw();
    demo_sleep(DEMO_PAUSE_DURATION_S);

    /* 编码器回读 */
    read_and_print_encoders();

    /* 清理 */
    disable_all_motors();
    deinit_all();

    printf("=== 演示结束 ===\n");
    return EXIT_SUCCESS;
}
