/**
 * @file    main.c
 * @brief   智能小车演示程序：自动演示 + 仪表盘
 * @author  Ltttttts
 *
 * ================================================================
 * TODO: 连接硬件后将下面的 0 改为 1
 * ================================================================
 */
#define HARDWARE_ENABLED 0

#include "serial_port.h"
#include "emm_v5_driver.h"
#include "kinematics.h"
#include "dashboard.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define WHEEL_RADIUS_M      (0.05)
#define HALF_LX_M           (0.12)
#define HALF_LY_M           (0.10)
#define ENCODER_PPR         (4000)

#define DEMO_MOVE_DURATION_S  (2U)
#define DEMO_PAUSE_DURATION_S (1U)

#define DIR_FACTOR_FL  (1.0)
#define DIR_FACTOR_FR  (1.0)
#define DIR_FACTOR_RL  (1.0)
#define DIR_FACTOR_RR  (1.0)

#if HARDWARE_ENABLED
static SerialPort_t  *s_port    = NULL;
static EmmMotor_t    *s_motor[4] = { NULL };
#endif
static Kinematics_t   s_kin;
static Dashboard_t    s_dash;

static void demo_sleep(uint32_t seconds)
{
    if (seconds > 60U) { return; }
    sleep(seconds);
}

/* ============ 初始化 / 反初始化 ============ */

static int init_all(const char *device)
{
#if HARDWARE_ENABLED
    size_t i;  int ret;
    uint8_t addrs[MOTOR_COUNT] = {
        MOTOR_ADDR_FL, MOTOR_ADDR_FR,
        MOTOR_ADDR_RL, MOTOR_ADDR_RR
    };
    s_port = serial_port_create(device, EMM_DEFAULT_BAUDRATE);
    if (s_port == NULL) { return ERR_NULL_PTR; }
    ret = serial_port_open(s_port);
    if (ret != ERR_OK) { return ret; }
    serial_port_flush(s_port);
    for (i = 0; i < MOTOR_COUNT; i++) {
        s_motor[i] = emm_motor_create(addrs[i], s_port);
        if (s_motor[i] == NULL) { return ERR_NULL_PTR; }
    }
#else
    (void)device;
#endif
    kinematics_init(&s_kin, WHEEL_RADIUS_M, HALF_LX_M,
                    HALF_LY_M, ENCODER_PPR);
    dash_init(&s_dash, WHEEL_RADIUS_M, "Demo 演示");
    return ERR_OK;
}

static void deinit_all(void)
{
#if HARDWARE_ENABLED
    size_t i;
    for (i = 0; i < MOTOR_COUNT; i++) {
        if (s_motor[i] != NULL) {
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

/* ============ 运动执行 ============ */

#if HARDWARE_ENABLED
static int move_wheels_sync(const double target_rpm[4])
{
    size_t i; int ret;
    for (i = 0; i < MOTOR_COUNT; i++) {
        ret = emm_motor_set_velocity(s_motor[i],
                         (int16_t)target_rpm[i],
                         EMM_DEFAULT_ACC, true);
        if (ret != ERR_OK) { return ret; }
    }
    return emm_motor_sync_trigger(s_motor, MOTOR_COUNT);
}
static void stop_all_motors(void)
{
    size_t i;
    for (i = 0; i < MOTOR_COUNT; i++) {
        (void)emm_motor_stop(s_motor[i], false);
    }
}
#else
static int move_wheels_sync(const double target_rpm[4])
{
    (void)target_rpm; return ERR_OK;
}
static void stop_all_motors(void) { }
#endif

static void do_movement(const char *label,
                        double vx, double vy, double omega,
                        uint32_t duration_s)
{
    RobotVelocity_t robot = { vx, vy, omega };
    WheelVelocity_t wheels;
    double rpm[4];
    double zero[4] = { 0, 0, 0, 0 };

    kinematics_inverse(&s_kin, &robot, &wheels);
    rpm[0] = wheels.rpm[0] * DIR_FACTOR_FL;
    rpm[1] = wheels.rpm[1] * DIR_FACTOR_FR;
    rpm[2] = wheels.rpm[2] * DIR_FACTOR_RL;
    rpm[3] = wheels.rpm[3] * DIR_FACTOR_RR;

    if (move_wheels_sync(rpm) != ERR_OK) return;

    demo_sleep(duration_s);

    dash_update(&s_dash, vx, vy, omega, rpm,
                (double)duration_s,
                "模拟", label);

    stop_all_motors();
    dash_update(&s_dash, 0, 0, 0, zero,
                0.0, "模拟", "待命");
}

/* ============ 主函数 ============ */

int main(int argc, char *argv[])
{
    const char *device;

    device = (argc >= 2) ? argv[1] : EMM_DEFAULT_DEVICE;

    if (init_all(device) != ERR_OK) {
        deinit_all();
        return EXIT_FAILURE;
    }

    demo_sleep(1U);

    do_movement("前进", 0.2,  0.0, 0.0, DEMO_MOVE_DURATION_S);
    do_movement("后退", -0.2, 0.0, 0.0, DEMO_MOVE_DURATION_S);
    do_movement("右平移", 0.0, -0.2, 0.0, DEMO_MOVE_DURATION_S);
    do_movement("左旋转", 0.0, 0.0, 0.5, DEMO_MOVE_DURATION_S);

    deinit_all();
    return EXIT_SUCCESS;
}
