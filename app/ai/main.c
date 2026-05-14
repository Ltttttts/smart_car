/**
 * @file    app/ai/main.c
 * @brief   AI 对话控制底盘程序
 * @author  Ltttttts
 *
 * 功能：
 *   用自然语言指挥小车运动。
 *   用户输入"向前走 3 秒"→ LLM 返回 JSON 指令
 *   → 解析出 vx/vy/omega/duration → 执行运动。
 *
 * 使用方式：
 *   1. 设置环境变量：
 *      export LLM_API_KEY="sk-xxxxx"
 *      export LLM_MODEL="deepseek-v4-flash"
 *   2. 运行：
 *      make run-ai
 *
 * 安全开关：
 *   顶部 HARDWARE_ENABLED 设为 0 时只模拟计算不驱动电机，
 *   设为 1 时才真正控制硬件。这样可以先在模拟模式下调试。
 */

#define HARDWARE_ENABLED 0
#define _GNU_SOURCE

#include "hal/serial_port.h"
#include "driver/emm_v5.h"
#include "control/kinematics.h"
#include "comm/llm_client.h"
#include "comm/json_helper.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>

/* ========== 底盘物理参数 ========== */
#define WHEEL_RADIUS_M      (0.05)
#define HALF_LX_M           (0.12)
#define HALF_LY_M           (0.10)
#define ENCODER_PPR         (4000)

/* ========== 各轮方向修正系数 ========== */
#define DIR_FACTOR_FL  (1.0)
#define DIR_FACTOR_FR  (1.0)
#define DIR_FACTOR_RL  (1.0)
#define DIR_FACTOR_RR  (1.0)

/* ========== 全局变量 ========== */
#if HARDWARE_ENABLED
static SerialPort_t *s_port = NULL;
static EmmMotor_t   *s_motor[4] = { NULL };
#endif
static Kinematics_t  s_kin;
static int           s_running = 1;

static void sigint_handler(int sig) { (void)sig; s_running = 0; }

/* ========== LLM 系统提示词 ========== */
/* 这个提示词告诉 LLM 它的角色和输出格式。
   核心要求：只输出 JSON，不要多余的文字说明。 */

static const char *SYSTEM_PROMPT =
    "你是智能小车的AI驾驶助手。用户用文字向你下达指令，"
    "你控制四轮麦克纳姆轮底盘移动。\n\n"
    "输出严格为JSON对象，不要包含其他文字：\n"
    "{\n"
    "  \"vx\": 前进速度(-1~1), 正=前进 负=后退\n"
    "  \"vy\": 侧移速度(-1~1), 正=左移 负=右移\n"
    "  \"omega\": 角速度(-1~1), 正=左转 负=右转\n"
    "  \"duration\": 持续时间(秒, 0=停止)\n"
    "  \"text\": \"你对用户说的话\"\n"
    "}\n\n"
    "示例：\n"
    "用户说「向前走」-> {\"vx\":0.3,\"vy\":0,\"omega\":0,"
    "\"duration\":3,\"text\":\"好的，向前移动\"}\n"
    "用户说「停」-> {\"vx\":0,\"vy\":0,\"omega\":0,"
    "\"duration\":0,\"text\":\"已停止\"}\n\n"
    "规则：\n"
    "- duration=0 时立即停止\n"
    "- 速度值必须在 -1.0 到 1.0 之间\n"
    "- 只说JSON，不要加解释";

/* ========== 发送速度指令 ========== */

/**
 * @brief  把 RPM 发给电机驱动器（硬件模式才真正发送）
 * @param  rpm  四个轮子的目标转速
 */
static void send_velocity(const double rpm[4])
{
#if HARDWARE_ENABLED
    size_t i;
    for (i = 0; i < MOTOR_COUNT; i++)
        (void)emm_motor_set_velocity(s_motor[i],
                     (int16_t)rpm[i], 200U, true);
    (void)emm_motor_sync_trigger(s_motor, MOTOR_COUNT);
#endif
    (void)rpm;
}

/**
 * @brief  让所有电机立即停止
 */
static void stop_now(void)
{
#if HARDWARE_ENABLED
    size_t i;
    for (i = 0; i < MOTOR_COUNT; i++)
        (void)emm_motor_stop(s_motor[i], false);
#endif
}

/* ========== 执行 LLM 返回的指令 ========== */

/**
 * @brief  解析并执行 LLM 返回的运动指令
 *
 * @param vx       前进速度（归一化 -1~1）
 * @param vy       侧移速度（归一化 -1~1）
 * @param omega    角速度（归一化 -1~1）
 * @param duration 持续时间（秒），<=0 表示停止
 */
static void execute_command(double vx, double vy,
                            double omega, double duration)
{
    RobotVelocity_t robot = { vx, vy, omega };
    WheelVelocity_t wheels;
    double rpm[4];

    /* 运动学逆解算：机器人速度 → 各轮 RPM */
    kinematics_inverse(&s_kin, &robot, &wheels);
    rpm[0] = wheels.rpm[0] * DIR_FACTOR_FL;
    rpm[1] = wheels.rpm[1] * DIR_FACTOR_FR;
    rpm[2] = wheels.rpm[2] * DIR_FACTOR_RL;
    rpm[3] = wheels.rpm[3] * DIR_FACTOR_RR;

    printf("[底盘] vx=%.2f vy=%.2f omega=%.2f  %.0fs\n",
           vx, vy, omega, duration);

    if (duration <= 0.0) {
        stop_now();
        printf("[底盘] 已停止\n");
        return;
    }

    send_velocity(rpm);
    sleep((unsigned int)duration);
    stop_now();
}

/* ========== 初始化 / 清理 ========== */

static int init_all(const char *device)
{
#if HARDWARE_ENABLED
    size_t i; int ret;
    uint8_t addrs[] = {1,2,3,4};
    s_port = serial_port_create(device, 115200);
    if (!s_port) return -1;
    ret = serial_port_open(s_port);
    if (ret != 0) return ret;
    serial_port_flush(s_port);
    for (i = 0; i < 4; i++) {
        s_motor[i] = emm_motor_create(addrs[i], s_port);
        if (!s_motor[i]) return -1;
    }
    for (i = 0; i < 4; i++) emm_motor_enable(s_motor[i]);
    printf("[硬件] 电机已使能\n");
#else
    (void)device;
    printf("[模拟] 不会实际驱动电机\n");
#endif
    kinematics_init(&s_kin, WHEEL_RADIUS_M, HALF_LX_M,
                    HALF_LY_M, ENCODER_PPR);
    return 0;
}

static void deinit_all(void)
{
#if HARDWARE_ENABLED
    size_t i;
    stop_now();
    for (i = 0; i < 4; i++) {
        if (s_motor[i])
            { emm_motor_disable(s_motor[i]); emm_motor_destroy(s_motor[i]); }
    }
    if (s_port) { serial_port_close(s_port); serial_port_destroy(s_port); }
#endif
}

/* ============ 主函数 ============ */

int main(int argc, char *argv[])
{
    LlmConfig_t   cfg;
    char          input[1024];
    const char   *device;
    int           ret;
    const char   *mode_str;

    device   = (argc >= 2) ? argv[1] : "/dev/ttyUSB0";
    mode_str = HARDWARE_ENABLED ? "硬件" : "模拟";

    signal(SIGINT, sigint_handler);
    signal(SIGTERM, sigint_handler);

    /* 加载 LLM 配置（从环境变量读取） */
    llm_cfg_default(&cfg);
    if (cfg.api_key[0] == '\0') {
        fprintf(stderr, "[错误] 请设置 LLM_API_KEY 环境变量\n");
        return 1;
    }

    /* 初始化硬件和运动学 */
    if (init_all(device) != 0) {
        deinit_all();
        return 1;
    }

    printf("\n===== AI 底盘控制 (%s) =====\n", mode_str);
    printf("  API: %s\n", cfg.api_url);
    printf("  模型: %s\n", cfg.model);
    printf("  输入指令控制小车，exit 退出\n");
    printf("==============================\n\n");

    /* ---- 对话循环 ---- */
    while (s_running) {
        LlmResponse_t llm_resp;

        printf("> ");
        fflush(stdout);

        if (!fgets(input, sizeof(input), stdin)) break;

        /* 去除末尾的换行符 */
        {
            size_t n = strlen(input);
            if (n > 0 && input[n - 1] == '\n') input[n - 1] = '\0';
        }

        if (strcmp(input, "exit") == 0 || strcmp(input, "quit") == 0)
            break;
        if (input[0] == '\0') continue;

        /* 调用 LLM API */
        printf("[LLM] 思考中...\n");
        ret = llm_chat(&cfg, SYSTEM_PROMPT, input, &llm_resp);
        if (ret != 0 || !llm_resp.ok) {
            printf("[LLM] 调用失败\n");
            continue;
        }

        printf("[LLM] 回复: %s\n", llm_resp.content);

        /* 解析 LLM 返回的 JSON，提取运动参数 */
        {
            double vx = 0, vy = 0, omega = 0, dur = 0;

            json_get_num(llm_resp.content, "vx", &vx);
            json_get_num(llm_resp.content, "vy", &vy);
            json_get_num(llm_resp.content, "omega", &omega);
            json_get_num(llm_resp.content, "duration", &dur);

            execute_command(vx, vy, omega, dur);

            /* 打印 LLM 生成的对用户说的话 */
            {
                char text[256] = "";
                if (json_get_str(llm_resp.content, "text",
                                 text, sizeof(text)) == 0) {
                    printf("[小车] %s\n", text);
                }
            }
        }
    }

    stop_now();
    deinit_all();
    printf("\n再见。\n");
    return 0;
}
