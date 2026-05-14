/**
 * @file    llm_client.h
 * @brief   LLM（大语言模型）API 客户端头文件
 * @author  Ltttttts
 *
 * 功能和用途：
 *   通过 HTTP 调用 OpenAI 兼容的 API（如 DeepSeek、通义千问等），
 *   让小车能用自然语言控制——你说"向前走"，LLM 返回 JSON 指令，
 *   小车解析后执行对应的运动。
 *
 * 环境变量配置（运行时通过 export 设置）：
 *   LLM_API_URL    API 地址，默认指向 OpenCode Go API
 *   LLM_API_KEY    API 密钥（必须设置，否则无法调用）
 *   LLM_MODEL      模型名，默认 deepseek-v4-flash
 *
 * 依赖：libcurl（用于 HTTP 请求）
 */

#ifndef SMART_CAR_LLM_CLIENT_H
#define SMART_CAR_LLM_CLIENT_H

#include <stddef.h>

/* ========== LLM 配置 ========== */
/* 从环境变量读取，也可以在代码中直接赋值 */
typedef struct {
    char api_url[256];    /* API 接口地址 */
    char api_key[256];    /* API 密钥 */
    char model[64];       /* 使用的模型名称 */
} LlmConfig_t;

/* ========== LLM 回复 ========== */
typedef struct {
    char content[2048];   /* LLM 返回的文本内容（choices[0].message.content） */
    int  ok;              /* 标志位：1 = 解析成功，0 = 失败 */
} LlmResponse_t;

/**
 * @brief  从环境变量加载 LLM 配置（LLM_API_URL / LLM_API_KEY / LLM_MODEL）
 * @param  cfg  输出参数：填充好的配置结构体
 */
void llm_cfg_default(LlmConfig_t *cfg);

/**
 * @brief  调用 LLM 聊天接口（Chat Completion API）
 *
 * 发一条用户消息 + 系统提示词给 LLM，拿到回复文本。
 * 对于小车控制，返回的文本应该是一个 JSON 格式的指令。
 *
 * @param  cfg    配置（API 地址、密钥、模型名）
 * @param  system 系统提示词（告诉 LLM 它是什么角色、输出什么格式）
 * @param  user   用户输入（自然语言指令，如"向前走 3 秒"）
 * @param  resp   输出参数：LLM 的回复内容
 * @return 0 成功，-1 失败
 */
int llm_chat(const LlmConfig_t *cfg,
             const char *system,
             const char *user,
             LlmResponse_t *resp);

#endif /* SMART_CAR_LLM_CLIENT_H */
