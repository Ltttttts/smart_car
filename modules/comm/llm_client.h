/**
 * @file    llm_client.h
 * @brief   LLM API 客户端（OpenAI 兼容格式，基于 libcurl）
 * @author  Ltttttts
 *
 * 环境变量:
 *   LLM_API_URL   默认 https://opencode.ai/zen/go/v1/chat/completions
 *   LLM_API_KEY   OpenCode Go / DeepSeek API Key
 *   LLM_MODEL     deepseek-v4-flash
 */

#ifndef SMART_CAR_LLM_CLIENT_H
#define SMART_CAR_LLM_CLIENT_H

#include <stddef.h>

/* ---- 配置 ---- */
typedef struct {
    char api_url[256];
    char api_key[256];
    char model[64];
} LlmConfig_t;

/* ---- 回复 ---- */
typedef struct {
    char content[2048];   /* choices[0].message.content */
    int  ok;              /* 1 = 成功 */
} LlmResponse_t;

/**
 * @brief  从环境变量加载 LLM 配置
 * @param  cfg  输出配置
 */
void llm_cfg_default(LlmConfig_t *cfg);

/**
 * @brief  调用 LLM API (chat completion)
 * @param  cfg    配置
 * @param  system 系统提示词
 * @param  user   用户输入
 * @param  resp   输出回复
 * @return 0 成功，-1 失败
 */
int llm_chat(const LlmConfig_t *cfg,
             const char *system,
             const char *user,
             LlmResponse_t *resp);

#endif /* SMART_CAR_LLM_CLIENT_H */
