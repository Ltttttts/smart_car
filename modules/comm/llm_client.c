/**
 * @file    llm_client.c
 * @brief   LLM API 客户端实现 — 用 libcurl 发 HTTP 请求
 * @author  Ltttttts
 *
 * 工作流程：
 *   1. 构建 OpenAI 兼容的 JSON 请求体（含 system prompt + user message）
 *   2. 通过 libcurl 发送 POST 请求到 LLM API
 *   3. 解析返回的 JSON，提取 choices[0].message.content
 *
 * 特别注意：
 *   请求体中的用户提示词和系统提示词需要进行 JSON 转义，
 *   避免其中的引号、换行符破坏 JSON 结构。
 */

#include "comm/llm_client.h"
#include "comm/json_helper.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <curl/curl.h>

/* ========== HTTP 响应收集回调 ========== */

/* 用于在回调函数和调用者之间传递数据的结构体 */
struct MemBuf { char *data; size_t len; };

/**
 * @brief  libcurl 的写回调函数：每收到一段响应体数据就追加到内存缓冲区
 *
 * 这个函数由 libcurl 自动调用，参数由 libcurl 传入。
 * 我们把响应体一段一段地收进动态分配的内存中。
 */
static size_t s_write_cb(void *ptr, size_t size,
                         size_t nmemb, void *userdata)
{
    struct MemBuf *mb = (struct MemBuf *)userdata;
    size_t chunk = size * nmemb;
    char *newp = realloc(mb->data, mb->len + chunk + 1);
    if (!newp) return 0;
    memcpy(newp + mb->len, ptr, chunk);
    mb->data = newp;
    mb->len  += chunk;
    mb->data[mb->len] = '\0';  /* 确保以 null 结尾，方便字符串操作 */
    return chunk;
}

/* ========== JSON 字符串转义 ========== */

/**
 * @brief  对字符串中的特殊字符做 JSON 转义
 *
 * 用户输入和系统提示词可能包含引号、换行等字符，
 * 如果不转义直接嵌入 JSON，会导致 JSON 语法错误。
 * 这个函数把 " \ \n \t \r 替换为 \" \\ \n \t \r。
 *
 * @param  src  原始字符串
 * @param  dst  转义后的字符串缓冲区
 * @param  sz   缓冲区大小
 */
static void s_esc_json(const char *src, char *dst, size_t sz)
{
    size_t di = 0;
    for (const char *p = src; *p && di < sz - 4; p++) {
        if (di >= sz - 3) { dst[di] = '\0'; return; }
        switch (*p) {
        case '"':  dst[di++] = '\\'; dst[di++] = '"';  break;
        case '\\': dst[di++] = '\\'; dst[di++] = '\\'; break;
        case '\n': dst[di++] = '\\'; dst[di++] = 'n';  break;
        case '\t': dst[di++] = '\\'; dst[di++] = 't';  break;
        case '\r': dst[di++] = '\\'; dst[di++] = 'r';  break;
        default:   dst[di++] = *p; break;
        }
    }
    dst[di] = '\0';
}

/* ========== 加载默认配置 ========== */

void llm_cfg_default(LlmConfig_t *cfg)
{
    const char *env;
    memset(cfg, 0, sizeof(*cfg));

    /* 从环境变量读取 API 地址，没有就用默认的 OpenCode Go API */
    env = getenv("LLM_API_URL");
    snprintf(cfg->api_url, sizeof(cfg->api_url), "%s",
             env ? env : "https://opencode.ai/zen/go/v1/chat/completions");

    /* API 密钥：必须设置，否则无法调用 */
    env = getenv("LLM_API_KEY");
    if (env) snprintf(cfg->api_key, sizeof(cfg->api_key), "%s", env);

    /* 模型名：默认用 deepseek-v4-flash */
    env = getenv("LLM_MODEL");
    snprintf(cfg->model, sizeof(cfg->model), "%s",
             env ? env : "deepseek-v4-flash");
}

/* ========== 从 API 回复中提取文本 ========== */

/**
 * @brief  从 API 的 JSON 回复中提取 choices[0].message.content 字段
 *
 * 收到的大概是这样：
 *   {"choices":[{"message":{"content":"{\"vx\":0.3,...}"}}]}
 * 我们要从中提取出 content 后面的字符串。
 *
 * 不用完整 JSON 解析器，用字符串搜索的方式：
 * 1. 找 "content" 关键字
 * 2. 找后面的冒号和引号
 * 3. 读取到下一个引号为止
 *
 * @param  raw    API 返回的完整 JSON 字符串
 * @param  out    输出缓冲区
 * @param  outsz  缓冲区大小
 * @return 0 成功，-1 解析失败
 */
static int s_extract_content(const char *raw, char *out, size_t outsz)
{
    const char *p;

    p = strstr(raw, "\"content\"");
    if (!p) return -1;
    p = strchr(p + 9, ':');
    if (!p) return -1;
    p++;
    while (*p && (unsigned char)*p <= ' ') p++;  /* 跳过空白 */
    if (*p != '"') return -1;
    p++;

    /* 读取字符串内容，处理转义（"text":"hello \"world\""） */
    size_t oi = 0;
    while (*p && oi < outsz - 1) {
        if (*p == '\\' && *(p + 1)) { p++; out[oi++] = *p++; continue; }
        if (*p == '"') break;
        out[oi++] = *p++;
    }
    out[oi] = '\0';

    /* 小车控制场景下，LLM 返回的应该是以 { 开头的 JSON */
    return (oi > 0 && out[0] == '{') ? 0 : -1;
}

/* ========== 核心函数 ========== */

int llm_chat(const LlmConfig_t *cfg,
             const char *system,
             const char *user,
             LlmResponse_t *resp)
{
    CURL *curl = NULL;
    struct curl_slist *headers = NULL;
    struct MemBuf mb = { NULL, 0 };
    char body[8192];
    char auth_hdr[512];
    int ret = -1;

    if (!cfg || !system || !user || !resp) return -1;
    memset(resp, 0, sizeof(*resp));

    if (cfg->api_key[0] == '\0') {
        fprintf(stderr, "[LLM] 错误: LLM_API_KEY 未设置\n");
        return -1;
    }

    /* ---- 构建 HTTP 请求体（JSON 格式）---- */
    {
        char sys_esc[2048], user_esc[2048];
        s_esc_json(system, sys_esc, sizeof(sys_esc));
        s_esc_json(user,   user_esc, sizeof(user_esc));

        /* OpenAI 兼容的 Chat Completion 请求格式 */
        snprintf(body, sizeof(body),
            "{"
            "\"model\":\"%s\","
            "\"messages\":["
            "{\"role\":\"system\",\"content\":\"%s\"},"
            "{\"role\":\"user\",\"content\":\"%s\"}"
            "]}",
            cfg->model, sys_esc, user_esc);
    }

    /* ---- 初始化 libcurl ---- */
    curl_global_init(CURL_GLOBAL_ALL);
    curl = curl_easy_init();
    if (!curl) goto cleanup;

    /* 设置 HTTP 头：认证和内容类型 */
    snprintf(auth_hdr, sizeof(auth_hdr),
             "Authorization: Bearer %s", cfg->api_key);
    headers = curl_slist_append(headers, auth_hdr);
    headers = curl_slist_append(headers,
                                "Content-Type: application/json");

    /* 配置 libcurl 选项 */
    curl_easy_setopt(curl, CURLOPT_URL, cfg->api_url);
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, body);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, (long)strlen(body));
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, s_write_cb);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &mb);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 60L);  /* 最长等 60 秒 */
    curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0L);  /* 跳过证书验证 */

    /* ---- 发送 HTTP 请求 ---- */
    {
        CURLcode rc = curl_easy_perform(curl);
        if (rc != CURLE_OK) {
            fprintf(stderr, "[LLM] HTTP 请求失败: %s\n",
                    curl_easy_strerror(rc));
            goto cleanup;
        }
    }

    /* ---- 检查 HTTP 状态码 ---- */
    {
        long http_code = 0;
        curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);
        if (http_code != 200) {
            fprintf(stderr, "[LLM] API 返回 HTTP %ld\n", http_code);
            if (mb.data) {
                fprintf(stderr, "[LLM] 响应: %s\n", mb.data);
            }
            goto cleanup;
        }
    }

    if (!mb.data || mb.len == 0) {
        fprintf(stderr, "[LLM] 响应为空\n");
        goto cleanup;
    }

    /* ---- 解析回复，提取 content 字段 ---- */
    if (s_extract_content(mb.data, resp->content,
                          sizeof(resp->content)) == 0) {
        resp->ok = 1;
        ret = 0;
    } else {
        fprintf(stderr, "[LLM] 无法解析 API 响应: %s\n", mb.data);
    }

cleanup:
    if (headers) curl_slist_free_all(headers);
    if (curl) curl_easy_cleanup(curl);
    free(mb.data);
    curl_global_cleanup();
    return ret;
}
