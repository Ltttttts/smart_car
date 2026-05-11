/**
 * @file    llm_client.c
 * @brief   LLM API 客户端实现 (libcurl + JSON 解析)
 * @author  Ltttttts
 */

#include "comm/llm_client.h"
#include "comm/json_helper.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <curl/curl.h>

/* ---- 回调：收集 HTTP 响应体 ---- */

struct MemBuf { char *data; size_t len; };

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
    mb->data[mb->len] = '\0';
    return chunk;
}

/* ---- JSON 字符串转义（" \ \n \t \r） ---- */

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

/* ---- 加载默认配置 ---- */

void llm_cfg_default(LlmConfig_t *cfg)
{
    const char *env;
    memset(cfg, 0, sizeof(*cfg));

    env = getenv("LLM_API_URL");
    snprintf(cfg->api_url, sizeof(cfg->api_url), "%s",
             env ? env : "https://opencode.ai/zen/go/v1/chat/completions");

    env = getenv("LLM_API_KEY");
    if (env) snprintf(cfg->api_key, sizeof(cfg->api_key), "%s", env);

    env = getenv("LLM_MODEL");
    snprintf(cfg->model, sizeof(cfg->model), "%s",
             env ? env : "deepseek-v4-flash");
}

/* ---- 从 API 回复中提取 choices[0].message.content ---- */

static int s_extract_content(const char *raw, char *out, size_t outsz)
{
    const char *p;

    p = strstr(raw, "\"content\"");
    if (!p) return -1;
    p = strchr(p + 9, ':');
    if (!p) return -1;
    p++;
    while (*p && (unsigned char)*p <= ' ') p++;
    if (*p != '"') return -1;
    p++;

    size_t oi = 0;
    while (*p && oi < outsz - 1) {
        if (*p == '\\' && *(p + 1)) { p++; out[oi++] = *p++; continue; }
        if (*p == '"') break;
        out[oi++] = *p++;
    }
    out[oi] = '\0';
    return (oi > 0 && out[0] == '{') ? 0 : -1;
}

/* ---- 核心 ---- */

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

    /* 构建请求体 */
    {
        char sys_esc[2048], user_esc[2048];
        s_esc_json(system, sys_esc, sizeof(sys_esc));
        s_esc_json(user,   user_esc, sizeof(user_esc));

        snprintf(body, sizeof(body),
            "{"
            "\"model\":\"%s\","
            "\"messages\":["
            "{\"role\":\"system\",\"content\":\"%s\"},"
            "{\"role\":\"user\",\"content\":\"%s\"}"
            "]}",
            cfg->model, sys_esc, user_esc);
    }

    /* 初始化 curl */
    curl_global_init(CURL_GLOBAL_ALL);
    curl = curl_easy_init();
    if (!curl) goto cleanup;

    snprintf(auth_hdr, sizeof(auth_hdr),
             "Authorization: Bearer %s", cfg->api_key);
    headers = curl_slist_append(headers, auth_hdr);
    headers = curl_slist_append(headers,
                                "Content-Type: application/json");

    curl_easy_setopt(curl, CURLOPT_URL, cfg->api_url);
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, body);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, (long)strlen(body));
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, s_write_cb);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &mb);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 60L);
    curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0L);

    {
        CURLcode rc = curl_easy_perform(curl);
        if (rc != CURLE_OK) {
            fprintf(stderr, "[LLM] HTTP 请求失败: %s\n",
                    curl_easy_strerror(rc));
            goto cleanup;
        }
    }

    /* 检查 HTTP 状态码 */
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

    /* 提取 content */
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
