/**
 * @file    json_helper.c
 * @brief   简易 JSON 字段提取实现
 * @author  Ltttttts
 */

#include "json_helper.h"
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

/**
 * @brief  在 json 中查找 "key":  并返回 value 起始位置
 */
static const char *s_find_key(const char *json, const char *key)
{
    const char *p;
    size_t klen = strlen(key);

    /*  搜索：空格 "key" : 值 */
    while ((p = strstr(json, key)) != NULL) {
        /* 确认前面是 '"' 且后面是 '"' (即匹配完整 key) */
        if (p > json && *(p - 1) == '"' && p[klen] == '"') {
            const char *v = p + klen + 1;  /* 跳过 "key" */
            while (*v && (unsigned char)*v <= ' ') v++;  /* 空格 */
            if (*v == ':') {
                v++;
                while (*v && (unsigned char)*v <= ' ') v++;
                return v;
            }
        }
        json = p + 1;
    }
    return NULL;
}

int json_get_str(const char *json, const char *key,
                 char *out, size_t outsz)
{
    const char *v;
    size_t len;

    if (!json || !key || !out || outsz == 0) return -1;

    v = s_find_key(json, key);
    if (!v) return -1;

    if (*v != '"') return -1;
    v++;
    len = 0;
    while (v[len] && v[len] != '"') {
        if (v[len] == '\\') len++; /* 跳过转义 */
        len++;
        if (len >= outsz) len = outsz - 1;
    }
    strncpy(out, v, len);
    out[len] = '\0';
    return 0;
}

int json_get_num(const char *json, const char *key,
                 double *out)
{
    const char *v;
    char *end = NULL;

    if (!json || !key || !out) return -1;

    v = s_find_key(json, key);
    if (!v) return -1;

    /* 检查是否为数字开头 */
    if (*v != '-' && *v != '+' && !isdigit((unsigned char)*v) &&
        *v != '.') return -1;

    *out = strtod(v, &end);
    if (end == v) return -1;  /* 没有任何数字被解析 */
    return 0;
}
