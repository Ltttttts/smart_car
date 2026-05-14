/**
 * @file    json_helper.c
 * @brief   简易 JSON 字段提取实现
 * @author  Ltttttts
 *
 * 实现方式：
 *   在 JSON 字符串中搜索 "key" 模式，然后提取 value。
 *   不做 JSON 语法校验，不处理嵌套，性能一般但够用。
 *
 * 适用场景：
 *   LLM 返回的扁平 JSON 对象，如 {"vx":0.3,"text":"你好","duration":3}
 */

#include "comm/json_helper.h"
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

/**
 * @brief  在 JSON 字符串中查找 "key": 模式，返回 value 的起始位置
 *
 * 搜索逻辑：
 *   1. 用 strstr 定位 key 第一次出现的位置
 *   2. 确认前面是双引号（"key"）
 *   3. 确认后面是双引号和冒号（"key":）
 *   4. 跳过空白，返回 value 的起始字符指针
 *
 * @param  json  完整的 JSON 字符串
 * @param  key   要查找的字段名
 * @return value 的起始位置指针，没找到返回 NULL
 */
static const char *s_find_key(const char *json, const char *key)
{
    const char *p;
    size_t klen = strlen(key);

    /* 遍历 json 中所有匹配 key 的位置 */
    while ((p = strstr(json, key)) != NULL) {
        /* 确认前面是 '"' 且后面是 '"'（即匹配完整的 key 名字） */
        if (p > json && *(p - 1) == '"' && p[klen] == '"') {
            const char *v = p + klen + 1;  /* 跳过 "key" */
            while (*v && (unsigned char)*v <= ' ') v++;  /* 跳过空格 */
            if (*v == ':') {               /* 找到冒号 */
                v++;
                while (*v && (unsigned char)*v <= ' ') v++;  /* 跳过冒号后的空格 */
                return v;                  /* 返回 value 的起始位置 */
            }
        }
        json = p + 1;  /* 没匹配上，继续往后找 */
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

    /* value 必须以引号开头 */
    if (*v != '"') return -1;
    v++;  /* 跳过开头的引号 */

    /* 读取直到遇到下一个未转义的引号 */
    len = 0;
    while (v[len] && v[len] != '"') {
        if (v[len] == '\\') len++; /* 跳过转义符 */
        len++;
        if (len >= outsz) len = outsz - 1;  /* 截断，防止溢出 */
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

    /* 检查 value 是否以数字相关字符开头 */
    if (*v != '-' && *v != '+' && !isdigit((unsigned char)*v) &&
        *v != '.') return -1;

    *out = strtod(v, &end);
    if (end == v) return -1;  /* 没有任何数字被解析出来 */
    return 0;
}
