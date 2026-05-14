/**
 * @file    json_helper.h
 * @brief   简易 JSON 字段提取工具（无第三方依赖，纯手写解析）
 * @author  Ltttttts
 *
 * 为什么需要这个？
 *   LLM 返回的是 JSON 格式字符串（如 {"vx":0.3,"text":"你好"}），
 *   但 C 标准库没有 JSON 解析功能。又不想为了一个小功能引入 cJSON 库，
 *   所以自己写了个精简版，只支持扁平（非嵌套）JSON 对象的字段提取。
 *
 * 限制：
 *   - 不支持嵌套对象和数组
 *   - 不支持转义字符（\" 会被当成普通字符）
 *   - 只做简单字符串匹配，不做完整语法校验
 */

#ifndef SMART_CAR_JSON_HELPER_H
#define SMART_CAR_JSON_HELPER_H

#include <stddef.h>

/**
 * @brief  从 JSON 字符串中提取指定 key 的字符串 value
 * @param  json   JSON 字符串，例如 {"vx":0.3,"text":"我向前走"}
 * @param  key    要查找的字段名，例如 "text"
 * @param  out    输出缓冲区（存放提取到的字符串值）
 * @param  outsz  输出缓冲区的大小（避免越界）
 * @return 0 成功，-1 未找到字段或格式错误
 */
int json_get_str(const char *json, const char *key,
                 char *out, size_t outsz);

/**
 * @brief  从 JSON 字符串中提取指定 key 的数字 value
 * @param  json   JSON 字符串
 * @param  key    要查找的字段名
 * @param  out    输出参数：提取到的数值（double 类型）
 * @return 0 成功，-1 未找到字段或格式错误
 */
int json_get_num(const char *json, const char *key,
                 double *out);

#endif /* SMART_CAR_JSON_HELPER_H */
