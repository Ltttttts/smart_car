/**
 * @file    json_helper.h
 * @brief   简易 JSON 字段提取（仅支持扁平对象）
 * @author  Ltttttts
 */

#ifndef SMART_CAR_JSON_HELPER_H
#define SMART_CAR_JSON_HELPER_H

#include <stddef.h>

/**
 * @brief  从 JSON 对象中提取字符串字段值
 * @param  json   JSON 字符串，如 {"vx":0.3,"text":"你好"}
 * @param  key    字段名，如 "text"
 * @param  out    输出缓冲区
 * @param  outsz  缓冲区大小
 * @return 0 成功，-1 未找到或格式错误
 */
int json_get_str(const char *json, const char *key,
                 char *out, size_t outsz);

/**
 * @brief  从 JSON 对象中提取数字字段值
 * @param  json   JSON 字符串
 * @param  key    字段名
 * @param  out    输出数值
 * @return 0 成功，-1 未找到
 */
int json_get_num(const char *json, const char *key,
                 double *out);

#endif /* SMART_CAR_JSON_HELPER_H */
