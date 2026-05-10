/**
 * @file    common.h
 * @brief   公共类型定义与全局常量
 * @author  Ltttttts
 */

#ifndef SMART_CAR_COMMON_H
#define SMART_CAR_COMMON_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* ---- 电机地址 ---- */
#define MOTOR_COUNT        (4U)
#define MOTOR_ADDR_FL      (1U)   /* 前左 */
#define MOTOR_ADDR_FR      (2U)   /* 前右 */
#define MOTOR_ADDR_RL      (3U)   /* 后左 */
#define MOTOR_ADDR_RR      (4U)   /* 后右 */

/* ---- 串口默认参数 ---- */
#define EMM_DEFAULT_BAUDRATE  (115200U)
#define EMM_DEFAULT_DEVICE    "/dev/ttyUSB0"

/* ---- 协议常量 ---- */
#define EMM_CHECKSUM_BYTE     (0x6B)
#define EMM_CMD_BUF_SIZE      (32U)
#define EMM_RESP_BUF_SIZE     (16U)

/* ---- 时序 ---- */
#define EMM_RESP_TIMEOUT_MS   (50U)
#define EMM_MOVE_TIMEOUT_MS   (5000U)

/* ---- 电机方向 ---- */
#define EMM_DIR_CW            (0U)   /* 顺时针 */
#define EMM_DIR_CCW           (1U)   /* 逆时针 */

/* ---- 速度限制 ---- */
#define EMM_MAX_RPM           (5000U)
#define EMM_MIN_RPM           (1U)
#define EMM_DEFAULT_ACC       (50U)
#define EMM_MAX_ACC           (255U)

/* ---- 统一错误码 ---- */
typedef enum {
    ERR_OK               =  0,
    ERR_NULL_PTR         = -1,   /* 空指针         */
    ERR_INVALID_PARAM    = -2,   /* 无效参数       */
    ERR_SERIAL_OPEN      = -3,   /* 串口打开失败   */
    ERR_SERIAL_WRITE     = -4,   /* 串口写入失败   */
    ERR_SERIAL_READ      = -5,   /* 串口读取失败   */
    ERR_SERIAL_TIMEOUT   = -6,   /* 串口超时       */
    ERR_MOTOR_FAULT      = -7,   /* 电机故障       */
    ERR_NOT_ENABLED      = -8,   /* 电机未使能     */
    ERR_OVERFLOW         = -9,   /* 缓冲区溢出     */
    ERR_BUSY             = -10,  /* 设备忙         */
    ERR_NOT_INITIALIZED  = -11,  /* 未初始化       */
} ErrorCode_t;

#endif /* SMART_CAR_COMMON_H */
