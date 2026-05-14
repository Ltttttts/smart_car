/**
 * @file    common.h
 * @brief   公共头文件 — 所有模块共享的类型定义、错误码与全局常量
 * @author  Ltttttts
 *
 * 整个智能小车项目的"字典"，所有模块都包含此文件。
 * 这里定义了电机地址、串口参数、协议常量、错误码等全局通用内容。
 */

#ifndef SMART_CAR_COMMON_H
#define SMART_CAR_COMMON_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* ========== 电机地址 ========== */
/* 四个麦克纳姆轮各有一个 EMM_V5 驱动器，挂在同一根 UART 总线上，
   通过 1~4 号地址区分。布局如下（俯视图）：
       前左(FL=2) ── 前右(FR=4)
       后左(RL=1) ── 后右(RR=3)     */
#define MOTOR_COUNT        (4U)
#define MOTOR_ADDR_FL      (2U)   /* 前左轮地址 */
#define MOTOR_ADDR_FR      (4U)   /* 前右轮地址 */
#define MOTOR_ADDR_RL      (1U)   /* 后左轮地址 */
#define MOTOR_ADDR_RR      (3U)   /* 后右轮地址 */

/* ========== 串口默认参数 ========== */
/* EMM_V5 驱动器的 UART 通信参数：
   波特率 921600，设备路径默认为 /dev/ttyS1（香橙派板载串口） */
#define EMM_DEFAULT_BAUDRATE  (921600U)
#define EMM_DEFAULT_DEVICE    "/dev/ttyS1"

/* ========== 协议常量 ========== */
#define EMM_CHECKSUM_BYTE     (0x6B)  /* 每帧末尾的固定校验字节 */
#define EMM_CMD_BUF_SIZE      (32U)   /* 发送指令缓冲区的最大长度 */
#define EMM_RESP_BUF_SIZE     (16U)   /* 接收回复缓冲区的最大长度 */

/* ========== 通信超时 ========== */
#define EMM_RESP_TIMEOUT_MS   (50U)   /* 等待电机回复的超时时间 */
#define EMM_MOVE_TIMEOUT_MS   (5000U) /* 运动指令的最大等待时间 */

/* ========== 电机方向 ========== */
#define EMM_DIR_CW            (0U)   /* 顺时针旋转 */
#define EMM_DIR_CCW           (1U)   /* 逆时针旋转 */

/* ========== 速度与加速度限制 ========== */
/* 步进电机速度范围 1~5000 RPM，加速度 0~255
   加速度值越大，电机从静止加速到目标速度越快 */
#define EMM_MAX_RPM           (5000U)
#define EMM_MIN_RPM           (1U)
#define EMM_DEFAULT_ACC       (50U)  /* 默认加速度（较保守的数值） */
#define EMM_MAX_ACC           (255U)

/* ========== 统一错误码 ========== */
/* 所有模块的函数都使用这套错误码，负值表示错误，0 表示成功。
   这样上层调用者只需判断 ret < 0 就知道出错了。 */
typedef enum {
    ERR_OK               =  0,   /* 成功 */
    ERR_NULL_PTR         = -1,   /* 收到了空指针参数 */
    ERR_INVALID_PARAM    = -2,   /* 参数值不合法 */
    ERR_SERIAL_OPEN      = -3,   /* 打不开串口设备 */
    ERR_SERIAL_WRITE     = -4,   /* 串口写入数据失败 */
    ERR_SERIAL_READ      = -5,   /* 串口读取数据失败 */
    ERR_SERIAL_TIMEOUT   = -6,   /* 串口通信超时（对方没回复） */
    ERR_MOTOR_FAULT      = -7,   /* 电机回复了错误数据 */
    ERR_NOT_ENABLED      = -8,   /* 电机还没有使能 */
    ERR_OVERFLOW         = -9,   /* 数据缓冲区溢出 */
    ERR_BUSY             = -10,  /* 设备正忙 */
    ERR_NOT_INITIALIZED  = -11,  /* 设备还没有初始化 */
} ErrorCode_t;

#endif /* SMART_CAR_COMMON_H */
