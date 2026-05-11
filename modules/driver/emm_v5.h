/**
 * @file    emm_v5.h
 * @brief   EMM_V5 闭环步进电机驱动（UART 通信协议封装）
 * @author  Ltttttts
 *
 * 4 个电机共用一根 UART 总线，通过地址区分（1/2/3/4）。
 * 支持同步运动：先以 sync=true 缓冲各电机指令，
 * 再调用 emm_motor_sync_trigger() 同时触发执行。
 */

#ifndef SMART_CAR_EMM_V5_H
#define SMART_CAR_EMM_V5_H

#include "common.h"

/* 前向声明（定义在 serial_port.h） */
typedef struct SerialPort SerialPort_t;

/* 不透明电机句柄 */
typedef struct EmmMotor EmmMotor_t;

/* 可读取的系统参数 */
typedef enum {
    EMM_PARAM_ENCODER_POS  = 0x31, /* 编码器校准值   */
    EMM_PARAM_TARGET_POS   = 0x33, /* 目标位置       */
    EMM_PARAM_CURRENT_VEL  = 0x35, /* 当前速度       */
    EMM_PARAM_CURRENT_POS  = 0x36, /* 当前位置       */
    EMM_PARAM_POS_ERROR    = 0x37, /* 位置误差       */
    EMM_PARAM_STATUS_FLAGS = 0x3A, /* 使能/零点标志   */
    EMM_PARAM_ORIGIN_FLAGS = 0x3B, /* 原点/堵转标志   */
    EMM_PARAM_BUS_VOLTAGE  = 0x24, /* 母线电压       */
    EMM_PARAM_CONF_STATE   = 0x42, /* 配置参数       */
} EmmParam_t;

/* ---- 生命周期 ---- */

/**
 * @brief  创建电机实例，绑定到指定串口
 * @param  addr  电机地址（1~254）
 * @param  port  共享的串口实例
 * @return 成功返回电机指针，失败返回 NULL
 */
EmmMotor_t *emm_motor_create(uint8_t addr, SerialPort_t *port);

/**
 * @brief  销毁电机实例
 * @param  self  电机实例（可传 NULL）
 */
void emm_motor_destroy(EmmMotor_t *self);

/* ---- 配置 ---- */

/**
 * @brief  设置控制模式（开环 / 闭环）
 * @param  self       电机实例
 * @param  ctrl_mode  1=开环，2=闭环
 * @param  persist    true 时写入 NVRAM 保存
 * @return ERR_OK 成功，负值为错误码
 */
int emm_motor_set_ctrl_mode(EmmMotor_t *self,
                            uint8_t ctrl_mode,
                            bool persist);

/* ---- 运动控制 ---- */

/**
 * @brief  使能电机输出
 * @param  self  电机实例
 * @return ERR_OK 成功，负值为错误码
 */
int emm_motor_enable(EmmMotor_t *self);

/**
 * @brief  关闭电机输出（自由转动）
 * @param  self  电机实例
 * @return ERR_OK 成功，负值为错误码
 */
int emm_motor_disable(EmmMotor_t *self);

/**
 * @brief  速度模式控制
 * @param  self  电机实例
 * @param  rpm   目标转速（-5000 ~ 5000，符号决定方向）
 * @param  acc   加速度斜坡 0~255（0=立即到位）
 * @param  sync  true=缓冲指令等待同步触发
 * @return ERR_OK 成功，负值为错误码
 */
int emm_motor_set_velocity(EmmMotor_t *self,
                           int16_t rpm,
                           uint8_t acc,
                           bool sync);

/**
 * @brief  位置模式控制
 * @param  self      电机实例
 * @param  pulses    脉冲数（正值 CW / 负值 CCW，绝对模式时忽略符号）
 * @param  rpm       移动速度 1~5000 RPM
 * @param  acc       加速度斜坡 0~255
 * @param  absolute  true=绝对位置模式，false=增量模式
 * @param  sync      true=缓冲指令等待同步触发
 * @return ERR_OK 成功，负值为错误码
 */
int emm_motor_set_position(EmmMotor_t *self,
                           int32_t pulses,
                           uint16_t rpm,
                           uint8_t acc,
                           bool absolute,
                           bool sync);

/**
 * @brief  立即停止（适用于所有控制模式）
 * @param  self  电机实例
 * @param  sync  true=缓冲指令等待同步触发
 * @return ERR_OK 成功，负值为错误码
 */
int emm_motor_stop(EmmMotor_t *self, bool sync);

/* ---- 同步触发 ---- */

/**
 * @brief  触发所有已缓冲的同步指令同时执行
 *
 * 向第一个电机的地址发送同步运动指令（0xFF 0x66），
 * 所有此前以 sync=true 下达的指令将同时启动。
 *
 * @param  motors  电机指针数组（必须共享同一根总线）
 * @param  count   数组长度
 * @return ERR_OK 成功，负值为错误码
 */
int emm_motor_sync_trigger(EmmMotor_t *motors[], size_t count);

/* ---- 状态读取 ---- */

/**
 * @brief  读取编码器校准值
 * @param  self   电机实例
 * @param  value  输出：编码器位置（脉冲数）
 * @return ERR_OK 成功，负值为错误码
 */
int emm_motor_read_encoder(EmmMotor_t *self, int32_t *value);

/**
 * @brief  读取当前位置
 * @param  self   电机实例
 * @param  value  输出：当前位置（脉冲数）
 * @return ERR_OK 成功，负值为错误码
 */
int emm_motor_read_position(EmmMotor_t *self, int32_t *value);

/**
 * @brief  读取当前速度
 * @param  self   电机实例
 * @param  value  输出：当前转速（RPM，带符号）
 * @return ERR_OK 成功，负值为错误码
 */
int emm_motor_read_velocity(EmmMotor_t *self, int16_t *value);

/**
 * @brief  读取位置误差
 * @param  self   电机实例
 * @param  value  输出：位置误差（脉冲数）
 * @return ERR_OK 成功，负值为错误码
 */
int emm_motor_read_error(EmmMotor_t *self, int16_t *value);

/**
 * @brief  将当前位置计数器清零
 * @param  self  电机实例
 * @return ERR_OK 成功，负值为错误码
 */
int emm_motor_reset_position(EmmMotor_t *self);

#endif /* SMART_CAR_EMM_V5_H */
