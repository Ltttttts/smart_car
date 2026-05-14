/**
 * @file    emm_v5.h
 * @brief   EMM_V5 闭环步进电机驱动头文件 — UART 通信协议封装
 * @author  Ltttttts
 *
 * 四个 EMM_V5 驱动器共用一根 UART 总线（像多台设备挂在同一根线上），
 * 通过 1~4 号地址来区分每个电机。
 *
 * 关键功能：
 *   1. 速度模式：设定 RPM 转速，电机持续旋转
 *   2. 位置模式：设定脉冲数，电机精确转动指定角度
 *   3. 同步运动：先缓冲指令，再一次性触发所有电机同时启动
 */

#ifndef SMART_CAR_EMM_V5_H
#define SMART_CAR_EMM_V5_H

#include "common.h"

/* 前向声明：告诉编译器 SerialPort_t 这个类型存在，定义在 serial_port.h 里 */
typedef struct SerialPort SerialPort_t;

/* 不透明电机句柄：外部代码看不到 EmmMotor 结构体的内部细节 */
typedef struct EmmMotor EmmMotor_t;

/* ========== 可读取的电机系统参数 ========== */
/* 这些枚举值实际上是 UART 协议中的"查询命令字节"。
   发一条读命令给电机，电机就会回复对应的参数值。
   比如发 0x36 给电机，它会回复当前的位置脉冲数。 */
typedef enum {
    EMM_PARAM_ENCODER_POS  = 0x31, /* 编码器校准后的位置值（脉冲数） */
    EMM_PARAM_TARGET_POS   = 0x33, /* 目标位置（闭环要到达的位置） */
    EMM_PARAM_CURRENT_VEL  = 0x35, /* 当前实时转速（RPM） */
    EMM_PARAM_CURRENT_POS  = 0x36, /* 当前位置（脉冲数，最常用的状态） */
    EMM_PARAM_POS_ERROR    = 0x37, /* 目标位置与当前位置的误差（脉冲数） */
    EMM_PARAM_STATUS_FLAGS = 0x3A, /* 状态标志位：使能/零点/报警等 */
    EMM_PARAM_ORIGIN_FLAGS = 0x3B, /* 回零状态、堵转检测标志 */
    EMM_PARAM_BUS_VOLTAGE  = 0x24, /* 电源母线电压（V） */
    EMM_PARAM_CONF_STATE   = 0x42, /* 当前配置参数 */
} EmmParam_t;

/* ---- 生命周期（创建 → 使用 → 销毁） ---- */

/**
 * @brief  创建一个电机实例，绑定到指定的串口
 * @param  addr  电机的总线地址（1~254），必须与驱动器拨码开关设置一致
 * @param  port  共享的串口实例（四个电机共用同一根串口总线）
 * @return 成功返回电机指针，失败返回 NULL
 */
EmmMotor_t *emm_motor_create(uint8_t addr, SerialPort_t *port);

/**
 * @brief  销毁电机实例，释放内存
 * @param  self  电机实例（传 NULL 也没关系）
 */
void emm_motor_destroy(EmmMotor_t *self);

/* ---- 配置 ---- */

/**
 * @brief  设置电机的控制模式
 * @param  self       电机实例
 * @param  ctrl_mode  1 = 开环模式（无反馈），2 = 闭环模式（编码器反馈）
 * @param  persist    true = 写入 NVRAM 永久保存，断电不丢失
 * @return ERR_OK 成功，负值为错误码
 * @note   正常使用时选闭环模式（2），驱动器会自动维持目标位置/速度
 */
int emm_motor_set_ctrl_mode(EmmMotor_t *self,
                            uint8_t ctrl_mode,
                            bool persist);

/* ---- 运动控制 ---- */

/**
 * @brief  使能电机输出（接通电流，电机可以转动）
 * @param  self  电机实例
 * @return ERR_OK 成功，负值为错误码
 */
int emm_motor_enable(EmmMotor_t *self);

/**
 * @brief  关闭电机输出（断开电流，电机可以自由转动）
 * @param  self  电机实例
 * @return ERR_OK 成功，负值为错误码
 */
int emm_motor_disable(EmmMotor_t *self);

/**
 * @brief  速度模式：让电机以指定的 RPM 持续旋转
 * @param  self  电机实例
 * @param  rpm   目标转速，范围 -5000 ~ 5000（负号 = 反转）
 * @param  acc   加速度斜坡 0~255（数值越大加速越快，0 = 立即到位）
 * @param  sync  true = 先缓冲在驱动器里，等同步触发再执行
 * @return ERR_OK 成功，负值为错误码
 */
int emm_motor_set_velocity(EmmMotor_t *self,
                           int16_t rpm,
                           uint8_t acc,
                           bool sync);

/**
 * @brief  位置模式：让电机精确转动指定的脉冲数
 * @param  self      电机实例
 * @param  pulses    目标脉冲数（正 = 顺时针，负 = 逆时针）
 * @param  rpm       移动过程中的速度 1~5000 RPM
 * @param  acc       加速度斜坡 0~255
 * @param  absolute  true = 绝对位置（相对零点），false = 增量（相对当前位置）
 * @param  sync      true = 缓冲等待同步触发
 * @return ERR_OK 成功，负值为错误码
 */
int emm_motor_set_position(EmmMotor_t *self,
                           int32_t pulses,
                           uint16_t rpm,
                           uint8_t acc,
                           bool absolute,
                           bool sync);

/**
 * @brief  立即停止电机（无论当前是什么控制模式）
 * @param  self  电机实例
 * @param  sync  true = 缓冲等待同步触发（用于多电机同时停止）
 * @return ERR_OK 成功，负值为错误码
 */
int emm_motor_stop(EmmMotor_t *self, bool sync);

/* ---- 同步触发 ---- */

/**
 * @brief  触发所有已缓冲的同步指令同时执行
 *
 * 使用场景：想让四个电机同时启动（而不是逐个启动）
 * 1. 先调用四次 emm_motor_set_velocity(..., sync=true) 缓冲指令
 * 2. 再调用本函数，四个电机就会同时开始运动
 *
 * @param  motors  电机指针数组（四个电机必须共享同一根串口）
 * @param  count   数组中电机的个数
 * @return ERR_OK 成功，负值为错误码
 */
int emm_motor_sync_trigger(EmmMotor_t *motors[], size_t count);

/* ---- 状态读取（用于里程计和故障检测） ---- */

/**
 * @brief  读取编码器的校准位置值
 * @param  self   电机实例
 * @param  value  输出参数：编码器当前位置（脉冲数）
 * @return ERR_OK 成功，负值为错误码
 */
int emm_motor_read_encoder(EmmMotor_t *self, int32_t *value);

/**
 * @brief  读取电机的当前位置（脉冲数）
 * @param  self   电机实例
 * @param  value  输出参数：当前位置
 * @return ERR_OK 成功，负值为错误码
 * @note   可用于推算车轮转过的角度，进而计算里程计
 */
int emm_motor_read_position(EmmMotor_t *self, int32_t *value);

/**
 * @brief  读取电机的当前实时转速
 * @param  self   电机实例
 * @param  value  输出参数：当前 RPM（带符号，负 = 反转）
 * @return ERR_OK 成功，负值为错误码
 */
int emm_motor_read_velocity(EmmMotor_t *self, int16_t *value);

/**
 * @brief  读取位置闭环控制的跟踪误差
 * @param  self   电机实例
 * @param  value  输出参数：位置误差（脉冲数）
 * @return ERR_OK 成功，负值为错误码
 * @note   误差越大说明负载越重或加速度太快，用于判断是否堵转
 */
int emm_motor_read_error(EmmMotor_t *self, int16_t *value);

/**
 * @brief  把电机的当前位置计数器清零（设为新的零点）
 * @param  self  电机实例
 * @return ERR_OK 成功，负值为错误码
 */
int emm_motor_reset_position(EmmMotor_t *self);

#endif /* SMART_CAR_EMM_V5_H */
