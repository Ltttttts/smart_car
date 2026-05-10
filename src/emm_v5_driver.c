/**
 * @file    emm_v5_driver.c
 * @brief   EMM_V5 闭环步进电机驱动实现
 * @author  Ltttttts
 *
 * 通信帧格式: [地址][命令字节][参数...][校验 0x6B]
 * 所有数值以大端字节序传输。
 *
 * 注意：回复数据长度基于常见协议惯例推测，
 * 正式使用前需对照 EMM_V5 手册验证。
 */

#include "emm_v5_driver.h"
#include "serial_port.h"

#include <stdlib.h>
#include <string.h>

/* ---- 命令字节定义 ---- */
#define EMM_CMD_ENABLE_CTRL       (0xF3U)
#define EMM_CMD_VEL_CTRL          (0xF6U)
#define EMM_CMD_POS_CTRL          (0xFDU)
#define EMM_CMD_STOP_NOW          (0xFEU)
#define EMM_CMD_SYNC_MOTION       (0xFFU)
#define EMM_CMD_RESET_POS         (0x0AU)
#define EMM_CMD_RESET_CLOG        (0x0EU)
#define EMM_CMD_MODIFY_MODE       (0x46U)

/* 双字节命令的子命令字节 */
#define EMM_SUBCMD_ENABLE_CTRL    (0xABU)
#define EMM_SUBCMD_STOP_NOW       (0x98U)
#define EMM_SUBCMD_SYNC_MOTION    (0x66U)
#define EMM_SUBCMD_RESET_POS      (0x6DU)
#define EMM_SUBCMD_RESET_CLOG     (0x52U)
#define EMM_SUBCMD_MODIFY_MODE    (0x69U)

/* 回复数据长度 */
#define EMM_RESP_DATA_POS_ENC     (4U)  /* 位置/编码器: 4 字节 */
#define EMM_RESP_DATA_VEL         (2U)  /* 速度:        2 字节 */
#define EMM_RESP_DATA_ERR         (2U)  /* 误差:        2 字节 */

/* ---- 电机对象 ---- */
struct EmmMotor {
    uint8_t        addr;  /* 总线地址 */
    SerialPort_t  *port;  /* 所属串口 */
};

/* ============ 内部辅助函数 ============ */

/**
 * @brief  向电机发送一帧完整指令
 */
static int s_send_frame(EmmMotor_t *self,
                        const uint8_t *cmd,
                        size_t len)
{
    return serial_port_write(self->port, cmd, len);
}

/**
 * @brief  接收回复帧，校验地址和校验和
 * @param  self      电机实例
 * @param  buf       输出缓冲区（存放数据负载）
 * @param  buf_size  缓冲区大小
 * @param  data_len  期望的数据负载长度
 * @param  out_len   输出：实际收到的总字节数
 * @return ERR_OK 成功，负值为错误码
 */
static int s_recv_response(EmmMotor_t *self,
                           uint8_t *buf,
                           size_t buf_size,
                           size_t data_len,
                           size_t *out_len)
{
    uint8_t  local[EMM_RESP_BUF_SIZE];
    size_t   expect;
    size_t   total;
    int      ret;

    /* 回复帧结构: [地址(1)][数据(data_len)][校验和(1)] */
    expect = 1U + data_len + 1U;
    if (expect > sizeof(local)) {
        return ERR_OVERFLOW;
    }

    ret = serial_port_read(self->port, local, expect,
                           EMM_RESP_TIMEOUT_MS);
    if (ret < 0) {
        return ret;
    }
    total = (size_t)ret;

    if (total < 2U) {
        return ERR_SERIAL_TIMEOUT;
    }

    /* 校验地址和校验和 */
    if ((local[0] != self->addr) ||
        (local[total - 1U] != EMM_CHECKSUM_BYTE)) {
        return ERR_MOTOR_FAULT;
    }

    /* 拷贝数据负载（地址与校验和之间的部分） */
    if (buf != NULL && buf_size >= (total - 2U)) {
        memcpy(buf, &local[1], total - 2U);
    }
    if (out_len != NULL) {
        *out_len = total;
    }

    return ERR_OK;
}

/**
 * @brief  发送参数查询指令并接收回复
 */
static int s_read_param(EmmMotor_t *self,
                        uint8_t param_cmd,
                        size_t data_len,
                        uint8_t *data_out)
{
    uint8_t cmd[EMM_CMD_BUF_SIZE];
    int     ret;

    cmd[0] = self->addr;
    cmd[1] = param_cmd;
    cmd[2] = EMM_CHECKSUM_BYTE;

    ret = s_send_frame(self, cmd, 3U);
    if (ret < 0) {
        return ret;
    }

    return s_recv_response(self, data_out, data_len,
                           data_len, NULL);
}

/**
 * @brief  从大端字节数组提取 int32
 */
static inline int32_t s_bytes_to_int32(const uint8_t *buf)
{
    return (int32_t)(((uint32_t)buf[0] << 24U) |
                     ((uint32_t)buf[1] << 16U) |
                     ((uint32_t)buf[2] << 8U)  |
                     ((uint32_t)buf[3]));
}

/**
 * @brief  从大端字节数组提取 int16
 */
static inline int16_t s_bytes_to_int16(const uint8_t *buf)
{
    return (int16_t)(((uint16_t)buf[0] << 8U) |
                     ((uint16_t)buf[1]));
}

/* ============ 生命周期 ============ */

EmmMotor_t *emm_motor_create(uint8_t addr, SerialPort_t *port)
{
    EmmMotor_t *self;

    if (port == NULL) {
        return NULL;
    }

    self = (EmmMotor_t *)calloc(1U, sizeof(EmmMotor_t));
    if (self == NULL) {
        return NULL;
    }

    self->addr = addr;
    self->port = port;

    return self;
}

void emm_motor_destroy(EmmMotor_t *self)
{
    if (self != NULL) {
        free(self);
    }
}

/* ============ 配置 ============ */

int emm_motor_set_ctrl_mode(EmmMotor_t *self,
                            uint8_t ctrl_mode,
                            bool persist)
{
    uint8_t cmd[EMM_CMD_BUF_SIZE];

    if (self == NULL) {
        return ERR_NULL_PTR;
    }

    cmd[0] = self->addr;
    cmd[1] = EMM_CMD_MODIFY_MODE;
    cmd[2] = EMM_SUBCMD_MODIFY_MODE;
    cmd[3] = (uint8_t)(persist ? 1U : 0U);   /* 是否保存到 NVRAM */
    cmd[4] = ctrl_mode;                      /* 1=开环 2=闭环   */
    cmd[5] = EMM_CHECKSUM_BYTE;

    return s_send_frame(self, cmd, 6U);
}

/* ============ 运动控制 ============ */

int emm_motor_enable(EmmMotor_t *self)
{
    uint8_t cmd[EMM_CMD_BUF_SIZE];

    if (self == NULL) {
        return ERR_NULL_PTR;
    }

    cmd[0] = self->addr;
    cmd[1] = EMM_CMD_ENABLE_CTRL;
    cmd[2] = EMM_SUBCMD_ENABLE_CTRL;
    cmd[3] = 1U;   /* 使能 */
    cmd[4] = 0U;   /* 立即执行（不等同步触发） */
    cmd[5] = EMM_CHECKSUM_BYTE;

    return s_send_frame(self, cmd, 6U);
}

int emm_motor_disable(EmmMotor_t *self)
{
    uint8_t cmd[EMM_CMD_BUF_SIZE];

    if (self == NULL) {
        return ERR_NULL_PTR;
    }

    cmd[0] = self->addr;
    cmd[1] = EMM_CMD_ENABLE_CTRL;
    cmd[2] = EMM_SUBCMD_ENABLE_CTRL;
    cmd[3] = 0U;   /* 失能 */
    cmd[4] = 0U;   /* 立即执行 */
    cmd[5] = EMM_CHECKSUM_BYTE;

    return s_send_frame(self, cmd, 6U);
}

int emm_motor_set_velocity(EmmMotor_t *self,
                           int16_t rpm,
                           uint8_t acc,
                           bool sync)
{
    uint8_t  cmd[EMM_CMD_BUF_SIZE];
    uint16_t abs_rpm;
    uint8_t  dir;

    if (self == NULL) {
        return ERR_NULL_PTR;
    }

    /* 根据 rpm 符号决定方向 */
    if (rpm >= 0) {
        dir     = EMM_DIR_CW;
        abs_rpm = (uint16_t)rpm;
    } else {
        dir     = EMM_DIR_CCW;
        abs_rpm = (uint16_t)(-rpm);
    }

    /* 限幅 */
    if (abs_rpm > EMM_MAX_RPM) {
        abs_rpm = EMM_MAX_RPM;
    }

    cmd[0] = self->addr;
    cmd[1] = EMM_CMD_VEL_CTRL;
    cmd[2] = dir;                           /* 方向           */
    cmd[3] = (uint8_t)(abs_rpm >> 8U);      /* 速度高字节     */
    cmd[4] = (uint8_t)(abs_rpm);            /* 速度低字节     */
    cmd[5] = acc;                           /* 加速度         */
    cmd[6] = (uint8_t)(sync ? 1U : 0U);     /* 同步标志       */
    cmd[7] = EMM_CHECKSUM_BYTE;

    return s_send_frame(self, cmd, 8U);
}

int emm_motor_set_position(EmmMotor_t *self,
                           int32_t pulses,
                           uint16_t rpm,
                           uint8_t acc,
                           bool absolute,
                           bool sync)
{
    uint8_t  cmd[EMM_CMD_BUF_SIZE];
    uint32_t abs_pulses;
    uint8_t  dir;

    if (self == NULL) {
        return ERR_NULL_PTR;
    }

    /* 根据脉冲数符号决定方向 */
    if (pulses >= 0) {
        dir        = EMM_DIR_CW;
        abs_pulses = (uint32_t)pulses;
    } else {
        dir        = EMM_DIR_CCW;
        abs_pulses = (uint32_t)(-pulses);
    }

    /* 限幅 */
    if (rpm > EMM_MAX_RPM) {
        rpm = EMM_MAX_RPM;
    }

    cmd[0]  = self->addr;
    cmd[1]  = EMM_CMD_POS_CTRL;
    cmd[2]  = dir;                          /* 方向             */
    cmd[3]  = (uint8_t)(rpm >> 8U);         /* 速度高字节       */
    cmd[4]  = (uint8_t)(rpm);               /* 速度低字节       */
    cmd[5]  = acc;                          /* 加速度           */
    cmd[6]  = (uint8_t)(abs_pulses >> 24U); /* 脉冲数 bit24-31  */
    cmd[7]  = (uint8_t)(abs_pulses >> 16U); /* 脉冲数 bit16-23  */
    cmd[8]  = (uint8_t)(abs_pulses >> 8U);  /* 脉冲数 bit8-15   */
    cmd[9]  = (uint8_t)(abs_pulses);        /* 脉冲数 bit0-7    */
    cmd[10] = (uint8_t)(absolute ? 1U : 0U);/* 相对/绝对标志    */
    cmd[11] = (uint8_t)(sync ? 1U : 0U);    /* 同步标志         */
    cmd[12] = EMM_CHECKSUM_BYTE;

    return s_send_frame(self, cmd, 13U);
}

int emm_motor_stop(EmmMotor_t *self, bool sync)
{
    uint8_t cmd[EMM_CMD_BUF_SIZE];

    if (self == NULL) {
        return ERR_NULL_PTR;
    }

    cmd[0] = self->addr;
    cmd[1] = EMM_CMD_STOP_NOW;
    cmd[2] = EMM_SUBCMD_STOP_NOW;
    cmd[3] = (uint8_t)(sync ? 1U : 0U);
    cmd[4] = EMM_CHECKSUM_BYTE;

    return s_send_frame(self, cmd, 5U);
}

int emm_motor_reset_position(EmmMotor_t *self)
{
    uint8_t cmd[EMM_CMD_BUF_SIZE];

    if (self == NULL) {
        return ERR_NULL_PTR;
    }

    cmd[0] = self->addr;
    cmd[1] = EMM_CMD_RESET_POS;
    cmd[2] = EMM_SUBCMD_RESET_POS;
    cmd[3] = EMM_CHECKSUM_BYTE;

    return s_send_frame(self, cmd, 4U);
}

/* ============ 同步触发 ============ */

int emm_motor_sync_trigger(EmmMotor_t *motors[], size_t count)
{
    uint8_t cmd[EMM_CMD_BUF_SIZE];

    if (motors == NULL || count == 0U) {
        return ERR_INVALID_PARAM;
    }

    /* 所有电机共享同一总线，向第一个电机地址发送触发即可 */
    cmd[0] = motors[0]->addr;
    cmd[1] = EMM_CMD_SYNC_MOTION;
    cmd[2] = EMM_SUBCMD_SYNC_MOTION;
    cmd[3] = EMM_CHECKSUM_BYTE;

    return s_send_frame(motors[0], cmd, 4U);
}

/* ============ 状态读取 ============ */

int emm_motor_read_encoder(EmmMotor_t *self, int32_t *value)
{
    uint8_t data[EMM_RESP_DATA_POS_ENC];
    int     ret;

    if (self == NULL || value == NULL) {
        return ERR_NULL_PTR;
    }

    ret = s_read_param(self, EMM_PARAM_ENCODER_POS,
                       EMM_RESP_DATA_POS_ENC, data);
    if (ret == ERR_OK) {
        *value = s_bytes_to_int32(data);
    }
    return ret;
}

int emm_motor_read_position(EmmMotor_t *self, int32_t *value)
{
    uint8_t data[EMM_RESP_DATA_POS_ENC];
    int     ret;

    if (self == NULL || value == NULL) {
        return ERR_NULL_PTR;
    }

    ret = s_read_param(self, EMM_PARAM_CURRENT_POS,
                       EMM_RESP_DATA_POS_ENC, data);
    if (ret == ERR_OK) {
        *value = s_bytes_to_int32(data);
    }
    return ret;
}

int emm_motor_read_velocity(EmmMotor_t *self, int16_t *value)
{
    uint8_t data[EMM_RESP_DATA_VEL];
    int     ret;

    if (self == NULL || value == NULL) {
        return ERR_NULL_PTR;
    }

    ret = s_read_param(self, EMM_PARAM_CURRENT_VEL,
                       EMM_RESP_DATA_VEL, data);
    if (ret == ERR_OK) {
        *value = s_bytes_to_int16(data);
    }
    return ret;
}

int emm_motor_read_error(EmmMotor_t *self, int16_t *value)
{
    uint8_t data[EMM_RESP_DATA_ERR];
    int     ret;

    if (self == NULL || value == NULL) {
        return ERR_NULL_PTR;
    }

    ret = s_read_param(self, EMM_PARAM_POS_ERROR,
                       EMM_RESP_DATA_ERR, data);
    if (ret == ERR_OK) {
        *value = s_bytes_to_int16(data);
    }
    return ret;
}
