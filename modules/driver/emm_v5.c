/**
 * @file    modules/driver/emm_v5.c
 * @brief   EMM_V5 闭环步进电机驱动实现
 * @author  Ltttttts
 *
 * 通信协议格式：
 *   每一帧数据由以下部分组成（从上位机发给驱动器）：
 *     [地址字节] [命令字节(1~2个)] [参数...] [校验和 0x6B]
 *
 *   驱动器回复：
 *     [地址字节] [数据...] [校验和 0x6B]
 *
 * 字节序：多字节数值使用大端（Big-Endian）传输，
 *         即高字节在前、低字节在后。
 *
 * 注意：
 *   回复长度是基于常见协议惯例推测的，如果实际使用中发现通信异常，
 *   请对照 EMM_V5 官方手册验证。
 */

#include "driver/emm_v5.h"
#include "hal/serial_port.h"

#include <stdlib.h>
#include <string.h>

/* ========== 命令字节定义 ========== */
/* 每个命令的功能说明见对应的 API 函数注释 */

/* 单字节命令 */
#define EMM_CMD_ENABLE_CTRL       (0xF3U)  /* 使能/失能控制 */
#define EMM_CMD_VEL_CTRL          (0xF6U)  /* 速度模式控制 */
#define EMM_CMD_POS_CTRL          (0xFDU)  /* 位置模式控制 */
#define EMM_CMD_STOP_NOW          (0xFEU)  /* 立即停止 */
#define EMM_CMD_SYNC_MOTION       (0xFFU)  /* 同步运动触发 */
#define EMM_CMD_RESET_POS         (0x0AU)  /* 清除位置计数器 */
#define EMM_CMD_RESET_CLOG        (0x0EU)  /* 清除堵转报警 */
#define EMM_CMD_MODIFY_MODE       (0x46U)  /* 修改控制模式 */

/* 双字节命令的子命令字节（跟在主命令后面，用于区分同一命令的不同操作） */
#define EMM_SUBCMD_ENABLE_CTRL    (0xABU)  /* 使能/失能操作的子命令 */
#define EMM_SUBCMD_STOP_NOW       (0x98U)  /* 停止操作的子命令 */
#define EMM_SUBCMD_SYNC_MOTION    (0x66U)  /* 同步触发的子命令 */
#define EMM_SUBCMD_RESET_POS      (0x6DU)  /* 清零位置的子命令 */
#define EMM_SUBCMD_RESET_CLOG     (0x52U)  /* 清除堵转的子命令 */
#define EMM_SUBCMD_MODIFY_MODE    (0x69U)  /* 修改模式的子命令 */

/* 回复数据中不同参数的长度（字节数） */
#define EMM_RESP_DATA_POS_ENC     (4U)  /* 位置/编码器值占用 4 字节（int32） */
#define EMM_RESP_DATA_VEL         (2U)  /* 速度值占用 2 字节（int16） */
#define EMM_RESP_DATA_ERR         (2U)  /* 位置误差占用 2 字节（int16） */

/* ========== 电机对象结构体 ========== */
struct EmmMotor {
    uint8_t        addr;  /* 本电机在 UART 总线上的地址（与拨码开关一致） */
    SerialPort_t  *port;  /* 指向共享串口的指针（四个电机共用同一个串口） */
};

/* ============ 内部辅助函数 ============ */

/**
 * @brief  通过串口向电机发送一帧指令数据
 */
static int s_send_frame(EmmMotor_t *self,
                        const uint8_t *cmd,
                        size_t len)
{
    return serial_port_write(self->port, cmd, len);
}

/**
 * @brief  从串口接收电机的回复帧，并校验地址和校验和
 *
 * 回复帧结构（从驱动器发给上位机）：
 *   [地址(1字节)] [数据负载(data_len字节)] [校验和(1字节)]
 *
 * @param  self      电机实例（用于获取地址做校验）
 * @param  buf       输出缓冲区：存放数据负载部分
 * @param  buf_size  输出缓冲区大小
 * @param  data_len  期望的数据负载长度
 * @param  out_len   输出参数：实际收到的总字节数（含地址和校验和）
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

    /* 预期长度 = 1 字节地址 + data_len 字节数据 + 1 字节校验和 */
    expect = 1U + data_len + 1U;
    if (expect > sizeof(local)) {
        return ERR_OVERFLOW;   /* 缓冲区装不下，防止溢出 */
    }

    ret = serial_port_read(self->port, local, expect,
                           EMM_RESP_TIMEOUT_MS);
    if (ret < 0) {
        return ret;            /* 读取失败或超时 */
    }
    total = (size_t)ret;

    /* 最短也得有地址 + 校验和 2 个字节 */
    if (total < 2U) {
        return ERR_SERIAL_TIMEOUT;
    }

    /* 校验：回复中的地址必须匹配、结尾必须是 0x6B */
    if ((local[0] != self->addr) ||
        (local[total - 1U] != EMM_CHECKSUM_BYTE)) {
        return ERR_MOTOR_FAULT;   /* 校验不过说明通信异常 */
    }

    /* 把地址和校验和之间的数据负载拷贝给调用者 */
    if (buf != NULL && buf_size >= (total - 2U)) {
        memcpy(buf, &local[1], total - 2U);
    }
    if (out_len != NULL) {
        *out_len = total;
    }

    return ERR_OK;
}

/**
 * @brief  发送参数查询指令，并接收电机回复的参数值
 *
 * 通用的"读参数"流程：
 *   上位机发： [地址] [参数命令] [校验和]
 *   驱动器回： [地址] [参数值]  [校验和]
 *
 * @param  self       电机实例
 * @param  param_cmd  要读取的参数命令（如 EMM_PARAM_CURRENT_POS = 0x36）
 * @param  data_len   参数值的字节数（位置=4，速度=2，误差=2）
 * @param  data_out   输出缓冲区：存放读到的参数值
 * @return ERR_OK 成功，负值为错误码
 */
static int s_read_param(EmmMotor_t *self,
                        uint8_t param_cmd,
                        size_t data_len,
                        uint8_t *data_out)
{
    uint8_t cmd[EMM_CMD_BUF_SIZE];
    int     ret;

    /* 构建查询指令：3 个字节 */
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
 * @brief  把大端字节序的 4 字节数组转换为 int32_t
 *
 * EMM_V5 的 32 位数值使用大端传输（网络字节序）：
 *   buf[0] = 最高8位（bit 24-31）
 *   buf[3] = 最低8位（bit 0-7）
 */
static inline int32_t s_bytes_to_int32(const uint8_t *buf)
{
    return (int32_t)(((uint32_t)buf[0] << 24U) |
                     ((uint32_t)buf[1] << 16U) |
                     ((uint32_t)buf[2] << 8U)  |
                     ((uint32_t)buf[3]));
}

/**
 * @brief  把大端字节序的 2 字节数组转换为 int16_t
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
        return NULL;   /* 没有串口就没法通信 */
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

    /* 命令帧：6 字节 */
    cmd[0] = self->addr;         /* 目标电机地址 */
    cmd[1] = EMM_CMD_MODIFY_MODE;       /* 修改模式命令 0x46 */
    cmd[2] = EMM_SUBCMD_MODIFY_MODE;    /* 子命令 0x69 */
    cmd[3] = (uint8_t)(persist ? 1U : 0U);   /* 1 = 写入 NVRAM 永久保存 */
    cmd[4] = ctrl_mode;                   /* 1 = 开环, 2 = 闭环 */
    cmd[5] = EMM_CHECKSUM_BYTE;          /* 固定校验字节 */

    return s_send_frame(self, cmd, 6U);
}

/* ============ 运动控制 ============ */

int emm_motor_enable(EmmMotor_t *self)
{
    uint8_t cmd[EMM_CMD_BUF_SIZE];

    if (self == NULL) {
        return ERR_NULL_PTR;
    }

    /* 使能命令：6 字节，让电机通电保持转矩 */
    cmd[0] = self->addr;
    cmd[1] = EMM_CMD_ENABLE_CTRL;
    cmd[2] = EMM_SUBCMD_ENABLE_CTRL;
    cmd[3] = 1U;   /* 1 = 使能 */
    cmd[4] = 0U;   /* 立即执行，不等待同步触发 */
    cmd[5] = EMM_CHECKSUM_BYTE;

    return s_send_frame(self, cmd, 6U);
}

int emm_motor_disable(EmmMotor_t *self)
{
    uint8_t cmd[EMM_CMD_BUF_SIZE];

    if (self == NULL) {
        return ERR_NULL_PTR;
    }

    /* 失能命令：电机断电，可以自由转动 */
    cmd[0] = self->addr;
    cmd[1] = EMM_CMD_ENABLE_CTRL;
    cmd[2] = EMM_SUBCMD_ENABLE_CTRL;
    cmd[3] = 0U;   /* 0 = 断电 */
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

    /* 根据 rpm 的正负号决定旋转方向 */
    if (rpm >= 0) {
        dir     = EMM_DIR_CW;       /* 正数 = 顺时针 */
        abs_rpm = (uint16_t)rpm;
    } else {
        dir     = EMM_DIR_CCW;      /* 负数 = 逆时针 */
        abs_rpm = (uint16_t)(-rpm); /* 取绝对值 */
    }

    /* 速度限幅：不能超过驱动器支持的最大值 */
    if (abs_rpm > EMM_MAX_RPM) {
        abs_rpm = EMM_MAX_RPM;
    }

    /* 速度命令帧：8 字节
       帧格式：[地址][0xF6][方向][速度高8位][速度低8位][加速度][同步标志][校验] */
    cmd[0] = self->addr;
    cmd[1] = EMM_CMD_VEL_CTRL;
    cmd[2] = dir;                           /* 0 = CW, 1 = CCW */
    cmd[3] = (uint8_t)(abs_rpm >> 8U);      /* 速度值的高字节（大端） */
    cmd[4] = (uint8_t)(abs_rpm);            /* 速度值的低字节 */
    cmd[5] = acc;                           /* 0~255, 越大加速越快 */
    cmd[6] = (uint8_t)(sync ? 1U : 0U);     /* 同步标志：1=缓冲等触发 */
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

    /* 根据脉冲数正负决定方向 */
    if (pulses >= 0) {
        dir        = EMM_DIR_CW;
        abs_pulses = (uint32_t)pulses;
    } else {
        dir        = EMM_DIR_CCW;
        abs_pulses = (uint32_t)(-pulses);
    }

    /* 速度限幅 */
    if (rpm > EMM_MAX_RPM) {
        rpm = EMM_MAX_RPM;
    }

    /* 位置命令帧：13 字节（最长的命令）
       帧格式：[地址][0xFD][方向][速度高8位][速度低8位][加速度]
               [脉冲数bit24-31][bit16-23][bit8-15][bit0-7]
               [相对/绝对][同步标志][校验] */
    cmd[0]  = self->addr;
    cmd[1]  = EMM_CMD_POS_CTRL;
    cmd[2]  = dir;
    cmd[3]  = (uint8_t)(rpm >> 8U);
    cmd[4]  = (uint8_t)(rpm);
    cmd[5]  = acc;
    cmd[6]  = (uint8_t)(abs_pulses >> 24U);  /* 脉冲数最高8位 */
    cmd[7]  = (uint8_t)(abs_pulses >> 16U);
    cmd[8]  = (uint8_t)(abs_pulses >> 8U);
    cmd[9]  = (uint8_t)(abs_pulses);          /* 脉冲数最低8位 */
    cmd[10] = (uint8_t)(absolute ? 1U : 0U); /* 1=绝对位置, 0=相对位置 */
    cmd[11] = (uint8_t)(sync ? 1U : 0U);     /* 同步标志 */
    cmd[12] = EMM_CHECKSUM_BYTE;

    return s_send_frame(self, cmd, 13U);
}

int emm_motor_stop(EmmMotor_t *self, bool sync)
{
    uint8_t cmd[EMM_CMD_BUF_SIZE];

    if (self == NULL) {
        return ERR_NULL_PTR;
    }

    /* 立即停止命令：5 字节 */
    cmd[0] = self->addr;
    cmd[1] = EMM_CMD_STOP_NOW;
    cmd[2] = EMM_SUBCMD_STOP_NOW;
    cmd[3] = (uint8_t)(sync ? 1U : 0U);  /* 是否等同步触发 */
    cmd[4] = EMM_CHECKSUM_BYTE;

    return s_send_frame(self, cmd, 5U);
}

int emm_motor_reset_position(EmmMotor_t *self)
{
    uint8_t cmd[EMM_CMD_BUF_SIZE];

    if (self == NULL) {
        return ERR_NULL_PTR;
    }

    /* 当前位置清零：4 字节 */
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

    /* 遍历所有电机，向每个电机的地址发送同步触发指令。
       因为每个 EMM_V5 驱动器只响应自己地址的指令，
       所以要逐个发送，不能只发一个地址。 */
    for (size_t i = 0; i < count; i++) {
        cmd[0] = motors[i]->addr;
        cmd[1] = EMM_CMD_SYNC_MOTION;
        cmd[2] = EMM_SUBCMD_SYNC_MOTION;
        cmd[3] = EMM_CHECKSUM_BYTE;
        int ret = s_send_frame(motors[i], cmd, 4U);
        if (ret < 0) {
            return ret;
        }
    }
    return ERR_OK;
}

/* ============ 状态读取 ============ */

int emm_motor_read_encoder(EmmMotor_t *self, int32_t *value)
{
    uint8_t data[EMM_RESP_DATA_POS_ENC];
    int     ret;

    if (self == NULL || value == NULL) {
        return ERR_NULL_PTR;
    }

    /* 发送 0x31 查询编码器位置，回复 4 字节数据 */
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

    /* 发送 0x36 查询当前位置，回复 4 字节数据 */
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

    /* 发送 0x35 查询当前速度，回复 2 字节数据 */
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

    /* 发送 0x37 查询位置误差，回复 2 字节数据 */
    ret = s_read_param(self, EMM_PARAM_POS_ERROR,
                       EMM_RESP_DATA_ERR, data);
    if (ret == ERR_OK) {
        *value = s_bytes_to_int16(data);
    }
    return ret;
}
