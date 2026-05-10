/**
 * @file    serial_port.h
 * @brief   Linux 串口硬件抽象层 — 基于 POSIX termios 的不透明封装
 * @author  Ltttttts
 */

#ifndef SMART_CAR_SERIAL_PORT_H
#define SMART_CAR_SERIAL_PORT_H

#include "common.h"

/* 不透明串口句柄，外部不可访问内部成员 */
typedef struct SerialPort SerialPort_t;

/* ---- 生命周期 ---- */

/**
 * @brief  创建串口对象（仅分配内存，不打开设备）
 * @param  device    设备路径，例如 "/dev/ttyUSB0"
 * @param  baudrate  波特率数值，例如 115200
 * @return 成功返回串口指针，失败返回 NULL
 */
SerialPort_t *serial_port_create(const char *device, uint32_t baudrate);

/**
 * @brief  销毁串口对象（若仍打开则先关闭）
 * @param  self  串口实例
 */
void serial_port_destroy(SerialPort_t *self);

/**
 * @brief  打开串口并配置为 8N1，无硬件流控
 * @param  self  串口实例
 * @return ERR_OK 成功，负值为错误码
 */
int serial_port_open(SerialPort_t *self);

/**
 * @brief  关闭串口描述符
 * @param  self  串口实例
 * @return ERR_OK 成功，负值为错误码
 */
int serial_port_close(SerialPort_t *self);

/* ---- 数据收发 ---- */

/**
 * @brief  向串口写入数据（阻塞，直到给定字节全部写入）
 * @param  self  串口实例
 * @param  data  待发送数据缓冲区
 * @param  len   待发送字节数
 * @return 成功返回写入字节数，失败返回负值错误码
 */
int serial_port_write(SerialPort_t *self,
                      const uint8_t *data,
                      size_t len);

/**
 * @brief  从串口读取数据，带超时（内部使用 select()）
 * @param  self        串口实例
 * @param  buf         接收缓冲区
 * @param  len         期望读取的最大字节数
 * @param  timeout_ms  超时时间（毫秒）
 * @return 成功返回实际读取字节数，超时返回 ERR_SERIAL_TIMEOUT
 */
int serial_port_read(SerialPort_t *self,
                     uint8_t *buf,
                     size_t len,
                     uint32_t timeout_ms);

/**
 * @brief  清空串口的收发缓冲区
 * @param  self  串口实例
 * @return ERR_OK 成功，负值为错误码
 */
int serial_port_flush(SerialPort_t *self);

#endif /* SMART_CAR_SERIAL_PORT_H */
