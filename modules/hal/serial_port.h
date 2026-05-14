/**
 * @file    serial_port.h
 * @brief   串口硬件抽象层头文件 — 让上层代码不依赖 Linux termios 细节
 * @author  Ltttttts
 *
 * 设计模式：不透明指针（Opaque Pointer）
 *   头文件只暴露 SerialPort_t 的类型名，不暴露结构体内部成员。
 *   所有操作只能通过 API 函数完成，无法直接访问 fd/device 等字段。
 *   好处：上层代码与底层实现解耦，未来想换 Windows/FreeRTOS 只需改 .c 文件。
 */

#ifndef SMART_CAR_SERIAL_PORT_H
#define SMART_CAR_SERIAL_PORT_H

#include "common.h"

/* ---- 不透明句柄 ---- */
/* 只声明类型存在，具体结构体定义在 .c 文件中。
   外部代码只能靠指针使用，看不到里面的成员变量。 */
typedef struct SerialPort SerialPort_t;

/* ---- 生命周期（创建 → 使用 → 销毁） ---- */

/**
 * @brief  创建串口对象（只分配内存、保存参数，不打开设备）
 * @param  device    设备路径，例如 "/dev/ttyUSB0" 或 "/dev/ttyS1"
 * @param  baudrate  波特率数值，例如 115200、921600
 * @return 成功返回串口指针，失败返回 NULL
 * @note   创建后还需要调用 serial_port_open() 才能真正连接硬件
 */
SerialPort_t *serial_port_create(const char *device, uint32_t baudrate);

/**
 * @brief  销毁串口对象，释放所有资源
 * @param  self  串口实例（传 NULL 也没事，函数内部会判断）
 * @note   如果设备还开着，会自动先关闭再释放
 */
void serial_port_destroy(SerialPort_t *self);

/**
 * @brief  打开串口设备，配置为 8N1 模式（8数据位、无校验、1停止位）
 * @param  self  串口实例
 * @return ERR_OK 成功，负值为错误码（见 common.h 的 ErrorCode_t）
 * @note   自动配置为原始模式（raw mode），不做行缓冲和回显处理
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
 * @brief  向串口发送数据（阻塞模式，直到全部字节写入完成才返回）
 * @param  self  串口实例
 * @param  data  待发送的数据缓冲区
 * @param  len   要发送的字节数
 * @return 成功返回实际写入的字节数，失败返回负值错误码
 * @note   内部会调用 tcdrain 等待发送完成，确保数据全部发出
 */
int serial_port_write(SerialPort_t *self,
                      const uint8_t *data,
                      size_t len);

/**
 * @brief  从串口接收数据（带超时机制，内部使用 select() 实现）
 * @param  self        串口实例
 * @param  buf         接收缓冲区
 * @param  len         期望读取的最大字节数
 * @param  timeout_ms  超时时间（毫秒），超时则返回 ERR_SERIAL_TIMEOUT
 * @return 成功返回实际读取到的字节数，失败返回负值错误码
 */
int serial_port_read(SerialPort_t *self,
                     uint8_t *buf,
                     size_t len,
                     uint32_t timeout_ms);

/**
 * @brief  清空串口的收发缓冲区（丢弃所有尚未处理的数据）
 * @param  self  串口实例
 * @return ERR_OK 成功，负值为错误码
 */
int serial_port_flush(SerialPort_t *self);

#endif /* SMART_CAR_SERIAL_PORT_H */
