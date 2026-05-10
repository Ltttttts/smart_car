/**
 * @file    serial_port.c
 * @brief   Linux 串口 HAL 层实现
 * @author  Ltttttts
 */

#include "serial_port.h"

#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

/* 设备路径最大长度 */
#define SERIAL_DEVICE_MAX_LEN  (256U)

struct SerialPort {
    int      fd;                            /* 文件描述符       */
    char     device[SERIAL_DEVICE_MAX_LEN]; /* 设备路径         */
    uint32_t baudrate;                      /* 波特率（整数值） */
};

/**
 * @brief  将整型波特率转换为 termios 的 speed_t 常量
 */
static speed_t s_baud_to_speed(uint32_t baud)
{
    switch (baud) {
    case 9600U:   return B9600;
    case 19200U:  return B19200;
    case 38400U:  return B38400;
    case 57600U:  return B57600;
    case 115200U: return B115200;
    case 230400U: return B230400;
    case 460800U: return B460800;
    case 921600U: return B921600;
    default:      return B115200;
    }
}

/* ============ 生命周期 ============ */

SerialPort_t *serial_port_create(const char *device, uint32_t baudrate)
{
    SerialPort_t *self;

    if (device == NULL) {
        return NULL;
    }

    self = (SerialPort_t *)calloc(1U, sizeof(SerialPort_t));
    if (self == NULL) {
        return NULL;
    }

    self->fd       = -1;
    self->baudrate = baudrate;
    strncpy(self->device, device, SERIAL_DEVICE_MAX_LEN - 1U);
    self->device[SERIAL_DEVICE_MAX_LEN - 1U] = '\0';

    return self;
}

void serial_port_destroy(SerialPort_t *self)
{
    if (self == NULL) {
        return;
    }
    if (self->fd >= 0) {
        serial_port_close(self);
    }
    free(self);
}

/* ============ 打开 / 关闭 ============ */

int serial_port_open(SerialPort_t *self)
{
    struct termios tty;
    int ret;

    if (self == NULL) {
        return ERR_NULL_PTR;
    }

    self->fd = open(self->device, O_RDWR | O_NOCTTY | O_SYNC);
    if (self->fd < 0) {
        return ERR_SERIAL_OPEN;
    }

    ret = tcgetattr(self->fd, &tty);
    if (ret != 0) {
        close(self->fd);
        self->fd = -1;
        return ERR_SERIAL_OPEN;
    }

    /* 设置输入输出波特率 */
    cfsetospeed(&tty, s_baud_to_speed(self->baudrate));
    cfsetispeed(&tty, s_baud_to_speed(self->baudrate));

    /* 8 位数据位，无校验，1 位停止位，无硬件流控 */
    tty.c_cflag &= (tcflag_t)(~PARENB);
    tty.c_cflag &= (tcflag_t)(~CSTOPB);
    tty.c_cflag &= (tcflag_t)(~CSIZE);
    tty.c_cflag |= (tcflag_t)(CS8);

    /* 禁用硬件流控（部分平台未定义 CRTSCTS，默认即为关闭） */
#ifdef CRTSCTS
    tty.c_cflag &= (tcflag_t)(~CRTSCTS);
#endif
    tty.c_cflag |= (tcflag_t)(CREAD | CLOCAL);

    /* 原始模式：关闭行缓冲、回显、信号处理 */
    tty.c_lflag &= (tcflag_t)(~(ICANON | ECHO | ECHOE | ISIG));
    tty.c_iflag &= (tcflag_t)(~(IXON | IXOFF | IXANY));
    tty.c_oflag &= (tcflag_t)(~OPOST);

    /* VMIN=0 非阻塞等待，VTIME=1 字节间超时 100ms */
    tty.c_cc[VMIN]  = 0U;
    tty.c_cc[VTIME] = 1U;

    ret = tcsetattr(self->fd, TCSANOW, &tty);
    if (ret != 0) {
        close(self->fd);
        self->fd = -1;
        return ERR_SERIAL_OPEN;
    }

    return ERR_OK;
}

int serial_port_close(SerialPort_t *self)
{
    if (self == NULL) {
        return ERR_NULL_PTR;
    }
    if (self->fd < 0) {
        return ERR_OK;
    }

    close(self->fd);
    self->fd = -1;
    return ERR_OK;
}

/* ============ 数据收发 ============ */

int serial_port_write(SerialPort_t *self,
                      const uint8_t *data,
                      size_t len)
{
    ssize_t written;

    if (self == NULL || data == NULL) {
        return ERR_NULL_PTR;
    }
    if (self->fd < 0) {
        return ERR_NOT_INITIALIZED;
    }
    if (len == 0U) {
        return ERR_OK;
    }

    written = write(self->fd, data, len);
    if (written < 0) {
        return ERR_SERIAL_WRITE;
    }

    return (int)written;
}

int serial_port_read(SerialPort_t *self,
                     uint8_t *buf,
                     size_t len,
                     uint32_t timeout_ms)
{
    fd_set         fds;
    struct timeval tv;
    ssize_t        n;
    int            ret;

    if (self == NULL || buf == NULL) {
        return ERR_NULL_PTR;
    }
    if (self->fd < 0) {
        return ERR_NOT_INITIALIZED;
    }
    if (len == 0U) {
        return ERR_OK;
    }

    FD_ZERO(&fds);
    FD_SET(self->fd, &fds);

    /* 设置超时 */
    tv.tv_sec  = (time_t)(timeout_ms / 1000U);
    tv.tv_usec = (suseconds_t)((timeout_ms % 1000U) * 1000U);

    ret = select(self->fd + 1, &fds, NULL, NULL, &tv);
    if (ret < 0) {
        return ERR_SERIAL_READ;
    }
    if (ret == 0) {
        return ERR_SERIAL_TIMEOUT;
    }

    n = read(self->fd, buf, len);
    if (n < 0) {
        return ERR_SERIAL_READ;
    }

    return (int)n;
}

/* ============ 辅助 ============ */

int serial_port_flush(SerialPort_t *self)
{
    if (self == NULL) {
        return ERR_NULL_PTR;
    }
    if (self->fd < 0) {
        return ERR_NOT_INITIALIZED;
    }

    /* 清空输入和输出缓冲区 */
    return tcflush(self->fd, TCIOFLUSH) == 0 ? ERR_OK
                                             : ERR_SERIAL_READ;
}
