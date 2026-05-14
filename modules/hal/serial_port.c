/**
 * @file    serial_port.c
 * @brief   串口硬件抽象层实现 — 底层用 Linux POSIX termios API
 * @author  Ltttttts
 *
 * 实现思路：
 *   封装 Linux 的 open()/read()/write()/tcsetattr() 等系统调用，
 *   对外提供简洁的创建-打开-读写-关闭-销毁生命周期。
 *
 * 关于 termios：
 *   Linux 串口默认是"行缓冲"模式（按回车才提交数据），
 *   这对电机控制来说完全不可用。我们需要设为"原始模式"（raw mode），
 *   让每个字节都立即收发，不做任何加工处理。
 */

#include "hal/serial_port.h"

#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

/* 设备路径的最大字符数 */
#define SERIAL_DEVICE_MAX_LEN  (256U)

/* ========== 结构体定义 ========== */
/* 头文件中只声明了 SerialPort_t 这个名字，具体的成员在这里定义。
   这就是"不透明指针"模式——外部代码看不到这些细节。 */
struct SerialPort {
    int      fd;                            /* 串口文件描述符（-1 表示未打开） */
    char     device[SERIAL_DEVICE_MAX_LEN]; /* 设备路径，例如 "/dev/ttyUSB0" */
    uint32_t baudrate;                      /* 波特率，例如 115200、921600 */
};

/**
 * @brief  把整数波特率转成 termios 库需要的 speed_t 常量
 *
 * Linux termios 的 cfsetospeed() 不接受整数波特率，
 * 它需要 B115200、B921600 这种宏常量。这个函数做转换。
 *
 * @param  baud  整数波特率
 * @return termios 的 speed_t 常量，找不到就默认返回 B115200
 */
static speed_t s_baud_to_speed(uint32_t baud)
{
    switch (baud) {
    case 9600U:   return B9600;
    case 19200U:  return B19200;
    case 38400U:  return B38400;
    case 57600U:  return B57600;
    case 115200U: return B115200;    /* 最常用的标准波特率 */
    case 230400U: return B230400;
    case 460800U: return B460800;
    case 921600U: return B921600;    /* EMM_V5 驱动器的推荐波特率 */
    default:      return B115200;    /* 不认识的就用 115200 保底 */
    }
}

/* ============ 生命周期 ============ */

SerialPort_t *serial_port_create(const char *device, uint32_t baudrate)
{
    SerialPort_t *self;

    /* 参数检查：设备路径不能为空 */
    if (device == NULL) {
        return NULL;
    }

    /* 分配内存并用 calloc 清零（避免未初始化的字段） */
    self = (SerialPort_t *)calloc(1U, sizeof(SerialPort_t));
    if (self == NULL) {
        return NULL;
    }

    self->fd       = -1;                   /* -1 表示"还没打开" */
    self->baudrate = baudrate;
    strncpy(self->device, device, SERIAL_DEVICE_MAX_LEN - 1U);
    self->device[SERIAL_DEVICE_MAX_LEN - 1U] = '\0'; /* 确保字符串以 \0 结尾 */

    return self;
}

void serial_port_destroy(SerialPort_t *self)
{
    if (self == NULL) {
        return;
    }
    /* 如果还开着，先关闭 */
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

    /* O_RDWR = 可读可写, O_NOCTTY = 不让此串口成为控制终端（否则 Ctrl+C 会发给串口） */
    self->fd = open(self->device, O_RDWR | O_NOCTTY);
    if (self->fd < 0) {
        return ERR_SERIAL_OPEN;
    }

    /* 先读取当前 termios 配置，再修改（只改需要改的位） */
    ret = tcgetattr(self->fd, &tty);
    if (ret != 0) {
        close(self->fd);
        self->fd = -1;
        return ERR_SERIAL_OPEN;
    }

    /* ---- 设置波特率 ---- */
    cfsetospeed(&tty, s_baud_to_speed(self->baudrate));
    cfsetispeed(&tty, s_baud_to_speed(self->baudrate));

    /* ---- 8N1 模式：8 数据位、无校验、1 停止位 ---- */
    tty.c_cflag &= (tcflag_t)(~PARENB);  /* 关闭校验位 */
    tty.c_cflag &= (tcflag_t)(~CSTOPB);  /* 1 位停止位（默认），清除就是 1 位 */
    tty.c_cflag &= (tcflag_t)(~CSIZE);   /* 先清空数据位设置 */
    tty.c_cflag |= (tcflag_t)(CS8);      /* 设置为 8 数据位 */

    /* 禁用硬件流控（RTS/CTS）——我们的电机不需要流控 */
#ifdef CRTSCTS
    tty.c_cflag &= (tcflag_t)(~CRTSCTS);
#endif
    /* CREAD = 使能接收, CLOCAL = 忽略调制解调器控制线 */
    tty.c_cflag |= (tcflag_t)(CREAD | CLOCAL);

    /* ---- 原始模式（Raw Mode）---- */
    /* 关闭：规范化输入（ICANON）、回显（ECHO）、信号转义（ISIG）。
       如果不关闭，串口会做行编辑，按回车才提交数据，这对二进制协议是灾难。 */
    tty.c_lflag &= (tcflag_t)(~(ICANON | ECHO | ECHOE | ISIG));
    tty.c_iflag &= (tcflag_t)(~(IXON | IXOFF | IXANY));  /* 关闭软件流控 */
    tty.c_oflag &= (tcflag_t)(~OPOST);                    /* 关闭输出处理（不改 \n）*/

    /* ---- 读取超时设置 ---- */
    /* VMIN=0, VTIME=1：如果读不到数据，最多等 100ms 就返回。
       这样读操作不会永远阻塞，上层能及时处理超时。 */
    tty.c_cc[VMIN]  = 0U;
    tty.c_cc[VTIME] = 1U;

    /* 把修改写回内核，TCSANOW = 立即生效，不等数据传完 */
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
        return ERR_OK;  /* 已经关了，也算成功 */
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
    const uint8_t *ptr;
    size_t         remain;

    if (self == NULL || data == NULL) {
        return ERR_NULL_PTR;
    }
    if (self->fd < 0) {
        return ERR_NOT_INITIALIZED;
    }
    if (len == 0U) {
        return ERR_OK;
    }

    /* 循环直到全部写入（处理部分写入和 EINTR 信号中断的情况） */
    ptr    = data;
    remain = len;
    while (remain > 0U) {
        ssize_t written;
        written = write(self->fd, ptr, remain);
        if (written < 0) {
            if (errno == EINTR)
                continue;               /* 被信号中断，重试 */
            return ERR_SERIAL_WRITE;
        }
        if (written == 0) {
            return ERR_SERIAL_WRITE;    /* 不应该发生，但防止死循环 */
        }
        ptr    += (size_t)written;
        remain -= (size_t)written;
    }

    /* tcdrain：等待串口 TX 缓冲区全部发送到物理线路上。
       不加这一行的话，write() 返回只是数据进了内核缓冲区，
       实际可能还没从串口发出去。 */
    tcdrain(self->fd);
    return (int)len;
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

    /* 用 select() 实现超时读取：
       监控串口文件描述符的可读状态，在 timeout_ms 毫秒内等待数据到达。 */
    FD_ZERO(&fds);
    FD_SET(self->fd, &fds);

    /* 把毫秒超时转成 timeval 结构体 */
    tv.tv_sec  = (time_t)(timeout_ms / 1000U);
    tv.tv_usec = (suseconds_t)((timeout_ms % 1000U) * 1000U);

    ret = select(self->fd + 1, &fds, NULL, NULL, &tv);
    if (ret < 0) {
        return ERR_SERIAL_READ;          /* select 本身出错 */
    }
    if (ret == 0) {
        return ERR_SERIAL_TIMEOUT;       /* 超时了，没有数据可读 */
    }

    /* 有数据可读，读取最多 len 个字节 */
    n = read(self->fd, buf, len);
    if (n < 0) {
        return ERR_SERIAL_READ;
    }

    return (int)n;                       /* 返回实际读到的字节数 */
}

/* ============ 辅助函数 ============ */

int serial_port_flush(SerialPort_t *self)
{
    if (self == NULL) {
        return ERR_NULL_PTR;
    }
    if (self->fd < 0) {
        return ERR_NOT_INITIALIZED;
    }

    /* TCIOFLUSH = 同时清空输入和输出缓冲区 */
    return tcflush(self->fd, TCIOFLUSH) == 0 ? ERR_OK
                                             : ERR_SERIAL_READ;
}
