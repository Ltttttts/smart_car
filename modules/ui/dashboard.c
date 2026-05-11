/**
 * @file    dashboard.c
 * @brief   终端仪表盘：里程积分 + ASCII 全帧刷新
 * @author  Ltttttts
 */
#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 199309L
#endif

#include "ui/dashboard.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <time.h>
#include <math.h>

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

#define DASH_W (55)

static const char *s_label[4] = { "FL", "FR", "RL", "RR" };

static double s_now_sec(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1.0e9;
}

void dash_init(Dashboard_t *dash, double wheel_radius,
               const char *title)
{
    memset(dash, 0, sizeof(*dash));
    dash->wheel_radius = (wheel_radius > 0.0) ? wheel_radius : 0.05;
    dash->last_time    = s_now_sec();
    snprintf(dash->title, sizeof(dash->title), "%s", title ?: "");
}

void dash_reset_odom(Dashboard_t *dash)
{
    for (int i = 0; i < 4; i++) dash->wheel_dist[i] = 0;
    dash->pos_x = dash->pos_y = dash->pos_yaw = 0;
}

/* ---- 输出一行：vsnprintf 后再用空格填满到 DASH_W ---- */
static void put_line(const char *fmt, ...)
{
    char buf[DASH_W + 1];
    int  len;

    va_list ap;
    va_start(ap, fmt);
    len = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (len < 0) len = 0;
    if (len > DASH_W) len = DASH_W;

    memset(buf + len, ' ', (size_t)(DASH_W - len));
    buf[DASH_W] = '\0';
    puts(buf);
}

void dash_update(Dashboard_t *dash,
                 double vx, double vy, double omega,
                 const double rpm[4],
                 double dt,
                 const char *mode_str,
                 const char *cmd_hint)
{
    double now = s_now_sec();
    double circ = 2.0 * M_PI * dash->wheel_radius;
    int i;

    /* 时间步长 */
    if (dt <= 0.0) dt = now - dash->last_time;
    dash->last_time = now;

    /* 里程积分 */
    if (dt > 0.0) {
        double c = cos(dash->pos_yaw), s = sin(dash->pos_yaw);
        dash->pos_x   += (vx * c - vy * s) * dt;
        dash->pos_y   += (vx * s + vy * c) * dt;
        dash->pos_yaw += omega * dt;
        for (i = 0; i < 4; i++)
            dash->wheel_dist[i] += fabs(rpm[i]) / 60 * circ * dt;
    }

    /* ==== 全帧刷新 ==== */
    printf("\033[2J\033[H\033[?25l");

    /* 边框顶 */
    put_line("+====================================================+");
    /* 标题 */
    put_line("|                  %-35s|", dash->title);
    /* 边框第二行 */
    put_line("|----------------------------------------------------|");
    /* 电机状态标题 */
    put_line("|                    电机状态                        |");
    /* 表头框 */
    put_line("|  +----------+-------------------+----------+       |");
    put_line("|  |  位置    |  目标转速(RPM)   |  里程(m)  |       |");
    /* 分割线 */
    put_line("|  +----------+-------------------+----------+       |");
    /* 4 行数据 */
    for (i = 0; i < 4; i++) {
        int r = (int)rpm[i];
        put_line("|  |  %-7s|     %+6d       |   %6.2f  |       |",
                 s_label[i], r, dash->wheel_dist[i]);
    }
    /* 表底框 */
    put_line("|  +----------+-------------------+----------+       |");
    /* 空行 */
    put_line("|                                                    |");
    /* 速度 */
    put_line("|  底盘速度: vx=%+7.2f  vy=%+7.2f  w=%+7.2f         |",
             vx, vy, omega);
    /* 里程 */
    put_line("|  底盘里程:  x=%+7.2f   y=%+7.2f  yaw=%+7.2f       |",
             dash->pos_x, dash->pos_y, dash->pos_yaw);
    /* 空行 */
    put_line("|                                                    |");
    /* 操作提示 */
    put_line("|  %-49s  |", cmd_hint ?: "");
    /* 速度提示 */
    put_line("|  速度: 线速 %.2f m/s  角速 %.2f rad/s             |",
             hypot(vx, vy), fabs(omega));
    /* 空行 */
    put_line("|                                                    |");
    /* 模式 */
    put_line("|  运行模式: %-12s|  按 X 退出                       |",
             mode_str ?: "?");
    /* 底边 */
    put_line("+====================================================+");

    fflush(stdout);
}

void dash_cleanup(void)
{
    printf("\033[?25h\n\n");
    fflush(stdout);
}
