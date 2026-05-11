/**
 * @file    dashboard.h
 * @brief   终端仪表盘：里程计算 + 2×2 电机状态 + 底盘状态显示
 * @author  Ltttttts
 */

#ifndef SMART_CAR_DASHBOARD_H
#define SMART_CAR_DASHBOARD_H

#include "common.h"

typedef struct {
    /* 底盘里程（积分累计） */
    double pos_x;           /* X 方向位移 (m)     */
    double pos_y;           /* Y 方向位移 (m)     */
    double pos_yaw;         /* 累计转角 (rad)     */

    /* 各轮里程 */
    double wheel_dist[4];   /* FL, FR, RL, RR (m) */

    /* 底盘参数 */
    double wheel_radius;    /* 轮半径 (m)         */

    /* 显示 */
    char   title[32];       /* 副标题             */

    /* 时间跟踪 */
    double last_time;       /* 上次更新时间戳 (s)  */
    int    frame_printed;   /* 是否已打印过框架    */
} Dashboard_t;

/* ---- 电机索引 ---- */
#define DASH_FL  (0)
#define DASH_FR  (1)
#define DASH_RL  (2)
#define DASH_RR  (3)

/**
 * @brief  初始化仪表盘
 * @param  dash          仪表盘实例
 * @param  wheel_radius  轮半径 (m)
 * @param  title         副标题，如"键盘遥控"
 */
void dash_init(Dashboard_t *dash, double wheel_radius,
               const char *title);

/**
 * @brief  重置里程计为 0
 * @param  dash  仪表盘实例
 */
void dash_reset_odom(Dashboard_t *dash);

/**
 * @brief  更新里程并刷新显示
 *
 * 应在每次电机速度指令更新后调用。
 * dt 为距离上次更新的时间差 (s)，内部自动计时时传 0。
 *
 * @param  dash      仪表盘实例
 * @param  vx        底盘前进速度 (m/s)
 * @param  vy        底盘横向速度 (m/s)
 * @param  omega     底盘角速度 (rad/s)
 * @param  rpm       各轮目标转速 [FL, FR, RL, RR]
 * @param  dt        时间步长 (s)，传 0 则内部计时
 * @param  mode_str  模式字符串，如"模拟"或"硬件"
 * @param  cmd_hint  命令提示字符串，显示在最下方
 */
void dash_update(Dashboard_t *dash,
                 double vx, double vy, double omega,
                 const double rpm[4],
                 double dt,
                 const char *mode_str,
                 const char *cmd_hint);

/**
 * @brief  清理仪表盘（恢复光标显示）
 */
void dash_cleanup(void);

#endif /* SMART_CAR_DASHBOARD_H */
