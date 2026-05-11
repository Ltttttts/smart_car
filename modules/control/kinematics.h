/**
 * @file    kinematics.h
 * @brief   麦克纳姆轮运动学（四轮全向底盘）
 * @author  Ltttttts
 *
 * 坐标系约定（俯视图）：
 *   +x = 前进方向   +y = 向左   +ω = 逆时针
 *
 * 车轮布局：
 *   前左(FL addr1) ---- 前右(FR addr2)
 *        |                  |
 *   后左(RL addr3) ---- 后右(RR addr4)
 *
 * 运动量均以 base_link 坐标系为参考。
 */

#ifndef SMART_CAR_KINEMATICS_H
#define SMART_CAR_KINEMATICS_H

#include "common.h"

/* ---- 底盘参数 ---- */
typedef struct {
    double   wheel_radius;       /* 轮半径 (m)           */
    double   lx;                 /* 前后半轴距 (m)        */
    double   ly;                 /* 左右半轴距 (m)        */
    int32_t  encoder_ppr;        /* 编码器每转脉冲数       */
} Kinematics_t;

/* ---- 速度表示 ---- */

/* 机器人速度（底盘坐标系） */
typedef struct {
    double vx;                   /* 前进速度 (m/s)       */
    double vy;                   /* 横向速度 (m/s)       */
    double omega;                /* 角速度 (rad/s)       */
} RobotVelocity_t;

/* 车轮转速，顺序 [FL, FR, RL, RR]，单位 RPM */
typedef struct {
    double rpm[4];
} WheelVelocity_t;

/* ---- API ---- */

/**
 * @brief  初始化运动学参数
 * @param  kin           运动学实例
 * @param  wheel_radius  轮半径 (m)
 * @param  lx            前后半轴距 (m)
 * @param  ly            左右半轴距 (m)
 * @param  encoder_ppr   编码器每转脉冲数
 */
void kinematics_init(Kinematics_t *kin,
                     double wheel_radius,
                     double lx,
                     double ly,
                     int32_t encoder_ppr);

/**
 * @brief  逆运动学：机器人速度 → 各轮转速 (RPM)
 * @param  kin     底盘参数
 * @param  robot   期望的机器人速度
 * @param  wheels  输出：各轮 RPM [FL, FR, RL, RR]
 */
void kinematics_inverse(const Kinematics_t *kin,
                        const RobotVelocity_t *robot,
                        WheelVelocity_t *wheels);

/**
 * @brief  正运动学：各轮转速 (RPM) → 机器人速度
 * @param  kin     底盘参数
 * @param  wheels  测量的各轮 RPM [FL, FR, RL, RR]
 * @param  robot   输出：估算的机器人速度
 */
void kinematics_forward(const Kinematics_t *kin,
                        const WheelVelocity_t *wheels,
                        RobotVelocity_t *robot);

/**
 * @brief  编码器增量 → 各轮转动角度 (rad)
 * @param  ppr        编码器每转脉冲数
 * @param  enc_delta  各轮编码器增量 [FL, FR, RL, RR]
 * @param  rad_out    输出：各轮转动弧度
 */
void kinematics_enc_to_rad(int32_t ppr,
                           const int32_t enc_delta[4],
                           double rad_out[4]);

#endif /* SMART_CAR_KINEMATICS_H */
