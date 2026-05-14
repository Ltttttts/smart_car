/**
 * @file    kinematics.c
 * @brief   麦克纳姆轮运动学实现
 * @author  Ltttttts
 *
 * 麦克纳姆轮的原理：
 *   普通轮子只能前后滚，麦克纳姆轮的外圈可以"被动侧滑"。
 *   通过四个轮子的不同转速组合，可以实现：
 *     - 前进/后退
 *     - 左右平移
 *     - 原地旋转
 *     - 斜向移动
 *   这就是"全向移动"（Omni-directional motion）。
 *
 * 逆运动学公式（机器人速度 → 各轮 RPM）：
 *   ω_fl = (1/R) * (vx - vy - (lx+ly)*ω)
 *   ω_fr = (1/R) * (vx + vy + (lx+ly)*ω)
 *   ω_rl = (1/R) * (vx + vy - (lx+ly)*ω)
 *   ω_rr = (1/R) * (vx - vy + (lx+ly)*ω)
 *
 *   其中：R = 轮半径，ω_xx = 车轮角速度 (rad/s)
 *         lx = 前后半轴距，ly = 左右半轴距
 *         vx/vy/omega = 机器人的目标速度
 *
 * RPM 与 rad/s 换算：
 *   RPM = (30 / π) × rad/s    （1 rad/s ≈ 9.55 RPM）
 *   rad/s = (π / 30) × RPM    （1 RPM ≈ 0.105 rad/s）
 */

#include "control/kinematics.h"
#include <math.h>

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

/* 角速度单位换算常量 */
#define RAD_PER_SEC_TO_RPM (30.0 / M_PI)  /* 弧度/秒 → 转/分钟 */
#define RPM_TO_RAD_PER_SEC (M_PI / 30.0)  /* 转/分钟 → 弧度/秒 */

void kinematics_init(Kinematics_t *kin,
                     double wheel_radius,
                     double lx,
                     double ly,
                     int32_t encoder_ppr)
{
    if (kin == NULL) {
        return;
    }

    /* 安全兜底：如果传入的参数不合理（<=0），使用默认值防止除零错误 */
    kin->wheel_radius = (wheel_radius > 0.0) ? wheel_radius : 0.05;
    kin->lx           = (lx > 0.0) ? lx : 0.1;
    kin->ly           = (ly > 0.0) ? ly : 0.1;
    kin->encoder_ppr  = (encoder_ppr > 0) ? encoder_ppr : 1000;
}

void kinematics_inverse(const Kinematics_t *kin,
                        const RobotVelocity_t *robot,
                        WheelVelocity_t *wheels)
{
    double inv_r;   /* 1 / R（轮半径的倒数，避免重复计算除法） */
    double l_sum;   /* lx + ly（半轴距之和） */

    if (kin == NULL || robot == NULL || wheels == NULL) {
        return;
    }

    inv_r  = 1.0 / kin->wheel_radius;
    l_sum  = kin->lx + kin->ly;

    /* 麦克纳姆轮逆运动学核心公式：
       每个轮子的角速度 = (1/R) × (vx ± vy ± (lx+ly)*omega)
       再乘以 30/π 转换为 RPM */

    /* FL（前左轮）：vx - vy - (lx+ly)*omega */
    wheels->rpm[0] = inv_r * (robot->vx - robot->vy
                              - l_sum * robot->omega)
                     * RAD_PER_SEC_TO_RPM;
    /* FR（前右轮）：vx + vy + (lx+ly)*omega */
    wheels->rpm[1] = inv_r * (robot->vx + robot->vy
                              + l_sum * robot->omega)
                     * RAD_PER_SEC_TO_RPM;
    /* RL（后左轮）：vx + vy - (lx+ly)*omega */
    wheels->rpm[2] = inv_r * (robot->vx + robot->vy
                              - l_sum * robot->omega)
                     * RAD_PER_SEC_TO_RPM;
    /* RR（后右轮）：vx - vy + (lx+ly)*omega */
    wheels->rpm[3] = inv_r * (robot->vx - robot->vy
                              + l_sum * robot->omega)
                     * RAD_PER_SEC_TO_RPM;
}

void kinematics_forward(const Kinematics_t *kin,
                        const WheelVelocity_t *wheels,
                        RobotVelocity_t *robot)
{
    double w[4];    /* 各轮角速度（rad/s） */
    double r;       /* 轮半径 */
    double inv_4l;  /* 1 / (4 * (lx+ly)) */

    if (kin == NULL || wheels == NULL || robot == NULL) {
        return;
    }

    /* 先把 RPM 转为 rad/s */
    w[0] = wheels->rpm[0] * RPM_TO_RAD_PER_SEC;
    w[1] = wheels->rpm[1] * RPM_TO_RAD_PER_SEC;
    w[2] = wheels->rpm[2] * RPM_TO_RAD_PER_SEC;
    w[3] = wheels->rpm[3] * RPM_TO_RAD_PER_SEC;

    r      = kin->wheel_radius;
    inv_4l = 1.0 / (4.0 * (kin->lx + kin->ly));

    /* 正运动学：从四个轮子速度反推底盘速度
       原理是逆运动学方程组的解（最小二乘解） */
    robot->vx    = (r / 4.0) * (w[0] + w[1] + w[2] + w[3]);
    robot->vy    = (r / 4.0) * (-w[0] + w[1] + w[2] - w[3]);
    robot->omega = r * inv_4l * (-w[0] + w[1] - w[2] + w[3]);
}

void kinematics_enc_to_rad(int32_t ppr,
                           const int32_t enc_delta[4],
                           double rad_out[4])
{
    double scale;

    if (enc_delta == NULL || rad_out == NULL || ppr == 0) {
        return;
    }

    /* 每个脉冲对应的弧度 = 2*PI / PPR */
    scale = (2.0 * M_PI) / (double)ppr;

    rad_out[0] = (double)enc_delta[0] * scale;
    rad_out[1] = (double)enc_delta[1] * scale;
    rad_out[2] = (double)enc_delta[2] * scale;
    rad_out[3] = (double)enc_delta[3] * scale;
}
