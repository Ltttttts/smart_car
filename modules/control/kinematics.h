/**
 * @file    kinematics.h
 * @brief   麦克纳姆轮运动学头文件 — 四轮全向底盘的数学建模
 * @author  Ltttttts
 *
 * 什么是运动学？
 *   - 逆运动学：已知"我想让机器人往哪个方向走多快"，算出"每个轮子该转多快"
 *   - 正运动学：已知"每个轮子实际转得多快"，算出"机器人实际在往哪走"
 *
 * 坐标系约定（从上方俯视小车）：
 *   +x 方向 = 车头前进方向
 *   +y 方向 = 向左平移
 *   +ω 正方向 = 逆时针旋转
 *
 * 车轮布局：
 *   前左(FL) ---- 前右(FR)
 *     |              |
 *   后左(RL) ---- 后右(RR)
 *
 * 注意：所有速度都是相对机器人自身坐标系（base_link）。
 * 如果机器人原地转了 90 度，它的 +x 方向也跟着转。
 */

#ifndef SMART_CAR_KINEMATICS_H
#define SMART_CAR_KINEMATICS_H

#include "common.h"

/* ========== 底盘物理参数 ========== */
typedef struct {
    double   wheel_radius;       /* 轮子半径，单位：米（m） */
    double   lx;                 /* 前后半轴距：从车中心到前/后轮的距离（m） */
    double   ly;                 /* 左右半轴距：从车中心到左/右轮的距离（m） */
    int32_t  encoder_ppr;        /* 编码器每转一圈产生的脉冲数（Pulse Per Revolution） */
} Kinematics_t;

/* ========== 速度表示 ========== */

/* 机器人速度：表达"我想让底盘怎么运动" */
typedef struct {
    double vx;                   /* 前进速度（m/s），正 = 向前 */
    double vy;                   /* 横向速度（m/s），正 = 向左 */
    double omega;                /* 角速度（rad/s），正 = 逆时针旋转 */
} RobotVelocity_t;

/* 车轮转速：4 个轮子的 RPM，顺序始终是 [FL, FR, RL, RR] */
typedef struct {
    double rpm[4];
} WheelVelocity_t;

/* ========== API 函数 ========== */

/**
 * @brief  初始化运动学参数（必须在使用前调用）
 * @param  kin           运动学实例指针
 * @param  wheel_radius  轮半径（m）
 * @param  lx            前后半轴距（m）
 * @param  ly            左右半轴距（m）
 * @param  encoder_ppr   编码器每转脉冲数
 * @note   如果传入的参数不合法（如半径 <= 0），会自动用默认值
 */
void kinematics_init(Kinematics_t *kin,
                     double wheel_radius,
                     double lx,
                     double ly,
                     int32_t encoder_ppr);

/**
 * @brief  逆运动学：把"机器人目标速度"换算成"每个轮子该转多快"
 *
 * 这是手柄遥控和 AI 控制最核心的转换函数。
 * 上层只需决定 vx/vy/omega，由这个函数算出四个轮子的 RPM。
 *
 * @param  kin     底盘参数（轮径、轴距等）
 * @param  robot   期望的机器人速度（vx, vy, omega）
 * @param  wheels  输出：四个轮子的目标 RPM [FL, FR, RL, RR]
 */
void kinematics_inverse(const Kinematics_t *kin,
                        const RobotVelocity_t *robot,
                        WheelVelocity_t *wheels);

/**
 * @brief  正运动学：把"轮子实际转速"反算出"机器人实际速度"
 *
 * 主要用于里程计推算：通过编码器读到的轮子转速，估算底盘的真实运动。
 * 比逆运动学算起来复杂一些，因为要解方程组。
 *
 * @param  kin     底盘参数
 * @param  wheels  测量的各轮 RPM [FL, FR, RL, RR]
 * @param  robot   输出：估算出的机器人速度
 */
void kinematics_forward(const Kinematics_t *kin,
                        const WheelVelocity_t *wheels,
                        RobotVelocity_t *robot);

/**
 * @brief  把编码器脉冲增量转换为车轮转动的弧度
 *
 * 用于从编码器数据计算里程计。
 * 比如编码器这次读到了 4000 个脉冲，除以 PPR 就知道轮子转了多少圈。
 *
 * @param  ppr        编码器每转脉冲数
 * @param  enc_delta  各轮的编码器脉冲增量 [FL, FR, RL, RR]
 * @param  rad_out    输出：各轮的转动弧度
 */
void kinematics_enc_to_rad(int32_t ppr,
                           const int32_t enc_delta[4],
                           double rad_out[4]);

#endif /* SMART_CAR_KINEMATICS_H */
