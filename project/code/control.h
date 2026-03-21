#ifndef __CONTROL_H__
#define __CONTROL_H__

#include <stdint.h>

// ----- 直立环(角度环)PID参数 -----
extern float Upright_Kp;        // P参数起主导恢复作用
extern float Upright_Kd;        // D参数起阻尼作用
extern float Target_Angle;      // 机械零度

// ----- 速度环PID参数 -----
extern float Velocity_Kp;       // 速度环比例系数
extern float Velocity_Ki;       // 速度环积分系数
extern float Target_Speed;      // 期望速度

// ----- 转向环(Yaw角)PD参数 -----
extern float Turn_Kp;           // 转向比例常数
extern float Turn_Kd;           // 转向微分常数
extern float Target_Yaw_Angle;  // 目标走直线的Yaw角度基准

// PID及电机PWM限幅
#define PWM_MAX  10000
#define PWM_MIN -10000

// 积分分离角度阈值（度）：超过此角度停止速度环积分
#define INTEGRAL_SEPARATION_ANGLE 20.0f

/**
 * @brief 直立环PD控制
 * @param current_angle 当前姿态角(如Pitch或Roll，由姿态解算/卡尔曼滤波得出)
 * @param current_gyro  当前绕该轴的角速度(由陀螺仪直接得出)
 * @return 直立环计算得到的PWM值
 */
float Control_Upright_PD(float current_angle, float current_gyro);

/**
 * @brief 速度环PI控制
 * @param current_speed 当前电机转速(可通过编码器测算)
 * @param current_angle 当前姿态角，用于积分分离（车倒地时不积分）
 * @return 速度环计算得到的PWM值
 */
float Control_Velocity_PI(float current_speed, float current_angle);

/**
 * @brief 串级总控制函数 (需要在定时器中断中周期性调用，例如每5ms一次)
 * @param current_angle 倾角
 * @param current_gyro  角速度
 * @param current_speed 电机编码器速度
 * @return 最终需作用到电机的占空比 (-10000 ~ +10000)
 */
int Control_Get_Total_Speed(float current_angle, float current_gyro, float current_speed);

/**
 * @brief 转向环PD控制 (计算两轮差速)
 * @param current_yaw 当前Yaw角度
 * @param yaw_gyro    当前Yaw轴角速度
 * @return 转向环计算得到的差速目标转速
 */
int Control_Get_Turn_Speed(float current_yaw, float yaw_gyro);

#endif // __CONTROL_H__
