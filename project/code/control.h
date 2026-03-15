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

extern int Use_Angle_Control;   // 状态机: 1: 角度控制(走绝对直线), 0: 角速度控制(转圈)
extern float Target_Turn_Rate;  // 当前的平滑目标角速度
extern float Desired_Turn_Rate; // 期望的最终目标角速度
extern float Turn_Rate_Step;    // 角速度渐变步长
extern float Turn_Rate_Kp;      // 角速度控制比例系数

// 自动转圈退出相关变量 (支持状态查询)
extern int Spin_Auto_Stop;           // 是否开启转完自动停
extern float Target_Spin_Angle;      // 期望转动的总角度 (度)
extern float Accumulated_Spin_Angle; // 当前已转动的积累角度 (度)
extern int Is_Stopping;              // 是否正在刹车标志

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

/**
 * @brief 原地转圈操作函数
 * @param enable     是否开启角速度转圈 (1:转圈, 0:关闭转圈并锁死当前角度)
 * @param turn_rate  期望的转圈角速度 (正数/负数决定方向)
 * @param current_yaw 当前角度 (关闭转圈时用于锁死新方向，防止回弹)
 */
void Control_Set_Spin(int enable, float turn_rate, float current_yaw);

#endif // __CONTROL_H__
