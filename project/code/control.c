/* control.c
 * 一级倒立摆/直立平衡车串级PID控制算法实现
 * 1. 直立环 (角度PD控制): 根据当前倾角和角速度计算需要输出的恢复力(PWM)
 * 2. 速度环 (速度PI控制): 根据电机速度抑制系统的整体位移偏差
 */

#include <stdint.h>
#include <math.h>

// ----- 直立环(角度环)PID参数 -----
// 经过调整，直立环现在输出期望轮子转速
float Upright_Kp = 0.1f; // 比例常数需重新整定以适应速度输出
float Upright_Kd =0.05f;  // 微分常数需重新整定以适应速度输出
float Target_Angle = -1.2f; // 机械零度(需根据实际重力校准后调整)

// ----- 速度环PID参数 -----
// 速度环为PI控制，输出用于微调期望角度（即产生俯仰角以加减速）
float Velocity_Kp = 0.08f;   // 比例常数需重新整定
float Velocity_Ki = 0.001f;  // 积分常数需重新整定
float Target_Speed = 0.0f; // 期望速度通常为0(静止不动)

// ----- 转向环(Yaw角)PD参数 -----
// 转向环用于控制小车走直线或实现指定角度转向
float Turn_Kp = 5.0f;      // 转向比例常数，根据响应速度整定
float Turn_Kd = 0.5f;      // 转向微分常数，用于抑制振荡
float Target_Yaw_Angle = 0.0f; // 目标Yaw角度，0度表示走绝对直线

// 积分分离角度阈值（度）：超过此角度停止速度环积分
#define INTEGRAL_SEPARATION_ANGLE 20.0f

// 转速输出限幅 (例如最大1500 RPM，根据电机实际能力修改)
#define TARGET_SPEED_MAX 1500 
#define TARGET_SPEED_MIN -1500

// 转向差速限幅 (如最大产生±500 RPM的差速)
#define TURN_SPEED_MAX 500
#define TURN_SPEED_MIN -500

/**
 * @brief 速度环PI控制 (计算角度偏差补偿)
 * @param current_speed 当前电机转速
 * @param current_angle 当前姿态角
 * @return 速度环计算得到的期望角度补偿值
 */
float Control_Velocity_PI(float current_speed, float current_angle) {
    static float velocity_err_last = 0.0f;
    static float speed_integral = 0.0f;
    
    // 计算速度偏差
    float speed_err = Target_Speed - current_speed;
    
    // 速度偏差滤波
    speed_err = speed_err * 0.7f + velocity_err_last * 0.3f;
    velocity_err_last = speed_err;
    
    // 积分分离
    if (fabsf(current_angle) < INTEGRAL_SEPARATION_ANGLE) {
        speed_integral += speed_err;
    } else {
        speed_integral = 0.0f;
    }
    
    // 积分限幅 (防止角度补偿过大，如最大不超过5度)
    if(speed_integral > 5000.0f) speed_integral = 5000.0f;
    if(speed_integral < -5000.0f) speed_integral = -5000.0f;
    
    // 速度环输出：补偿到目标直立角度上的值
    float angle_compensation = (Velocity_Kp * speed_err) + (Velocity_Ki * speed_integral);
    
    // 输出限幅 (补偿倾角范围)
    if(angle_compensation > 10.0f) angle_compensation = 10.0f;
    if(angle_compensation < -10.0f) angle_compensation = -10.0f;
    
    return angle_compensation;
}

/**
 * @brief 直立环PD控制 (计算目标转速)
 * @param current_angle 当前姿态角(如Pitch或Roll，由姿态解算/卡尔曼滤波得出)
 * @param current_gyro  当前绕该轴的角速度(由陀螺仪直接得出)
 * @param angle_compensation 速度环输出的角度补偿
 * @return 直立环计算得到的目标转速
 */
float Control_Upright_PD(float current_angle, float current_gyro, float angle_compensation) {
    // 结合速度环的补偿调整总体目标角度
    float target_angle_adj = Target_Angle + angle_compensation;
    float angle_err = current_angle - target_angle_adj;
    
    // 直立PD计算公式： 转速 = Kp * 角度偏差 + Kd * 角速度
    float target_speed = (Upright_Kp * angle_err) + (Upright_Kd * current_gyro);
    
    // 直立环输出转速限幅
    if (target_speed > TARGET_SPEED_MAX) target_speed = TARGET_SPEED_MAX;
    if (target_speed < TARGET_SPEED_MIN) target_speed = TARGET_SPEED_MIN;
    
    return target_speed;
}

/**
 * @brief 串级总控制函数 (需要在定时器中断中周期性调用，例如每5ms一次)
 * @param current_angle 倾角
 * @param current_gyro  角速度
 * @param current_speed 电机编码器实际速度 (rad/s)
 * @return 最终需作用到电机的目标转速 (rad/s)
 */
int Control_Get_Total_Speed(float current_angle, float current_gyro, float current_speed) {
    // 1. 获取速度环输出 (目标角度微调基准)
    float angle_compensation = Control_Velocity_PI(current_speed, current_angle);
    
    // 2. 将角度补偿传递给直立环，直接输出目标转速
    float target_motor_speed_f = Control_Upright_PD(current_angle, current_gyro, angle_compensation);
    
    int target_motor_speed = (int)target_motor_speed_f;
    
    // 3. 总死区与转速限幅
    if (target_motor_speed > TARGET_SPEED_MAX) target_motor_speed = TARGET_SPEED_MAX;
    if (target_motor_speed < TARGET_SPEED_MIN) target_motor_speed = TARGET_SPEED_MIN;
    
    return target_motor_speed;
}

/**
 * @brief 转向环PD控制 (计算两轮差速)
 * @param current_yaw 当前Yaw角度(由姿态解算影响走直线的倾斜或偏航)
 * @param yaw_gyro    当前Yaw轴角速度
 * @return 转向环计算得到的差速目标转速 (加在左轮、减在右轮)
 */
int Control_Get_Turn_Speed(float current_yaw, float yaw_gyro) {
    float yaw_err = current_yaw - Target_Yaw_Angle;
    
    // 转向PD计算公式： 转速差 = Kp * 误差 + Kd * 角速度微调
    float turn_speed_f = (Turn_Kp * yaw_err) + (Turn_Kd * yaw_gyro);
    
    int turn_speed = (int)turn_speed_f;
    
    // 转向差速限幅，防止打滑或倾覆
    if (turn_speed > TURN_SPEED_MAX) turn_speed = TURN_SPEED_MAX;
    if (turn_speed < TURN_SPEED_MIN) turn_speed = TURN_SPEED_MIN;
    
    return turn_speed;
}
