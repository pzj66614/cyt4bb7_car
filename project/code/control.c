/* control.c
 * 一级倒立摆/直立平衡车串级PID控制算法实现
 * 1. 直立环 (角度PD控制): 根据当前倾角和角速度计算需要输出的恢复力(PWM)
 * 2. 速度环 (速度PI控制): 根据电机速度抑制系统的整体位移偏差
 */

#include <stdint.h>
#include <math.h>

// ----- 直立环(角度环)PID参数 -----
// 经过调整，直立环现在输出期望轮子转速 (RPM)
// 原为rad/s参数, 现乘以 9.549296 使其匹配 RPM 个量级输出
float Upright_Kp = -70.0f; // 滑动是因为软了兜不住，适当加大Kp (从-21 加回 -26)
float Upright_Kd = -120.0f; // Kd保持刚降下来的参数不产生抽搐
float Target_Angle = -5.5f; // 机械零度(需根据实际重力校准后调整)

// ----- 速度环PID参数 -----
// 速度环为PI控制，输出用于微调期望角度（即产生俯仰角以加减速）
// 原因为输入误差变成了RPM(放大9.549倍), 而输出仍需要是相同的补偿角度, 故参数需要除以 9.549296 (即乘以 0.10472)
float Velocity_Kp = 0.075f;   // 从0.08往回调一点，防止速度环太猛引起整体低频震荡
float Velocity_Ki = 0.00035f; // 积分类同
float Target_Speed = 60.0f; // 期望速度(RPM级别)，静止为0

// ----- 转向环(Yaw角)PD参数 -----
// 转向环用于控制小车走直线或实现指定角度转向
// 原为rad/s参数, 现乘以 9.549296 使其匹配 RPM 差速输出
float Turn_Kp = 1.0f;  // 0.1 * 9.549
float Turn_Kd = 0.1f;  // 0.001 * 9.549
float Target_Yaw_Angle = 0.0f; // 目标Yaw角度，0度表示走绝对直线

// 状态机：区分角度锁定控制和角速度控制
int Use_Angle_Control = 1;     // 1: 角度控制(限定角度走直), 0: 角速度控制(转圈或角速度走直线)
float Target_Turn_Rate = 0.0f; // 当前的平滑目标角速度
float Desired_Turn_Rate = 10.0f; // 最终期望达到的目标角速度
float Turn_Rate_Step = 5.0f;   // 每次控制周期角速度最大变化步长 (根据控制频率调节)
float Turn_Rate_Kp = 2.5f;     // 角速度控制比例系数

// 自动转圈退出相关变量
int Spin_Auto_Stop = 0;           // 是否开启转完自动停
float Target_Spin_Angle = 0.0f;   // 期望转动的总角度 (度)
float Accumulated_Spin_Angle = 0.0f; // 当前已转动的积累角度 (度)
float Last_Yaw_Angle = 0.0f;      // 前一次的Yaw角，用于计算增量
int Is_Stopping = 0;              // 是否正在刹车标志

// 积分分离角度阈值（度）：超过此角度停止速度环积分
#define INTEGRAL_SEPARATION_ANGLE 20.0f

// 转速输出限幅 (例如最大1000 RPM，根据电机实际能力修改)
#define TARGET_SPEED_MAX 1000 
#define TARGET_SPEED_MIN -1000

// 转向差速限幅 (如最大产生±300 RPM的差速)
#define TURN_SPEED_MAX 300
#define TURN_SPEED_MIN -300

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
    
    // 速度偏差滤波 (对高频噪声做缓解)
    speed_err = speed_err * 0.7f + velocity_err_last * 0.3f;
    velocity_err_last = speed_err;
    
    // *** 关键修复：当误差很小接近 0 时，清除积分，或者设置死区，防止残余积分死顶 ***
    // 如果小车本来应该停下，却怎么也停不彻底，往往是因为 speed_integral 积累了一些垃圾值退不回来
    // 这里加一个抗积分饱和与积分清零的智能死区
    if (fabsf(speed_err) < 2.0f && fabsf(Target_Speed) < 0.1f) {
        speed_integral *= 0.8f; // 如果目标是静止，且当前速度已经很慢了，让积分快速衰减
    } else {
        // 积分分离 (防大角度失控)
        if (fabsf(current_angle) < INTEGRAL_SEPARATION_ANGLE) {
            speed_integral += speed_err;
        } else {
            speed_integral = 0.0f;
        }
    }
    
    // 原来的积分限幅对于 RPM 来说 5000 太容易满了，扩大抗积分饱和的容量
    if(speed_integral > 50000.0f) speed_integral = 50000.0f;
    if(speed_integral < -50000.0f) speed_integral = -50000.0f;
    
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
    
    // 2. 将角度补偿传递给直立环，直接输出目标转速(用于产生恢复力/角速度)
    float target_motor_speed_f = Control_Upright_PD(current_angle, current_gyro, angle_compensation);
    
    int target_motor_speed = (int)target_motor_speed_f;
    
    // 3. 总死区与转速限幅
    if (target_motor_speed > TARGET_SPEED_MAX) target_motor_speed = TARGET_SPEED_MAX;
    if (target_motor_speed < TARGET_SPEED_MIN) target_motor_speed = TARGET_SPEED_MIN;
    
    return target_motor_speed;
}

/**
 * @brief 原地转圈操作函数
 * @param enable     是否开启角速度转圈 (1:转圈, 0:关闭转圈并锁死当前角度)
 * @param turn_rate  期望的转圈角速度 (正数/负数决定方向)
 * @param current_yaw 当前角度 (关闭转圈时用于锁死新方向，防止回弹)
 */
void Control_Set_Spin(int enable, float turn_rate, float current_yaw) {
    if (enable) {
        Use_Angle_Control = 0;
        Desired_Turn_Rate = turn_rate; // 设置为期望角速度，让其平滑过渡
        Target_Speed = 0.0f; // 清零前进速度，保证原地打转
        
        // 开启以 720 度(两圈)为目标的自动累计
        Spin_Auto_Stop = 1;
        Target_Spin_Angle = 720.0f; // 定死两圈
        Accumulated_Spin_Angle = 0.0f;
        Last_Yaw_Angle = current_yaw;
        Is_Stopping = 0;
    } else {
        Use_Angle_Control = 1;
        Desired_Turn_Rate = 0.0f;
        Target_Turn_Rate = 0.0f; // 瞬间清零
        Target_Yaw_Angle = current_yaw; // 更新为当前角度
        Spin_Auto_Stop = 0;
        Is_Stopping = 0;
    }
}

/**
 * @brief 转向环控制 (支持角度锁定与角速度控制)
 * @param current_yaw 当前Yaw角度
 * @param yaw_gyro    当前Yaw轴角速度
 * @return 转向环计算得到的差速目标转速 (加在左轮、减在右轮)
 */
int Control_Get_Turn_Speed(float current_yaw, float yaw_gyro) {
    float turn_speed_f = 0.0f;
    
    if (Use_Angle_Control) {
        // --- 模式：角度控制 (走绝对直线或限定角度) ---
    float yaw_err = current_yaw - Target_Yaw_Angle;
        turn_speed_f = (Turn_Kp * yaw_err) + (Turn_Kd * yaw_gyro);
    } else {
        // --- 模式：角速度控制 (原地转圈或角速度走直线) ---
        
        // 1. 自动计算转圈累计角度 (处理超限制后引发刹车)
        if (Spin_Auto_Stop) {
            float delta_yaw = current_yaw - Last_Yaw_Angle;
            // 处理偏航角跨越 ±180 或 0~360 导致的差值跳变
            if (delta_yaw > 180.0f) delta_yaw -= 360.0f;
            else if (delta_yaw < -180.0f) delta_yaw += 360.0f;
            
            Accumulated_Spin_Angle += fabsf(delta_yaw);
            Last_Yaw_Angle = current_yaw;
            
            // 如果累计转角超过了目标(如720度)，则开始平滑刹车
            if (Accumulated_Spin_Angle >= Target_Spin_Angle && !Is_Stopping) {
                Desired_Turn_Rate = 0.0f; // 触发刹车，目标角速度置零
                Is_Stopping = 1;
            }
        }

        // 2. 角速度目标值渐变平滑过渡 (斜坡函数缓启动/缓停止)
        if (Target_Turn_Rate < Desired_Turn_Rate) {
            Target_Turn_Rate += Turn_Rate_Step;
            if (Target_Turn_Rate > Desired_Turn_Rate) Target_Turn_Rate = Desired_Turn_Rate;
        } else if (Target_Turn_Rate > Desired_Turn_Rate) {
            Target_Turn_Rate -= Turn_Rate_Step;
            if (Target_Turn_Rate < Desired_Turn_Rate) Target_Turn_Rate = Desired_Turn_Rate;
        }
        
        // 3. 缓降结束后，自动切换回限定角度模式锁死
        if (Is_Stopping && fabsf(Target_Turn_Rate) < 0.1f && fabsf(Desired_Turn_Rate) < 0.1f) {
            Use_Angle_Control = 1;
            Spin_Auto_Stop = 0;
            Is_Stopping = 0;
            Target_Yaw_Angle = current_yaw; // 将最后停下时的角度锁死，防止回弹
        }

        // 4. 最终误差进入比例控制器
        float rate_err = Target_Turn_Rate - yaw_gyro;
        turn_speed_f = Turn_Rate_Kp * rate_err;
        
        // 动态更新 Target_Yaw_Angle，即使中途意外强行切回角度控制，也不会疯转回弹
        if (!Is_Stopping && !Use_Angle_Control) {
            Target_Yaw_Angle = current_yaw; 
        }
    }
    
    int turn_speed = (int)turn_speed_f;
    
    // 转向差速限幅，防止打滑或倾覆
    if (turn_speed > TURN_SPEED_MAX) turn_speed = TURN_SPEED_MAX;
    if (turn_speed < TURN_SPEED_MIN) turn_speed = TURN_SPEED_MIN;
    
    return turn_speed;
}
