#ifndef __SERVO_H__
#define __SERVO_H__

#include "zf_common_headfile.h"

// 定义主板上舵机对应引脚
#define SERVO_MOTOR_PWM1            (TCPWM_CH09_P05_0)
#define SERVO_MOTOR_PWM2            (TCPWM_CH10_P05_1)
#define SERVO_MOTOR_PWM3            (TCPWM_CH11_P05_2)
#define SERVO_MOTOR_PWM4            (TCPWM_CH12_P05_3)
#define SERVO_MOTOR_FREQ            (50 )                                       // 定义主板上舵机频率  请务必注意范围 50-300

#define SERVO_MOTOR_L_MAX           (50 )                                       // 定义主板上舵机活动范围 角度
#define SERVO_MOTOR_R_MAX           (150)                                       // 定义主板上舵机活动范围 角度

// ------------------ 舵机占空比计算方式 ------------------
#define SERVO_MOTOR_DUTY(x)         ((float)PWM_DUTY_MAX/(1000.0/(float)SERVO_MOTOR_FREQ)*(0.5+(float)(x)/90.0))

#if (SERVO_MOTOR_FREQ<50 || SERVO_MOTOR_FREQ>300)
    #error "SERVO_MOTOR_FREQ ERROR!"
#endif

extern float servo_motor_duty_1;
extern float servo_motor_duty_2;
extern float servo_motor_duty_3;
extern float servo_motor_duty_4;

// 初始化四个舵机PWM，并设置初始值
void servo_init(float duty1, float duty2, float duty3, float duty4);

// 更新四个舵机占空比
void servo_set_duty(float duty1, float duty2, float duty3, float duty4);

#endif
