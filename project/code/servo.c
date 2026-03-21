#include "servo.h"

// 假设车头方向电机驱动板方向，1号舵机为车头左，2号为车头右，3号为车尾左，4号为车尾右
// 舵机以贴纸方向为下的话，顺时针转动角度减小
float servo_motor_duty_1 = 65.0;
float servo_motor_duty_2 = 50.0;
float servo_motor_duty_3 = 40.0;
float servo_motor_duty_4 = 30.0;
//这个占空比是目前工作角度的占空比
void servo_set_duty(float duty1, float duty2, float duty3, float duty4)
{
    servo_motor_duty_1 = duty1;
    servo_motor_duty_2 = duty2;
    servo_motor_duty_3 = duty3;
    servo_motor_duty_4 = duty4;

    pwm_set_duty(SERVO_MOTOR_PWM1, (uint16)SERVO_MOTOR_DUTY(servo_motor_duty_1));
    pwm_set_duty(SERVO_MOTOR_PWM2, (uint16)SERVO_MOTOR_DUTY(servo_motor_duty_2));
    pwm_set_duty(SERVO_MOTOR_PWM3, (uint16)SERVO_MOTOR_DUTY(servo_motor_duty_3));
    pwm_set_duty(SERVO_MOTOR_PWM4, (uint16)SERVO_MOTOR_DUTY(servo_motor_duty_4));
}

void servo_init(float duty1, float duty2, float duty3, float duty4)
{
    pwm_init(SERVO_MOTOR_PWM1, SERVO_MOTOR_FREQ, 0);
    pwm_init(SERVO_MOTOR_PWM2, SERVO_MOTOR_FREQ, 0);
    pwm_init(SERVO_MOTOR_PWM3, SERVO_MOTOR_FREQ, 0);
    pwm_init(SERVO_MOTOR_PWM4, SERVO_MOTOR_FREQ, 0);

    servo_set_duty(duty1, duty2, duty3, duty4);
}
