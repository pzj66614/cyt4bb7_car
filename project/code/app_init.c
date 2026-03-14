#include "app_init.h"
#include "zf_common_headfile.h"
#include "control.h"
#include "servo.h"
#include "imu_fusion.h"
#include "zf_device_imu660rb.h"
#include "small_drive_uart_control.h"

// 宏定义
#define CONTROL_PERIOD_MS       5
#define CONTROL_FREQ_HZ         200
#define PRINT_PERIOD_MS         100

// 外部变量声明
extern volatile int g_control_enable;
extern volatile float g_pitch_angle;
extern volatile float g_pitch_gyro;
extern volatile float g_yaw_angle;
extern volatile float g_yaw_gyro;
extern volatile int g_target_speed;
extern volatile int g_turn_speed;

// 外部函数声明
extern void system_init(void);

void car_system_init(void)
{
    // 系统初始化
    clock_init(SYSTEM_CLOCK_250M);  
    debug_init();                       
    
    // 打印启动信息
    printf("\r\n========================================\r\n");
    printf("   轮腿小车 PID Balance Control\r\n");
    printf("   Control Frequency: %d Hz (%d ms)\r\n", CONTROL_FREQ_HZ, CONTROL_PERIOD_MS);
    printf("   Architecture: ISR Flag + Main Loop Task\r\n");
    printf("   Dual-MCU: CYT4BB7 <-> CYT2BL3 (FOC Driver)\r\n");
    printf("========================================\r\n");
    
    // 各模块初始化
    system_init();
    
    // 舵机初始化 - 使用servo.c中预设的角度值
    servo_init(servo_motor_duty_1, servo_motor_duty_2, 
               servo_motor_duty_3, servo_motor_duty_4);
    printf("Servo initialized: FL=%.1f, FR=%.1f, RL=%.1f, RR=%.1f\r\n",
           servo_motor_duty_1, servo_motor_duty_2, 
           servo_motor_duty_3, servo_motor_duty_4);
    
    // IMU660RB初始化
    uint8_t imu_init_status = imu660rb_init();
    if(imu_init_status != 0) {
        printf("IMU660RB init failed! Error: %d\r\n", imu_init_status);
        while(1); // 初始化失败，停止
    }
    printf("IMU660RB initialized\r\n");
    
    // 等待IMU稳定
    printf("Waiting for IMU stabilization...\r\n");
    system_delay_ms(500);
    
    // IMU零偏校准
    printf("Calibrating IMU zero bias...\r\n");
    imu_zero_bias_init(200);  // 采样200次
    printf("IMU zero bias calibration completed\r\n");
    
    // 卡尔曼滤波器初始化
    imu_fusion_init();
    printf("IMU fusion initialized\r\n");
    
    // 双机通信初始化（与CYT2BL3建立UART通信）
    small_driver_uart_init();  // 内部包含 uart_init() 和 uart_rx_interrupt()
    printf("Dual-MCU UART initialized: 460800 baud (UART4)\r\n");
    
    // 配置PIT定时器中断，5ms控制周期
    pit_ms_init(PIT_CH0, CONTROL_PERIOD_MS);
    printf("PIT timer initialized: %d ms period\r\n", CONTROL_PERIOD_MS);
    
    // 启动控制
    g_control_enable = 1;
    printf("\r\n>>> Control Started! <<<&\r\n\r\n");
}

void car_debug_print(void)
{
    static uint32_t print_counter = 0;
    print_counter++;
    
    if(print_counter >= PRINT_PERIOD_MS)
    {
        print_counter = 0;
        
        // 打印调试信息
        printf("P:%5.1f Pg:%5.1f | Y:%5.1f Yg:%5.1f | LSpd:%5d RSpd:%5d | Tgt:%d Turn:%d\r\n",
               g_pitch_angle, g_pitch_gyro,
               g_yaw_angle, g_yaw_gyro,
               motor_value.receive_left_speed_data,
               motor_value.receive_right_speed_data,
               g_target_speed,
               g_turn_speed);
    }
}
