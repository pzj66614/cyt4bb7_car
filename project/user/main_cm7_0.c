/*********************************************************************************************************************
* CYT4BB Opensourec Library (基于 CYT4BB 开源库) 是一个基于官方 SDK 接口的单片机开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 CYT4BB 开源库的一部分
*
* CYT4BB 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL (GNU General Public License) 的条款
* 即 GPL 的第3版（或 GPL3.0），或任何后续版本，重新分发和/或修改它
*
* 开源库的发布是希望它能有用，但未提供任何担保
* 甚至没有对适销性或特定用途适用性的暗示担保
* 有关详情参见 GPL
*
* 您应在此开源库发布时同时收到一份 GPL 的副本
* 如果没有，请参阅 <https://www.gnu.org/licenses/>
*
* 特别声明
* 本开源库使用 GPL3.0 开源许可证协议 该声明只针对当前版本
* 使用的中文版本 中文翻译文件见 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件
* 许可证副本见 libraries 文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序，但请注意保护逐飞科技的版权和著作权
*
* 文件名称          main_cm7_0
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹下 version 文件 版本说明
* 适用环境          IAR 9.40.1
* 适用平台          CYT4BB
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2024-1-4       pudding            first version
* 2026-3-8       modified           重构轮腿小车PID直立控制，双机通信架构
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "control.h"
#include "servo.h"
#include "imu_fusion.h"
#include "zf_device_imu660rb.h"
#include "small_drive_uart_control.h"  // 双机通信驱动
#include "app_init.h"

// **************************** 宏定义 ****************************

// 控制周期定义 5ms = 200Hz
#define CONTROL_PERIOD_MS       5
#define CONTROL_FREQ_HZ         200

// 调试打印周期 100ms
#define PRINT_PERIOD_MS         100

// 双机通信使用的UART (定义在 small_drive_uart_control.h 中)
// SMALL_DRIVER_UART = UART_4
// SMALL_DRIVER_BAUDRATE = 460800
// SMALL_DRIVER_TX = UART4_RX_P14_0
// SMALL_DRIVER_RX = UART4_TX_P14_1

// **************************** 全局变量 ****************************

// 控制标志（由中断设置，主循环查询）
volatile uint8_t g_control_flag = 0;        // 控制任务触发标志

// 编码器数据（从CYT2BL3接收）
volatile float g_motor_speed = 0;            // 平均电机速度

// 控制状态
volatile int g_control_enable = 0;           // 控制使能标志

// 调试数据
volatile float g_pitch_angle = 0;            // 俯仰角
volatile float g_pitch_gyro = 0;             // 俯仰角速度
volatile float g_yaw_angle = 0;              // Yaw角(影响偏航)
volatile float g_yaw_gyro = 0;               // Yaw角速度
volatile int g_target_speed = 0;             // 目标主转速（前进后退）
volatile int g_turn_speed = 0;               // 目标转向差速（左右偏差）

// **************************** 函数声明 ****************************

void system_init(void);
void control_task(void);                      // 主循环控制任务
void control_isr_handler(void);               // 中断处理函数（仅设置标志）

// **************************** 函数实现 ****************************

/**
 * @brief 系统初始化
 */
void system_init(void)
{
    // 禁用全局中断进行初始化
    interrupt_global_disable();
    
    // 使能全局中断
    interrupt_global_enable(0);
}

/**
 * @brief 计算速度平均值（用于PID控制）
 * 从CYT2BL3接收的左右电机速度计算平均值
 */
float calculate_average_speed(void)
{
    // 从双机通信结构体获取左右电机速度
    // motor_value.receive_left_speed_data 和 receive_right_speed_data 在中断中自动更新
    return (float)(motor_value.receive_left_speed_data + motor_value.receive_right_speed_data) / 2.0f;
}

// **************************** 控制任务 ****************************

/**
 * @brief 控制任务 - 在主循环中执行
 * 执行周期由PIT定时器控制（5ms），通过 g_control_flag 标志触发
 * 
 * 数据流向：
 * 1. 从IMU读取姿态 → PID计算 → 得到目标转速 g_target_speed
 * 2. 通过相应的发送函数(后续需要根据需求替换)发送目标转速给 CYT2BL3
 * 3. CYT2BL3 执行FOC驱动电机，并可能通过串口返回实际转速
 * 4. 实际转速在中断中存入 motor_value.receive_left/right_speed_data
 */
void control_task(void)
{
    // 检查控制标志，未设置则直接返回
    if(!g_control_flag) return;
    
    // 清除标志（必须在开头清除，防止遗漏下一次触发）
    g_control_flag = 0;
    
    // 采集IMU数据（加速度计和陀螺仪）
    imu660rb_get_acc();
    imu660rb_get_gyro();
    
    // 卡尔曼滤波更新姿态，dt = 5ms = 0.005s
    imu_fusion_update(NULL, 0.005f);
    
    // 获取滤波后的姿态角和角速度
    float pitch_angle = imu_get_pitch_angle();
    float pitch_gyro = imu_get_pitch_gyro();
    
    // 获取Yaw角信息用于转向差速控制
    float yaw_angle = imu_get_yaw_angle();
    float yaw_gyro = imu_get_yaw_gyro();
    
    // 获取电机速度（从CYT2BL3接收的实际转速）
    g_motor_speed = calculate_average_speed();
    
    // 更新全局调试变量
    g_pitch_angle = pitch_angle;
    g_pitch_gyro = pitch_gyro;
    g_yaw_angle = yaw_angle;
    g_yaw_gyro = yaw_gyro;
    
    // 如果控制使能，计算并输出速度
    if(g_control_enable) {
        // 1. 直立环 + 速度环 -> 决定基础共模转速
        int target_speed = Control_Get_Total_Speed(pitch_angle, pitch_gyro, g_motor_speed);
        g_target_speed = target_speed;
        
        // 2. 转向环(Yaw) -> 决定差速
        int turn_speed = Control_Get_Turn_Speed(yaw_angle, yaw_gyro);
        g_turn_speed = turn_speed;
        
        // 3. 差速叠加到底盘左右轮上
        // 注意加减方向：需根据实际车体转向方向进行测试，如果反向则调换 + / -
        int left_target = target_speed - turn_speed;
        int right_target = target_speed + turn_speed;

        // 发送带差速的目标转速给CYT2BL3（双机通信）
        small_driver_set_speed((int16_t)left_target, (int16_t)right_target);
    } else {
        // 控制未使能，停止电机
        g_target_speed = 0;
        g_turn_speed = 0;
        small_driver_set_speed(0, 0);
    }
}

// **************************** 中断处理 ****************************

/**
 * @brief 控制中断处理函数
 * 注意：此函数在 cm7_0_isr.c 中的 pit0_ch0_isr() 中被调用
 * 仅设置标志，所有实际工作在 control_task() 中完成
 */
void control_isr_handler(void)
{
    // 仅设置控制标志，让主循环执行控制任务
    // 避免在中断中执行耗时操作，防止中断阻塞
    g_control_flag = 1;
}

// **************************** 主函数 ****************************

int main(void)
{
    // 系统初始化
    car_system_init();
    
    // 主循环
    while(true)
    {
        // 执行控制任务（由中断标志触发，5ms周期）
        // 该任务会：
        // 1. 读取IMU数据进行姿态解算
        // 2. 获取CYT2BL3返回的电机速度
        // 3. PID计算得到目标转速
        // 4. 将差速目标转速发送至CYT2BL3
        control_task();
        
        // 每100ms打印一次调试信息
        car_debug_print();
        
        // 喂狗（如果使能了看门狗）
        // system_watchdog_feed();
        
        // 主循环延时（不能太长，确保能及时响应控制标志）
        system_delay_ms(1);
    }
}
