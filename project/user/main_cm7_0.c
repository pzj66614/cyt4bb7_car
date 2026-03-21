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

/*********************************************************************************************************************
* 文件名称          main_cm7_0
* 功能说明          科目一“绕桩”主函数（集成 project_one_vision）
********************************************************************************************************************/

/******************************************************************************
 * main_cm7_0.c
 *
 * 核心架构变化：
 * 控制闭环 100% 在 PIT 中断里执行，5ms 一次
 * 视觉处理在主循环里执行，不管多慢都不影响平衡
 *****************************************************************************/

#include "zf_common_headfile.h"
#include "control.h"
#include "servo.h"
#include "imu_fusion.h"
#include "zf_device_imu660rb.h"
#include "zf_device_mt9v03x.h"
#include "zf_device_ips114.h"
#include "small_drive_uart_control.h"
#include "project_one_vision.h"

#define CONTROL_PERIOD_MS   5
#define VISION_YAW_LIMIT    8.0f
#define VISION_YAW_ALPHA    0.15f
#define PRINT_DIV           500
#define DISPLAY_DIV         30

extern float Target_Yaw_Angle;
extern float Target_Speed;

volatile uint8_t g_control_enable = 0;

volatile float g_pitch_angle  = 0.0f;
volatile float g_pitch_gyro   = 0.0f;
volatile float g_yaw_angle    = 0.0f;
volatile float g_yaw_gyro     = 0.0f;
volatile int   g_target_speed = 0;
volatile int   g_turn_speed   = 0;

static float g_vision_enter_yaw     = 0.0f;
static float g_vision_yaw_filtered  = 0.0f;

void control_isr_handler(void)
{
    imu660rb_get_acc();
    imu660rb_get_gyro();
    imu_fusion_update(NULL, 0.005f);

    float pitch_angle = imu_get_pitch_angle();
    float pitch_gyro  = imu_get_pitch_gyro();
    float yaw_angle   = imu_get_yaw_angle();
    float yaw_gyro    = imu_get_yaw_gyro();

    float motor_speed = (float)(motor_value.receive_left_speed_data +
                                motor_value.receive_right_speed_data) / 2.0f;

    g_pitch_angle = pitch_angle;
    g_pitch_gyro  = pitch_gyro;
    g_yaw_angle   = yaw_angle;
    g_yaw_gyro    = yaw_gyro;

    if(g_control_enable)
    {
        int target_speed = Control_Get_Total_Speed(pitch_angle, pitch_gyro, motor_speed);
        int turn_speed   = Control_Get_Turn_Speed(yaw_angle, yaw_gyro);

        g_target_speed = target_speed;
        g_turn_speed   = turn_speed;

        int left_target  = target_speed - turn_speed;
        int right_target = target_speed + turn_speed;

        small_driver_set_speed((int16_t)left_target, (int16_t)right_target);
    }
}

int main(void)
{
    uint32_t print_cnt   = 0;
    uint32_t display_cnt = 0;

    clock_init(SYSTEM_CLOCK_250M);
    debug_init();

    printf("\r\n=== Balance(PIT_CH1) + Vision + Display ===\r\n");

    interrupt_global_disable();
    interrupt_global_enable(0);

    servo_init(servo_motor_duty_1, servo_motor_duty_2,
               servo_motor_duty_3, servo_motor_duty_4);

    if(imu660rb_init() != 0) while(1);
    system_delay_ms(500);
    imu_zero_bias_init(200);
    imu_fusion_init();
    printf("IMU ready\r\n");

    small_driver_uart_init();
    printf("Motor UART ready\r\n");

    if(mt9v03x_init() != 0) while(1);
    printf("Camera ready\r\n");

    ips114_init();
    printf("IPS114 ready\r\n");

    project_one_vision_init();
    project_one_vision_enable();
    printf("Vision ready\r\n");

    Target_Speed     = 0.0f;
    Target_Yaw_Angle = 0.0f;
    g_vision_enter_yaw    = imu_get_yaw_angle();
    g_vision_yaw_filtered = 0.0f;
    Target_Yaw_Angle      = g_vision_enter_yaw;

    // 控制用 PIT_CH1，避开摄像头的 CH0
    pit_ms_init(PIT_CH1, CONTROL_PERIOD_MS);

    g_control_enable = 1;
    printf("Control on PIT_CH1\r\n");

    while(1)
    {
        // ======== 视觉处理（低优先级）========
        if(mt9v03x_finish_flag)
        {
            mt9v03x_finish_flag = 0;
            project_one_vision_run(mt9v03x_image);

            float raw_yaw = g_project_one_yaw;
            if(raw_yaw > VISION_YAW_LIMIT)
            {
                raw_yaw = VISION_YAW_LIMIT;
            }
            if(raw_yaw < -VISION_YAW_LIMIT)
            {
                raw_yaw = -VISION_YAW_LIMIT;
            }

            g_vision_yaw_filtered = g_vision_yaw_filtered * (1.0f - VISION_YAW_ALPHA)
                                  + raw_yaw * VISION_YAW_ALPHA;

            Target_Yaw_Angle = g_vision_enter_yaw + g_vision_yaw_filtered;
        }

        // ======== 降频刷屏（不阻塞控制）========
        display_cnt++;
        if(display_cnt >= DISPLAY_DIV)
        {
            display_cnt = 0;

            ips114_show_gray_image(0, 0, (const uint8 *)g_project_one_binary_image,
                                   PROJECT_ONE_IMAGE_W, PROJECT_ONE_IMAGE_H,
                                   135, 86, 128);

            ips114_show_string(0, 88, "St:");
            ips114_show_int(24, 88, g_project_one_state, 2);

            ips114_show_string(50, 88, "Blk:");
            ips114_show_int(82, 88, g_project_one_block_count, 2);

            ips114_show_string(0, 104, "Err:");
            ips114_show_int(30, 104, g_project_one_error, 4);

            ips114_show_string(70, 104, "Yaw:");
            ips114_show_float(102, 104, g_vision_yaw_filtered, 2, 2);
        }

        // ======== 降频打印 ========
        print_cnt++;
        if(print_cnt >= PRINT_DIV)
        {
            print_cnt = 0;
            printf("P:%.1f Y:%.1f | TS:%d Turn:%d | Err:%d Vyaw:%.2f\r\n",
                   g_pitch_angle, g_yaw_angle,
                   g_target_speed, g_turn_speed,
                   g_project_one_error,
                   g_vision_yaw_filtered);
        }

        system_delay_ms(1);
    }
}