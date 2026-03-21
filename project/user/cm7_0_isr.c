/*********************************************************************************************************************
* CYT4BB Opensourec Library
* Copyright (c) 2022 SEEKFREE
*
* File name: cm7_0_isr
* Company: Chengdu Seekfree Technology Co., Ltd.
* Version: See version file in libraries/doc folder
* Environment: IAR 9.40.1
* Platform: CYT4BB
* Store link: https://seekfree.taobao.com/
*
* Modification record:
* Date       Author     Note
* 2024-1-9   pudding    first version
* 2024-5-14  pudding    Added 12 PIT timer interrupts
* 2026-3-8   modified   重构为中断标志模式，添加双机通信UART接收中断
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "small_drive_uart_control.h"

// External declaration for control flag
extern volatile uint8_t g_control_flag;

// **************************** PIT ISR Functions ****************************

// 声明主循环控制中断处理函数
extern void control_isr_handler(void);

/**
 * @brief PIT0_CH0 中断服务函数
 * 控制周期定时器 (5ms = 200Hz)
 */
void pit0_ch0_isr(void)
{
    // 清除中断标志位
    pit_isr_flag_clear(PIT_CH0);

    // 调用控制中断处理函数（设置标志位，主循环执行具体任务）
    
}
 

void pit0_ch1_isr(void)
{
    pit_isr_flag_clear(PIT_CH1);
    control_isr_handler();
}

void pit0_ch2_isr(void)
{
    pit_isr_flag_clear(PIT_CH2);
}

void pit0_ch10_isr(void)
{
    pit_isr_flag_clear(PIT_CH10);
}

void pit0_ch11_isr(void)
{
    pit_isr_flag_clear(PIT_CH11);
}

void pit0_ch12_isr(void)
{
    pit_isr_flag_clear(PIT_CH12);
}

void pit0_ch13_isr(void)
{
    pit_isr_flag_clear(PIT_CH13);
}

void pit0_ch14_isr(void)
{
    pit_isr_flag_clear(PIT_CH14);
}

void pit0_ch15_isr(void)
{
    pit_isr_flag_clear(PIT_CH15);
}

void pit0_ch16_isr(void)
{
    pit_isr_flag_clear(PIT_CH16);
}

void pit0_ch17_isr(void)
{
    pit_isr_flag_clear(PIT_CH17);
}

void pit0_ch18_isr(void)
{
    pit_isr_flag_clear(PIT_CH18);
}

void pit0_ch19_isr(void)
{
    pit_isr_flag_clear(PIT_CH19);
}

void pit0_ch20_isr(void)
{
    pit_isr_flag_clear(PIT_CH20);
    tsl1401_collect_pit_handler();
}

void pit0_ch21_isr(void)
{
    pit_isr_flag_clear(PIT_CH21);
    tsl1401_collect_pit_handler();
}

// **************************** GPIO External Interrupt Functions ****************************

void gpio_0_exti_isr(void)
{
}

void gpio_1_exti_isr(void)
{
    if(exti_flag_get(P01_0))
    {
    }
    if(exti_flag_get(P01_1))
    {
    }
}

void gpio_2_exti_isr(void)
{
    if(exti_flag_get(P02_0))
    {
    }
    if(exti_flag_get(P02_4))
    {
    }
}

void gpio_3_exti_isr(void)
{
}

void gpio_4_exti_isr(void)
{
}

void gpio_5_exti_isr(void)
{
}

void gpio_6_exti_isr(void)
{
}

void gpio_7_exti_isr(void)
{
}

void gpio_8_exti_isr(void)
{
}

void gpio_9_exti_isr(void)
{
}

void gpio_10_exti_isr(void)
{
}

void gpio_11_exti_isr(void)
{
}

void gpio_12_exti_isr(void)
{
}

void gpio_13_exti_isr(void)
{
}

void gpio_14_exti_isr(void)
{
}

void gpio_15_exti_isr(void)
{
}

void gpio_16_exti_isr(void)
{
}

void gpio_17_exti_isr(void)
{
}

void gpio_18_exti_isr(void)
{
}

void gpio_19_exti_isr(void)
{
}

void gpio_20_exti_isr(void)
{
}

void gpio_21_exti_isr(void)
{
}

void gpio_22_exti_isr(void)
{
}

void gpio_23_exti_isr(void)
{
}

// **************************** UART ISR Functions ****************************

void uart0_isr(void)
{
    if(Cy_SCB_GetRxInterruptMask(get_scb_module(UART_0)) & CY_SCB_UART_RX_NOT_EMPTY)
    {
        Cy_SCB_ClearRxInterrupt(get_scb_module(UART_0), CY_SCB_UART_RX_NOT_EMPTY);
        
#if DEBUG_UART_USE_INTERRUPT
        debug_interrupr_handler();
#endif
    }
    else if(Cy_SCB_GetTxInterruptMask(get_scb_module(UART_0)) & CY_SCB_UART_TX_DONE)
    {
        Cy_SCB_ClearTxInterrupt(get_scb_module(UART_0), CY_SCB_UART_TX_DONE);
    }
}

void uart1_isr(void)
{
    if(Cy_SCB_GetRxInterruptMask(get_scb_module(UART_1)) & CY_SCB_UART_RX_NOT_EMPTY)
    {
        Cy_SCB_ClearRxInterrupt(get_scb_module(UART_1), CY_SCB_UART_RX_NOT_EMPTY);
        wireless_module_uart_handler();
    }
    else if(Cy_SCB_GetTxInterruptMask(get_scb_module(UART_1)) & CY_SCB_UART_TX_DONE)
    {
        Cy_SCB_ClearTxInterrupt(get_scb_module(UART_1), CY_SCB_UART_TX_DONE);
    }
}

void uart2_isr(void)
{
    if(Cy_SCB_GetRxInterruptMask(get_scb_module(UART_2)) & CY_SCB_UART_RX_NOT_EMPTY)
    {
        Cy_SCB_ClearRxInterrupt(get_scb_module(UART_2), CY_SCB_UART_RX_NOT_EMPTY);
        gnss_uart_callback();
    }
    else if(Cy_SCB_GetTxInterruptMask(get_scb_module(UART_2)) & CY_SCB_UART_TX_DONE)
    {
        Cy_SCB_ClearTxInterrupt(get_scb_module(UART_2), CY_SCB_UART_TX_DONE);
    }
}

void uart3_isr(void)
{
    if(Cy_SCB_GetRxInterruptMask(get_scb_module(UART_3)) & CY_SCB_UART_RX_NOT_EMPTY)
    {
        Cy_SCB_ClearRxInterrupt(get_scb_module(UART_3), CY_SCB_UART_RX_NOT_EMPTY);
    }
    else if(Cy_SCB_GetTxInterruptMask(get_scb_module(UART_3)) & CY_SCB_UART_TX_DONE)
    {
        Cy_SCB_ClearTxInterrupt(get_scb_module(UART_3), CY_SCB_UART_TX_DONE);
    }
}

/**
 * @brief UART4 中断服务函数
 * 用于接收 CYT2BL3 返回的电机速度数据
 * 在 small_driver_uart_control.c 中的 uart_control_callback() 处理数据
 */
void uart4_isr(void)
{
    if(Cy_SCB_GetRxInterruptMask(get_scb_module(UART_4)) & CY_SCB_UART_RX_NOT_EMPTY)
    {
        Cy_SCB_ClearRxInterrupt(get_scb_module(UART_4), CY_SCB_UART_RX_NOT_EMPTY);
        // 调用双机通信接收回调函数
        // 解析CYT2BL3返回的7字节速度数据帧
        uart_control_callback();
    }
    else if(Cy_SCB_GetTxInterruptMask(get_scb_module(UART_4)) & CY_SCB_UART_TX_DONE)
    {
        Cy_SCB_ClearTxInterrupt(get_scb_module(UART_4), CY_SCB_UART_TX_DONE);
    }
}
