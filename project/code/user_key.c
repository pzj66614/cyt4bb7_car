#include "user_key.h"

// 定义四个按键引脚，对应 main.c 中的 KEY1~KEY4
#define KEY1_PIN          (P20_0)
#define KEY2_PIN          (P20_1)
#define KEY3_PIN          (P20_2)
#define KEY4_PIN          (P20_3)

#define KEY_FILTER_TIME   (2)       // 20ms 消抖 (2*10ms)
#define KEY_LONG_TIME     (100)     // 1000ms 算长按 (100*10ms)
#define KEY_DOUBLE_TIME   (30)      // 300ms 内连按两次算双击 (30*10ms)

// 引脚数组，方便循环遍历
static const uint32 key_pins[USER_KEY_NUM] = {KEY1_PIN, KEY2_PIN, KEY3_PIN, KEY4_PIN};

// 将状态机变量转为数组，每个按键拥有独立的跟踪状态
static uint8 key_state[USER_KEY_NUM] = {0};
static uint16 key_timer[USER_KEY_NUM] = {0};
static key_event_e current_event[USER_KEY_NUM] = {KEY_EVENT_NONE};

void user_key_init(void)
{
    // 循环初始化四个按键为上拉输入
    for(int i = 0; i < USER_KEY_NUM; i++) 
    {
        gpio_init(key_pins[i], GPI, GPIO_HIGH, GPI_PULL_UP);
    }
}

// 建议将此函数放在 10ms 周期执行的定时器中断内
void user_key_scan_10ms(void)
{
    // 遍历四个按键
    for(int i = 0; i < USER_KEY_NUM; i++)
    {
        uint8 pin_level = gpio_get_level(key_pins[i]); // 0为按下，1为松开

        switch(key_state[i])
        {
            case 0: // 空闲状态
                if (pin_level == 0) 
                {
                    key_state[i] = 1;      // 进入消抖阶段
                    key_timer[i] = 0;
                }
                break;
                
            case 1: // 消抖确认
                if (pin_level == 0) 
                {
                    key_state[i] = 2;      // 确认按下
                    key_timer[i] = 0;
                } 
                else 
                {
                    key_state[i] = 0;      // 可能是干扰
                }
                break;
                
            case 2: // 等待松开或判定长按
                if (pin_level == 0) 
                {
                    key_timer[i]++;
                    if (key_timer[i] >= KEY_LONG_TIME) 
                    {
                        current_event[i] = KEY_EVENT_LONG_PRESS; // 触发长按
                        key_state[i] = 5; // 等待长按松开状态
                    }
                } 
                else // 在长按时间到达前松开
                {
                    key_timer[i] = 0;
                    key_state[i] = 3; // 进入等待双击阶段
                }
                break;
                
            case 3: // 等待双击
                if (pin_level == 0) 
                {
                    key_state[i] = 4; // 双击状态的消抖
                } 
                else 
                {
                    key_timer[i]++;
                    if (key_timer[i] >= KEY_DOUBLE_TIME) 
                    {
                        current_event[i] = KEY_EVENT_SHORT_PRESS; // 超时未第二次按下，仅为短按
                        key_state[i] = 0;
                    }
                }
                break;
                
            case 4: // 双击确认
                if (pin_level == 0) 
                {
                    current_event[i] = KEY_EVENT_DOUBLE_CLICK; // 确认为双击
                    key_state[i] = 5; 
                } 
                else 
                {
                    key_state[i] = 3;
                }
                break;
                
            case 5: // 等待彻底松开
                if (pin_level != 0) 
                {
                    key_state[i] = 0; // 回到空闲
                }
                break;
        }
    }
}

key_event_e user_key_get_event(user_key_id_e key_id)
{
    // 边界保护
    if (key_id >= USER_KEY_NUM) return KEY_EVENT_NONE;

    key_event_e ev = current_event[key_id];
    if (ev != KEY_EVENT_NONE) 
    {
        current_event[key_id] = KEY_EVENT_NONE; // 读后清零，防止重复读取
    }
    return ev;
}
