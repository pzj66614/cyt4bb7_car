#ifndef _USER_KEY_H_
#define _USER_KEY_H_

#include "zf_common_headfile.h"

// 定义按键枚举，用于指定读取哪个按键
typedef enum {
    USER_KEY_1 = 0,
    USER_KEY_2,
    USER_KEY_3,
    USER_KEY_4,
    USER_KEY_NUM // 按键总数
} user_key_id_e;

// 定义按键事件类型
typedef enum {
    KEY_EVENT_NONE = 0,
    KEY_EVENT_SHORT_PRESS,
    KEY_EVENT_LONG_PRESS,
    KEY_EVENT_DOUBLE_CLICK
} key_event_e;

// 初始化按键
void user_key_init(void);

// 状态机扫描函数（请放在 10ms 定时器中断中运行，或在主循环不阻塞的情况下调用）
void user_key_scan_10ms(void);

// 获取按键事件（调用后会自动清除该按键的事件旗标）
key_event_e user_key_get_event(user_key_id_e key_id);

#endif
