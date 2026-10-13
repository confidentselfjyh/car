#ifndef _KEY_H_
#define _KEY_H_

#include "zf_driver_gpio.h"
#include "zf_driver_delay.h"

#define KEY1  gpio_get_level(A0)
#define KEY2  gpio_get_level(C3)
#define KEY3  gpio_get_level(A2)
#define KEY4  gpio_get_level(A1)
#define KEY5  gpio_get_level(A3)
#define KEY6  gpio_get_level(B10)
#define KEY7  gpio_get_level(C1)

#define KEY_MODE_SINGLE   0
#define KEY_MODE_CONTINUE 1

/* 按键事件编号（低电平表示按下） */
#define KEY_MIDDLE_PRES         1       //KEY1  中
#define KEY_UP_PRES             2       //KEY2  上
#define KEY_RIGHT_PRES          3       //KEY3  右
#define KEY_DOWN_PRES           4       //KEY4  下
#define KEY_LEFT_PRES           5       //KEY5  左
#define KEY6_PRES               6       //KEY6  菜单
#define KEY7_PRES               7       //KEY7  启动/停止

/* 按键事件锁存，由 Key_Tick 在 1ms 中断里写入，主循环用 Key_Get 取走 */
extern volatile uint8_t Key_Num;        // 短按事件
extern volatile uint8_t Key_Long_Num;   // 长按事件（按住 1 秒）

void    KEY_Init (void);
void    Key_Tick (void);                // 每 1ms 调用一次，非阻塞
uint8_t Key_Get  (void);                // 取走一次按键事件，取完自动清零

/* 旧的阻塞式扫描函数，内部有 system_delay_ms(10)，不要再在中断里用了 */
u8      KEY_Scan (u8 mode);

#endif
