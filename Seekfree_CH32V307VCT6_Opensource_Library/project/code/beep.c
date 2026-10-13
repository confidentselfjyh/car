#include "beep.h"

/* BEEP_PIN 统一在 control.h 里定义(当前是 C13), 这里不要再重定义。
   原来这里写的是 B7, 有两个问题:
     1. 把头文件里的值覆盖掉, 每次编译都报 "BEEP_PIN redefined" 警告
     2. B7 是 TFT180 的复位脚(TFT180_RES_PIN), 真拿它驱动蜂鸣器会把屏幕不停复位 */
//uint8 BeepCount = 0;
void beepstart()
{
    //打开蜂鸣器
    gpio_set_level(BEEP_PIN,1);
    //关闭蜂鸣器
    // gpio_set_level(BEEP_PIN,0);
}
void beepinit()
{
    gpio_init(BEEP_PIN, GPO, 0, GPO_PUSH_PULL);
}
