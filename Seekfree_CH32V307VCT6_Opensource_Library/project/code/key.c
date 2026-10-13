/*
 * key.c
 *
 *  Created on: 2022年7月12日
 *      Author: 86159
 */
#include "key.h"

// 全局按键事件变量
volatile uint8_t Key_Num = 0;
volatile uint8_t Key_Long_Num = 0;
uint8_t KEY_SCAN_MODE = KEY_MODE_SINGLE; // 默认单次触发

//-------------------------------------------------------------------------------------------------------------------
//  @brief      GPIO初始化
//  @param      pin         选择的引脚 (可选择范围由 common.h 内GPIO_PIN_enum枚举值确定)
//  @param      dir         引脚的方向   输出：GPO   输入：GPI
//  @param      dat         引脚初始化时设置的电平状态，输出时有效 0：低电平 1：高电平
//  @param      pinconf     引脚配置（可设置参数由zf_gpio.h文件内GPIOSPEED_enum与GPIOMODE_enum枚举值确定，多个条件使用 | 相或）
//  @return     void
//  Sample usage:           gpio_init(D0, GPO, 1, GPIO_PIN_CONFIG);//D0初始化为GPIO功能、输出模式、输出高电平、速度100MHZ 推挽输出
//-------------------------------------------------------------------------------------------------------------------
void KEY_Init(void)
{
    gpio_init(C1,  GPI, 1, GPIO_INT_CONFIG);    // KEY7  启动/停止
    gpio_init(A0,  GPI, 1, GPIO_INT_CONFIG);    // KEY1  中
    gpio_init(C3,  GPI, 1, GPIO_INT_CONFIG);    // KEY2  上
    gpio_init(A2,  GPI, 1, GPIO_INT_CONFIG);    // KEY3  右
    gpio_init(A1,  GPI, 1, GPIO_INT_CONFIG);    // KEY4  下
    gpio_init(A3,  GPI, 1, GPIO_INT_CONFIG);    // KEY5  左
    gpio_init(B10, GPI, 1, GPIO_INT_CONFIG);    // KEY6  菜单（原来漏了这句，按键6一直没生效）
}

//按键处理函数
//返回按键值
//mode:0,不支持连续按;1,支持连续按;
//返回值：
//0，没有任何按键按下
//KEY0_PRES，KEY0按下
//KEY1_PRES，KEY1按下
//WKUP_PRES，WK_UP按下
//注意此函数有响应优先级,KEY0>KEY1>WK_UP!!
u8 KEY_Scan(u8 mode)
{
    static u8 key_up=1;//按键按松开标志
    if(mode)key_up=1;  //支持连按
    if(key_up&&(KEY7==0||KEY1==0||KEY2==0||KEY3==0||KEY4==0||KEY5==0||KEY6==0))
    {
        system_delay_ms(10);//去抖动10ms
        key_up=0;
             if(KEY3==0)return KEY_RIGHT_PRES;//右
        else if(KEY1==0)return KEY_MIDDLE_PRES;//中
        else if(KEY4==0)return KEY_DOWN_PRES;//下
        else if(KEY5==0)return KEY_LEFT_PRES;//左
        else if(KEY2==0)return KEY_UP_PRES;//上
        else if(KEY6==0)return KEY6_PRES;//6
        else if(KEY7==0)return KEY7_PRES;//7
    }
    else if(KEY7==1&&KEY1==1&&KEY2==1&&KEY3==1&&KEY4==1&&KEY5==1&&KEY6==1)key_up=1;//删除这段可长按
    return 0;// 无按键按下
}


/**
 * @brief  按键定时扫描函数
 * @param  无
 * @retval 无
 * @note   必须每 1ms 调用一次
 */
void Key_Tick(void)
{
    static uint8_t scan_count = 0;

    /*
     * 已确认的按键状态：
     * 1 表示松开
     * 0 表示按下
     */
    static uint8_t key_state[7] =
    {
        1, 1, 1, 1, 1, 1, 1
    };

    /*
     * 消抖计数。
     */
    static uint8_t key_filter_count[7] =
    {
        0, 0, 0, 0, 0, 0, 0
    };

    /*
     * 按住时间计数，单位为 20ms。
     */
    static uint8_t key_hold_count[7] =
    {
        0, 0, 0, 0, 0, 0, 0
    };

    uint8_t i;

    /*
     * 每 20ms 扫描一次按键。
     */
    scan_count++;

    if (scan_count < 20)
    {
        return;
    }

    scan_count = 0;

    /*
     * 原始按键电平。
     */
    uint8_t key_raw[7] =
    {
        KEY1,
        KEY2,
        KEY3,
        KEY4,
        KEY5,
        KEY6,
        KEY7
    };

    /*
     * 与按键引脚相对应的按键值。
     */
    uint8_t key_code[7] =
    {
        KEY_MIDDLE_PRES, // KEY1
        KEY_UP_PRES,     // KEY2
        KEY_RIGHT_PRES,  // KEY3
        KEY_DOWN_PRES,   // KEY4
        KEY_LEFT_PRES,   // KEY5
        KEY6_PRES,       // KEY6
        KEY7_PRES        // KEY7
    };

    for (i = 0; i < 7; i++)
    {
        /*
         * 原始状态与确认状态不同，开始消抖。
         */
        if (key_raw[i] != key_state[i])
        {
            key_filter_count[i]++;

            /*
             * 连续两次扫描状态相同：
             * 2 × 20ms = 40ms。
             */
            if (key_filter_count[i] >= 2)
            {
                key_filter_count[i] = 0;
                key_state[i] = key_raw[i];
                key_hold_count[i] = 0;

                /*
                 * 检测到按键确认按下。
                 * 低电平表示按下。
                 */
                if (key_state[i] == 0)
                {
                    Key_Num = key_code[i];
                }
            }
        }
        else
        {
            key_filter_count[i] = 0;

            /*
             * 按键保持按下。
             */
            if (key_state[i] == 0)
            {
                if (key_hold_count[i] < 255)
                {
                    key_hold_count[i]++;
                }

                /*
                 * 50 × 20ms = 1000ms。
                 */
                if (key_hold_count[i] == 50)
                {
                    Key_Long_Num = key_code[i];
                }

                /*
                 * 连续触发模式。
                 * 按住超过 200ms 后，每 100ms 产生一次事件。
                 */
                if (KEY_SCAN_MODE == KEY_MODE_CONTINUE)
                {
                    if (key_hold_count[i] >= 10 &&
                        key_hold_count[i] % 5 == 0)
                    {
                        Key_Num = key_code[i];
                    }
                }
            }
            else
            {
                key_hold_count[i] = 0;
            }
        }
    }
}


/**
 * @brief  取走一次按键事件
 * @param  无
 * @retval 按键编号，没有事件时返回 0
 * @note   Key_Tick 只负责把事件锁存进 Key_Num，由主循环调用本函数取走。
 *         取走后 Key_Num 清零，所以一次按下只会被处理一次。
 */
uint8_t Key_Get(void)
{
    uint8_t key = Key_Num;

    if (key != 0)
    {
        Key_Num = 0;
    }

    return key;
}
