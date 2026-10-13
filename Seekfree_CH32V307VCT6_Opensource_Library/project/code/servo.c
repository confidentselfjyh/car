/*
 * servo.c
 *
 *  Created on: 2024年9月22日
 *      Author: 付外茂
 */
#include "servo.h"

int16 medille     = 6710;       // 舵机中值，菜单3可调
int16 servo_limit = 1700;       // 打舵限幅，菜单3可调


void servo_ctrl(int16 angle)
{
    int16 temp_angle = angle;
    int32 duty;

    temp_angle = _limit(temp_angle, -servo_limit, servo_limit);

    duty = (int32)medille - (int32)temp_angle;

    // 中值和限幅都能在菜单里改，这里兜一下底，防止占空比顶出 0 ~ PWM_DUTY_MAX
    if (duty < 0)               duty = 0;
    if (duty > PWM_DUTY_MAX)    duty = PWM_DUTY_MAX;

    pwm_set_duty(TIM2_PWM_MAP1_CH1_A15, (uint32)duty);
}

void servo_init(void)
{
    pwm_init(TIM2_PWM_MAP1_CH1_A15,1000,medille);
}
