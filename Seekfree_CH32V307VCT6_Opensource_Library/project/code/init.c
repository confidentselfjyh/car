/*
 * init.c
 *
 *  Created on: 2024年10月26日
 *      Author: 付外茂
 */

#ifndef INIT_C_
#define INIT_C_

#include "init.h"
void Init(void)
{
    //按键初始化
    KEY_Init();
    //屏幕初始化
    tft180_set_dir(TFT180_CROSSWISE_180);
    tft180_init();
    tft180_set_font(TFT180_6X8_FONT);
    //电机初始化
    //printf("wheel TEST INIT...\r\n");
    wheel_init();
    //编码器初始化
    //printf("encoder TEST INIT...\r\n");
    encoder_init();
    //舵机初始化
    servo_init();
    //摄像头
    mt9v03x_init();
    mt9v03x_set_exposure_time(128);//曝光时间设置，括号内参数自行根据情况修改
    JumpInit();
    //蜂鸣器 C13   EN-L D12   EN-R D15
    //初始化蜂鸣器
    //beepinit();
    //pid初始化
    PID_init();

    //装载固化的标准赛宽(Base_Width_Default), 这样上电不用重新标定
    Width_Calib_LoadDefault();

    //陀螺仪
    //mpu6050_init();

    //定时器中断初始化，服务函数在isr.c
    pit_ms_init(TIM5_PIT, 1);//开启一个5ms的中断1
    pit_ms_init(TIM6_PIT, 10);//编码器，开启一个10ms的中断

}

#endif /* INIT_C_ */

