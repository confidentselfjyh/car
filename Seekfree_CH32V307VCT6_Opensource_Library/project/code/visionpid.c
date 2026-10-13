/*
 * visionpid.c
 *
 *  Created on: 2024年9月22日
 *      Author: 付外茂
 */

#include "visionpid.h"

// typedef struct
// {
//    float kp;
//    float ki;
//    float kd;

//    float error_curr;   //当前误差
//    float error_last;   //上一次的误差
//    float sum_error;    //积分累加

//    float limit_out_abs; //输出限幅
// }PosiPidNode;

PosiPidNode servo_pid = {.limit_out_abs = 1700},motor_pid = {.limit_out_abs = 10000},stop_pid = {.limit_out_abs = 10000},gyro_pid = {.limit_out_abs = 10000};
InrcPidNode left_pid = {.limit_out_abs = 10000},right_pid = {.limit_out_abs = 10000};

float LocaPid(PosiPidNode *pid,float input,float target)
{
    float ep=0.0, ei=0.0, ed=0.0;
    float pidout=0.0;

    pid ->error_curr = target - input;
    //ep = 0.7*pid ->error_curr+0.3*pid ->error_last ;//一阶滤波
    ep = pid->error_curr;
    pid ->sum_error += ep;
    pid->sum_error=(pid ->sum_error<3500?(pid ->sum_error<(-3500)?(-3500):pid ->sum_error):3500);//积分限幅

    ei = pid ->sum_error;
    ed = ep - pid ->error_last;


    pidout = pid->kp * ep + pid->ki * ei + pid->kd * ed;
    //输出限幅
    if(pidout > pid ->limit_out_abs)
    {
        pidout = pid ->limit_out_abs;
    }
    else if(pidout < (- pid ->limit_out_abs))
    {
        pidout = - pid ->limit_out_abs;
    }
    //输出限幅

    pid ->error_last =ep;
    return pidout;
}

void SetLocaPid(PosiPidNode *pid, float kp, float ki, float kd)
{
    pid ->kp = kp;
    pid ->ki = ki;
    pid ->kd = kd;
}


//增量式pid
float InrcPid(InrcPidNode *pid, float curr, float expc)
{
    float ep=0, ei=0, ed=0;
    float inrc_out=0;

    pid ->error_curr = expc - curr;

    ep = pid->error_curr - pid->error_last;
    ei = pid->error_curr;
    ed = pid->error_curr - 2*pid->error_last + pid->error_pre;

    inrc_out = pid->kp * ep + pid->ki * ei + pid->kd * ed;
    if(inrc_out > pid ->limit_out_abs)
    {
        inrc_out = pid ->limit_out_abs;
    }
    else if(inrc_out < - pid ->limit_out_abs)
    {
        inrc_out = - pid ->limit_out_abs;
    }

    pid->error_pre = pid->error_last;
    pid->error_last = pid->error_curr;

    return inrc_out;
}


void SetInrcPid(InrcPidNode *pid, float kp, float ki, float kd)
{
    pid ->kp = kp;
    pid ->ki = ki;
    pid ->kd = kd;
}



void PID_init(void)
{
    SetLocaPid(&servo_pid,90,0,200);
    // SetLocaPid(&motor_pid,0,0,0);
    // SetLocaPid(&stop_pid, 0, 0, 0);
    // SetLocaPid(&gyro_pid,0,0,0);
    SetInrcPid(&left_pid,90, 9, 0);
    SetInrcPid(&right_pid,90, 9, 0);

    // 顺手把历史状态也清掉，这样菜单里的“恢复默认PID”可以随时调用
    servo_pid.error_curr = 0.0f;
    servo_pid.error_last = 0.0f;
    servo_pid.sum_error  = 0.0f;

    left_pid.error_curr = 0.0f;
    left_pid.error_last = 0.0f;
    left_pid.error_pre  = 0.0f;

    right_pid.error_curr = 0.0f;
    right_pid.error_last = 0.0f;
    right_pid.error_pre  = 0.0f;
}


