/*
 * visionpid.h
 *
 *  Created on: 2024年9月22日
 *      Author: 付外茂
 */

#ifndef VISIONPID_H_
#define VISIONPID_H_

#include "zf_common_headfile.h"
#include "Ourcode_headfile.h"

typedef struct
{
    float kp;
    float ki;
    float kd;

    float error_curr;   //当前误差
    float error_last;   //上一次的误差
    float sum_error;    //误差和

    float limit_out_abs; //输出限幅
}PosiPidNode;

typedef struct
{
    float kp;
    float ki;
    float kd;

    float error_curr;   //当前误差（e_0）
    float error_last;   //上一次误差(e_1)
    float error_pre;    //前一次的偏差(e_2)

    float limit_out_abs; //输出限幅
}InrcPidNode;//增量式

extern PosiPidNode servo_pid,motor_pid,stop_pid,gyro_pid;
extern InrcPidNode left_pid,right_pid;

float LocaPid(PosiPidNode *pid,float input,float target);
float InrcPid(InrcPidNode *pid, float expc, float curr);
void SetLocaPid(PosiPidNode *pid, float kp, float ki, float kd);
void SetInrcPid(InrcPidNode *pid, float kp, float ki, float kd);
void PID_init(void);

#endif /* VISIONPID_H_ */

