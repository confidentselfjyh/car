/*
 * servo.h
 *
 *  Created on: 2024年9月22日
 *      Author: 付外茂
 */

#ifndef BSP_SERVO_H_
#define BSP_SERVO_H_

#include "zf_common_headfile.h"
#include "Ourcode_headfile.h"

extern int16 medille;       // 舵机中值
extern int16 servo_limit;   // 打舵限幅

void servo_ctrl(int16 angle);
void servo_init(void);

#endif /* BSP_SERVO_H_ */
