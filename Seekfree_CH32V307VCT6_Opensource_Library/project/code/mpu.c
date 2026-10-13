/*
 * mpu.c
 *
 *  Created on: 2024年10月26日
 *      Author: 付外茂
 */
#include "mpu.h"
extern float acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z;

void mpu_getvalue()
{
    mpu6050_get_acc();
    mpu6050_get_gyro();
    acc_x=mpu6050_acc_transition(mpu6050_acc_x);
    acc_y=mpu6050_acc_transition(mpu6050_acc_y);
    acc_z=mpu6050_acc_transition(mpu6050_acc_z);
    gyro_x=mpu6050_gyro_transition(mpu6050_gyro_x);
    gyro_y=mpu6050_gyro_transition(mpu6050_gyro_y);
    gyro_z=mpu6050_gyro_transition(mpu6050_gyro_z);
    }

