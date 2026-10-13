#ifndef _ENCODER_H_
#define _ENCODER_H_
#include "zf_common_headfile.h"

//设置左右的编号
#define ENCODER_LEFT 0
#define ENCODER_RIGH 1
//设置正方向
#define ENCODER_LEFT_FRONT (-1)
#define ENCODER_RIGH_FRONT (1)

//初始化
void encoder_init(void);
//获取一路数值
int encoder_get(uint8 ch);



#endif
