#include "encoder.h"
//ye

void encoder_init(void)
{
    //D11作为方向引脚，D9作为脉冲信号脚
    encoder_dir_init(TIM9_ENCOEDER, TIM9_ENCOEDER_MAP3_CH1_D9, TIM9_ENCOEDER_MAP3_CH2_D11);
    //E11作为方向引脚，E9作为脉冲信号脚
    encoder_dir_init(TIM1_ENCOEDER,  TIM1_ENCOEDER_MAP3_CH1_E9, TIM1_ENCOEDER_MAP3_CH2_E11);
}

//获取一路的数值
int encoder_get(uint8 ch)
{
    int cnt;
    if(ch)
    {
        cnt=(ch==ENCODER_LEFT?ENCODER_LEFT_FRONT:ENCODER_RIGH_FRONT)*encoder_get_count (TIM9_ENCOEDER);
        encoder_clear_count (TIM9_ENCOEDER);

    }
    else
    {
        cnt= (ch==ENCODER_RIGH?ENCODER_LEFT_FRONT:ENCODER_RIGH_FRONT)*encoder_get_count (TIM1_ENCOEDER);
        encoder_clear_count (TIM1_ENCOEDER);
    }
    return cnt;
}

