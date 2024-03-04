#include "encoder.h"
#include "tim.h"

/**************************************************************************
Function: Read the encoder count
Input   : The timer
Output  : Encoder value (representing speed)
函数功能：读取编码器计数
入口参数：定时器
返回  值：编码器数值(代表速度)
**************************************************************************/
int Read_Encoder(unsigned char TIMX) {
  int Encoder_TIM;
  switch (TIMX) {
  case 2:
    Encoder_TIM = (short)TIM2->CNT;
    TIM2->CNT = 0;
    break;
  case 3:
    Encoder_TIM = (short)TIM3->CNT;
    TIM3->CNT = 0;
    break;
  case 4:
    Encoder_TIM = (short)TIM4->CNT;
    TIM4->CNT = 0;
    break;
  case 5:
    Encoder_TIM = (short)TIM5->CNT;
    TIM5->CNT = 0;
    break;
  default:
    Encoder_TIM = 0;
  }
  return Encoder_TIM;
}

void encoder_tim_init() {
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
}
