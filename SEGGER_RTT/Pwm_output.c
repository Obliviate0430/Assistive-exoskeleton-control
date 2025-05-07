#include "stm32f4xx_hal.h"
#include "main.h"
#include "tim.h"

// 设置PWM占空比的函数，channel为通道号(例如TIM_CHANNEL_1)，pulse为占空比值，取值0~99
void Set_PWM_DutyCycle(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t pulse)
{
  TIM_OC_InitTypeDef sConfigOC = {0};

  sConfigOC.OCMode = TIM_OCMODE_PWM1;// 设置PWM模式
  sConfigOC.Pulse = pulse; // 新的占空比值
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;// 设置有效电平为高电平
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;// 不使用快速模式
  HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, channel);// 重新配置PWM输出
  HAL_TIM_PWM_Start(htim, channel); // 重新启动PWM输出
}

//助力指示灯亮灭函数
void LED_ON(int side)//side=1,左脚；side=2,右脚
{
  if(side == 1)//
  {
//    Set_PWM_DutyCycle(&htim4, TIM_CHANNEL_1, 100);//左侧灯亮
//    Set_PWM_DutyCycle(&htim4, TIM_CHANNEL_2, 0);
  }
  else
  {
//    Set_PWM_DutyCycle(&htim4, TIM_CHANNEL_2, 100);
//    Set_PWM_DutyCycle(&htim4, TIM_CHANNEL_1, 0);
  }
}

void LED_OFF(void)//灭灯函数
{
//  Set_PWM_DutyCycle(&htim4, TIM_CHANNEL_1, 0);
//  Set_PWM_DutyCycle(&htim4, TIM_CHANNEL_2, 0);
}