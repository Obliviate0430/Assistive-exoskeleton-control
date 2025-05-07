#include "skeleton.h"
#include "algorithm.h"
#include "uart_debug.h"
#include "my_exti.h"
#include "motor_can.h"
#include "imu_can.h"
#include "main.h"
#include "cJSON.h"
#include <stdlib.h>
#include "usart.h"
#include "IterativeLearning.h"
#include "tim.h"
enum SkeletonStateTypeDef Skeleton_State = SKELETON_OFF;

int force_reference;

int position_reference_R,position_reference_L;
int motor_position_real,motor_velocity_real;
float x;
float ThetaL,ThetaR;
float init_pos_middle;
float init_pos_left;
float Force_L;
// extern float Fd_Left;
// extern float Fd_Right;

float t1=0,t2,t3,t4;

//*****************�1�7�1�7�1�7�1�7�1�7�1�7
// extern float var_accy_L;
// extern float var_accy_R;
//*****************

State current_state = NoAssistance;
Input input = NoFlag;
void stateTransition(State current, Input input)
{
	switch(current) 
	{
        case NoAssistance:
            if(input == NoFlag) 
			{
				enableFlagLeft = 0;
				enableFlagRight = 0;
                force_reference=ZeroPoint;
            } else if(input == LeftFlag) {
				enableFlagLeft = 1;
				enableFlagRight = 0;
				force_reference=skeletonAlgorithmLeft();
				current_state = LeftAssistance; 
				ClearError();
//				UpdateSampleRate();
            } else if(input == RightFlag) {
				enableFlagLeft = 0;
				enableFlagRight = 1;
				force_reference=skeletonAlgorithmRight();
				current_state = RightAssistance; 
				ClearError();
            }
            break;
        case LeftAssistance:
            if(input == LeftFlag) {
				enableFlagLeft = 1;
				enableFlagRight = 0;
				force_reference=skeletonAlgorithmLeft();
            } else if(input == RightFlag) {
				enableFlagLeft = 0;
				enableFlagRight = 1;
				force_reference=skeletonAlgorithmRight();
                current_state = RightAssistance; 
				Fd_Left=0;
				ClearError();
            } else if(input == NoFlag) {
				enableFlagLeft = 0;
				enableFlagRight = 0;
				force_reference=ZeroPoint;
				current_state = NoAssistance;
				Fd_Left=0;
				Fd_Right=0;
				ClearError();
//				UpdateSampleRate();
			}
            break;
		case RightAssistance:
			if(input == RightFlag) {
				enableFlagLeft = 0;
				enableFlagRight = 1;
				force_reference=skeletonAlgorithmRight();
            } else if(input == LeftFlag) {
				enableFlagLeft = 1;
				enableFlagRight = 0;
				force_reference=skeletonAlgorithmLeft();
                current_state = LeftAssistance; 
				Fd_Right=0;
				ClearError();
            }
			else if(input == NoFlag) {
				enableFlagLeft = 0;
				enableFlagRight = 0;
				force_reference=ZeroPoint;
                current_state = NoAssistance; 
				Fd_Left=0;
				Fd_Right=0;
				ClearError();
            }
            break;
    }
}


void skeletonApp(void){
		
	switch(Skeleton_State){
		
		case SKELETON_OFF:
		HAL_GPIO_WritePin(GPIOH,GPIO_PIN_2,GPIO_PIN_SET);//
		HAL_GPIO_WritePin(GPIOH,GPIO_PIN_4,GPIO_PIN_SET);//

		printf("----- Skeleton power on ------------------------------\r\n");
		Skeleton_State=SKELETON_INIT;
		printf("----- Start initial ----------------------------------\r\n");
		HAL_Delay(1000);
		break;
		case SKELETON_INIT:
		initMotorZeroPoint();
		Skeleton_State=SKELETON_WORKING;//跳过初始化
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,GPIO_PIN_RESET);//LED_R
		if(Skeleton_State==SKELETON_WORKING)
		{
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,GPIO_PIN_RESET);//LED_R
			printf("----- Initial done -----------------------------------\r\n");
			printf("----- Start Working ----------------------------------\r\n");
			HAL_Delay(1000);
			RCycletick=0;
			LCycletick=0;
			LGrytick=0;
			RGrytick=0;
			__HAL_TIM_SET_COUNTER(&htim3, 0);
			__HAL_TIM_SET_COUNTER(&htim4, 0);
			HAL_TIM_Base_Start_IT(&htim4);// 启动定时器
			HAL_TIM_Base_Start_IT(&htim3);// 启动定时器
		}

		break;
		case SKELETON_WORKING:
		// HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14,GPIO_PIN_RESET); //LED_G
		UpdataGryZBuffer();//更新IMU数据
		CycleTimerTickLeft();//计算步态周期及检测HeelStrike
		CycleTimerTickRight();
		stateTransition(current_state, input);//有限状态机，切换左右脚助力
		
//		comm_can_set_pos(Motor_Controller_ID, (float)(force_reference));//发送命令让电机转动

		 if (RCycletick>=1500)//测试定时器，无实际助力
		 {
		 	input=RightFlag;
			
			TimerRight.Cycletime=RCycletick;
		 	RCycletick=0;
			
		}
//		 if (LCycletick>=2000)//测试定时器，无实际助力
//		 {
//		 	input=LeftFlag;
//			
//			TimerLeft.Cycletime=LCycletick;
//		 	LCycletick=0;
//			
//		}
		motor_position_real=motorrealposition();
//		motor_velocity_real=motorrealvelocity();
		force_left = (float)skeletonAdDmaBuf[0]*(3.3/10)*250*0.4536*9.8/4096;
		force_right = (float)skeletonAdDmaBuf[1]*(3.3/10)*250*0.4536*9.8/4096;
		// printf("forceL=%.2f,forceR=%.2f,FdL=%.2f,FdR=%.2f,Input=%d,force_reference=%d \r\n",force_left,force_right,Fd_Left,Fd_Right,50*input,force_reference);
		// json_pack();//发送json数据
//		printf("Fd=%.2f,forceL=%d,LError=%.2f,Tick=%.2f \r\n",Fd_Left,force_reference,LError[LClearIndex],Tick);
		// printf("Fd=%.2f,forceL=%d \r\n",Fd_Left,force_reference);
		// printf("enableFlagLeft=%d,enableFlagRight=%d,force_reference=%d\r\n",enableFlagLeft,enableFlagRight,force_reference);		
		break;
		case SKELETON_ERROR:
			
		break;
		}
	}
