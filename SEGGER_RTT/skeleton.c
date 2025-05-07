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
#include "math.h"
#include "Insole.h"
#include "Pwm_output.h"
#include "my_math.h"
#include "motor_control.h"
#include "pid.h"
#include "algorithm.h"
enum SkeletonStateTypeDef Skeleton_State = SKELETON_OFF;

PID_Regulator_t  CMLEFT1 =  MOTOR_DEFAULT;  //电机pid控制初始化
PID_Regulator_t  CMRIGHT0 = MOTOR_DEFAULT;

// int velocity_reference=0;//输出参考速度
int force_reference = 0;	 // 输出参考位置
int32_t OutPutPos = 0;		 // 输出位置
int force_reference_spd = 0; // 输出参考速度
int Global_Motor_Position = 0;
int LastOutPutPos = 0;
int position_reference_R, position_reference_L;
int motor_position_real, motor_velocity_real;
float x;
float ThetaL, ThetaR;
float init_pos_middle;
float init_pos_left;
float Force_L;

float tighten_force = 0.13;  //预紧力
float tighten_speed = 10;  //预紧速度


float speed_limit_high = 50;//预紧速度限制
float speed_limit_low = -60.0;//预紧速度限制



static int tighten_flag = 0;

static int tighten_flag1 = 0;

static int tighten_flag2 = 0;

static int init123_flag = 0;
float Freq; // 测试正弦波频率
float t1 = 0, t2, t3, t4;

State current_state = NoAssistance;
Input input = NoFlag;
State previous_state = NoAssistance;
void stateTransition(State current, Input input)
{
	// No Assistance State
	if (current == NoAssistance)
	{
		if (input == NoFlag)
		{
			// Global_Motor_Position = OutPutPos;
			force_reference = 0;
			// force_reference_spd = 0;
			// LED_OFF();
		}
		else if (input == LeftFlag)
		{
			// Global_Motor_Position = OutPutPos;
			MotorPositionGernerate(&LeftForce, &TimerLeft);
			force_reference = LeftForce.Motor_position;
			force_reference_spd = LeftForce.Motor_vel;
			current_state = LeftAssistance;
		}
		else if (input == RightFlag)
		{
			// Global_Motor_Position = OutPutPos;
			MotorPositionGernerate(&RightForce, &TimerRight);
			force_reference = RightForce.Motor_position;
			force_reference_spd = RightForce.Motor_vel;
			current_state = RightAssistance;
		}
	}
	// Left Assistance State
	else if (current == LeftAssistance)
	{
		if (input == LeftFlag)
		{
			MotorPositionGernerate(&LeftForce, &TimerLeft);
			force_reference = LeftForce.Motor_position;
			force_reference_spd = LeftForce.Motor_vel;
			RightForce.Force_desire = 0;
			// LED_ON(1);
		}
		else if (input == RightFlag)
		{
			// Global_Motor_Position = OutPutPos;
			MotorPositionGernerate(&RightForce, &TimerRight);
			force_reference = RightForce.Motor_position;
			force_reference_spd = RightForce.Motor_vel;
			current_state = RightAssistance;
			ClearControllerPos(&LeftForce);
		}
		else if (input == NoFlag)
		{
			// Global_Motor_Position = OutPutPos;
			force_reference = 0;
			// force_reference_spd = 0;
			current_state = NoAssistance;
			ClearControllerPos(&LeftForce);
			Global_Motor_Position = Global_Motor_Position;
			LastOutPutPos = LastOutPutPos + 2 * SWITCH_THRESHOLD * RightForce.Motor_direction;
			LeftForce.Force_desire = 0;

			// ReleaseCable(LeftForce.Motor_direction);
		}
	}
	// Right Assistance State
	else if (current == RightAssistance)
	{
		if (input == RightFlag)
		{
			MotorPositionGernerate(&RightForce, &TimerRight);
			force_reference = RightForce.Motor_position;
			force_reference_spd = RightForce.Motor_vel;
			LeftForce.Force_desire = 0;
			// LED_ON(2);
		}
		else if (input == LeftFlag)
		{
			MotorPositionGernerate(&LeftForce, &TimerLeft);
			force_reference = LeftForce.Motor_position;
			force_reference_spd = LeftForce.Motor_vel;
			current_state = LeftAssistance;
			ClearControllerPos(&RightForce);
		}
		else if (input == NoFlag)
		{
			force_reference = 0;
			// force_reference_spd = 0;
			current_state = NoAssistance;
			ClearControllerPos(&RightForce);
			Global_Motor_Position = Global_Motor_Position;
			LastOutPutPos = LastOutPutPos + 2 * SWITCH_THRESHOLD * LeftForce.Motor_direction;
			RightForce.Force_desire = 0;
			// ReleaseCable(RightForce.Motor_direction);
		}
	}
	if (current_state != current)
	{
		Global_Motor_Position = LastOutPutPos;
	}
	//	debugPrintPending("current:%d input:%d Global_Motor_Position:%d force_reference:%d OutPutPos:%d\r\n", current, input, Global_Motor_Position, force_reference, OutPutPos);
}
void motor_init(int i){
	if(i != 0 && i != 1) return;
	cmd[i].id=i; 			
	cmd[i].mode=1;
	cmd[i].T=0;
	cmd[i].W=0;
	cmd[i].Pos=0;
	cmd[i].K_P=0;
	cmd[i].K_W=0;
	SERVO_Send_recv(&cmd[i], &data[i]);
}

void motor_stop(int i){
	if(i != 0 && i != 1) return;
	cmd[i].mode=0;
	SERVO_Send_recv(&cmd[i], &data[i]);
}

void motor_zeroTorque(int i){
	if(i != 0 && i != 1) return;
	cmd[i].T = 0.0; 
	cmd[i].W = 0.0; 
	cmd[i].Pos = 0.0; 
	cmd[i].K_P = 0.0; 
	cmd[i].K_W = 0.0; 
  SERVO_Send_recv(&cmd[i], &data[i]);
}
void motor_Torque(int i,float torque){
	if(i != 0 && i != 1) return;
	cmd[i].T = torque; 
	cmd[i].W = 0.0; 
	cmd[i].Pos = 0.0; 
	cmd[i].K_P = 0.0; 
	cmd[i].K_W =  0.0033f;//0 
  SERVO_Send_recv(&cmd[i], &data[i]);
}

void motor_damping(int i){
	if(i != 0 && i != 1) return;
	cmd[i].T = 0.0; 
	cmd[i].W = 0.0; 
	cmd[i].Pos = 0.0; 
	cmd[i].K_P = 0.0; 
	cmd[i].K_W = 0.02; 
  SERVO_Send_recv(&cmd[i], &data[i]);
}

void motor_profilePosition(int i, float pos_rad){ 
	if(i != 0 && i != 1) return;
	cmd[i].T = 0.0; 
	cmd[i].W = 0.0; 
	cmd[i].Pos = pos_rad*6.33f; 
	cmd[i].K_P = 0.1f; 
	cmd[i].K_W = 0.0; 
  SERVO_Send_recv(&cmd[i], &data[i]);
}
void motor_profileVelocity(int i, float vel_rad){ //线速度 rad/s
	if(i != 0 && i != 1) return;
	cmd[i].T = 0.0; 
	cmd[i].W = vel_rad; 
	cmd[i].Pos = 0; 
	cmd[i].K_P = 0; 
	cmd[i].K_W = 0.2f;//0.2
  SERVO_Send_recv(&cmd[i], &data[i]);
}

void motor_tighten(int i){
	if(i != 0 && i != 1) return;
	cmd[i].T = 0.0; 
	cmd[i].W = 0.0; 
	cmd[i].Pos = -10; 
	cmd[i].K_P = 0.01; 
	cmd[i].K_W = 0.0; 
  SERVO_Send_recv(&cmd[i], &data[i]);
}


void CMPID_Init()
{
		CMLEFT1.kp=3;
		CMLEFT1.kd=0.2;

		
		CMRIGHT0.kp=3;
	  CMRIGHT0.kd=0.2;
	   //	CMRIGHT0.ki=0.02;
	
//		CMRIGHT0.kp=0.35;
//    //	CMRIGHT0.ki=0.02;
//	  CMRIGHT0.kd=0.5;
}



void skeletonApp(void)
{
	
	switch (Skeleton_State)
	{

	case SKELETON_OFF:
		
	    motor_stop(0);   //0=right 1=left
	    motor_stop(1); 
		// HAL_GPIO_WritePin(GPIOH, GPIO_PIN_2, GPIO_PIN_SET); //
		// HAL_GPIO_WritePin(GPIOH, GPIO_PIN_4, GPIO_PIN_SET); //

		debugPrintPending("----- Skeleton power on ------------------------------\r\n");
		Skeleton_State = SKELETON_INIT;
		debugPrintPending("----- Start initial ----------------------------------\r\n");
		HAL_Delay(500);
		break;
	case SKELETON_INIT:
		__HAL_TIM_SET_COUNTER(&htim3, 0);  // 清零定时器
//		HAL_TIM_Base_Start_IT(&htim3);
//	  Skeleton_State = SKELETON_WORKING;//调试用
//		break;
		//		initMotorZeroPoint();
		//		initMotorZeroPointSingleSide();//单边初始化
		// system_stiffness_test();
		// PreLoad();									         // 初始化预紧
		// comm_can_set_origin(Motor_Controller_ID, (uint8_t)1); // 设置此时位置为原�??
		if (Skeleton_State == SKELETON_INIT)
		{
			if(init123_flag==0)//只运行一遍
			{
			  HAL_Delay(1000);
				debugPrintPending("----- ones done -----------------------------------\r\n");
				motor_init(0);
	      		motor_init(1);  //电机初始化
			TimerLeft.Tick = 0; // 初始化时钟
			TimerRight.Tick = 0;
//			TimerLeft.Flag = 1; // 定义输入Flag
//			TimerRight.Flag = 2;
//			LeftForce.Motor_direction = 1; // 定义电机旋转方向
//			RightForce.Motor_direction = -1;
//			LeftForce.ForceSensorID = 0;
//			RightForce.ForceSensorID = 1;
//			LeftForce.Force_peak = 80;
//			RightForce.Force_peak = 80;
				LeftForce.Force_tighten = 10;  //预紧力大小
				RightForce.Force_tighten = 10;
				CMPID_Init();
				//CMRIGHT0.Reset(&CMRIGHT0);
				
				CMRIGHT0.ref=tighten_force;// 阻力设置

				
				__HAL_TIM_SET_COUNTER(&htim3, 0); // 清零定时器
				HAL_TIM_Base_Start_IT(&htim3);
				init123_flag=1;
			}
			
      debugPrintPending("----- Waiting tighten -----------------------------------\r\n");
      
        //   force_left = (float)skeletonAdDmaBuf[0]*388.29/4095;
		//   force_right = (float)skeletonAdDmaBuf[1]*388.29/4095; //O=LEFT 1=RIGHT  拉力传感器

			
			
			if(force_left<LeftForce.Force_tighten&&tighten_flag1==0)
			{ 
				motor_profileVelocity(1,-tighten_speed); 
			}
			else
			{
				motor_profileVelocity(1,0); 
				tighten_flag1=1;             //未预紧=0  预紧完毕=1
				debugPrintPending("----- right tighten -----------------------------------\r\n");
			}
			
			if(force_right<RightForce.Force_tighten&&tighten_flag2==0)
			{ 
				//motor_profileVelocity(0,0); 
				motor_profileVelocity(0,tighten_speed); 
			}
			else
			{
				debugPrintPending("----- left tighten -----------------------------------\r\n");
				motor_profileVelocity(0,0); 
				tighten_flag2=1;             //未预紧=0  预紧完毕=1
			}
			
	  if(tighten_flag1==1&&tighten_flag2==1)
	  {
				tighten_flag=1;             //未预紧=0  预紧完毕=1
				Skeleton_State = SKELETON_WORKING; 
				debugPrintPending("----- Initial done -----------------------------------\r\n");
				debugPrintPending("----- Start Working ----------------------------------\r\n");
	  }
		else
		{
				//Skeleton_State = SKELETON_WORKING;//调试用
				debugPrintPending("----- Waiting tighten -----------------------------------\r\n");
		}
//			if(force_left<LeftForce.Force_tighten||force_right<RightForce.Force_tighten&&tighten_flag==0)//两侧同时预张紧
//			{ 
//				

//				motor_profileVelocity(0,5); 
//				motor_profileVelocity(1,-5);   //

//			}
//			else
//			{
//				motor_profileVelocity(0,0); 
//				motor_profileVelocity(1,0);
//				tighten_flag=1;             //未预紧=0  预紧完毕=1
//				Skeleton_State = SKELETON_WORKING; 
//				debugPrintPending("----- Initial done -----------------------------------\r\n");
//				debugPrintPending("----- Start Working ----------------------------------\r\n");
//			}
			// Motor_Enable();
		}
		break;
//**********************************************************************************************************************
	case SKELETON_WORKING:

//	if(data[0].Pos>200||data[0].Pos<-200||data[1].Pos>200||data[1].Pos<-200)//保护超限
//	{
//		motor_zeroTorque(0);
//		motor_zeroTorque(1);
////			motor_stop(0);
////		  motor_stop(1);
//	
//	}
	
//	else
	

	{ if(TimerLeft.Cycletime)
		{
		TimerLeft.Phase = ((float)TimerLeft.Tick / (float)TimerLeft.Cycletime)*100.0;		   // 计算步态相位
	  }
		if(TimerRight.Cycletime)
		{
   	TimerRight.Phase = ((float)TimerRight.Tick / (float)TimerRight.Cycletime)*100.0;	
	  }
		
		
		
		CMLEFT1.ref=ReferenceForce(TimerRight.Phase);
		//CMLEFT1.ref=0.1;
		force_left=(float)skeletonAdDmaBuf[0]*388.29/4095;
		CMLEFT1.fdb = (float)force_left/144.0;
		PID_Calc(&CMLEFT1);
		

		CMRIGHT0.ref=ReferenceForce(TimerLeft.Phase);// 阻力设置 
		//CMRIGHT0.ref=0.1;// 阻力设置 
		
	  //force_right=ReferenceForce(TimerRight.Phase);
		force_right = (float)skeletonAdDmaBuf[1]*388.29/4095;
		//CMRIGHT0.fdb = (float)force_motor_right;
		CMRIGHT0.fdb = (float)force_right/144.0;
		PID_Calc(&CMRIGHT0);
		
		
//	  	motor_Torque(1,-0.0);
//	  	motor_Torque(0,0.0);
// 
//		
		if(ReferenceForce(TimerLeft.Phase))
			motor_Torque(1,-(tighten_force+CMLEFT1.output));
		else
			if(data[1].W<speed_limit_low ||data[1].W>speed_limit_high)
				motor_Torque(1,-(tighten_force+CMLEFT1.output)*0.4);
		  else 
				motor_Torque(1,-(tighten_force+CMLEFT1.output));
			
			
			
			
			
			
        //motor_Torque(1,-tighten_force);

//				motor_Torque(1,-(CMLEFT1.output));
		
		
		//CMRIGHT0.Calc(&CMRIGHT0);
		//motor_Torque(0,0);
		
		
		    //motor_Torque(0,tighten_force);
		//0.13+0.2
			
			
			
		if(ReferenceForce(TimerRight.Phase))
			motor_Torque(0,tighten_force+CMRIGHT0.output);
		else
			if(data[0].W<speed_limit_low ||data[0].W>speed_limit_high)
				motor_Torque(0,(tighten_force+CMRIGHT0.output)*0.4
			
			);
		  else 
				motor_Torque(0,tighten_force+CMRIGHT0.output);
			}
		//

	

	
	
//*********************************步态计算代码*************************************************************************************
		UpdataGryZBuffer();//更新IMU数据
		CycleTimerTickLeft();//计算步态周期
		CycleTimerTickRight();

//**********************************************************************************************************************
		
		
//		motor_getinfo()
//		force_left = (float)skeletonAdDmaBuf[0]*388.29/4095;
//		force_right = (float)skeletonAdDmaBuf[1]*388.29/4095; //O=LEFT 1=RIGHT 
	 // debugPrintPending("LeftForce=%f,RightForce=%f,Torgue_left=%f,Torgue_right=%f \r\n",  force_left, force_right, data[0].T, data[1].T);
	
			//debugPrintPending("force:%f,%f,%f,%f,%f \n",  force_left, force_motor_left, force_right, force_motor_right,data[0].Pos);
	
		//debugPrintPending("time=%d,LeftForce=%f,RightForce=%f,DesirePos=%d,RealPos=%d,Freq=%f \r\n", TimerRight.Tick, force_left, force_right, OutPutPos, motor_position_real, Freq);

		//**********************************************************************************************************************

		//********无步态输入下生成步态周期，测试代码**************************************************************************************************************
		// LastOutPutPos = OutPutPos;
		// if (TimerRight.Tick >= 1000) // 测试定时器，无实际助力
		// {
		// 	input = RightFlag;
		// 	TimerRight.Cycletime = TimerRight.Tick;
		// 	TimerRight.Tick = 0;
		// }

		// if (TimerLeft.Tick >= 1000) // 测试定时器，无实际助力
		// {
		// 	input = LeftFlag;
		// 	TimerLeft.Cycletime = TimerLeft.Tick;
		// 	TimerLeft.Tick = 0;
		// }
		// OutPutPos = Global_Motor_Position + force_reference; // 计算电机输出位置

		//**********************************************************************************************************************

		//************正常运行代码**********************************************************************************************************
		// stateTransition(current_state, input); // 有限状态机，切换左右脚助力，现移至中断中运行

		// 		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14,GPIO_PIN_RESET); //LED_G，运行指示灯
		// 		LastOutPutPos = OutPutPos;
		// 		UpdataPressureInsole();//更新压力数据
		

		
		
		
		// 		OutPutPos = Global_Motor_Position + force_reference; // 计算电机输出位置
		// 		comm_can_set_pos_spd(Motor_Controller_ID, (float)(OutPutPos), force_reference_spd, 30000);
		// 		force_left = (float)skeletonAdDmaBuf[0] * (3.3 / 10) * 250 * 0.4536 * 9.8 / 4096;
		// 		force_right = (float)skeletonAdDmaBuf[1] * (3.3 / 10) * 250 * 0.4536 * 9.8 / 4096;
		// 		debugPrintPending("t=%d,LF=%f,RF=%f,FLd=%f,FRd=%f,DP=%d,RP=%d, \r\n", Global_time,force_left, force_right, LeftForce.Force_desire, RightForce.Force_desire,OutPutPos, motor_position_real);
		// //		debugPrintPending("LTH=%d,RTH=%d, \r\n", LeftInsole.HeelValue, RightInsole.HeelValue);

		//**********************************************************************************************************************

		//****************调试位置速度环代码******************************************************************************************************
		// comm_can_set_pos_spd(Motor_Controller_ID, (float)(OutPutPos), force_reference_spd, 32767); // 位置、速度、加速度
		// comm_can_set_pos_spd(Motor_Controller_ID, (float)(100), (int16_t)OutPutPos/6.55, 1); // 位置、速度、加速度
		// comm_can_set_pos(Motor_Controller_ID, (float)(OutPutPos));
		// 输出正弦波
		// force_reference  = 300*sin((double)(RCycletick)/2000.0*3.1415*2);
		// force_reference_spd = 300*cos((double)(RCycletick)/2000.0*3.1415*2);

		// motor_receive(&motor_data.motor_pos, &motor_data.motor_spd, &motor_data.motor_cur, RxData_Motor); // 接收电机数据
		// motor_position_real = motor_data.motor_pos;														  // 更新电机实际位置
		// motor_velocity_real = motor_data.motor_spd;
		//**********************************************************************************************************************
		break;
	case SKELETON_ERROR:

		break;
	}
}


