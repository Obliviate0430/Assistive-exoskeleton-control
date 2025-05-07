#ifndef __SKELETON_H
#define __SKELETON_H

#include "pid.h"

extern void skeletonApp(void);
	
enum SkeletonStateTypeDef
{
  	SKELETON_INIT = 0,
  	SKELETON_WORKING = 1,
	  SKELETON_OFF = 2,
	  SKELETON_ERROR = 3
};

typedef enum {
	NoAssistance = 0,//不助力
	LeftAssistance = 1,//左侧助力
	RightAssistance = 2,//右侧助力
}State;

typedef enum {
	NoFlag = 0,//无助力FLAG
	LeftFlag = 1,//左侧助力FLAG
	RightFlag = 2,//右侧助力FLAG
}Input;

//extern void SoftReset(void);
extern enum SkeletonStateTypeDef Skeleton_State;
extern int motor_position_real;
extern int motor_velocity_real;
extern int force_reference_R,force_reference_L;
extern int position_reference_R,position_reference_L;
extern float t1,t2;
extern Input input;//有限状态机输入变量
extern State current_state;//当前助力状态

extern int force_reference;
extern int OutPutPos;			  // 输出位置
extern int force_reference_spd; // 输出参考速度
extern int Global_Motor_Position;

extern PID_Regulator_t CMLEFT1;
extern PID_Regulator_t CMRIGHT0;
#endif
