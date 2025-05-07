#ifndef __ALGORITHM_H
#define __ALGORITHM_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include "Insole.h"
#define CycleTimeBufferSize 3
typedef struct
{
	volatile unsigned short int HeelStrikeTick;		 // 脚跟触地时的tick
	volatile unsigned short int LastHeelStrikeTick;	 // 上一次脚跟触地时的tick
	volatile unsigned short int Last2HeelStrikeTick; // 上上次脚跟触地时的tick
	volatile unsigned short int PushOffTick;
	volatile unsigned short int ToeOffTick; // 脚尖离地时的tick
	volatile unsigned short int Cycletime;
	volatile unsigned short int OppositeHeelStrikeTick;
	volatile unsigned short int OppositeLastHeelStrikeTick;
	int HeelStikeDected;		 // 脚跟触地检测
	int FootDected;				 // 脚掌触地检测
	uint32_t LastHeelStrikeTime; // 上一次脚跟着地的时间
	uint32_t LastToeOffTime;	 // 上一次脚尖离地的时间
	int CycletimeBuffer[CycleTimeBufferSize];
	uint32_t Tick;	 // 周期计数
	float Phase; // 步态相位
	int Flag; //输入flag，用于左右步态切换
} CycleTimerMeasureTypedef;

typedef struct
{
	float Force_desire;		   // 期望力
	float Force_error;		   // 力误差
	int   Force_peak;			   // 力峰值
	float Motor_position;	   // 电机位置
	float Motor_vel;		   // 电机速度
	float Motor_position_last; // 上一时刻电机位置
	int   Motor_direction;//电机旋转方向
	float Desired_acc;	   // 期望加速度
	float Desired_vel;	   // 期望速度
	float Desired_pos;	   // 期望位置
	int   ForceSensorID;     
	float Force_tighten;    //力传感器紧绷力
} ForceControlTypedef;		   // 力控制参数

typedef struct
{
	int LeftFlag;
	int RightFlag;
	int LTick; // 左侧周期计数
	int RTick; // 右侧周期计数

} ExoskeletonParaTypeDef;

extern CycleTimerMeasureTypedef TimerLeft;
extern CycleTimerMeasureTypedef TimerRight;

extern ExoskeletonParaTypeDef Exo;
extern ForceControlTypedef LeftForce;
extern ForceControlTypedef RightForce;

extern int enableFlagLeft;
extern int enableFlagRight;

extern int position_reference_L;
extern int position_reference_R;
extern float virtual_inertia;
extern float virtual_damping;
extern float virtual_stiffness;
// extern uint32_t LCycletick, RCycletick;
// extern uint32_t LtickCnt, RtickCnt, LGrytick, RGrytick;
extern float Fd_Left, Fe_Left;
extern float Fd_Right, Fe_Right;
// extern uint32_t Fp_right;
//**********************************************************//
//

extern void UpdataGryZBuffer(void);
extern void CycleTimerTickLeft(void);
extern void CycleTimerTickRight(void);
extern void AverageCycleTime(CycleTimerMeasureTypedef *Timer);// 步态周期均值滤波
// extern int skeletonAlgorithmLeft(void);
// extern int skeletonAlgorithmRight(void);
// extern int skeletonAlgorithmFdRight(void);
extern int force2motorposition(int p);
extern int FeedforwardLeft(void);
extern int FeedforwardRight(void);
extern void InitializeImuAngle(void);
extern int CycleTimeCalRight(void);
extern int CycleTimeCalLeft(void);
extern void json_analysis(void);
extern void json_pack(void);
extern void UpdataPressureInsole(void); //
extern float ReferenceForce(float Phase);
extern void AddmitanceControl(ForceControlTypedef *Controller);
extern void ClearControllerAddmitance(ForceControlTypedef *Controller);
extern void ClearControllerPos(ForceControlTypedef *Controller);
#endif
