#ifndef _PID_REGULATOR_H_
#define _PID_REGULATOR_H_
#include "stm32f4xx.h"
#include "struct_typedef.h"

#define VAL_LIMIT(val, min, max)\
if(val<=min)\
{\
	val = min;\
}\
else if(val>=max)\
{\
	val = max;\
}\

#define MOTOR_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	0.0f,\
	0.0f,\
	0.0f,\
	0,\
	0,\
	0,\
	5000,\
	3500,\
	1500,\
	0,\
	1000,\
	0,\
	0,\
	0,\
}\

typedef struct PID_Regulator_t
{
	float ref;
	float fdb;
	float err[2];
	float kp;
	float ki;
	float kd;
	float componentKp;
	float componentKi;
	float componentKd;
	float componentKpMax;
	float componentKiMax;
	float componentKdMax;
	float output;
	float outputMax;
	float kp_offset;
	float ki_offset;
	float kd_offset;
	float ec;
	float detkp;
	float detki;
	float detkd;
	float maxdetkp;
	float maxdetki;
	float maxdetkd;
	float error;
  int rule_kp[7][7];
	int rule_ki[7][7];
	int rule_kd[7][7];
	int Reset_i;
	float diedzone;
	
}PID_Regulator_t;

typedef struct
{
    int16_t forward_back_ref ;
    int16_t left_right_ref;
} XTL_Regulator;

void PID_Reset(PID_Regulator_t *pid);
void PID_Calc(PID_Regulator_t *pid);
#endif

