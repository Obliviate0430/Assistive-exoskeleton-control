#include "main.h"
#include "pid.h"
#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }



void PID_Calc(struct PID_Regulator_t *pid)
{
	pid->err[0] = pid->ref - pid->fdb;
	
	if((pid->err[0] >= pid->diedzone) || (pid->err[0] < -pid->diedzone))
	{
		pid->componentKp = pid->kp * pid->err[0];
		VAL_LIMIT(pid->componentKp,-pid->componentKpMax,pid->componentKpMax)
//		if((pid->err[0] * pid->err[1] < 0))
//		{
//			pid->ki = 0;
//		}
		pid->componentKi+= pid->ki * pid->err[0];
		VAL_LIMIT(pid->componentKi,-pid->componentKiMax,pid->componentKiMax)
		
		
		pid->componentKd = pid->kd * ( pid->err[0] - pid->err[1] );

		//pid->componentKd = pid->kd * ( pid->err[0] - pid->err[1] )*0.7+pid->componentKd*0.3;
		VAL_LIMIT(pid->componentKd,-pid->componentKdMax,pid->componentKdMax)
		pid->output=pid->componentKp + pid->componentKi + pid->componentKd;
	}
	else
	{
	}
	
	VAL_LIMIT(pid->output,-pid->outputMax,pid->outputMax)
	
	pid->err[1] = pid->err[0];
}

void PID_Reset(PID_Regulator_t *pid)
{
	pid->ref 		= 0;
	pid->fdb		= 0;
	
	pid->err[0] = 0;
	pid->err[1]	= 0;
	pid->componentKp = 0;
	pid->componentKi = 0;
	pid->componentKd = 0;
	pid->output			 = 0;
}
