/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             ������CAN�жϽ��պ��������յ������,CAN���ͺ������͵���������Ƶ��.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef MOTOR_CAN_H
#define MOTOR_CAN_H

#include "struct_typedef.h"
#include "main.h"
//**********************************************************//			
#define PI 3.141592
#define SWITCH_THRESHOLD 5 //左右助力切换阈值，单位：度
//**********************************************************//	
typedef enum {
	CAN_PACKET_SET_DUTY = 0,  //
	CAN_PACKET_SET_CURRENT,   //
	CAN_PACKET_SET_CURRENT_BRAKE,  //
	CAN_PACKET_SET_RPM,            //
	CAN_PACKET_SET_POS,            //
	CAN_PACKET_SET_ORIGIN_HERE,    //
	CAN_PACKET_SET_POS_SPD         //
} CAN_PACKET_ID;
//Tmotorak60-6 motor data
typedef struct
{
	float motor_pos;
	float motor_spd;
	float motor_cur;
	float motor_temp;
	uint8_t motor_error;
} motor_measure_t;

//**********************************************************//		
//
extern motor_measure_t motor_data;
extern uint8_t RxData_Motor[8];
extern uint8_t Motor_Controller_ID;
extern uint8_t init_flag;
extern uint8_t stiffness_test_flag;
extern float Motor_RPM_right;
extern float Motor_RPM_left;
extern float Motor_RPM_stop;
extern int motorrealposition(void);
extern int motorrealvelocity(void);

extern void comm_can_set_current(uint8_t controller_id, float current);
extern void comm_can_set_rpm(uint8_t controller_id, float rpm);
extern void comm_can_set_pos(uint8_t controller_id, float pos);
extern void comm_can_set_pos_spd(uint8_t controller_id, float pos,int16_t spd, int16_t RPA);
extern void comm_can_set_origin(uint8_t controller_id, uint8_t set_origin_mode);
extern void motor_receive(float* motor_pos,float* motor_spd,float* motor_cur,uint8_t Data[]);
extern void initMotorZeroPoint(void);
extern void initSkeletonAdBuf(void);
extern void initMotorZeroPointSingleSide(void);
extern void system_stiffness_test(void);
extern void PreLoad(void);//初始化预紧
extern void ReleaseCable(int Direction);
//**********************************************************//		
extern volatile uint16_t skeletonAdDmaBuf[2];//拉力传感器AD采集数据
extern float force_left;
extern float force_right;
extern float force_motor_left;
extern float force_motor_right;

extern int ZeroPoint;
extern void  initSkeletonAdBuf(void);
extern void outputSineWave(float amplitude, double frequency, int t);
extern void pack_cmd(float p_des, float v_des, float kp, float kd, float t_ff);
extern int float_to_uint(float x, float x_min, float x_max, unsigned int bits);
extern float uint_to_float(int x_int, float x_min, float x_max, int bits);
extern void comm_can_transmit_STDid(uint32_t id, const uint8_t *data, uint8_t len);
extern void unpack_reply(uint8_t RxData_Motor[],float *motor_pos, float *motor_spd, float *motor_cur, float *Temperature);
extern void Motor_Enable();
extern void Motor_Disable();
#endif
