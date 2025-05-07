
#include "motor_can.h"
#include "main.h"
#include "adc.h"
#include "math.h"
#include "uart_debug.h"
#include "skeleton.h"
#include "algorithm.h"
#include "Vcan.h"
#include <stdio.h>

#define PI 3.14159265
extern CAN_HandleTypeDef hcan1;
//**********************************************************//
// 力传感器变量定义
volatile uint16_t skeletonAdDmaBuf[2] = {0}; // 力传感器反馈大小
float force_left = 0.0f;
float force_right = 0.0f;
float force_motor_left = 0.0f;
float force_motor_right = 0.0f;
//**********************************************************//
// 电机变量定义
CAN_RxHeaderTypeDef header_Motor;
uint8_t Motor_Controller_ID = 1;
uint8_t RxData_Motor[8] = {0};
motor_measure_t motor_data;
float Motor_RPM_right = 0.0f; // 电机逆时针转拉右脚
float Motor_RPM_left = 0.0f;  // 电机顺时针转拉左脚
float Motor_RPM_stop = 0.0f;  // 电机停止转动
uint8_t init_flag = 0;
uint8_t stiffness_test_flag = 0;
extern float init_pos_middle;
extern float init_pos_left;
int ZeroPoint = 3; // 设置零点未成功情况下，记录当前位置作为零点
int offset_cnt = 10, offset_cntR = 10;
//**********************************************************//

//****************************运控模式******************************//
void pack_cmd(float p_des, float v_des, float kp, float kd, float t_ff)
{
	/// limit data to be within bounds //
	uint8_t data[8];
	float P_MIN = -12.5f;
	float P_MAX = 12.5f;
	float V_MIN = -45.0f;
	float V_MAX = 45.0f;
	float T_MIN = -15.0f;
	float T_MAX = 15.0f;
	float Kp_MIN = 0;
	float Kp_MAX = 500.0f;
	float Kd_MIN = 0;
	float Kd_MAX = 5.0f;
	float Test_Pos = 0.0f;
	p_des = fminf(fmaxf(P_MIN, p_des), P_MAX);
	v_des = fminf(fmaxf(V_MIN, v_des), V_MAX);
	kp = fminf(fmaxf(Kp_MIN, kp), Kp_MAX);
	kd = fminf(fmaxf(Kd_MIN, kd), Kd_MAX);
	t_ff = fminf(fmaxf(T_MIN, t_ff), T_MAX);
	/// convert floats to unsigned ints ///
	int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
	int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
	int kp_int = float_to_uint(kp, Kp_MIN, Kp_MAX, 12);
	int kd_int = float_to_uint(kd, Kd_MIN, Kd_MAX, 12);
	int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);
	/// pack ints into the can buffer ///
	data[0] = p_int >> 8;							// 位置高 8
	data[1] = p_int & 0xFF;							// 位置低 8
	data[2] = v_int >> 4;							// 速度高 8 位
	data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8); // 速度低 4 位 KP 高 4 位
	data[4] = kp_int & 0xFF;						// KP 低 8 位
	data[5] = kd_int >> 4;							// Kd 高 8 位
	data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8); // KP 低 4 位扭矩高 4 位
	data[7] = t_int & 0xff;							// 扭矩低 8 位
	comm_can_transmit_STDid(Motor_Controller_ID, data, 8);
}
void Motor_Enable() // 使能电机，控制电机前需调用
{
	uint8_t data[8];
	data[0] = 0xff;
	data[1] = 0xff;
	data[2] = 0xff;
	data[3] = 0xff;
	data[4] = 0xff;
	data[5] = 0xff;
	data[6] = 0xff;
	data[7] = 0xfc;
	comm_can_transmit_STDid(Motor_Controller_ID, data, 8);
}
void Motor_Disable() // 失能电机
{
	uint8_t data[8];
	data[0] = 0xff;
	data[1] = 0xff;
	data[2] = 0xff;
	data[3] = 0xff;
	data[4] = 0xff;
	data[5] = 0xff;
	data[6] = 0xff;
	data[7] = 0xfd;
	comm_can_transmit_STDid(Motor_Controller_ID, data, 8);
}
// 运控模式下浮点型转换为无符号整型
int float_to_uint(float x, float x_min, float x_max, unsigned int bits)
{
	/// Converts a float to an unsigned int, given range and number of bits ///
	float span = x_max - x_min;
	if (x < x_min)
		x = x_min;
	else if (x > x_max)
		x = x_max;
	return (int)((x - x_min) * ((float)((1 << bits) / span)));
}
// 运控模式下解析电机数据
void unpack_reply(uint8_t RxData_Motor[], float *motor_pos, float *motor_spd, float *motor_cur, float *Temperature)
{
	/// unpack ints from can buffer ///
	float P_MIN = -12.5f;
	float P_MAX = 12.5f;
	float V_MIN = -45.0f;
	float V_MAX = 45.0f;
	float I_MAX = 15.0f;
	int id = RxData_Motor[0];									  // 驱动 ID 号
	int p_int = (RxData_Motor[1] << 8) | RxData_Motor[2];		  // 电机位置数据
	int v_int = (RxData_Motor[3] << 4) | (RxData_Motor[4] >> 4);  // 电机速度数据
	int i_int = ((RxData_Motor[4] & 0xF) << 8) | RxData_Motor[5]; // 电机扭矩数据
	int T_int = RxData_Motor[6];
	/// convert ints to floats ///
	float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
	float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
	float i = uint_to_float(i_int, -I_MAX, I_MAX, 12);
	float T = T_int;
	if (id == 1)
	{
		*motor_pos = p; // 根据 ID 号读取对应数据
		*motor_spd = v;
		*motor_cur = i;
		*Temperature = T - 40; // 温度范围-40~215
	}
}
// 运控模式转换浮点型
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/// converts unsigned int to float, given range and number of bits ///
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}
// 运控模式发送CAN信息
void comm_can_transmit_STDid(uint32_t id, const uint8_t *data, uint8_t len)
{ // 电机CAN信息发送函数
	uint32_t send_mail_box;
	uint8_t i = 0;
	uint8_t tx_data[8];
	if (len > 8)
	{
		len = 8;
	}
	CAN_TxHeaderTypeDef TXHeader;
	TXHeader.StdId = 1;
	TXHeader.IDE = CAN_ID_STD; // CAN_ID_STD
	TXHeader.ExtId = id;
	TXHeader.RTR = CAN_RTR_DATA;
	TXHeader.DLC = len;
	for (i = 0; i < len; i++)
		tx_data[i] = data[i];
	HAL_CAN_AddTxMessage(&hcan1, &TXHeader, tx_data, &send_mail_box);
}
//**********************伺服模式************************************//

// 电机函数
void buffer_append_int32(uint8_t *buffer, int32_t number, int32_t *index)
{ // 电机官方打包程序
	buffer[(*index)++] = number >> 24;
	buffer[(*index)++] = number >> 16;
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

void buffer_append_int16(uint8_t *buffer, int16_t number, int16_t *index)
{
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}
void comm_can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len)
{ // 电机CAN信息发送函数
	uint32_t send_mail_box;
	uint8_t i = 0;
	uint8_t tx_data[8];
	if (len > 8)
	{
		len = 8;
	}
	CAN_TxHeaderTypeDef TXHeader;
	TXHeader.StdId = 0;
	TXHeader.IDE = CAN_ID_EXT; // CAN_ID_STD
	TXHeader.ExtId = id;
	TXHeader.RTR = CAN_RTR_DATA;
	TXHeader.DLC = len;
	for (i = 0; i < len; i++)
		tx_data[i] = data[i];
	HAL_CAN_AddTxMessage(&hcan1, &TXHeader, tx_data, &send_mail_box);
}

void Motor_Callback(CAN_HandleTypeDef *hcan)
{ // CAN通讯下电机原始状态接受
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header_Motor, RxData_Motor) != HAL_OK)
	{
		Error_Handler();
	}
	__HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void comm_can_set_current(uint8_t controller_id, float current)
{ // 伺服模式下电流环模式

	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(current * 1000.0f), &send_index);
	comm_can_transmit_eid(controller_id |
							  ((uint32_t)CAN_PACKET_SET_CURRENT << 8),
						  buffer, send_index);
}

void comm_can_set_rpm(uint8_t controller_id, float rpm)
{ // 伺服模式下速度环模式
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)rpm, &send_index);
	comm_can_transmit_eid(controller_id |
							  ((uint32_t)CAN_PACKET_SET_RPM << 8),
						  buffer, send_index);
}

void comm_can_set_pos(uint8_t controller_id, float pos)
{ // 伺服模式下位置环模式
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(pos * 10000.0), &send_index);
	comm_can_transmit_eid(controller_id |
							  ((uint32_t)CAN_PACKET_SET_POS << 8),
						  buffer, send_index);
}

void comm_can_set_pos_spd(uint8_t controller_id, float pos, int16_t spd, int16_t RPA)
{
	int32_t send_index = 0;
	int16_t send_index1 = 4;
	uint8_t buffer[8];
	buffer_append_int32(buffer, (int32_t)(pos * 10000.0), &send_index);
	buffer_append_int16(buffer, spd / 10.0, &send_index1);
	buffer_append_int16(buffer, RPA / 10.0, &send_index1);
	comm_can_transmit_eid(controller_id |
							  ((uint32_t)CAN_PACKET_SET_POS_SPD << 8),
						  buffer, send_index1);
}

void comm_can_set_origin(uint8_t controller_id, uint8_t set_origin_mode)
{ // 伺服模式下设置原点模式
	int32_t send_index = 1;
	uint8_t buffer;
	buffer = set_origin_mode;
	comm_can_transmit_eid(controller_id |
							  ((uint32_t)CAN_PACKET_SET_ORIGIN_HERE << 8),
						  &buffer, send_index);
}

void motor_receive(float *motor_pos, float *motor_spd, float *motor_cur, uint8_t Data[])
{ // 处理电机原始数据
	int16_t pos_int = Data[0] << 8 | Data[1];
	int16_t spd_int = Data[2] << 8 | Data[3];
	int16_t cur_int = Data[4] << 8 | Data[5];

	//	wave_form_data[0] = pos_int;
	//	shanwai_send_wave_form();

	*motor_pos = (float)(pos_int * 0.1f);  // 电机位置/单位度
	*motor_spd = (float)(spd_int * 10.0f); // 电机速度/单位rpm
	*motor_cur = (float)(cur_int * 0.01f); // 电机电流/单位A
										   //		*motor_temp= Data[6] ;//电机温度
										   //		*motor_error= Data[7] ;//电机故障码
}
//**********************************************************//
// 电机实际位置、速度
int motorrealposition(void)
{
	float motor_real_position;
	motor_receive(&motor_data.motor_pos, &motor_data.motor_spd, &motor_data.motor_cur, RxData_Motor);
	motor_real_position = Round(motor_data.motor_pos);
	return motor_real_position;
}

int motorrealvelocity(void)
{
	float motor_real_velocity;
	motor_receive(&motor_data.motor_pos, &motor_data.motor_spd, &motor_data.motor_cur, RxData_Motor);
	motor_real_velocity = Round(motor_data.motor_spd);
	return motor_real_velocity;
}
//**********************************************************//
// 初始化电机零点
void initMotorZeroPoint(void)
{
	static int i = 0;
	force_left = (float)skeletonAdDmaBuf[0] * (3.3f / 10.0f) * 250 * 0.4536f * 9.8f / 4096.0f;
	force_right = (float)skeletonAdDmaBuf[1] * (3.3f / 10.0f) * 250 * 0.4536f * 9.8f / 4096.0f;
	//	debugPrintPending("force_left=%f force_right=%f \r\n",force_left,force_right);
	Motor_RPM_right = 1500.0f; // 电机逆时针转拉右脚
	Motor_RPM_left = -1500.0f; // 电机顺时针转拉左脚
	//	float init_pos_left,init_pos_right,init_pos_middle;
	if (init_flag == 0)
	{
		comm_can_set_rpm(Motor_Controller_ID, Motor_RPM_right);
	}
	if ((fabs(force_right - 10.0f) < 1) && (init_flag == 0)) // 30N阈值
	{
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET); // LED_R
		motor_receive(&motor_data.motor_pos, &motor_data.motor_spd, &motor_data.motor_cur, RxData_Motor);
		offset_cntR = motor_data.motor_pos;
		init_flag = 1;
	}
	if (init_flag == 1)
	{
		comm_can_set_rpm(Motor_Controller_ID, Motor_RPM_left);
	}
	//	comm_can_set_pos(Motor_Controller_ID, (float)(i));
	//	i++;
	if ((fabs(force_left - 10.0f) < 1) && (init_flag == 1)) // 30N阈值
	{
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_RESET); // LED_G
		motor_receive(&motor_data.motor_pos, &motor_data.motor_spd, &motor_data.motor_cur, RxData_Motor);
		init_pos_left = motor_data.motor_pos;
		init_flag = 2;
		comm_can_set_rpm(Motor_Controller_ID, Motor_RPM_stop);
		//		init_pos_middle=init_pos_left;
		//		comm_can_set_pos(Motor_Controller_ID,init_pos_middle);//位置模式下转到中间

		//		comm_can_set_origin(Motor_Controller_ID, 1);//设置此时位置为原点

		offset_cnt = init_pos_left;
		ZeroPoint = (offset_cnt + offset_cntR) / 2;
		//		offset_cntR = 0;
		//		HAL_Delay(1000);
		comm_can_set_pos(Motor_Controller_ID, ZeroPoint);
		HAL_Delay(50);
		Skeleton_State = SKELETON_WORKING;
	}
	HAL_Delay(5);
}

void initMotorZeroPointSingleSide(void) // 只对一侧初始化，仅限于鲍登线断的时候
{
	static int i = 0;
	force_left = (float)skeletonAdDmaBuf[0] * (3.3f / 10.0f) * 250 * 0.4536f * 9.8f / 4096.0f;
	force_right = (float)skeletonAdDmaBuf[1] * (3.3f / 10.0f) * 250 * 0.4536f * 9.8f / 4096.0f;
	//	debugPrintPending("force_left=%f force_right=%f \r\n",force_left,force_right);
	Motor_RPM_right = 1500.0f; // 电机逆时针转拉右脚
	Motor_RPM_left = -1500.0f; // 电机顺时针转拉左脚
	//	float init_pos_left,init_pos_right,init_pos_middle;

	if (init_flag == 0)
	{
		comm_can_set_rpm(Motor_Controller_ID, Motor_RPM_right);
	}

	if ((fabs(force_right - 10.0f) < 1) && (init_flag == 0)) // 30N阈值
	{
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_RESET); // LED_G
		motor_receive(&motor_data.motor_pos, &motor_data.motor_spd, &motor_data.motor_cur, RxData_Motor);
		offset_cntR = motor_data.motor_pos;
		init_flag = 2;
		comm_can_set_rpm(Motor_Controller_ID, Motor_RPM_stop);

		ZeroPoint = (offset_cntR) / 2;

		comm_can_set_pos(Motor_Controller_ID, 0);
		HAL_Delay(50);
		Skeleton_State = SKELETON_WORKING;
	}
	HAL_Delay(5);
}

//**********************************************************//
// 释放鲍登线
void ReleaseCable(int Direction)
{
	comm_can_set_pos(Motor_Controller_ID, ZeroPoint - Direction * SWITCH_THRESHOLD);
}
//**********************************************************//
// 初始化预紧，左右各转动一定角度
void PreLoad(void)
{
	comm_can_set_pos(Motor_Controller_ID, 3 * SWITCH_THRESHOLD);
	HAL_Delay(1000);
	comm_can_set_pos(Motor_Controller_ID, -3 * SWITCH_THRESHOLD);
	HAL_Delay(1000);
	comm_can_set_pos(Motor_Controller_ID, 0);
	motor_receive(&motor_data.motor_pos, &motor_data.motor_spd, &motor_data.motor_cur, RxData_Motor);
	HAL_Delay(500);
	// ZeroPoint = motor_data.motor_pos;
}
//**********************************************************//
// 系统刚度测试

void system_stiffness_test(void)
{
	static float pos_stiffness;
	int Diameter = 62;		   // 绕线轮直径，单位mm
	Motor_RPM_right = 1500.0f; // 电机逆时针转拉右脚
	Motor_RPM_left = -500.0f;  // 电机逆时针转拉左脚
	force_right = (float)skeletonAdDmaBuf[1] * (3.3f / 10.0f) * 250 * 0.4536f * 9.8f / 4096.0f;
	force_left = (float)skeletonAdDmaBuf[0] * (3.3 / 10) * 250 * 0.4536 * 9.8 / 4096;

	if ((force_right < 100.0f) && (stiffness_test_flag == 0))
	{
		comm_can_set_rpm(Motor_Controller_ID, Motor_RPM_right);
		motor_receive(&motor_data.motor_pos, &motor_data.motor_spd, &motor_data.motor_cur, RxData_Motor);
		pos_stiffness = motor_data.motor_pos / 360.0f * PI * Diameter; // 单位由度转化为mm
		//debugPrintPending("%f;%f \r\n", force_right, pos_stiffness);
		HAL_Delay(5);
	}

	//	if(fabs(force_left-100.0f)<0.3) stiffness_test_flag=1;
	//	if((force_left<100.0f)&&(stiffness_test_flag==1))
	//	{
	//		comm_can_set_rpm(Motor_Controller_ID, Motor_RPM_left );
	//		motor_receive(&motor_data.motor_pos,&motor_data.motor_spd,&motor_data.motor_cur,RxData_Motor);
	//		pos_stiffness=motor_data.motor_pos/360.0f*2*PI*15;//单位由度转化为mm
	//		debugPrintPending("force_right=%f pos_stiffness=%f \r\n",force_right,pos_stiffness);
	//		if(fabs(pos_stiffness-0.0f)<1)
	//		{
	//			comm_can_set_rpm(Motor_Controller_ID, Motor_RPM_stop);
	//			stiffness_test_flag=2;
	//		}
	//	}
}

//**********************************************************//
// 输出正弦曲线.amplitude:振幅，frequency:频率，duration:持续时间
void outputSineWave(float amplitude, double frequency, int t)
{
	float sample = t / 1000; // 采样频率1000Hz
	float velocity = 0;
	OutPutPos = (int)(amplitude * sin(2 * PI * frequency * (double)t / 1000.0));
	velocity = amplitude * cos(2 * PI * frequency * t / 1000.0);
//	comm_can_set_pos_spd(Motor_Controller_ID, (float)(OutPutPos), 1000*velocity, 30000);
	comm_can_set_pos(Motor_Controller_ID,(float)OutPutPos);
}

//**********************************************************//
// 力传感器
void initSkeletonAdBuf(void)
{
	if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&skeletonAdDmaBuf, 2) != HAL_OK)
	{
		Error_Handler();
	}
}
