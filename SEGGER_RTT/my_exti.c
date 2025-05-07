#include "my_exti.h"
#include "imu_can.h"
#include "motor_can.h"
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
//CAN通讯中断回调函数
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
//	if(hcan->Instance==hcan1.Instance)
//		Motor_Callback(hcan);
	if(hcan->Instance==hcan2.Instance)
		IMU_Callback(hcan);
}
