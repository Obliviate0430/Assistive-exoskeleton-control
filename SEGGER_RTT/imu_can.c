#include "imu_can.h"
#include "string.h"
#include "math.h"
CAN_RxHeaderTypeDef canRxHeader;
CanRxMsgTypeDef CanRxMsg;

IMU_RawMessage RPY_msg1,RPY_msg2,RPY_msg3,RPY_msg4;
IMU_RawMessage GyroXYZ_msg1,GyroXYZ_msg2,GyroXYZ_msg3,GyroXYZ_msg4;
IMU_RawMessage AccXYZ_msg1,AccXYZ_msg2,AccXYZ_msg3,AccXYZ_msg4;


IMU_Information imu_info1,imu_info2,imu_info3,imu_info4;
static uint8_t RxData[6]={0};

float RPY_c = 0.0078;
float Gyro_c = 0.002;
float Acc_c = 0.0039;



void IMU_Callback(CAN_HandleTypeDef *hcan){
	if(HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&(CanRxMsg.header),RxData)!=HAL_OK){
		Error_Handler();
	}
    switch(CanRxMsg.header.StdId){
		case 0x9: getRaw_RPY(IMU_1);break;    //欧拉角
		case 0x13: getRaw_RPY(IMU_2);break;
		case 0x11: getRaw_GyroXYZ(IMU_1);break;		     //角速度	
		case 0x14: getRaw_GyroXYZ(IMU_2);break;
		case 0x12: getRaw_AccXYZ(IMU_1);break;     //free加速度
		case 0x15: getRaw_AccXYZ(IMU_2);break;
//			
//		case 0x11: getRaw_RPY(IMU_1);break;
//		case 0x21: getRaw_RPY(IMU_2);break;
//		case 0x31: getRaw_RPY(IMU_3);break;
//		case 0x41: getRaw_RPY(IMU_4);break;
//		case 0x12: getRaw_GyroXYZ(IMU_1);break;
//		case 0x22: getRaw_GyroXYZ(IMU_2);break;
//		case 0x32: getRaw_GyroXYZ(IMU_3);break;
//		case 0x42: getRaw_GyroXYZ(IMU_4);break;
//		case 0x13: getRaw_AccXYZ(IMU_1);break;
//		case 0x23: getRaw_AccXYZ(IMU_2);break;
//		case 0x33: getRaw_AccXYZ(IMU_3);break;
//		case 0x43: getRaw_AccXYZ(IMU_4);break;
		default: Error_Handler();
	}
  __HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}


void getRaw_RPY(enum IMU_ID id){
	switch(id){
		case IMU_1: memcpy(RPY_msg1.data,RxData,6); break;
		case IMU_2: memcpy(RPY_msg2.data,RxData,6); break;
		case IMU_3: memcpy(RPY_msg3.data,RxData,6); break;
		case IMU_4: memcpy(RPY_msg4.data,RxData,6); break;
		default: Error_Handler();break;
	}
}

void getRaw_GyroXYZ(enum IMU_ID id){
	switch(id){
		case IMU_1: memcpy(GyroXYZ_msg1.data,RxData,6); break;
		case IMU_2: memcpy(GyroXYZ_msg2.data,RxData,6); break;
		case IMU_3: memcpy(GyroXYZ_msg3.data,RxData,6); break;
		case IMU_4: memcpy(GyroXYZ_msg4.data,RxData,6); break;
		default: Error_Handler();break;
	}
}

void getRaw_AccXYZ(enum IMU_ID id){
	switch(id){
		case IMU_1: memcpy(AccXYZ_msg1.data,RxData,6); break;
		case IMU_2: memcpy(AccXYZ_msg2.data,RxData,6); break;
		case IMU_3: memcpy(AccXYZ_msg3.data,RxData,6); break;
		case IMU_4: memcpy(AccXYZ_msg4.data,RxData,6); break;
		default: Error_Handler();break;
	}
}

void IMU_data_analysis(void){
	uint8_t b2_rpy[2],b2_gyro[2],b2_acc[2];
	for(int i=0;i<3;i++){
		for(int j=0;j<2;j++){
			b2_rpy[j] = RPY_msg1.data[i*2+j];
			b2_gyro[j] = GyroXYZ_msg1.data[i*2+j];
			b2_acc[j] = AccXYZ_msg1.data[i*2+j];
		}
		imu_info1.rpy[i]=byte2float(b2_rpy,RPY_c);
		imu_info1.gyro[i]=byte2float(b2_gyro,Gyro_c);
		imu_info1.acc[i]=byte2float(b2_acc,Acc_c);
	}
	for(int i=0;i<3;i++){
		for(int j=0;j<2;j++){
			b2_rpy[j] = RPY_msg2.data[i*2+j];
			b2_gyro[j] = GyroXYZ_msg2.data[i*2+j];
			b2_acc[j] = AccXYZ_msg2.data[i*2+j];
		}
		imu_info2.rpy[i]=byte2float(b2_rpy,RPY_c);
		imu_info2.gyro[i]=byte2float(b2_gyro,Gyro_c);
		imu_info2.acc[i]=byte2float(b2_acc,Acc_c);
	}
	for(int i=0;i<3;i++){
		for(int j=0;j<2;j++){
			b2_rpy[j] = RPY_msg3.data[i*2+j];
			b2_gyro[j] = GyroXYZ_msg3.data[i*2+j];
			b2_acc[j] = AccXYZ_msg3.data[i*2+j];
		}
		imu_info3.rpy[i]=byte2float(b2_rpy,RPY_c);
		imu_info3.gyro[i]=byte2float(b2_gyro,Gyro_c);
		imu_info3.acc[i]=byte2float(b2_acc,Acc_c);
	}
	for(int i=0;i<3;i++){
		for(int j=0;j<2;j++){
			b2_rpy[j] = RPY_msg4.data[i*2+j];
			b2_gyro[j] = GyroXYZ_msg4.data[i*2+j];
			b2_acc[j] = AccXYZ_msg4.data[i*2+j];
		}
		imu_info4.rpy[i]=byte2float(b2_rpy,RPY_c);
		imu_info4.gyro[i]=byte2float(b2_gyro,Gyro_c);
		imu_info4.acc[i]=byte2float(b2_acc,Acc_c);
	}
}


float byte2float(uint8_t* byte_arr,float c){
	uint8_t sign = byte_arr[0]>>7;
	uint16_t b;
	if( sign == 1 ){
		b = 32768 - (((byte_arr[0] & 0x7f) << 8) | byte_arr[1]);
	}else if( sign == 0 ){
		b = ((byte_arr[0] & 0x7f) << 8) | byte_arr[1];
	}
	return (float)pow(-1,sign)*b*c;
}
