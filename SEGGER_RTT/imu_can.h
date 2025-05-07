#ifndef IMU_H
#define IMU_H
#include "main.h"
typedef struct{
	CAN_RxHeaderTypeDef header;
	char canData[6];
	unsigned int timeStamp;
}CanRxMsgTypeDef;


typedef struct{
	uint8_t data[6];
}IMU_RawMessage;

typedef struct{
	float rpy[3];
	float gyro[3];
	float acc[3];
}IMU_Information;

enum IMU_ID{IMU_1=1,IMU_2=2,IMU_3=3,IMU_4=4};
extern IMU_Information imu_info1, imu_info2, imu_info3, imu_info4;

void IMU_Callback(CAN_HandleTypeDef *hcan);
void getRaw_RPY(enum IMU_ID);
void getRaw_GyroXYZ(enum IMU_ID);
void getRaw_AccXYZ(enum IMU_ID);
void IMU_data_analysis(void);
float byte2float(uint8_t *byte_arr,float c);
#endif
