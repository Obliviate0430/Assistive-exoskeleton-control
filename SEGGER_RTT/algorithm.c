#include "algorithm.h"
#include "imu_can.h"
#include "uart_debug.h"
#include "motor_can.h"
#include <stdio.h>
#include <string.h>
#include "skeleton.h"
#include "cJSON.h"
#include <stdlib.h>
#include "my_math.h"
#include "IterativeLearning.h"
#include "Insole.h"
#include "stdlib.h"																			   //引入随机数函数
#define ROUND_TO_UINT16(x) ((uint16_t)(x) + 0.5) > (x) ? ((uint16_t)(x)) : ((uint16_t)(x) + 1) // 将浮点数x四舍五入为uint16_t

//**********************************************************//
// 
#define SKELETON_AD_SAMPLE_FRAME 4
#define SKELETON_IMU_SAMPLE_FRAME 3

#define HeelStrike_Threhold 1000 // 鞋垫HeelStrike阈值
#define ToeOff_Threhold 1000	 // 鞋垫ToeOff阈值
float GRY_RIGHT_THRESHOLD = -30; //
float GRY_LEFT_THRESHOLD = 30;	 //
float ACC_THRESHOLD = 10;		 //
float GryOverflowTime = 80;		 //

float LorRFlag = 0;	 //
extern char receive; //
//**********************************************************//
//
int enableFlagLeft = 0;
int enableFlagRight = 0;
float GryZ_L[SKELETON_AD_SAMPLE_FRAME] = {0};
float GryZ_R[SKELETON_AD_SAMPLE_FRAME] = {0};
float AccY_L[SKELETON_IMU_SAMPLE_FRAME] = {0};
float AccY_R[SKELETON_IMU_SAMPLE_FRAME] = {0};
	
CycleTimerMeasureTypedef TimerLeft = {0, 0, 0, 0, 0, 0, 0, 0};
CycleTimerMeasureTypedef TimerRight = {0, 0, 0, 0, 0, 0, 0, 0};

// float StartTime = 20;	   // 助力开始时间
// float FirstPeakTime = 40;  // 第一次力峰值时间
// float SecondPeakTime = 50; // 第二次力峰值时间
// float EndTime = 60;		   // 助力结束时间

// 定义参数
float StartTime = 0;
float FirstPeakStart = 0;
float FirstPeakEnd = 0;
float FirstEndTime = 0;
float SecondStartTime = 35;
float SecondPeakStart = 45;
float SecondPeakEnd = 55;
float SecondEndTime = 65;

float Fp_first = 0;//峰值力
float Fp_second = 0.2;

float delt_t = 0.01; // 控制频率，10ms输出一次电机位置

ExoskeletonParaTypeDef Exo;		//
ForceControlTypedef LeftForce;	// 左侧力控参数
ForceControlTypedef RightForce; // 右侧力控参数

//**********************************************************//

extern int ZeroPoint;				// 电机零点
extern int offset_cnt, offset_cntR; // 左右侧偏移量，预紧位置

//**********************************************************//
//
const float PiR = 1.0472; // 4 * 3.14159 * 15mm /180deg =1.0472  4�ȶ�Ӧ�Ļ���
const float dt = 0.002;
const float PreloadForce = 0; // 预紧力
const float Max_vel = 40000;  // 40000/6/14=476，电机最大转速

float virtual_inertia = 0.01; // 虚拟质量 0.009
float virtual_damping = 0.2; // 虚拟阻尼 0.009
float virtual_stiffness = 2.4; // 虚拟刚度 0.15

float Fp_right = 100; // 峰值力
float Fp_left = 100;	 // 峰值力

float Fd_Left = 0, Fe_Left = 0; // 期望力和误差
float Fd_Right = 0, Fe_Right = 0;
uint32_t LtickCnt = 1, RtickCnt = 1; //

const float da = 60; // 踝关节转动半径

//**********************************************************//
//
void UpdataGryZBuffer(void) // 更新陀螺仪数据
{
	IMU_data_analysis();
	for (int i = 0; i < SKELETON_AD_SAMPLE_FRAME - 1; i++)
	{
		GryZ_L[i] = GryZ_L[i + 1];
		GryZ_R[i] = GryZ_R[i + 1];
	}
	for (int i = 0; i < SKELETON_IMU_SAMPLE_FRAME - 1; i++) // SKELETON_IMU_SAMPLE_FRAME   3
	{
		AccY_L[i] = AccY_L[i + 1];
		AccY_R[i] = AccY_R[i + 1];
	}
	GryZ_L[SKELETON_AD_SAMPLE_FRAME - 1] = imu_info1.gyro[2] * 180 / 3.1415926;  //SKELETON_AD_SAMPLE_FRAME    4
	GryZ_R[SKELETON_AD_SAMPLE_FRAME - 1] = imu_info2.gyro[2] * 180 / 3.1415926;  //[0]-Z axis ; [1]-Y axis  ;w[2]-X axis
	AccY_L[SKELETON_IMU_SAMPLE_FRAME - 1] = imu_info1.acc[1]; // [0]-Z axis ; [1]-Y axis  ;w[2]-X axis  //SKELETON_IMU_SAMPLE_FRAME   3
	AccY_R[SKELETON_IMU_SAMPLE_FRAME - 1] = imu_info2.acc[1];
}

//**********************************************************//
void UpdataPressureInsole(void) // 更新压力鞋垫数据
{
	//
	if (LeftInsole.ProcessFlag == 0) // uart8，接收到数据后再进行处理
	{
		ProcessInsoleData(&LeftInsole);						// 处理鞋垫数据
		memset(LeftInsole.Buffer, 0x00, INSOLEBUFFER_SIZE); // 清空缓存区
															// printf("1 \r\n"); // 计算采样频率
	}
	if (RightInsole.ProcessFlag == 0) // uart6
	{
		ProcessInsoleData(&RightInsole); // 处理鞋垫数据
		memset(RightInsole.Buffer, 0x00, INSOLEBUFFER_SIZE);
	}
}

//**********************************************************//
//
void AverageCycleTime(CycleTimerMeasureTypedef *Timer) // 步态周期均值滤波
{
	int AverageTime;
	int SumTime = 0;
	int count = 0;
	for (int i = 0; i < 3; i++)
	{
		if (Timer->CycletimeBuffer[i] > 500 && Timer->CycletimeBuffer[i] < 2000) // 只取有效值平均
		{
			SumTime += Timer->CycletimeBuffer[i];
			count++;
		}
	}
	AverageTime = (int)(SumTime / count);
	 if (AverageTime <= 300 || AverageTime >= 1500) // 步态周期异常时，修正步态周期
	 {AverageTime = 800;}
	// return AverageTime;
	Timer->Cycletime = AverageTime; // 计算平均步态周期
}

//**********************************************************//

void CycleTimerTickLeft(void)
{
   int HeelStrikeTick=SKELETON_IMU_SAMPLE_FRAME-2; //SKELETON_IMUGRY_SAMPLE_FRAME=4;SKELETON_IMU_SAMPLE_FRAME=3; HeelStrikeTick=1
  
   //static uint32_t Cycletick;
	 static uint32_t Grytickleft = 0;
	 static uint32_t GryLFlag = 0;
	 static float m_accy_L;
	 static float var_accy_L;	
	
	 m_accy_L=(AccY_L[HeelStrikeTick-1]+AccY_L[HeelStrikeTick]+AccY_L[HeelStrikeTick+1])*0.333333;
	 var_accy_L=((AccY_L[HeelStrikeTick-1]-m_accy_L)*(AccY_L[HeelStrikeTick-1]-m_accy_L)+(AccY_L[HeelStrikeTick]-m_accy_L)*(AccY_L[HeelStrikeTick]-m_accy_L)+(AccY_L[HeelStrikeTick+1]-m_accy_L)*(AccY_L[HeelStrikeTick+1]-m_accy_L))*0.333333;
	if( GryZ_L[HeelStrikeTick-1]>0 && GryZ_L[HeelStrikeTick]>0 && GryZ_L[HeelStrikeTick+1]<=0 && GryZ_L[HeelStrikeTick+2]<=0)
	{
		if(Grytickleft>160 )
		{
		  GryLFlag=1;
		  Grytickleft=0;
		}
	}
	Grytickleft++;
	
	if( var_accy_L>5 && GryLFlag==1)//40  15
   { 
 		enableFlagLeft=1;
		GryLFlag=0; 
//      if(Cycletick>150 && Cycletick<500 )  //Sample frequency = 400 Hz , so Cycletick is about 400+-200
//      {
//          		  enableFlagLeft=1;
//				 // TimerLeft.Cycletime=Cycletick;// Gait period calculation			  				  
//      }
//			Cycletick=0;
//					
   }
}

void CycleTimerTickRight(void)
{
   int HeelStrikeTick=SKELETON_IMU_SAMPLE_FRAME-2; //SKELETON_IMUGRY_SAMPLE_FRAME=4;SKELETON_IMU_SAMPLE_FRAME=3; HeelStrikeTick=1
   
   //static uint32_t Cycletick;
	 static uint32_t Grytickright = 0;
	 static uint32_t GryRFlag = 0;
	 static float m_accy_R;
	 static float var_accy_R;
	
  m_accy_R=(AccY_R[HeelStrikeTick-1]+AccY_R[HeelStrikeTick]+AccY_R[HeelStrikeTick+1])*0.333333;
	var_accy_R=((AccY_R[HeelStrikeTick-1]-m_accy_R)*(AccY_R[HeelStrikeTick-1]-m_accy_R)+(AccY_R[HeelStrikeTick]-m_accy_R)*(AccY_R[HeelStrikeTick]-m_accy_R)+(AccY_R[HeelStrikeTick+1]-m_accy_R)*(AccY_R[HeelStrikeTick+1]-m_accy_R))*0.333333;
	
  debugPrintPending("force:%d,%d,%f,%f,%f \n",  TimerRight.Cycletime,TimerRight.Tick,TimerRight.Phase,force_right,data[0].T);

	if( GryZ_R[HeelStrikeTick-1]<0 && GryZ_R[HeelStrikeTick]<0 && GryZ_R[HeelStrikeTick+1]>=0 && GryZ_R[HeelStrikeTick+2]>=0)
	{
		if(Grytickright>160 )
		{
			  GryRFlag=1;
		    Grytickright=0;
		}
	}
	Grytickright++;
	
   if( var_accy_R>5 && GryRFlag==1) //40  15
   { 
		 enableFlagRight=1;	
		 GryRFlag=0;
//      if(Cycletick>150 && Cycletick<500) 
//      {
//          enableFlagRight=1;			  
//          TimerRight.Cycletime=Cycletick; // Gait period calculation	 				  
//      }
//			Cycletick=0;
   }
}

//**********************************************************//



void MotorPositionGernerate(ForceControlTypedef *Controller, CycleTimerMeasureTypedef *Timer) // 
 {
// 	int Compensate_state = 1; // 导纳控制补偿状态,1为补偿，0为不补偿
// 	float Motor_position, Motor_vel;
// 	float Displacement; // 计算电机位移
// 	float Phase;
// 	float Compensate_position;										   // 导纳控制补偿位置
// 	Displacement = (float)ForceToDisplacement(Controller->Force_peak); // 计算峰值力对应的位移
// 	Phase = (float)Timer->Tick / (float)Timer->Cycletime * 100;		   // 计算步态相位
// 	Controller->Force_desire = ReferenceForce(Phase);				   // 生成三次样条参考力曲线
// 	if (Phase > 0 && Phase < StartTime)
// 	{
// 		Motor_position = Linear_interpolation(0, 0, StartTime, offset_cnt, Phase); // 线性插值电机位置
// 		// Motor_vel = Linear_Velocity_interpolation(0, 0, StartTime, offset_cnt); // 计算电机速度
// 		Motor_vel = 32767;
// 	}
// 	else if (Phase > StartTime && Phase < FirstPeakTime)
// 	{
// 		Motor_position = Linear_interpolation(StartTime, offset_cnt, FirstPeakTime, Displacement, Phase); // 线性插值电机位置
// 																										  // Motor_vel = Linear_Velocity_interpolation(StartTime, offset_cnt, FirstPeakTime, Displacement); // 计算电机速度
// 		Motor_vel = 32767;
// 	}
// 	else if (Phase > FirstPeakTime && Phase < SecondPeakTime)
// 	{
// 		Motor_position = Displacement;  //
// 		Motor_vel = 0;				    //
// 	}
// 	else if (Phase > SecondPeakTime && Phase < EndTime)
// 	{
// 		Motor_position = Linear_interpolation(SecondPeakTime, Displacement, EndTime, Displacement, Phase);
// 		Motor_position = Displacement;

// 		// Motor_vel = Linear_Velocity_interpolation(SecondPeakTime, Displacement, EndTime, Displacement-5); // 计算电机速度
// 		Motor_vel = -32767;
// 		Compensate_state = 0;
// 	}
// 	else if (Phase >= EndTime)
// 	{
// 		// Motor_position = Displacement-DegreeToDisplacement(10); //
// 		Motor_position = Displacement; //
// 		// Motor_vel = 0; // 计算电机速度
// 		Motor_vel = -32767;
// 		Compensate_state = 0;
// 		input = NoFlag;
// 	}
// 	Controller->Force_error = Controller->Force_desire - (float)skeletonAdDmaBuf[Controller->ForceSensorID] * (3.3 / 10) * 250 * 0.4536 * 9.8 / 4096; //
// 	// Controller->Force_error = 20;//

// 	// Controller->Force_error = 60 - abs(1*Controller->Motor_position)+rand()%20;//测试用，假设为弹簧模型
// 	if (Compensate_state == 1)
// 	{
// 		AddmitanceControl(Controller);//导纳补偿
// 		Compensate_position = Controller->Desired_pos;
// 		// Compensate_position = 0;
// 	}
// 	else
// 	{
// 		Compensate_position = 0;
// 		ClearControllerAddmitance(Controller); // 清空控制参数
// 	}
// 	// if (Motor_position == 0)
// 	// {
// 	// 	Motor_position = 0;
// 	// }

// 	Motor_position = DisplacementToDegree(Motor_position + Compensate_position); // 位移转换为电机角度
// 	// Motor_vel = DisplacementToDegree(Motor_vel);		   //位移速度转换为电机角速度
// 	// Motor_vel = 10000;
// 	Controller->Motor_position_last = Controller->Motor_position; // 记录上一时刻电机位置
// 	Controller->Motor_position = Motor_position * (Controller->Motor_direction);				  // 输出电机位置

// 	if ((Controller->Motor_position - Controller->Motor_position_last)*(Controller->Motor_direction) < 0)//防止反转
// 	{
// 		Controller->Motor_position = Controller->Motor_position_last;//电机位置不变
// 	}
// 	else
// 	{
// 		Controller->Motor_position = Motor_position * (Controller->Motor_direction);				  // 输出电机位置
// 	}
// 	// Controller->Motor_position = Motor_position * (Controller->Motor_direction);				  // 输出电机位置
// 	Controller->Motor_vel = Motor_vel * (Controller->Motor_direction);							  // 电机速度
}

//**********************************************************//
// 生成三次样条参考力曲线


float ReferenceForce(float Phase)
{


    if (Phase > StartTime && Phase < FirstPeakStart)
    {
        return cubic_interpolation(StartTime, 0, FirstPeakStart, Fp_first, Phase);
    }
    else if (Phase >= FirstPeakStart && Phase <= FirstPeakEnd)
    {
        return Fp_first;
    }
    else if (Phase > FirstPeakEnd && Phase < FirstEndTime)
    {
        return cubic_interpolation(FirstPeakEnd, Fp_first, FirstEndTime, 0, Phase);
    }
    else if (Phase > FirstEndTime && Phase < SecondStartTime)
    {
        return 0;
    }
    else if (Phase >= SecondStartTime && Phase < SecondPeakStart)
    {
        return cubic_interpolation(SecondStartTime, 0, SecondPeakStart, Fp_second, Phase);
    }
    else if (Phase >= SecondPeakStart && Phase <= SecondPeakEnd)
    {
        return Fp_second;
    }
    else if (Phase > SecondPeakEnd && Phase < SecondEndTime)
    {
        return cubic_interpolation(SecondPeakEnd, Fp_second, SecondEndTime, 0, Phase);
    }
    else
    {
        return 0;
    }
}



// float ReferenceForce(float Phase)
// {
// 	static float PeakTime = 50;
// 	if (Phase > StartTime && Phase < PeakTime)
// 	{
// 		return cubic_interpolation(StartTime, 0, PeakTime, Fp_left, Phase);
// 	}
// 	else if (Phase > PeakTime && Phase < EndTime)
// 	{
// 		return cubic_interpolation(PeakTime, Fp_left, EndTime, 0, Phase);
// 	}
// 	else
// 	{
// 		return 0;
// 	}
// }

void AddmitanceControl(ForceControlTypedef *Controller)//导纳控制器
{
	float desired_acc = 0, desired_vel = 0, desired_pos = 0;
	float Force_error;
	desired_acc = Controller->Desired_acc;
	desired_vel = Controller->Desired_vel;
	desired_pos = Controller->Desired_pos;
	Force_error = Controller->Force_error;
	if (Controller->Force_desire > 0 && Force_error > 10)
	{
		// Admittance control  (5N/mm)
		desired_acc = (1 / virtual_inertia) * (Force_error - virtual_damping * desired_vel - virtual_stiffness * desired_pos);
		desired_vel = desired_vel + desired_acc * dt;
		if (desired_vel >= Max_vel)
		{
			desired_vel = Max_vel;
		}
		if (desired_vel <= (-1) * Max_vel)
		{
			desired_vel = (-1) * Max_vel;
		}
		desired_pos = desired_pos + desired_vel * dt;
	}
	else
	{
		desired_acc = 0;
		desired_vel = 0;
		desired_pos = 0.5 * desired_pos;
	}
	Controller->Desired_acc = desired_acc;
	Controller->Desired_vel = desired_vel;
	Controller->Desired_pos = desired_pos; // 更新期望位置
}

void ClearControllerAddmitance(ForceControlTypedef *Controller)
{
	Controller->Desired_acc = 0;
	Controller->Desired_vel = 0;
	Controller->Desired_pos = 0;

}
void ClearControllerPos(ForceControlTypedef *Controller)
{
	Controller->Motor_position_last = 0;
	Controller->Motor_position = 0;
}

//**********************************************************//
//
// int skeletonAlgorithmLeft(void)
// {
// 	static float desired_acc = 0, desired_vel = 0;
// 	static float desired_pos = 0;
// 	static float pe_left = 0;
// 	static uint32_t displacement, cnt;										  //
// 	static uint32_t start = 20, peak1 = 35, peak2 = 50, peak3 = 40, end = 60; // result(unit-0.1ms)
// 	static float p_left = 0;
// 	static float ILC_left = 0; //
// 	static float Tick = 0;	   //-

// 	Tick = (float)LtickCnt / (float)TimerLeft.Cycletime * 100; //
// 	if (enableFlagLeft == 1)								   //
// 	{
// 		if (LastRightPos <= 10)
// 		{
// 			LastRightPos = 0;
// 		}
// 		RtickCnt = 0;
// 		enableFlagLeft = 2;
// 		displacement = ForceToDisplacement(Fp_left);
// 		// cnt = Round(displacement*360/(3.1415926*30));//
// 		cnt = DisplacementToDegree(displacement);
// 		p_left = ZeroPoint;
// 		;
// 	}
// 	if (Tick > 0 && Tick < start && enableFlagLeft == 2)
// 	{
// 		//		p_left = Linear_interpolation(0,-LastRightPos,start,offset_cnt,RtickCnt);
// 		p_left = Linear_interpolation(0, ZeroPoint, start, offset_cnt, Tick);
// 		//		p_left = offset_cnt;//
// 		Fd_Left = cubic_interpolation(0, 0, start, PreloadForce, Tick);
// 		//		Fd_Left = 0;
// 	}
// 	else if (Tick >= start && Tick < peak1 && enableFlagLeft == 2)
// 	{
// 		p_left = Linear_interpolation(start, offset_cnt, peak1, cnt + offset_cnt, Tick);
// 		// Fd_Left = cubic_interpolation(start,20,peak3,Fp_left,Tick);
// 		Fd_Left = cubic_interpolation(start, PreloadForce, peak3, Fp_left, Tick);
// 	}

// 	else if (Tick >= peak1 && Tick < peak2 && enableFlagLeft == 2)
// 	{
// 		p_left = cnt + offset_cnt;
// 		if (Tick < peak3)
// 		{
// 			Fd_Left = cubic_interpolation(start, PreloadForce, peak3, Fp_left, Tick);
// 		}
// 		else
// 		{
// 			Fd_Left = cubic_interpolation(peak3, Fp_left, end, 0, Tick);
// 		}
// 	}
// 	else if (Tick >= peak2 && Tick < end && enableFlagLeft == 2)
// 	{
// 		p_left = Linear_interpolation(peak2, cnt + offset_cnt, end, ZeroPoint, Tick);
// 		Fd_Left = cubic_interpolation(peak3, Fp_left, end, 0, Tick);
// 	}

// 	else if (Tick >= end && enableFlagLeft == 2)
// 	{
// 		p_left = ZeroPoint;
// 		Fd_Left = 0;
// 		Tick = 1;
// 		LtickCnt = 1;
// 		//			enableFlagLeft = 0;
// 		input = RightFlag; //
// 	}

// 	if (skeletonAdDmaBuf[0] <= 30)
// 	{
// 		skeletonAdDmaBuf[0] = 0;
// 	}																						 // ignore fluctuation
// 	Fe_Left = Fd_Left - (float)skeletonAdDmaBuf[0] * (3.3 / 10) * 250 * 0.4536 * 9.8 / 4096; // 0.4536Ӣ��תΪkg�ĵ�λ
// 	if (Fd_Left > 0)
// 	{
// 		// Admittance control  (5N/mm)
// 		desired_acc = (1 / virtual_inertia) * (Fe_Left - virtual_damping * desired_vel - virtual_stiffness * desired_pos);
// 		desired_vel = desired_vel + desired_acc * dt;
// 		if (desired_vel >= Max_vel)
// 		{
// 			desired_vel = Max_vel;
// 		}
// 		if (desired_vel <= (-1) * Max_vel)
// 		{
// 			desired_vel = (-1) * Max_vel;
// 		}
// 		desired_pos = desired_pos + desired_vel * dt;
// 	}
// 	else
// 	{
// 		desired_acc = 0;
// 		desired_vel = 0;
// 		desired_pos = 0.5 * desired_pos;
// 	}
// 	pe_left = DisplacementToDegree(desired_pos); //
// 	// ILC_left = OutputGain(Tick);//
// 	pe_left = 0; //
// 	// p_left=0;//
// 	LastLeftPos = p_left + pe_left; //

// 	return (LastLeftPos);
// }

//**********************************************************//
//
int force2motorposition(int p)
{
	int motor_pos;
	float coefficient = 3.0f;
	motor_pos = -Round(p / coefficient * 360 / (3.1415926 * 30));
	return motor_pos;
}

void json_pack(void) //
{
	static uint8_t number_data;
	cJSON *usr;
	char *data;

	usr = cJSON_CreateObject();																//
	cJSON_AddItemToObject(usr, "FpL", cJSON_CreateNumber(Fp_left));							//
	cJSON_AddItemToObject(usr, "FpR", cJSON_CreateNumber(Fp_right));						//
	cJSON_AddItemToObject(usr, "forceL", cJSON_CreateNumber(ROUND_TO_UINT16(force_left)));	//
	cJSON_AddItemToObject(usr, "forceR", cJSON_CreateNumber(ROUND_TO_UINT16(force_right))); //
	cJSON_AddItemToObject(usr, "Fd_Left", cJSON_CreateNumber(ROUND_TO_UINT16(Fd_Left)));	//
	cJSON_AddItemToObject(usr, "Fd_Right", cJSON_CreateNumber(ROUND_TO_UINT16(Fd_Right)));	//
	cJSON_AddItemToObject(usr, "Gait", cJSON_CreateNumber(50 * input));						//
	cJSON_AddItemToObject(usr, "DesPos", cJSON_CreateNumber(force_reference));
	cJSON_AddItemToObject(usr, "RealPos", cJSON_CreateNumber(motor_data.motor_pos));
	// cJSON_AddItemToObject(usr, "Fl", cJSON_CreateString("ok"));			 		//
	// data = cJSON_Print(usr);   //
	data = cJSON_PrintUnformatted(usr); //
	printf("%s\n", data);				// ͨ
	cJSON_Delete(usr);
	free(data);
}

void jsJSONReceive(cJSON *json, float *var) //
{
	//	if (json != NULL&&json->valuedouble != NULL) {
	if (json != NULL)
	{
		*var = json->valuedouble;
		// printf("var=%f",*var);
		//    printf("\r\n tst:%d", *var);
	}
	else
	{
		// printf("\r\nFailed to get value for 'var'.");
	}
}

void json_analysis(void) //
{

	cJSON *json, *json1, *json2, *json3, *json4, *json5, *json6, *json7, *json8, *json9, *json10;
	//	float tst;
	//    char* out="{\"one\":\"long\",\"two\":\"2\",\"three\":3}";

	json = cJSON_Parse(&receive); //
	if (json != NULL)
	{
		/***************************/
		// char *out_data = cJSON_Print(json);   //
		// printf("%s",out_data);
		//{"inertia":0.009,"damping":0.009,"stiffness":0.6}
		//{"GRYL":0.7,"GRYR":-0.7,"ACC":10,"Overflow":120}
		//		json1 = cJSON_GetObjectItem( json , "inertia" );  //
		//		json2 = cJSON_GetObjectItem( json , "damping" );//
		//		json3 = cJSON_GetObjectItem( json , "stiffness" );//
		//		json4 = cJSON_GetObjectItem( json , "GRYL" );
		//		json7 = cJSON_GetObjectItem( json , "GRYR" );
		//		json5 = cJSON_GetObjectItem( json , "ACC" );
		//		json6 = cJSON_GetObjectItem( json , "Overflow" );
		//		json8 = cJSON_GetObjectItem( json , "LorR" );
		json9 = cJSON_GetObjectItem(json, "Fp_right");
		json10 = cJSON_GetObjectItem(json, "Fp_left");
		//		jsJSONReceive(json1,&virtual_inertia);
		//		jsJSONReceive(json2,&virtual_damping);
		//		jsJSONReceive(json3,&virtual_stiffness);
		//		jsJSONReceive(json4, &GRY_LEFT_THRESHOLD);
		//       jsJSONReceive(json5, &ACC_THRESHOLD);
		//       jsJSONReceive(json6, &GryOverflowTime);
		//		jsJSONReceive(json7, &GRY_RIGHT_THRESHOLD);
		//		jsJSONReceive(json8, &LorRFlag);
		jsJSONReceive(json9, &Fp_right);
		jsJSONReceive(json10, &Fp_left);
		//		tst = json_two->valuedouble;
		//		printf("\r\ninertia=%f,damping=%f,stiffness=%f",virtual_inertia,virtual_damping,virtual_stiffness);
		cJSON_Delete(json); // 清空内存
	}
}
