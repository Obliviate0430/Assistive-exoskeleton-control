/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "adc.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include "imu_can.h"
#include "motor_can.h"
#include "struct_typedef.h"
#include "uart_debug.h"
#include "my_exti.h"
#include "skeleton.h"
#include "motor_control.h"
#include "buzzer.h"
#include "SEGGER_RTT.h"
#include <math.h>
#include "algorithm.h"
//#include "rt_nonfinite.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum{
	EXAMPLE_BUZZER_START,
	EXAMPLE_BUZZER_STOP,
	EXAMPLE_BUZZER_RINGTONE,
	EXAMPLE_BUZZER_NO_LOOP,
	EXAMPLE_BUZZER_LOOP
}example_buzzer_e;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define QUEUE_SIZE 60


#define RXBUFFERSIZE 256
 MOTOR_send cmd[2];   //
 MOTOR_recv data[2];
#define USB_MAX_RECEIVE_LEN 64
uint8_t rxData[USB_MAX_RECEIVE_LEN] = {0};
uint32_t rxLen = 0;
char receive[RXBUFFERSIZE];
#define PWM_TIM		&htim12
#define PWM_CHN		TIM_CHANNEL_1
#ifndef USE_STATIC_MEM_ALLOCATION
buzzer_t mBuzzer;
#endif
buzzer_t *Buzzer;
example_buzzer_e example = EXAMPLE_BUZZER_STOP;
uint8_t nextPattern;

extern volatile float VELX[IMU_SAMPLE_FRAME], ANGX[IMU_SAMPLE_FRAME];
static uint32_t timer1_count;
static uint32_t gait50_count[GAIT_SAMPLE_FRAME];
static uint32_t gait85_count[GAIT_SAMPLE_FRAME];
static uint32_t GAIT_PERIOD;

uint8_t uart6Data = 0;
uint8_t ucRxBuffer6[30] = {0};
static int LED_ON[8];


//========================//
//double val_KW_set = 0.02;  //walk
float val_KW_set = 0.015;   //
float val_KW_down = 0.0008;
double val_W_set = 4;        //w = value*3.14*6.33
double val_W_down = 8;
int interp_L_set_index = 0;
int interp_R_set_index = 0;
int interp_L_down_index = 0;
int interp_R_down_index = 0;
float value = 0.0;
int interpolation_count = 29;
float interpolation_set_output[29];
float interpolation_down_output[29];
int interpolation_length;
int key_state_L=1;
int key_state_R=1;
int key_state_L_rising=0;
int key_state_L_falling=0;
int key_state_R_rising=0;
int key_state_R_falling=0;
//================================//

struct SAcc
{
	short a[3];
	short T;
};
struct SGyro
{
	short w[3];
	short T;
};
struct SAngle
{
	short Angle[3];
	short T;
};
void CopeSerial2Data_left(unsigned char ucData); //IMU
void CopeSerial2Data_right(unsigned char ucData);//IMU
void CopeSerial2Data_back(unsigned char ucData); //IMU
void sendcmd(uint8_t data[3]);
void get_data_left();
void get_data_right();
void get_data_back();
uint8_t rx_buff1; //IMUĴ
uint8_t rx_buff2; //IMUĴ
uint8_t rx_buff3; //IMUĴ
struct SAcc ACC_left;
struct SAngle Angle_left;
struct SGyro Gyro_left;
struct SAcc ACC_right;
struct SAngle Angle_right;
struct SGyro Gyro_right;
struct SAcc ACC_back;
struct SAngle Angle_back;
struct SGyro Gyro_back;
char YAWCMD[3] = {0XFF,0XAA,0X52};
char ACCCMD[3] = {0XFF,0XAA,0X67};
char SLEEPCMD[3] = {0XFF,0XAA,0X60};
char UARTMODECMD[3] = {0XFF,0XAA,0X61};
char IICMODECMD[3] = {0XFF,0XAA,0X62};



// button
int button_value_1 = 0;
int button_value_2 = 0;

int nub = 0;//

//Ԥ
//double dv[18] = {-74.2566, -3.1805, 0.2656, -0.6792, -1.9531, 26.5502, -3.0761, -13.3154, -0.2646, -0.0556, -21.0571, -62.0117, -2.7246, -24.6918, -0.0698, -0.0903, 4.1503, -116.638};
float fv[10];
double label;

// 廷ζнṹ
struct CircularQueue
{
	float items[QUEUE_SIZE];
	int head;
	int tail;
};

struct CircularQueue angle_L_queue;
struct CircularQueue angle_R_queue;
struct CircularQueue angle_diff_queue;

// ʼ
void initQueue(struct CircularQueue* q)
{
	// ԪΪ0
	for (int i = 0; i < QUEUE_SIZE; i++)
	{
		q->items[i] = 0.0;
	}
	q->head = 0;
	q->tail = 0;
}

// Ӳ
void enqueue(struct CircularQueue* q, float item) 
{
    // ж϶Ƿ
    if ((q->tail + 1) % QUEUE_SIZE == q->head) {
//        printf("Queue is full. Dequeuing the last item to enqueue a new one.\n");
        // Զ
        q->head = (q->head + 1) % QUEUE_SIZE;
    }

    // Ԫصβ
    q->items[q->tail] = item;
    // ƶβָ
    q->tail = (q->tail + 1) % QUEUE_SIZE;
}

// Ӳ
int dequeue(struct CircularQueue* q) 
{
    // ж϶ǷΪ
    if (q->head == q->tail) {
//        printf("Queue is empty.\n");
        return -1; // -1ʾΪ
    }

    // 
    int item = q->items[q->head];
    // 
    q->items[q->head] = 0;
    // 
    q->head = (q->head + 1) % QUEUE_SIZE;

    return item;
}

// 
void printQueue(struct CircularQueue* q) 
{
    printf("Queue: ");
    int index = q->head;
    while (index != q->tail) {
        printf("%f ", q->items[index]);
        index = (index + 1) % QUEUE_SIZE;
    }
    printf("\n");
}


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);






void gait_update_period(){
	
	GAIT_PERIOD = (gait50_count[2] - gait50_count[0])/2;
}

/*********************************************************************************************/

/*********************************************************************************************/
void cubicSplineInterpolation(float start, float end, int count, float* output)
{
	float t;
	float step = 1.0 / (count - 1);

	for (int i = 0; i < count; i++)
	{
		t = i * step;

		float a = 2 * pow(t, 3) - 3 * pow(t, 2) + 1;
		float b = -2 * pow(t, 3) + 3 * pow(t, 2);
		float c = pow(t, 3) - 2 * pow(t, 2) + t;
		float d = pow(t, 3) - pow(t, 2);
		output[i] = start * a + end * b + ((end - start) / 6) * (c - d);
	}
}




//ʱʶ
//

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) 
{
  static int count=0;
	static int integer=1;
	force_left = (float)skeletonAdDmaBuf[0]*388.29/4095;
	force_right = (float)skeletonAdDmaBuf[1]*388.29/4095; //O=LEFT 1=RIGHT 
	
	force_motor_left =(float)data[1].T*144.7;
	force_motor_right=(float)data[0].T*144.7;

	//safety();
	
	
	if (htim == (&htim3)) //电机控制命令
	{	
		//motor_getinfo();
		//debugPrintPending("----- htim3 start -----------------------------------\r\n");
//		force_left = (float)skeletonAdDmaBuf[0]*388.29/4095;
//		force_right = (float)skeletonAdDmaBuf[1]*388.29/4095; //O=LEFT 1=RIGHT 
		count++;
		integer=count%2;
		if(integer!=0)
		{
      //motor_profilePosition(0,3.14);//+=出线  -=进线
//		cmd[0].T=0;
//		cmd[0].K_W = 0.03;
//		cmd[0].W=0;
		 SERVO_Send_recv(&cmd[0], &data[0]);
		}
		else
		{
		 SERVO_Send_recv(&cmd[1], &data[1]);
		}

   if(count>100000)//防止溢出
   count=0;
	}
	if (htim == (&htim4))//步态周期计数	 400hz
	{
    if(enableFlagLeft==1)
		{
			enableFlagLeft=0;
			 if(TimerLeft.Tick>200 && TimerLeft.Tick<700 )  //200 550
			 {
				TimerLeft.Cycletime=TimerLeft.Tick;
				TimerLeft.Tick=0; 
			 }
			 TimerLeft.Tick=0;
		}
		
		if(enableFlagRight==1)
		{
			enableFlagRight=0;
			 if(TimerRight.Tick>200 && TimerRight.Tick<700 )
			 {
				TimerRight.Cycletime=TimerRight.Tick;
				TimerRight.Tick=0;
			 }
			 TimerRight.Tick=0;
		}
	
    	TimerLeft.Tick++;
		TimerRight.Tick++;
	
	
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
	MX_CAN1_Init();
	MX_CAN2_Init();
  MX_USART6_UART_Init();
  MX_UART7_Init();
  MX_TIM1_Init();
  MX_UART8_Init();
  MX_TIM12_Init();
  MX_TIM9_Init();
  MX_USART3_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
	MX_TIM4_Init();
  MX_TIM5_Init();
	MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	can_filter_init();                                   // CAN通讯过滤�??
	initSkeletonAdBuf();//ADC
  //nitDebugBuf();
//	HAL_UART_Receive_IT(&huart8, (uint8_t *)&ucData_L, 1);
   HAL_GPIO_WritePin(GPIOH, POWER1_CTRL_Pin, GPIO_PIN_SET);
   HAL_GPIO_WritePin(GPIOH, POWER2_CTRL_Pin, GPIO_PIN_SET);
   HAL_GPIO_WritePin(GPIOH, POWER3_CTRL_Pin, GPIO_PIN_SET);
   HAL_GPIO_WritePin(GPIOH, POWER4_CTRL_Pin, GPIO_PIN_SET);
//	 HAL_TIM_Base_Start_IT(&htim3);
//   motor_init(0);
//	 motor_init(1);  //电机初始化
//	
//	cmd[0].id=0; 			//ָṹ
//  cmd[0].mode=1;
//	cmd[0].T=0.0;
//	cmd[0].W=0;
////	cmd[0].W=6.28318f * 6.33f;
//	cmd[0].Pos=0;
//	cmd[0].K_P=0;
//	cmd[0].K_W=0;
//	SERVO_Send_recv(&cmd[0], &data[0]);
//	cmd[1].id=1; 			
//	cmd[1].mode=1;
//	cmd[1].T=0.0;
//	cmd[1].W=0;
////	cmd[1].W=6.28318f * 6.33f;
//	cmd[1].Pos=0;
//	cmd[1].K_P=0;
//	cmd[1].K_W=0;
//  SERVO_Send_recv(&cmd[1], &data[1]);
	unsigned int count = 0;




  while (1)
  {	
		skeletonApp();
    //motor_profilePosition(0,6);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
   */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
//  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

//  /** Configure the main internal regulator output voltage
//  */
//  __HAL_RCC_PWR_CLK_ENABLE();
//  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

//  /** Initializes the RCC Oscillators according to the specified parameters
//  * in the RCC_OscInitTypeDef structure.
//  */
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
//  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
//  RCC_OscInitStruct.PLL.PLLM = 6;
//  RCC_OscInitStruct.PLL.PLLN = 168;
//  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
//  RCC_OscInitStruct.PLL.PLLQ = 7;
//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//  {
//    Error_Handler();
//  }

//  /** Initializes the CPU, AHB and APB buses clocks
//  */
//  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

//  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
//  {
//    Error_Handler();
//  }
}

/* USER CODE BEGIN 4 */

static int sys_count = 0;
void sysTickTask(void)
{
//	sys_count++;
//	for(int i=0; i<8; i++){
//		LED_ON[i]--;
//		if(LED_ON[i] > 0) HAL_GPIO_WritePin(GPIOG, (uint16_t)2<<i, GPIO_PIN_RESET);
//		else HAL_GPIO_WritePin(GPIOG, (uint16_t)2<<i, GPIO_PIN_SET);
//		if(LED_ON[i] < 0) LED_ON[i] = 0;
//	}
//	if(sys_count%1000 == 0){
//		HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
//		debugPrintf("sys_time: %.3f sec\r\n",sys_count/1000.f);	
//				
//	}
//	if(sys_count%5000 == 0){
////		buzzer_start(Buzzer, 2500, 500, BUZZER_LOOP_OFF);
//		debugPrintf("buzzer_start\r\n");	
//	}
	
}

void sendcmd(uint8_t imu_data[3])
{
//	static uint8_t tx_buff;
//	for(int i=0;i<3;i++)
//	{
//		tx_buff = imu_data[i];
//		HAL_UART_Transmit(&huart2,&tx_buff,1,0Xff);
//		HAL_UART_Receive_IT(&huart2,&rx_buff1,1);
//		HAL_UART_Transmit(&huart3,&tx_buff,1,0Xff);
//		HAL_UART_Receive_IT(&huart3,&rx_buff2,1);
//		HAL_UART_Transmit(&huart7,&tx_buff,1,0Xff);
//		HAL_UART_Receive_IT(&huart7,&rx_buff3,1);
//	}
	
}



//IMUݴ
void CopeSerial2Data_left(unsigned char ucData)
{
	static unsigned char ucRxBuffer_left[250];
	static unsigned char ucRxCnt_left = 0;	
	
	ucRxBuffer_left[ucRxCnt_left++]=ucData;	//յݴ뻺
	if (ucRxBuffer_left[0]!=0x55) //ͷԣ¿ʼѰ0x55ͷ
	{
		ucRxCnt_left=0;
		return;
	}
	if (ucRxCnt_left<11) {return;}//ݲ11򷵻
	else
	{
		switch(ucRxBuffer_left[1])//жݣȻ俽ӦĽṹУЩݰҪͨλ򿪶Ӧ󣬲ܽյݰ
		{
			//memcpyΪԴڴ濽"string.h"ջַݽṹ棬ӶʵݵĽ
			case 0x51:	memcpy(&ACC_left,&ucRxBuffer_left[2],8);break;
			case 0x52:	memcpy(&Gyro_left,&ucRxBuffer_left[2],8);break;
			case 0x53:	memcpy(&Angle_left,&ucRxBuffer_left[2],8);break;

		}
		ucRxCnt_left=0;//ջ
	}
}

void CopeSerial2Data_right(unsigned char ucData)
{
	static unsigned char ucRxBuffer_right[250];
	static unsigned char ucRxCnt_right = 0;	
	
	ucRxBuffer_right[ucRxCnt_right++]=ucData;	//յݴ뻺
	if (ucRxBuffer_right[0]!=0x55) //ͷԣ¿ʼѰ0x55ͷ
	{
		ucRxCnt_right=0;
		return;
	}
	if (ucRxCnt_right<11) {return;}//ݲ11򷵻
	else
	{
		switch(ucRxBuffer_right[1])//жݣȻ俽ӦĽṹУЩݰҪͨλ򿪶Ӧ󣬲ܽյݰ
		{
			//memcpyΪԴڴ濽"string.h"ջַݽṹ棬ӶʵݵĽ
			case 0x51:	memcpy(&ACC_right,&ucRxBuffer_right[2],8);break;
			case 0x52:	memcpy(&Gyro_right,&ucRxBuffer_right[2],8);break;
			case 0x53:	memcpy(&Angle_right,&ucRxBuffer_right[2],8);break;

		}
		ucRxCnt_right=0;//ջ
	}
}

void CopeSerial2Data_back(unsigned char ucData)
{
	static unsigned char ucRxBuffer_back[250];
	static unsigned char ucRxCnt_back = 0;	
	
	ucRxBuffer_back[ucRxCnt_back++]=ucData;	//յݴ뻺
	if (ucRxBuffer_back[0]!=0x55) //ͷԣ¿ʼѰ0x55ͷ
	{
		ucRxCnt_back=0;
		return;
	}
	if (ucRxCnt_back<11) {return;}//ݲ11򷵻
	else
	{
		switch(ucRxBuffer_back[1])//жݣȻ俽ӦĽṹУЩݰҪͨλ򿪶Ӧ󣬲ܽյݰ
		{
			//memcpyΪԴڴ濽"string.h"ջַݽṹ棬ӶʵݵĽ
			case 0x51:	memcpy(&ACC_back,&ucRxBuffer_back[2],8);break;
			case 0x52:	memcpy(&Gyro_back,&ucRxBuffer_back[2],8);break;
			case 0x53:	memcpy(&Angle_back,&ucRxBuffer_back[2],8);break;

		}
		ucRxCnt_back=0;//ջ
	}
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
