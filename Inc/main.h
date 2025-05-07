/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "motor_control.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdarg.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define BEEP_TUNE        TIM12->ARR
#define BEEP_CTRL        TIM12->CCR1
static const uint32_t IMU_SAMPLE_FRAME = 20;	
static const uint32_t MEDIAN = 0.5*IMU_SAMPLE_FRAME;
static const uint32_t THRESHOLD = 0.8*MEDIAN;
static const uint32_t GAIT_SAMPLE_FRAME = 3;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern MOTOR_send cmd[2];   //
extern MOTOR_recv data[2];
extern uint8_t ucData_L[2],ucData_R[2];
extern uint8_t ucRxBuffer_L[50];
extern float accxf, accyf, acczf, velxf, velyf, velzf, angxf, angyf, angzf;
extern volatile float VELX[IMU_SAMPLE_FRAME], ANGX[IMU_SAMPLE_FRAME];
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
uint32_t get_time_ms(void);
void beep_set_tune(uint16_t tune, uint16_t ctrl);
int32_t beep_set_times(uint8_t times);
int32_t beep_set_tick(uint32_t tick);
int32_t beep_ctrl_times(void *argc);
void sysTickTask(void);


/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define POWER1_CTRL_Pin GPIO_PIN_2
#define POWER1_CTRL_GPIO_Port GPIOH
#define POWER2_CTRL_Pin GPIO_PIN_3
#define POWER2_CTRL_GPIO_Port GPIOH
#define POWER3_CTRL_Pin GPIO_PIN_4
#define POWER3_CTRL_GPIO_Port GPIOH
#define LED8_Pin GPIO_PIN_8
#define LED8_GPIO_Port GPIOG
#define POWER4_CTRL_Pin GPIO_PIN_5
#define POWER4_CTRL_GPIO_Port GPIOH
#define LED7_Pin GPIO_PIN_7
#define LED7_GPIO_Port GPIOG
#define LED6_Pin GPIO_PIN_6
#define LED6_GPIO_Port GPIOG
#define LED5_Pin GPIO_PIN_5
#define LED5_GPIO_Port GPIOG
#define LED4_Pin GPIO_PIN_4
#define LED4_GPIO_Port GPIOG

#define SP3485_RE_Pin GPIO_PIN_10
#define SP3485_RE_GPIO_Port GPIOF

#define LED2_Pin GPIO_PIN_2
#define LED2_GPIO_Port GPIOG
#define BUTTON_4_Pin GPIO_PIN_3
#define BUTTON_4_GPIO_Port GPIOC
#define BUTTON_4_EXTI_IRQn EXTI3_IRQn
#define BUTTON_Pin GPIO_PIN_2
#define BUTTON_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_1
#define LED1_GPIO_Port GPIOG
#define BEEP_CTRL_Pin GPIO_PIN_6
#define BEEP_CTRL_GPIO_Port GPIOH
#define TIM2_CH2_Pin GPIO_PIN_1
#define TIM2_CH2_GPIO_Port GPIOA
#define BUTTON_1_Pin GPIO_PIN_4
#define BUTTON_1_GPIO_Port GPIOA
#define BUTTON_1_EXTI_IRQn EXTI4_IRQn
#define BUTTON_2_Pin GPIO_PIN_5
#define BUTTON_2_GPIO_Port GPIOA
#define BUTTON_2_EXTI_IRQn EXTI9_5_IRQn
#define LED_RED_Pin GPIO_PIN_11
#define LED_RED_GPIO_Port GPIOE
#define BUTTON_3_Pin GPIO_PIN_1
#define BUTTON_3_GPIO_Port GPIOB
#define BUTTON_3_EXTI_IRQn EXTI1_IRQn
#define LED_GREEN_Pin GPIO_PIN_14
#define LED_GREEN_GPIO_Port GPIOF

/* USER CODE BEGIN Private defines */
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))



/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
