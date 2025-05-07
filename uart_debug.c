#include "uart_debug.h"
#include "stdio.h"
#include "stdarg.h"
#include "stm32f4xx_hal_uart.h"
//在 uart.c文件中定义变量，在其他需要使用变量的文件中用extern声明为外部变量，避免重复定义
extern UART_HandleTypeDef huart7;
static char tempBuf[SINGLE_MSG_SIZE] = {0};
void debugPrintPending(char *fmt,...){
	//可变长度参数
	va_list argp;
	uint32_t n = 0;
	// 以fmt为起始地址，获取第一个参数的首地址
	va_start(argp, fmt);
	n = vsprintf((char *) tempBuf, fmt, argp);
	// argp置0
	va_end(argp);

	if (HAL_UART_Transmit(&huart7, (uint8_t *) tempBuf, n, 1000) != HAL_OK) {
		/* Transfer error in transmission process */
		Error_Handler();
	}

}
