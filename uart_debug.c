#include "uart_debug.h"
#include "stdio.h"
#include "stdarg.h"
#include "stm32f4xx_hal_uart.h"
//�� uart.c�ļ��ж����������������Ҫʹ�ñ������ļ�����extern����Ϊ�ⲿ�����������ظ�����
extern UART_HandleTypeDef huart7;
static char tempBuf[SINGLE_MSG_SIZE] = {0};
void debugPrintPending(char *fmt,...){
	//�ɱ䳤�Ȳ���
	va_list argp;
	uint32_t n = 0;
	// ��fmtΪ��ʼ��ַ����ȡ��һ���������׵�ַ
	va_start(argp, fmt);
	n = vsprintf((char *) tempBuf, fmt, argp);
	// argp��0
	va_end(argp);

	if (HAL_UART_Transmit(&huart7, (uint8_t *) tempBuf, n, 1000) != HAL_OK) {
		/* Transfer error in transmission process */
		Error_Handler();
	}

}
