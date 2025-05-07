#ifndef __INSOLE_H
#define __INSOLE_H

#include <stdint.h>

#define PACKET_SIZE 60   //鞋垫数据包大小
#define PACKETS_TO_RECEIVE 4  //鞋垫数据包个数
#define INSOLEBUFFER_SIZE (PACKET_SIZE * PACKETS_TO_RECEIVE) //鞋垫数据缓存大小

#define InsoleValueBuffer 4
typedef struct
{
	uint32_t HeelValue;//后跟
	uint32_t ReerValue;//后脚掌
	uint32_t ForwardValue;//前脚掌
	uint32_t ToeValue;//脚尖
	uint8_t Buffer[INSOLEBUFFER_SIZE];//数据缓存
	uint8_t	ProcessFlag;//数据处理完成flag
	uint32_t HistoryValue[InsoleValueBuffer];//历史数据，脚跟数据
} InsoleMeasured;

extern InsoleMeasured LeftInsole,RightInsole;//两侧鞋垫数据
extern void ProcessInsoleData(InsoleMeasured* Insole);//处理鞋垫数据

#endif
