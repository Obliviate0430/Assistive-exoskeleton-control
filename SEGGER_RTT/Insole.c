
#include <stdint.h>
//#include "usart.h"
#include "main.h"
#include "Insole.h"


#define FRAME_HEADER_1 0xAA
#define FRAME_HEADER_2 0x55
#define FRAME_SIZE 60
#define NUM_PRESSURE_POINTS 28
#define NUM_PACKETS_PER_FRAME 4

InsoleMeasured LeftInsole,RightInsole;//两侧鞋垫数据

uint8_t ValidPointA[28] = {0,1,1,1,1,1,0,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
uint8_t ValidPointB[28] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
uint8_t ValidPointC[28] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
uint8_t ValidPointD[28] = {0,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,0,0,0,0,0,0,0,0};

// 接收串口数据并更新数组


//void LeftProcessInsoleData(uint8_t* buffer) {
//    const uint8_t frameHeader[] = {0x3C, 0xAA, 0x55};
//    const size_t headerSize = sizeof(frameHeader);
//    size_t i = 0;//

//    while (i <= INSOLEBUFFER_SIZE - PACKET_SIZE) {
//        // 在buffer中搜索帧头
//        if (memcmp(&buffer[i], frameHeader, headerSize) == 0) {
//            // 找到帧头，处理数据包
//            uint8_t area = buffer[i + 3]; // 获取区域代码
//            // 这里根据区域代码处理相应的数据...
//            processAreaData(&buffer[i + 4], area);
//            i += PACKET_SIZE; // 移动到下一个数据包的起始位置
//        } else {
//            i++; // 继续搜索帧头
//        }
//    }
//	// LeftProcessFlag = 1;
//}

void ProcessInsoleData(InsoleMeasured* Insole) {
    const uint8_t frameHeader[] = {0x3C, 0xAA, 0x55};
    const size_t headerSize = sizeof(frameHeader);
    size_t i = 0;
//    uint8_t SeqNum;
    uint8_t* ValidPoints;
	uint32_t* SaveValue;//鞋垫数据保存地址
    uint32_t TotalAdc = 0;//区域总adc值
    uint16_t adc_values[28]; // 每个区域有28个测量点
    // 解析数据，合并高低位
    uint8_t* buffer = Insole->Buffer;//数据缓存
    uint8_t areaData[56];
    while (i <= INSOLEBUFFER_SIZE - PACKET_SIZE) 
    {
        // 在buffer中搜索帧头
        if (memcmp(&buffer[i], frameHeader, headerSize) == 0) {
            // 找到帧头，处理数据包
            uint8_t area = buffer[i + 3]; // 获取区域代码
            // areaData 指向56字节的数据
            // SeqNum = &buffer[i + 4];//采样次数序列[0-255]
            memcpy(&areaData,&buffer[i+5],56);
            switch (area)//根据Area代码选择保存的区域 
            {
	        	case 0x41:
	        		SaveValue = &Insole->ToeValue;
	        		ValidPoints = ValidPointA;
	        		break;
	        	case 0x42:
	        		SaveValue = &Insole->ForwardValue;
	        		ValidPoints = ValidPointB;
	        		break;
	        	case 0x43:
	        		SaveValue = &Insole->ReerValue;
	        		ValidPoints = ValidPointC;
	        		break;
	        	case 0x44:
	        		SaveValue = &Insole->HeelValue;
	        		ValidPoints = ValidPointD;
                    
	        		break;
	        	default:;
            }
            TotalAdc = 0;//清空
            for (int j = 0; j < 28; j++) 
            {
	        	if (ValidPoints[j])//仅对有效点处理
                { // 只处理有效的数据点
                	uint16_t low_byte = areaData[j * 2];//
                	uint16_t high_byte = areaData[j * 2 + 1];
                	uint16_t adc_value = (high_byte << 8) | low_byte;//
                	// adc_values[i] = adc_value;
                	TotalAdc += adc_value;
	        	}
            }
	        *SaveValue = TotalAdc;//
            // processAreaData(&buffer[i + 4], area);
            i += PACKET_SIZE; // 移动到下一个数据包的起始位置
        } else {
            i++; // 继续搜索帧头
        }
    }
	Insole->ProcessFlag = 1;//数据处理完成flag
    // printf('1\r\n');//计算采样频率
}


// 这里是处理每个区域数据的示例函数
// void processAreaData(InsoleMeasured* Insole,uint8_t* areaData, char areaCode) {
//     // areaData 指向56字节的数据
//     uint32_t TotalAdc = 0;
//     uint16_t adc_values[28]; // 每个区域有28个测量点
//     // 解析数据，合并高低位
// 	uint8_t* ValidPoints;
// 	uint32_t* SaveValue;
//     switch (areaCode) {
// 		case 0x41:
// 			SaveValue = &Insole->ToeValue;
// 			ValidPoints = ValidPointA;
// 			break;
// 		case 0x42:
// 			SaveValue = &Insole->ForwardValue;
// 			ValidPoints = ValidPointB;
// 			break;
// 		case 0x43:
// 			SaveValue = &Insole->ReerValue;
// 			ValidPoints = ValidPointC;
// 			break;
// 		case 0x44:
// 			SaveValue = &Insole->HeelValue;
// 			ValidPoints = ValidPointD;
// 			break;
// 		default:;
//     }
//     for (int i = 0; i < 28; i++) {
// 		if (ValidPoints[i]) { // 只处理有效的数据点
//         	uint16_t low_byte = areaData[i * 2];//
//         	uint16_t high_byte = areaData[i * 2 + 1];
//         	uint16_t adc_value = (high_byte << 8) | low_byte;//
//         	// adc_values[i] = adc_value;
//         	TotalAdc += adc_value;
// 			// if (areaCode == 0x41) {
// 			// 	Senbuff[i] = adc_value;
// 			// }
// 		}
//     }
// 	*SaveValue = TotalAdc;
// }

