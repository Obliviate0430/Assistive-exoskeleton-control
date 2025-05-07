#include "usart.h"
#include "motor_control.h"
#include "crc_ccitt.h"
#include "stdio.h"

#define SATURATE(_IN, _MIN, _MAX) {\
 if (_IN < _MIN)\
 _IN = _MIN;\
 else if (_IN > _MAX)\
 _IN = _MAX;\
 } 

	float pos0,vel0,tor0;
	float pos1,vel1,tor1;
//uint32_t crc32_core(uint32_t* ptr, uint32_t len)
//{
//    uint32_t xbit = 0;
//    uint32_t data = 0;
//    uint32_t CRC32 = 0xFFFFFFFF;
//    const uint32_t dwPolynomial = 0x04c11db7;
//    for (uint32_t i = 0; i < len; i++)
//    {
//        xbit = 1 << 31;
//        data = ptr[i];
//        for (uint32_t bits = 0; bits < 32; bits++)
//        {
//            if (CRC32 & 0x80000000)
//            {
//                CRC32 <<= 1;
//                CRC32 ^= dwPolynomial;
//            }
//            else
//                CRC32 <<= 1;
//            if (data & xbit)
//                CRC32 ^= dwPolynomial;

//            xbit >>= 1;
//        }
//    }
//    return CRC32;
//}

int modify_data(MOTOR_send *motor_s)
{
    motor_s->hex_len = 17;
    motor_s->motor_send_data.head[0] = 0xFE;
    motor_s->motor_send_data.head[1] = 0xEE;
	
//		SATURATE(motor_s->id,   0,    15);
//		SATURATE(motor_s->mode, 0,    7);
		SATURATE(motor_s->K_P,  0.0f,   25.599f);
		SATURATE(motor_s->K_W,  0.0f,   25.599f);
		SATURATE(motor_s->T,   -127.99f,  127.99f);
		SATURATE(motor_s->W,   -804.00f,  804.00f);
		SATURATE(motor_s->Pos, -411774.0f,  411774.0f);

    motor_s->motor_send_data.mode.id   = motor_s->id;
    motor_s->motor_send_data.mode.status  = motor_s->mode;
    motor_s->motor_send_data.comd.k_pos  = motor_s->K_P/25.6f*32768;
    motor_s->motor_send_data.comd.k_spd  = motor_s->K_W/25.6f*32768;
    motor_s->motor_send_data.comd.pos_des  = motor_s->Pos/6.2832f*32768;
    motor_s->motor_send_data.comd.spd_des  = motor_s->W/6.2832f*256;
    motor_s->motor_send_data.comd.tor_des  = motor_s->T*256;
    motor_s->motor_send_data.CRC16 = crc_ccitt(0, (uint8_t *)&motor_s->motor_send_data, 15);
    return 0;
}

int extract_data(MOTOR_recv *motor_r)
{

    if(motor_r->motor_recv_data.CRC16 !=
        crc_ccitt(0, (uint8_t *)&motor_r->motor_recv_data, 14)){
        uartPrintf("[WARNING] Receive data CRC error");
        motor_r->correct = 0;
        return motor_r->correct;
    }
    else
		{
        motor_r->motor_id = motor_r->motor_recv_data.mode.id;
        motor_r->mode = motor_r->motor_recv_data.mode.status;
        motor_r->Temp = motor_r->motor_recv_data.fbk.temp;
        motor_r->MError = motor_r->motor_recv_data.fbk.MError;
        motor_r->W = ((float)motor_r->motor_recv_data.fbk.speed/256)*6.2832f ;
        motor_r->T = ((float)motor_r->motor_recv_data.fbk.torque) / 256;
        motor_r->Pos = 6.2832f*((float)motor_r->motor_recv_data.fbk.pos) / 32768;
				motor_r->footForce = motor_r->motor_recv_data.fbk.force;
				motor_r->correct = 1;
//				uartPrintf("motor ID:%d, pos:%.3f, vel:%.3f, tor:%.3f\r\n",motor_r->motor_id,motor_r->Pos,motor_r->W,motor_r->T);
			if(motor_r->motor_id==0)
			{
				pos0 = motor_r->Pos;
				vel0 = motor_r->W;
				tor0 = motor_r->T;
			}
			if(motor_r->motor_id==1)
			{
				pos1 = motor_r->Pos;
				vel1 = motor_r->W;
				tor1 = motor_r->T;
			}
//			uartPrintf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",pos0,vel0,tor0,pos1,vel1,tor1); // �������
			return motor_r->correct;
    }
}


extern uint8_t uart6Data;
extern uint8_t ucRxBuffer6[30] ;
HAL_StatusTypeDef SERVO_Send_recv(MOTOR_send *pData, MOTOR_recv *rData)
{
    uint16_t rxlen = 0;

    modify_data(pData);
//    
////		SET_485_DE_UP();
		SET_485_RE_UP();
    HAL_UART_Transmit(&huart6, (uint8_t *)pData, sizeof(pData->motor_send_data), 2); 
//		

	  	SET_485_RE_DOWN();
//		SET_485_DE_DOWN();
//    HAL_UARTEx_ReceiveToIdle(&huart6, (uint8_t *)rData, sizeof(rData->motor_recv_data), &rxlen, 10);hal
		HAL_UARTEx_ReceiveToIdle_DMA(&huart6, (uint8_t *)rData, sizeof(rData->motor_recv_data));
	
	
//    if(rxlen == 0)
//      return HAL_TIMEOUT;

//    if(rxlen != sizeof(rData->motor_recv_data))
//			return rxlen;

    uint8_t *rp = (uint8_t *)&rData->motor_recv_data;
    if(rp[0] == 0xFD && rp[1] == 0xEE)			
    {
        rData->correct = 1;
        extract_data(rData);
        return HAL_OK;
    }
		else{
			  rData->correct = 0;
//				uartPrintf("rp[0]:%#x, rp[1]:%#x\r\n",rp[0],rp[1]);
		}
//    extract_data(rData);
    return HAL_ERROR;
		
		
}


void motor_getinfo(void) //mode: 0:空闲,  1:闭环FOC控制
{  
    //实际给FOC的指令力矩为：
    // K_P*delta_Pos + K_W*delta_W + T
	static int count1=0;
	static int integer1=0;
  count1++;
	integer1=count1%2;
	if(integer1!=0)

		{
		SERVO_Send_recv(&cmd[0], &data[0]);
		}
		
	else
	
		{
		SERVO_Send_recv(&cmd[1], &data[1]);
		}

   if(count1>100000)//防止溢出
   count1=0;
}


//void motor_control(int mode,int id1,int id2, double T,double pos_rad,double vel_rad) //mode: 0:空闲,  1:闭环FOC控制
//{  
//    //实际给FOC的指令力矩为：
//    // K_P*delta_Pos + K_W*delta_W + T
//	static int count1=0;
//	static int integer1=0;
//    count1++;
//	integer1=count1%2;
//	if(integer1!=0)
//	{
//	    switch (mode)
//		{
//		case 0:
//			motor_stop(id1);
//			break;
//		case 1:
//			motor_profilePosition(id1,pos_rad);
//			break;
//		case 2:
//			motor_profileVelocity(id1,vel_rad);
//			break;

//		default:
//			break;
//		}
//        motor_profilePosition(0,3.14);//+=出线  -=进线
////		cmd[0].T=0;
////		cmd[0].K_W = 0.03;
////		cmd[0].W=0;
////		SERVO_Send_recv(&cmd[0], &data[0]);
//		 // HAL_Delay(100);
//	}
//		else
//		{
//         switch (mode)
//		 {
//		 case 0:
//			motor_stop(id2);
//			break;

//		 case 1:
//			motor_profilePosition(id2,pos_rad);
//			break;
//		 case 2:
//			motor_profileVelocity(id2,vel_rad);
//			break;

//		 default:
//			break;
//		 }
//		
//		SERVO_Send_recv(&cmd[1], &data[1]);
//		}

//   if(count1>100000)//防止溢出
//   count1=0;
//}


