#ifndef __RS3485_H
#define __RS3485_H

#include "stm32f0xx.h"
#include "communication.h"

#ifdef USE_UART2
extern uint8_t UART2RevData[MAXCOMSIZE];
extern uint16_t UART2Time_1ms;
extern uint16_t UART2RXDataLenth;//UART1接受数据长度
#endif

//extern uint8_t RS485_RX_BUF[MAXCOMSIZE];


//模式控制
#define RS485EN_L()	 	  GPIO_ResetBits(GPIOF, GPIO_Pin_4)
#define RS485EN_H()  		GPIO_SetBits(GPIOF, GPIO_Pin_4)

void RS485_Init(uint32_t buad);
HAL_StatusTypeDef RS485_Data_API(const void* data, uint16_t datasize);
HAL_StatusTypeDef RS485_Data_SDSES(uint32_t len, uint16_t cmdr, uint8_t state, const void* data);
//void RS485_Send_Data1(uint8_t *buf, uint8_t len);


#endif






//extern uint8_t RS485_RX_BUF[64]; 		//接收缓冲,最大64个字节
//extern uint8_t RS485_RX_CNT;   			//接收到的数据长度










