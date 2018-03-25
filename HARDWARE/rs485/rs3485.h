#ifndef __RS3485_H
#define __RS3485_H

#include "stm32f0xx.h"
#include "communication.h"

#ifdef USE_UART2
extern uint8_t UART2RevData[MAXCOMSIZE];
extern uint16_t UART2Time_1ms;
extern uint16_t UART2RXDataLenth;//UART1接受数据长度
#endif



//模式控制
#define RS485EN_L()	 	  GPIO_ResetBits(GPIOF, GPIO_Pin_4)
#define RS485EN_H()  		GPIO_SetBits(GPIOF, GPIO_Pin_4)

void RS485_Init(uint32_t buad);//功能：485初始化，包括使能端
HAL_StatusTypeDef RS485_Data_API(const void* data, uint16_t datasize);//功能：485发送数据，只做显示用，测试用
HAL_StatusTypeDef RS485_Data_SDSES(uint32_t len, uint16_t cmdr, uint8_t state, const void* data);//功能：485发送数据，适用于神思，自动加入SDsEs,crc

#endif



//extern uint8_t RS485_RX_BUF[MAXCOMSIZE];
//void RS485_Send_Data1(uint8_t *buf, uint8_t len);
//extern uint8_t RS485_RX_BUF[64]; 		//接收缓冲,最大64个字节
//extern uint8_t RS485_RX_CNT;   			//接收到的数据长度










