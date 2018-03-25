#ifndef __RS232_H__
#define __RS232_H__

#include "stm32f0xx.h"
#include "communication.h"

#ifdef USE_UART1
extern uint8_t UART1RevData[MAXCOMSIZE];
extern uint16_t UART1Time_1ms;
extern uint16_t UART1RXDataLenth;//UART1接受数据长度
#endif


//#define MAXREVSIZE 512		//定义最大接收字节数 512

//extern uint16_t UART1RXDataLenth；//UART1接受数据长度
//HAL_StatusTypeDef UARTFaultStatus; //串口报超时

//extern uint8_t UART1RevData[MAXREVSIZE];
//extern uint8_t UART2RevData[MAXREVSIZE];

void RS232_Init(uint32_t buad);



#endif


