#ifndef __RS232_H__
#define __RS232_H__

#include "stm32f0xx.h"

#ifdef USE_UART1
extern uint8_t UART1RevData[MAXCOMSIZE];
extern uint16_t UART1Time_1ms;
extern uint16_t UART1RXDataLenth;//UART1�������ݳ���
#endif


//#define MAXREVSIZE 512		//�����������ֽ��� 512

//extern uint16_t UART1RXDataLenth��//UART1�������ݳ���
//HAL_StatusTypeDef UARTFaultStatus; //���ڱ���ʱ

//extern uint8_t UART1RevData[MAXREVSIZE];
//extern uint8_t UART2RevData[MAXREVSIZE];

void RS232_Init(uint32_t buad);



#endif


