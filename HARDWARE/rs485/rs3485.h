#ifndef __RS3485_H
#define __RS3485_H

#include "stm32f0xx.h"

#ifdef USE_UART2
extern uint8_t UART2RevData[MAXCOMSIZE];
extern int UART2Time_1ms;
extern uint16_t UART2RXDataLenth;//UART1�������ݳ���
#endif

//ģʽ����
#define RS485EN_L()	 	  GPIO_ResetBits(GPIOF, GPIO_Pin_4)
#define RS485EN_H()  		GPIO_SetBits(GPIOF, GPIO_Pin_4)


void RS485_Init(uint32_t buad);
//void RS485_Send_Data(uint8_t *buf,uint8_t len);
//void RS485_Receive_Data(uint8_t *buf );



#endif






//extern uint8_t RS485_RX_BUF[64]; 		//���ջ���,���64���ֽ�
//extern uint8_t RS485_RX_CNT;   			//���յ������ݳ���









