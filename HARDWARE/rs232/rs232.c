#include "rs232.h"
#include "communication.h"
#include "timer.h"


//���ڳ�ʱȫ�ֺ���
#ifdef USE_UART1
uint8_t UART1RevData[MAXCOMSIZE];
int UART1Time_1ms = 0;
uint16_t UART1RXDataLenth = 0;
#endif

void USART1_IRQHandler(void)
{
    uint8_t tempdata;
//	USART_ClearITPendingBit(USART1,USART_IT_ORE);//ORE�ж���� �����������ʱ���������
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        tempdata = USART_ReceiveData(USART1);
        UART1RevData[UART1RXDataLenth++] = tempdata;
        if(UART1RXDataLenth >= MAXCOMSIZE)
        {
            UART1RXDataLenth--;
            UART1Time_1ms = 1000;//ֹͣ��ʱ
        }
        else
            UART1Time_1ms = 1;//������ʱ
    }
}

void RS232_Init(uint32_t buad) {

    UART_Initial(RS232, buad );
}

//void USART1_IRQHandler(void)
//{
//    uint8_t tempdata;
//    USART_ClearITPendingBit(USART1, USART_IT_ORE); //ORE�ж���� �����������ʱ���������
//    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
//    {
//        tempdata = USART_ReceiveData(USART1);
//        UART1RevData[UART1RXDataLenth] = tempdata;
//        UART1RXDataLenth++;
//        if(UART1RXDataLenth >= MAXREVSIZE)//���ձ�ը��
//        {
//            UART1RXDataLenth = UART1RXDataLenth - 1;
//            UARTFaultStatus = HAL_UART1RXFULL;
//            time3Usart1ms = 0;//ֹͣ��ʱ
//        }
//        else
//            time3Usart1ms = 1;//������ʱ
//    }
//}

