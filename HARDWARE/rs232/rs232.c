#include "rs232.h"
#include "timer.h"

//串口超时全局函数
#ifdef USE_UART1
uint8_t UART1RevData[MAXCOMSIZE];
uint16_t UART1Time_1ms = 1;
uint16_t UART1RXDataLenth = 0;
#endif

void USART1_IRQHandler(void)
{
    uint8_t tempdata;
//	USART_ClearITPendingBit(USART1,USART_IT_ORE);//ORE中断清除 否则大量数据时会出现死机
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        tempdata = USART_ReceiveData(USART1);
        UART1RevData[UART1RXDataLenth++] = tempdata;
        if(UART1RXDataLenth >= MAXCOMSIZE)
        {
            UART1RXDataLenth--;
            UART1Time_1ms = 0;//停止计时
        }
        else
            UART1Time_1ms = 1;//启动计时
    }
}

void RS232_Init(uint32_t buad) {

    UART_Initial(RS232, buad );
}

//void USART1_IRQHandler(void)
//{
//    uint8_t tempdata;
//    USART_ClearITPendingBit(USART1, USART_IT_ORE); //ORE中断清除 否则大量数据时会出现死机
//    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
//    {
//        tempdata = USART_ReceiveData(USART1);
//        UART1RevData[UART1RXDataLenth] = tempdata;
//        UART1RXDataLenth++;
//        if(UART1RXDataLenth >= MAXREVSIZE)//接收爆炸，
//        {
//            UART1RXDataLenth = UART1RXDataLenth - 1;
//            UARTFaultStatus = HAL_UART1RXFULL;
//            time3Usart1ms = 0;//停止计时
//        }
//        else
//            time3Usart1ms = 1;//启动计时
//    }
//}

