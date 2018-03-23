#include "rs3485.h"
#include "timer.h"
#include "delay.h"
////���ջ�����
uint8_t RS485_RX_BUF[64];  	//���ջ���,���64���ֽ�.
////���յ������ݳ���
uint8_t RS485_RX_CNT = 0;

#ifdef USE_UART2
uint8_t UART2RevData[MAXCOMSIZE];
uint16_t UART2Time_1ms = 1;
uint16_t UART2RXDataLenth = 0;
#endif

void USART2_IRQHandler(void)
{
    uint8_t tempdata;

//    USART_ClearITPendingBit(USART1, USART_IT_ORE); //ORE�ж���� �����������ʱ���������
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        tempdata = USART_ReceiveData(USART2);
//        UART2RevData[UART2RXDataLenth++] = tempdata;
        UART2RevData[UART2RXDataLenth] = tempdata;
        UART2RXDataLenth++;
        if(UART2RXDataLenth >= MAXCOMSIZE)
        {
            UART2RXDataLenth--;
            UART2Time_1ms = 0;//ֹͣ��ʱ
        }
        else
            UART2Time_1ms = 1;//������ʱ
    }
}


//void USART2_IRQHandler(void)
//{
//    uint8_t res;

//    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) //���յ�����
//    {
//        res = USART_ReceiveData(USART2); 	//��ȡ���յ�������
//        if(RS485_RX_CNT < 64)
//        {
//            RS485_RX_BUF[RS485_RX_CNT] = res;		//��¼���յ���ֵ
//            RS485_RX_CNT++;						//������������1
//        }
//    }
//}


//RS485��ѯ���յ�������
//buf:���ջ����׵�ַ
//len:���������ݳ���
void RS485_Receive_Data(uint8_t *buf)
{
    uint8_t rxlen = RS485_RX_CNT;
    uint8_t i = 0;
//    *len = 0;				//Ĭ��Ϊ0
    Delay_ms(10);		//�ȴ�10ms,��������10msû�н��յ�һ������,����Ϊ���ս���
    if(rxlen == RS485_RX_CNT && rxlen) //���յ�������,�ҽ��������
    {
        for(i = 0; i < rxlen; i++)
        {
            buf[i] = RS485_RX_BUF[i];
        }
        RS485_RX_CNT = 0;		//����
    }
}


void RS485_Init(uint32_t buad)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF, ENABLE);	//ʹ��Fʱ��
    //    //EN�ų�ʼ��
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; //ģʽ�����
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //������ͣ��������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOF, &GPIO_InitStructure);

    UART_Initial(RS485, buad );

    RS485EN_L();//Ĭ��Ϊ����ģʽ
}


HAL_StatusTypeDef RS485_Send_Data(const void* data, uint16_t datasize)
{
    HAL_StatusTypeDef temp;
    RS485EN_H();//
    temp = TransmitData_API(RS485, data, datasize);
    RS485EN_L();//Ĭ��Ϊ����ģʽ
    return temp;
}





//RS485����len���ֽ�.
//buf:�������׵�ַ
//len:���͵��ֽ���(Ϊ�˺ͱ�����Ľ���ƥ��,���ｨ�鲻Ҫ����64���ֽ�)
void RS485_Send_Data1(uint8_t *buf, uint8_t len)
{
    uint8_t t;
    RS485EN_H();	//����Ϊ����ģʽ
    for(t = 0; t < len; t++)		//ѭ����������
    {
        while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
        USART_SendData(USART2, buf[t]);
    }

    while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
    RS485_RX_CNT = 0;
    RS485EN_L();				//����Ϊ����ģʽ
}







////RS485��ѯ���յ�������
////buf:���ջ����׵�ַ
////len:���������ݳ���
//void RS485_Receive_Data(uint8_t *buf)
//{
//    uint8_t rxlen = RS485_RX_CNT;
//    uint8_t i = 0;
////    *len = 0;				//Ĭ��Ϊ0
//    Delay_ms(10);		//�ȴ�10ms,��������10msû�н��յ�һ������,����Ϊ���ս���
//    if(rxlen == RS485_RX_CNT && rxlen) //���յ�������,�ҽ��������
//    {
//        for(i = 0; i < rxlen; i++)
//        {
//            buf[i] = RS485_RX_BUF[i];
//        }
////        *len = RS485_RX_CNT;	//��¼�������ݳ���
//        RS485_RX_CNT = 0;		//����
//    }
//}


