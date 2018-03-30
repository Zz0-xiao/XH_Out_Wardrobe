#include "rs3485.h"
#include "timer.h"
#include "delay.h"
#include "utility.h"
////接收缓存区
//uint8_t RS485_RX_BUF[MAXCOMSIZE];  	//接收缓冲,最大64个字节.
////接收到的数据长度
//uint8_t RS485_RX_CNT = 0;

#ifdef USE_UART2
uint8_t UART2RevData[MAXCOMSIZE];
uint16_t UART2Time_1ms = 1;
uint16_t UART2RXDataLenth = 0;
#endif

void USART2_IRQHandler(void)
{
    uint8_t tempdata;
//    USART_ClearITPendingBit(USART1, USART_IT_ORE); //ORE中断清除 否则大量数据时会出现死机
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        tempdata = USART_ReceiveData(USART2);
        UART2RevData[UART2RXDataLenth] = tempdata;
        UART2RXDataLenth++;
        if(UART2RXDataLenth >= MAXCOMSIZE)
        {
            UART2RXDataLenth--;
            UART2Time_1ms = 500;//停止计时
        }
        else
            UART2Time_1ms = 1;//启动计时
    }
}

/*******************************
名称：RS485_Data_API()
功能：485发送数据，只做显示用，测试用
参数：uint16_t datasize，只是data长度,函数中自动加5 若为0 则全不选上
返回：HAL_StatusTypeDef communication.h
*******************************/

HAL_StatusTypeDef RS485_Data_API(const void* data, uint16_t datasize)
{
    HAL_StatusTypeDef temp;
    RS485EN_H();//
    temp = TransmitData_API(RS485, data, datasize);
    RS485EN_L();//默认为接收模式
    return temp;
}

/*******************************
名称：RS485_Data_SDSES()
功能：485发送数据，适用于神思，自动加入SDsEs,crc
参数：	uint32_t 长度，只是data长度,函数中自动加5
			cmdr  命令
			state状态值
			data  发送数据,不包含任何协议内容，只是data数据，或数组
返回：HAL_StatusTypeDef communication.h
*******************************/
HAL_StatusTypeDef RS485_Data_SDSES(uint32_t len, uint16_t cmdr, uint8_t state, const void* data)
{
    HAL_StatusTypeDef temp;
    RS485EN_H();//
    temp = TransmitData_SDSES(RS485, len, cmdr, state, data);
    RS485EN_L();//默认为接收模式
    return temp;
}

/*******************************
名称：RS485_Init()
功能：485初始化，包括使能端
参数：	buad 波特率
返回：无
*******************************/
void RS485_Init(uint32_t buad)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF, ENABLE);	//使能F时钟
    //    //EN脚初始化
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; //模式：输出
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //输出类型，推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOF, &GPIO_InitStructure);

    UART_Initial(RS485, buad );

    RS485EN_L();//默认为接收模式
}



//RS485发送len个字节.
//buf:发送区首地址
//len:发送的字节数(为了和本代码的接收匹配,这里建议不要超过64个字节)
//void RS485_Send_Data1(uint8_t *buf, uint8_t len)
//{
//    uint8_t t;
//    RS485EN_H();	//设置为发送模式
//    for(t = 0; t < len; t++)		//循环发送数据
//    {
//        while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
//        USART_SendData(USART2, buf[t]);
//    }

//    while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
//    RS485_RX_CNT = 0;
//    RS485EN_L();				//设置为接收模式
//}


////RS485查询接收到的数据
////buf:接收缓存首地址
////len:读到的数据长度
//void RS485_Receive_Data(uint8_t *buf)
//{
//    uint8_t rxlen = RS485_RX_CNT;
//    uint8_t i = 0;
////    *len = 0;				//默认为0
//    Delay_ms(10);		//等待10ms,连续超过10ms没有接收到一个数据,则认为接收结束
//    if(rxlen == RS485_RX_CNT && rxlen) //接收到了数据,且接收完成了
//    {
//        for(i = 0; i < rxlen; i++)
//        {
//            buf[i] = RS485_RX_BUF[i];
//        }
////        *len = RS485_RX_CNT;	//记录本次数据长度
//        RS485_RX_CNT = 0;		//清零
//    }
//}


//void USART2_IRQHandler(void)
//{
//    uint8_t res;

//    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) //接收到数据
//    {
//        res = USART_ReceiveData(USART2); 	//读取接收到的数据
//        if(RS485_RX_CNT < 64)
//        {
//            RS485_RX_BUF[RS485_RX_CNT] = res;		//记录接收到的值
//            RS485_RX_CNT++;						//接收数据增加1
//        }
//    }
//}

//RS485查询接收到的数据
//buf:接收缓存首地址
//len:读到的数据长度
//void RS485_Receive_Data(uint8_t *buf)
//{
//    uint8_t rxlen = UART2RXDataLenth;
//    uint8_t i = 0;
//    if(rxlen == UART2RXDataLenth && rxlen) //接收到了数据,且接收完成了
//    {
//        for(i = 0; i < rxlen; i++)
//        {
//            buf[i] = UART2RevData[i];
//        }
//        UART2RXDataLenth = 0;		//清零
//        BuffReset_API(UART2RevData, MAXCOMSIZE);
//    }
//}
