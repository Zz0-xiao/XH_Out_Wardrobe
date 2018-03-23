#include "communication.h"
#include "rs232.h"
#include "rs3485.h"
#include "utility.h"
#include "string.h"
#include "timer.h"

/************全局变量定义区**************/

/************外部变量定义区*************/
//<神思协议定义
//<s>协议头
uint8_t HEAD[] =  "SDsEs";
//<s>协议尾
uint8_t END[]  =  "ETX";
uint8_t HEADSIZE = sizeof(HEAD) / sizeof(uint8_t) - 1;
uint8_t ENDSIZE = sizeof(END) / sizeof(uint8_t) - 1;

/*************END***********************/
HAL_StatusTypeDef CheckCrc(uint8_t *p)
{
    uint16_t crc = 0;
    uint8_t  crc_out[2] = {0, 0};
    uint32_t len;
    uint8_t  value;
    uint16_t i;
    uint8_t  tmp;

    len = (p[HEADSIZE] << 24) + (p[HEADSIZE + 1] << 26) + (p[HEADSIZE + 2] << 8) + p[HEADSIZE + 3] + 2;
    i = 0;
    while (len--)
    {
        crc = crc16one(p[i + 6], crc);
        i++;
    }
    crc_out[0] = crc / 256;
    crc_out[1] = crc % 256;
    value = p[i + 6];
    tmp = p[i + 1 + 6];
    if((crc_out[0] == value) && (crc_out[1] == tmp))
    {
        return HAL_OK;
    }
    return HAL_CRCERROR;
}

/*******************************
名称：CheckProtocol(uint8_t* pbuff)
功能：检查协议，并不声明
参数：USART_TypeDef *uart 串口号 uint8_t* pbuff 接收全部数据，不偏移
返回：HAL_StatusTypeDef communication.h
*******************************/
HAL_StatusTypeDef CheckProtocol(USART_TypeDef* huart, uint8_t* pbuff)
{
    int i;
    uint16_t lenth;
//    if(UART1RXDataLenth < HEADSIZE)
//        return HAL_HEADERROR;
    for(i = 0; i < HEADSIZE; i++)
    {
        if(pbuff[i] != HEAD[i])
            return HAL_HEADERROR;
    }
    lenth = (pbuff[HEADSIZE] << 24) + (pbuff[HEADSIZE + 1] << 26) + (pbuff[HEADSIZE + 2] << 8) + pbuff[HEADSIZE + 3];

    if( huart == USART1)
    {
#ifdef USE_UART1
        if(UART1RXDataLenth - 9 < lenth)
            return HAL_BUSY;
#endif
    }
    else if(huart == USART2)
    {
#ifdef USE_UART2
        if(UART2RXDataLenth - 9 < lenth)
            return HAL_BUSY;
#endif
    }
	if(CheckCrc(pbuff)!= HAL_OK)
		return HAL_CRCERROR;
    return HAL_OK;
}

/*******************************
名称：HAL_StatusTypeDef	UART_Initial(USART_TypeDef* huart,int buad)
功能：串口初始化
参数：UART_HandleTypeDef *huart串口结构体
			int buad 波特率
返回：HAL_StatusTypeDef
			详见communication.h
*******************************/
void	UART_Initial(USART_TypeDef* huart, int buad)
{
    USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);					//使能USART1，GPIOA时钟

    if(huart == USART1)
    {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

        GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
        NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    }
    else
    {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

        GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;

        NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    }

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannelPriority = 0;		//抢占优先级0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		//IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);										//根据指定的参数初始化VIC寄存器

    USART_InitStructure.USART_BaudRate = buad;								//串口波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//收发模式

    USART_Init(huart, &USART_InitStructure);			//初始化串口
    USART_ITConfig(huart, USART_IT_RXNE, ENABLE);//开启串口接受中断

    USART_Cmd(huart, ENABLE);						//使能串口
}



/*************全局函数定义******************/

/*******************************
名称：Main_Process();
功能：主循环函数，main中进入
参数：null
返回：null
*******************************/
//int count = 0;
//void Main_Process(void)
//{
//	HAL_StatusTypeDef processResult = HAL_INI;
//	#ifdef USE_UART1
//	if(UART1Time2_1ms>30)
//	{
////		TransmitData_API(HAL_USB,"UART1 Received",0,SENDNOPROTOCOL);
//		UART1Time2_1ms = 0;
//		if(UART1RXDataLenth>7)
//			processResult = ColorUpdate(UART1RevData);//协议处理函数

//		UART1RXDataLenth = 0;
////		ResultSend(CurrentPort,processResult);
//		BuffReset_API(UART1RevData,MAXCOMSIZE);
//	}
//	#endif
//	#ifdef USE_UART2
//	if(UART2Time2_1ms>30)
//	{
//		UART_TransmitData_API(USART1,"U2,REC",0,SENDNOPROTOCOL);
//		UART2time3_100us = 0;
//		CurrentPort = HAL_USART2;
//		processResult=CheckProtocol(CurrentPort,UART2RevData);
//		if(processResult == HAL_OK)
//		{
//			processResult = Protocol_Process(CurrentPort,UART2RevData);//协议处理函数
//		}
//		else
//			processResult = HAL_BUSY;
//		UART2RXDataLenth = 0;
//		ResultSend(CurrentPort,UART2RevData,processResult);
//		BuffReset_API(UART2RevData,MAXCOMSIZE);
//	}
//	#endif
//
//
//	if(time2Counter1ms-1 > 100)//100ms执行一次
//	{
////		if(time2Counter1ms>150)TransmitData_API(CurrentPort,"process miss handled",0,SENDNOPROTOCOL);
//
//
//		time2Counter1ms = TIMESTART;
//	}
//}

/*******************************
名称：ResultSend();
功能：回复函数；
参数：
返回：见communication.h
*******************************/
//void ResultSend(ComPort comPort,HAL_StatusTypeDef result)
//{
//	if(result == HAL_OK)
//		TransmitData_SDSES(comPort,0,SDSCmd,OPSUCCESS,NULL);
//}

/*******************************
名称：UART_TransmitData_API();;
功能：串口发送数据
参数：UART_HandleTypeDef *huart串口结构体
			const void* data外部接收数据指针
			uint16_t datasize发送数据大小，防止溢出
			PARAMETER sendmode 发送模式，具体参数见结构体
返回：HAL_StatusTypeDef communication.h
*******************************/
HAL_StatusTypeDef TransmitData_API(USART_TypeDef* huart, const void* data, uint16_t datasize)
{
    uint16_t i, timeout;
    uint8_t* pdata = (uint8_t*) data;
//    USART_TypeDef* huart = USART1;
    if(datasize == 0)
        datasize = strlen((char*)pdata);

//		sendLen = datasize;
//			memcpy(SendBuff,pdata,sendLen);

//    if(comPort == HAL_USART1)
//        huart = USART1;
//    if(comPort == HAL_USART2)
//        huart = USART2;

    for(i = 0; i < datasize; i++)
    {
        USART_SendData(huart, pdata[i]);
        timeout = 0;
        while(USART_GetFlagStatus(huart, USART_FLAG_TXE) == RESET)
        {
            timeout++;
            if(timeout == SystemCoreClock / 1000) //发送超时1ms
                return HAL_TIMEOUT;

#ifdef USE_IWDG
            IWDG_ReloadCounter();
#endif
        }
    }

    return HAL_OK;
}

/************************END******************************/
/*******************************
名称：UART_TransmitData_SDSES()
功能：串口发送数据，适用于神思，自动加入SDsEs,crc
参数：UART_HandleTypeDef *huart串口结构体
			uint32_t 长度，只是data长度,函数中自动加5
			cmdr  命令
			data  发送数据,不包含任何协议内容，只是data数据，或数组
返回：HAL_StatusTypeDef communication.h
*******************************/
uint8_t SendBuff[512];

HAL_StatusTypeDef TransmitData_SDSES(USART_TypeDef* huart, uint32_t len, uint16_t cmdr, uint8_t state, const void* data)
{
    uint16_t i, crc16 = 0, sendLen = 0;
    uint8_t lenth[4];
    uint8_t cmd[2];
    uint8_t crc[2];
    uint8_t* pdata = (uint8_t*) data;
    if(len == 0)
        len = strlen((char*)pdata);
    //头 SDsEs 5
    memcpy(SendBuff, HEAD, HEADSIZE);
    sendLen += HEADSIZE;
    //长度 4字节
    len += 5;            //长度已加5
    for(i = 0; i < 4; i++)
    {
        lenth[i] = (len >> ((3 - i) * 8)) & 0xff;
    }
    memcpy(SendBuff + sendLen, lenth, 4);
    sendLen += 4;
    //CMDR 2字节
    cmd[0] = cmdr >> 8;
    cmd[1] = cmdr & 0xff;
    memcpy(SendBuff + sendLen, cmd, 2);
    sendLen += 2;

    //state 1字节
    SendBuff[sendLen] = state;
    sendLen++;

    //data len字节
    memcpy(SendBuff + sendLen, pdata, len - 5);
    sendLen += len - 5;

    //crc 2字节
    for(i = 0; i < sendLen - 5; i++)
    {
        crc16 = crc16one(SendBuff[i + 5], crc16);
    }
    crc[0] = crc16 >> 8;
    crc[1] = crc16 & 0xFF;
    memcpy(SendBuff + sendLen, crc, 2);
    sendLen += 2;
    TransmitData_API(huart, SendBuff, sendLen);
    return HAL_OK;
}

