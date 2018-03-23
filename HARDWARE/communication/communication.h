#ifndef __COMMUNICATION_H__
#define __COMMUNICATION_H__

#include "stm32f0xx.h"

/**********************结构体定义区********************/

//	#define RVERSION_SDSES 0x7F01
//	#define RHEART_SDSES 0x7F02
//	#define RSENDOUT_SDSES 0x2100
//	#define RMOTOR_SDSES 0x2101
//	#define RSENSOR_SDSES 0x2102
//
//
//全局标志位结构体
typedef enum
{
    HAL_INI = 0,
    HAL_OK,
    HAL_ERROR,
    HAL_BUSY,
    HAL_TIMEOUT,

    HAL_UART1TIMEOUT,//串口1接收超时
    HAL_UART2TIMEOUT,//串口2接收超时
    HAL_UART1RXFULL,//串口1接收满
    HAL_UART2RXFULL,//串口2接收满
    HAL_TIMMISS,

    HAL_NULL,//没有接收到数据
    HAL_HEADERROR,//收到头错误
    HAL_LENGTHERROR,//长度错误
    HAL_DATAERROR,//数据内容有误
    HAL_ENDERROR, //头正确，尾错误
    HAL_XORERROR,//校验位错误
    HAL_CRCERROR,

    HAL_ENABLE,
    HAL_DISABLE,

} HAL_StatusTypeDef;
//数据传输标志位结构体
//typedef enum
//{
//    SENDNOPROTOCOL = 0,//发送函数专用，发送数据是否添加头尾，以及协议
//    SENDPROTOCOL,
//    SENDNOHEAD,
//    SENDNOEND,

//    UARTSETNOPARAMETER,//UART初始化专用，是否带参数初始化
//    UARTSETPARAMETER
//} PARAMETER;


/***************通讯参数定义区*********************/
#define RS232 USART1
#define RS485 USART2

//接收缓冲区大小
#define MAXCOMSIZE 512

//USART1使能
#define USE_UART1
//串口1波特率
#define UART1BUAD 9600
//USART2使能

#define USE_UART2
//串口1波特率
#define UART2BUAD 9600
//看门狗开启
#define USE_IWDG

//回复命令，
#define OPSUCCESS 0x90
#define OPFAILED  0x6D
/**********************END********************/

/**********************函数体定义区********************/

void	UART_Initial(USART_TypeDef* huart, int buad);


/*******************************
名称：TransmitData_API();;
功能：串口发送数据，适用于神思旭辉，自动加入SUNRISE头，以及0x0d，0x0a结尾
参数：ComPort comPort通讯口结构体
			uint8_t *pdata外部接收数据指针
			UARTRevDef UARTRec同返回参数
			uint16_t datasize发送数据大小，防止溢出

返回：UARTResult
				接收状态
			  HAL_OK       = 接收成功
				HAL_ERROR    = 接收数据超出缓冲区，需要重新发送
				HAL_BUSY     = 串口没准备好
				HAL_TIMEOUT  = reserve
		Size 接收数据大小
*******************************/
//HAL_StatusTypeDef TransmitData_API(ComPort comPort, const void* data, uint16_t datasize );
HAL_StatusTypeDef TransmitData_API(USART_TypeDef* huart, const void* data, uint16_t datasize);

/*******************************
名称：TransmitData_SDSES);;
功能：串口发送数据，适用于神思，自动加入SDsEs,crc
参数：UART_HandleTypeDef *huart串口结构体
			uint32_t 长度，data长度,函数中自动加5
			cmdr  命令
			data  发送数据
返回：HAL_StatusTypeDef
		Size 接收数据大小
*******************************/
//HAL_StatusTypeDef TransmitData_SDSES(ComPort comPort, uint32_t len, uint16_t cmdr, uint8_t state, const void* data);
HAL_StatusTypeDef TransmitData_SDSES(USART_TypeDef* huart, uint32_t len, uint16_t cmdr, uint8_t state, const void* data);

HAL_StatusTypeDef CheckCrc(uint8_t *p);

HAL_StatusTypeDef CheckProtocol(USART_TypeDef* huart, uint8_t* pbuff);

#endif
