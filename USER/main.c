#include "delay.h"
#include "timer.h"
#include "rs232.h"
#include "utility.h"
#include "motor.h"
#include "sensor.h"
#include "rs3485.h"

#include "communication.h"

// TIM14,TIM16 PWM 频率设定
#define INIHz 1000

HAL_StatusTypeDef Protocol_Process(uint8_t* pbuff);

HAL_StatusTypeDef processResult = HAL_INI;


void ResultSend(uint8_t* pbuff, HAL_StatusTypeDef result);
void Main_Process(void);
void Reset(void);
void MotorTime(void);


static void IWDG_Config(void)
{
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_Prescaler_256);
    IWDG_SetReload(2000 * 40 / 256); //2s
    IWDG_ReloadCounter();
    IWDG_Enable();
}

int main(void)
{
    Delay_init();
    TIM3_Initial();
    TIM14_Initial(INIHz);
    TIM16_Initial(INIHz);
//    IWDG_Config();

    SENSOR_Init();
    Motor_Init();

    RS232_Init(UART1BUAD);
    RS485_Init(UART2BUAD);

    RS485_Data_API("XH-Wardrobe-V2.5", 0);

    while (1)
    {
        if(UART2Time_1ms > 30)
        {
            UART2Time_1ms = 1;

            processResult = CheckProtocol(RS485, UART2RevData);

            if(processResult == HAL_OK)
            {
                processResult = Protocol_Process(UART2RevData);//协议处理函数
            }

            UART2RXDataLenth = 0;
            BuffReset_API(UART2RevData, MAXCOMSIZE);
        }
    }
}

/*******************************
名称：Protocol_Process(uint8_t* pbuff,);
功能：协议处理函数
参数：协议数据缓存区pbuff
返回：处理结果，可以在communication.h中添加
*******************************/
//53 44 73 45 73 00 00 00 09 21 01 ff ff 04 00 2a 82 da
//53 44 73 45 73 00 00 00 09 20 02 ff ff 04 00 2a F4 5B
//53 44 73 45 73 00 00 00 06 A1 00 6D 00 2C 4C 2A D4 15 CD 01 00 00 06 A1 00 6D 00 2C 31
HAL_StatusTypeDef Protocol_Process(uint8_t* pbuff)
{
    HAL_StatusTypeDef processResult;
    uint16_t cmdr;

    cmdr = ((uint16_t)pbuff[9] << 8) + pbuff[10];

    switch(cmdr)
    {
    case 0x2101:
        RS485_Data_API("11ab", 0);
        break;
    case 0x2002:
        RS485_Data_API("22cd", 0);
        break;
    case 0x2003:
        RS485_Data_API("33ww", 0);
        break;
    case 0x01:
        RS485_Data_API("444d", 0);
        break;
    case 0x02:
        RS485_Data_API("55g5", 0);
        break;
    default:
        break;
    }
    return processResult;
}


/*******************************
名称：ResultSend(uint8_t* pbuff, uint16_t buffSize)
功能：流程结果上传
参数：pbuff 接收缓冲区，用于非神思协议上传回复命令，result 结果
返回：null
*******************************/
//void ResultSend(uint8_t* pbuff, HAL_StatusTypeDef result)
//{
//		uint8_t dataSDSES[10];
//	  if(motorErrorFlag != MOTOR_OK)
//		{
//			UART_TransmitData_API(USART1,"57MOTOR ERROR",0,SENDNOPROTOCOL);
//			MotorDrive57(MOTORV,MOTOR_STOP);
//			MotorDrive57(MOTORH,MOTOR_STOP);
//			return;
//		}
//		if(result == HAL_ERROR)
//		{
//			UART_TransmitData_SDSES(USART1,0,0xA100,OPFAILED,"");
//		}
//		if(result == HAL_OK)
//		{
//			UART_TransmitData_SDSES(USART1,0,0xA100,OPSUCCESS,"");
//		}
//		if(result == HAL_TIMEOUT)//电机运作超时
//		{
//			motorErrorFlag = MOTOR_OK;
//			MotorDrive57(MOTORV,MOTOR_STOP);
//			MotorDrive57(MOTORH,MOTOR_STOP);
//			dataSDSES[0]=0x02;
//			UART_TransmitData_SDSES(USART1,1,0xA100,OPFAILED,dataSDSES);
//		}
//		if(result == HAL_DCTIMEOUT)
//		{
//			dataSDSES[0] = 0x01;//无货
//			UART_TransmitData_SDSES(USART1,1,0xA100,OPFAILED,dataSDSES);
//		}
//		if(result == HAL_CRCERROR)
//			UART_TransmitData_API(USART1,"CRC ERROR",0,SENDNOPROTOCOL);
//		if(result == HAL_BUSY)
//			UART_TransmitData_API(USART1,"ERROR BUSY",0,SENDNOPROTOCOL);
//
//		cV = 0;
//}


