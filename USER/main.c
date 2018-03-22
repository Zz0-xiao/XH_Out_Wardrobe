#include "delay.h"
#include "timer.h"
#include "rs232.h"
#include "utility.h"
#include "motor.h"
#include "sensor.h"
#include "rs3485.h"
// TIM14,TIM16 PWM 频率设定
#define INIHz 1000

HAL_StatusTypeDef Protocol_Process(uint8_t* pbuff);

void ResultSend(uint8_t* pbuff, HAL_StatusTypeDef result);
void Main_Process(void);
void Reset(void);
void MotorTime(void);


uint8_t rs485buf[5]  ;
uint8_t rs485buf3[10] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};


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

    uint16_t cmdr;

    Delay_init();

    TIM3_Initial();
    TIM14_Initial(INIHz);
    TIM16_Initial(INIHz);
//    UART_Initial(USART1, 9600);
//    UART_TransmitData_API(USART1, "XH-Wardrobe-V2.5", 0, SENDNOPROTOCOL);
//    IWDG_Config();

    SENSOR_Init();
    Motor_Init();
	
		RS232_Init(UART1BUAD);
    RS485_Init(UART2BUAD);
	
    while (1)
    {
//        if(time3Usart1ms > 1000)
//        {
//            RS485_Send_Data(rs485buf, 6); //发送5个字节
//            time3Usart1ms = 1;
//        }

//        RS485_Receive_Data(rs485buf);
        cmdr = rs485buf[0];
//			cmdr = ((uint16_t)rs485buf[9]<<8)+rs485buf[10];
        switch(cmdr)
        {
        case 0x01:
//           rs485TransmitData_API("aaaa", 0);
            break;
        case 0x02:
//            rs485TransmitData_API("bbb", 0);
            break;
        case 0x03:
//           rs485TransmitData_API("ccccccccc", 0);
            break;
        case 0x2011:

            break;
        case 0x2018:

            break;
        default:
            break;
        }

        rs485buf[0]=0x00;

    }
}

/*******************************
名称：Protocol_Process(uint8_t* pbuff,);
功能：协议处理函数
参数：协议数据缓存区pbuff
返回：处理结果，可以在communication.h中添加
*******************************/
//const uint32_t DEV_ID = 0xffffffff;//初始必须为ffffffff,否则写入不成功

//HAL_StatusTypeDef Protocol_Process(uint8_t* pbuff)
//{
//	HAL_StatusTypeDef processResult;
//	uint16_t cmdr;
//	uint8_t senddata;
//	uint32_t setID;
//	uint8_t sensorStatus[11];
//	int i = 0;
//	cmdr = ((uint16_t)pbuff[9]<<8)+pbuff[10];
//	indexKHz = 0;
//	cV = 0;
//	switch(cmdr)
//	{
//		case RSENDOUT_SDSES:
//			processResult = SendOut(&pbuff[11]);
//			break;
//		case RMOTOR_SDSES:
//			processResult = Debug_Process(&pbuff[11]);
//			break;
//		case RSENSOR_SDSES:
//			for(i=0;i<6;i++)
//			{
//				if(GPIO_ReadInputDataBit(VSENSOR[i].GPIOx,VSENSOR[i].GPIO_Pin)==Bit_RESET)
//					sensorStatus[i]='1'+i;
//				else
//					sensorStatus[i]='0';
//			}
//			for(i=6;i<8;i++)
//			{
//				if(GPIO_ReadInputDataBit(HSENSOR[i-6].GPIOx,HSENSOR[i-6].GPIO_Pin)==Bit_RESET)
//					sensorStatus[i]='1'+i-6;
//				else
//					sensorStatus[i]='0';
//			}
//			for(i=8;i<11;i++)
//			{
//				if(GPIO_ReadInputDataBit(INFRARE[i-8].GPIOx,INFRARE[i-8].GPIO_Pin)==Bit_SET)
//					sensorStatus[i]='1'+i-8;
//				else
//					sensorStatus[i]='0';
//			}
//			UART_TransmitData_API(USART1,"\r\nVERTICAL:",0,SENDNOPROTOCOL);
//			UART_TransmitData_API(USART1,sensorStatus,6,SENDNOPROTOCOL);
//			UART_TransmitData_API(USART1,"\r\nHORIZENTAL:",0,SENDNOPROTOCOL);
//			UART_TransmitData_API(USART1,sensorStatus+6,2,SENDNOPROTOCOL);
//			UART_TransmitData_API(USART1,"\r\nINFRARE:",0,SENDNOPROTOCOL);
//			UART_TransmitData_API(USART1,sensorStatus+8,3,SENDNOPROTOCOL);
//			break;
//		case 0x2011:
//			senddata = motorCurrentPosition[MOTORV]+'0';
//			UART_TransmitData_API(USART1,"MOTORV POSITION",0,SENDNOPROTOCOL);
//			UART_TransmitData_API(USART1,&senddata,1,SENDNOPROTOCOL);
//			UART_TransmitData_API(USART1,"MOTORV END",0,SENDNOPROTOCOL);
//		  motorErrorFlag = MOTOR_OK;
//			processResult = HAL_OK;
//			break;
//		case 0x2018:
//			UART_TransmitData_API(USART1,"XH-Wardrobe-V2.5",0,SENDNOPROTOCOL);
//			break;
//		default:
//			break;
//	}
//	return processResult;
//    return HAL_OK;
//}


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


