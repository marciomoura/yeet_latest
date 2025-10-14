#include "main.h"
/*********************************************************************************************************
** USART初始化
*********************************************************************************************************/
 /*
  * @brief  
  * @param  None
  * @retval None
  */


 /* Private macro -------------------------------------------------------------*/
 /* Private variables ---------------------------------------------------------*/

/************************************************************************************************
*串口驱动使用说明																				*
*																								*
*初始化函数																						*
*串口1：Init_RS232_1(波特率，发送字节，触发接收标志的字节数)									*
*串口2：Init_RS232_2(波特率，发送字节，触发接收标志的字节数)									*
*串口3：Init_RS485(波特率，发送字节，触发接收标志的字节数)										*
*波特率不得大于7500000，发送字节＜512，接收数据<256。											*
*																								*
*接收函数																						*
*串口1：FlagStatus RS232_Rx_1(uint8_t* RxData)													*
*串口2：FlagStatus RS232_Rx_2(uint8_t* RxData)													*
*串口3：FlagStatus RS485_Rx(uint8_t* RxData)													*
*接收缓冲中数据＞触发事件的字节则返回1， RxData是用户的接收缓冲。								*
*																								*
*串口1：void RS232_1_Tx(void)																	*
*串口2：void RS232_2_Tx(void)																	*
*串口3：void RS485_Tx(void)																		*
*只写全局变量RS232_1_Tx_Flag/RS232_2_Tx_Flag/RS485_Tx_Flag，用户置位后启动发送，启动后自动清零。*
*只读全局变量RS232_1_Tx_Enable_Flag/RS232_2_Tx_Enable_Flag/RS485_Tx_Enable_Flag，启动发送后     *
*自动清零，发送完成后置位。为0时禁止修改发送数据区。Usart1_Tx_Data/Usart2_Tx_Data是发送数据区。	*				
************************************************************************************************/
#include <stm32f4xx.h>

#define Tx_Buf_Size 512
#define Rx_Buf_Size 512

uint8_t Usart1_Tx_Data[Tx_Buf_Size];
uint8_t Usart1_Rx_Data[Tx_Buf_Size];
uint8_t Usart2_Tx_Data[Rx_Buf_Size];
uint8_t Usart2_Rx_Data[Rx_Buf_Size];
uint8_t Usart3_Tx_Data[Rx_Buf_Size];
uint8_t Usart3_Rx_Data[Rx_Buf_Size];
uint16_t RS232_1_Rx_Count,RS232_2_Rx_Count;
uint16_t RS485_Rx_Count;
uint32_t RS232_1_Tx_Num;
uint32_t RS232_1_Rx_Threshold;
uint32_t RS485_Tx_Num;
uint32_t RS485_Rx_Threshold;
uint8_t RS232_1_Tx_Flag,RS232_2_Tx_Flag;
uint8_t RS232_1_Tx_Enable_Flag,RS232_2_Tx_Enable_Flag;
uint8_t RS485_Tx_Flag;
uint8_t RS485_Tx_Enable_Flag;


/************************************************************************************
*RS232_1_Init_Para																	*
*初始化COM1数据区和相关参数。														*
************************************************************************************/
void RS232_1_Init_Para(void)
{
	RS232_1_Rx_Count=0;
	RS232_1_Tx_Flag=0;
	RS232_1_Tx_Enable_Flag=1;
}

/************************************************************************************
*RS232_2_Init_Para																	*
*初始化COM2数据区和相关参数。														*
************************************************************************************/
void RS232_2_Init_Para(void)
{
	RS232_2_Rx_Count=0;
	RS232_2_Tx_Flag=0;
	RS232_2_Tx_Enable_Flag=1;
	RS485_Rx_Count=0;
}

/************************************************************************************
*RS485_Init_Para																	*
*初始化COM3数据区和相关参数。														*
************************************************************************************/
void RS485_Init_Para(uint32_t Tx_Num, uint32_t Rx_Threshold)
{
	uint32_t i;

	for (i=0;i<Tx_Buf_Size;i++)
	{
		Usart3_Tx_Data[i]=(uint8_t)(i & 0xff);
	}
	for (i=0;i<Rx_Buf_Size;i++)
	{
		Usart3_Rx_Data[i]=0;
	}
	RS485_Rx_Count=0;//数据区已处理数据的指针
	RS485_Tx_Flag=0;
	RS485_Tx_Enable_Flag=1;
	RS485_Tx_Num=Tx_Num;
	RS485_Rx_Threshold=Rx_Threshold;
}

/************************************************************************************
*RS232_1_Init_Pin																		*
*初始化RS232相关引脚。																*
*USART1, Tx-PA9, Rx-PA10。 															*
************************************************************************************/
void RS232_1_Init_Pin()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);//启用USART1时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//启用USART1的Rx、Tx以及USART2的Tx引脚时钟
	
  	//USART1:Tx
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);//PA9指定给USART1
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//USART1:Rx
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);//PA10指定给USART1
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);	
}

/************************************************************************************
*RS232_2_Init_Pin																	*
*初始化RS232相关引脚。																*
*USART2, Tx-PB10, Rx-PB11。														*
************************************************************************************/
void RS232_2_Init_Pin()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);//启用USART3时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//启用USART3的Tx、Rx引脚时钟
  	
  	//USART3:Tx
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);//PB10指定给USART3
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//USART3:Rx
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);//PB11指定给USART3
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);	
}

/************************************************************************************
*RS485_Init_Pin																		*
*初始化RS485相关引脚。																*
*USART2, Tx-PD5, Rx-PD6。															*
************************************************************************************/
void RS485_Init_Pin(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);//启用USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//启用USART2的Tx、Rx引脚时钟
  	
  	//USART2:Tx
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);//PD5指定给USART2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOD, &GPIO_InitStructure);

	//USART2:Rx
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);//PD6指定给USART2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  	GPIO_Init(GPIOD, &GPIO_InitStructure);	
}
				
/************************************************************************************
*RS232_1_Init_Port																	*
*初始化COM1，设置波特率。															*
*************************************************************************************
*BaudRate：0~7,500,000																*
*Parity: USART_Parity_No, USART_Parity_Even, USART_Parity_Odd						*
*WORD_LENGTH: USART_WordLength_8b, USART_WordLength_9b								*
*StopBits: USART_StopBits_1/USART_StopBits_0_5/USART_StopBits_2/USART_StopBits_1_5	*
************************************************************************************/
void RS232_1_Init_Port(uint32_t baudrate,uint16_t parity,uint16_t wordlength,uint16_t stopbits)
{
	USART_InitTypeDef USART_InitStructure;

	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_Parity = parity;
	USART_InitStructure.USART_WordLength = wordlength;
	USART_InitStructure.USART_StopBits = stopbits;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//禁用硬件流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
	USART_Cmd(USART1,ENABLE);  //打开串口
}
/************************************************************************************
*RS232_2_Init_Port																	*
*初始化COM2，设置波特率。															*
*************************************************************************************
*BaudRate：0~7,500,000																*
*Parity: USART_Parity_No, USART_Parity_Even, USART_Parity_Odd						*
*WORD_LENGTH: USART_WordLength_8b, USART_WordLength_9b								*
*StopBits: USART_StopBits_1/USART_StopBits_0_5/USART_StopBits_2/USART_StopBits_1_5	*
************************************************************************************/
void RS232_2_Init_Port(uint32_t baudrate,uint16_t parity,uint16_t wordlength,uint16_t stopbits)
{
	USART_InitTypeDef USART_InitStructure;

	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_Parity = parity;
	USART_InitStructure.USART_WordLength = wordlength;
	USART_InitStructure.USART_StopBits = stopbits;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//禁用硬件流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);
	USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
	USART_Cmd(USART3,ENABLE);  //打开串口
}
/************************************************************************************
*RS485_Init_Port																	*
*初始化COM3，设置波特率。															*
*************************************************************************************
*BaudRate：0~7,500,000																*
*Parity: USART_Parity_No, USART_Parity_Even, USART_Parity_Odd						*
*WORD_LENGTH: USART_WordLength_8b, USART_WordLength_9b								*
*StopBits: USART_StopBits_1/USART_StopBits_0_5/USART_StopBits_2/USART_StopBits_1_5	*
************************************************************************************/
void RS485_Init_Port(uint32_t BaudRate)
{
	USART_InitTypeDef USART_InitStructure;

	USART_InitStructure.USART_BaudRate = BaudRate;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//禁用硬件流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);
	USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
	USART_Cmd(USART2,ENABLE);  //打开串口
}

/************************************************************************************
*RS232_1_Init_DMA																		*
*初始化串口1（RS232）的DMA发送功能。															*
*USART1_Tx-DMA2_Stream7_Channel4													*
************************************************************************************/
void RS232_1_Init_DMA_Tx(void)	
{
	DMA_InitTypeDef  DMA_InitStruct;
	
	//DMA_DeInit(DMA2_Stream7);		//禁用数据流，进入设置状态
	DMA_StructInit(&DMA_InitStruct);//缺省值填充初始化参数
	DMA_InitStruct.DMA_Channel=DMA_Channel_4;
	DMA_InitStruct.DMA_PeripheralBaseAddr=(uint32_t)&USART1->DR;
	DMA_InitStruct.DMA_Memory0BaseAddr=(uint32_t)Usart1_Tx_Data;
	DMA_InitStruct.DMA_DIR=DMA_DIR_MemoryToPeripheral;//发送是内存到外设
	DMA_InitStruct.DMA_BufferSize=512;
	DMA_InitStruct.DMA_PeripheralInc=DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_MemoryInc=DMA_MemoryInc_Enable;//内存地址递增
	DMA_InitStruct.DMA_PeripheralDataSize	=DMA_PeripheralDataSize_Byte;
	DMA_InitStruct.DMA_MemoryDataSize=DMA_MemoryDataSize_Byte;
	DMA_InitStruct.DMA_Mode=DMA_Mode_Normal;
	DMA_InitStruct.DMA_Priority=DMA_Priority_High;
	DMA_InitStruct.DMA_FIFOMode= DMA_FIFOMode_Disable;
	DMA_InitStruct.DMA_FIFOThreshold= DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStruct.DMA_MemoryBurst = DMA_PeripheralBurst_Single;
	
	DMA_Init(DMA2_Stream7, &DMA_InitStruct);
	//DMA_Cmd(DMA2_Stream7, ENABLE);	
}

/************************************************************************************
*RS232_2_Init_DMA															*
*初始化串口2（RS232）的DMA发送功能。											*
*USART3_Tx-DMA1_Stream3_Channel4	                                       *
************************************************************************************/
void RS232_2_Init_DMA_Tx(void)	
{
	DMA_InitTypeDef  DMA_InitStruct;
	
//	DMA_DeInit(DMA1_Stream3);		//禁用数据流，进入设置状态
	DMA_StructInit(&DMA_InitStruct);//缺省值填充初始化参数
	DMA_InitStruct.DMA_Channel=DMA_Channel_4;
	DMA_InitStruct.DMA_PeripheralBaseAddr=(uint32_t)&USART3->DR;
	DMA_InitStruct.DMA_Memory0BaseAddr=(uint32_t)Usart2_Tx_Data;
	DMA_InitStruct.DMA_DIR=DMA_DIR_MemoryToPeripheral;//发送是内存到外设
	DMA_InitStruct.DMA_BufferSize=512;
	DMA_InitStruct.DMA_PeripheralInc=DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_MemoryInc=DMA_MemoryInc_Enable;//内存地址递增
	DMA_InitStruct.DMA_PeripheralDataSize	=DMA_PeripheralDataSize_Byte;
	DMA_InitStruct.DMA_MemoryDataSize=DMA_MemoryDataSize_Byte;
	DMA_InitStruct.DMA_Mode=DMA_Mode_Normal;
	DMA_InitStruct.DMA_Priority=DMA_Priority_High;
	DMA_InitStruct.DMA_FIFOMode= DMA_FIFOMode_Disable;
	DMA_InitStruct.DMA_FIFOThreshold= DMA_FIFOThreshold_1QuarterFull;
 	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStruct.DMA_MemoryBurst = DMA_PeripheralBurst_Single;

	DMA_Init(DMA1_Stream3, &DMA_InitStruct);
	//DMA_Cmd(DMA1_Stream3, ENABLE);
}

/************************************************************************************
*RS232_1_Init_DMA_Rx																	*
*初始化串口1（RS232）的DMA接收功能。															*
*USART1_Rx-DMA2_Stream5_Channel4													*
************************************************************************************/
void RS232_1_Init_DMA_Rx(void)	
{
	DMA_InitTypeDef  DMA_InitStruct;
	
	DMA_DeInit(DMA2_Stream5);		//禁用数据流，进入设置状态
	DMA_StructInit(&DMA_InitStruct);//缺省值填充初始化参数
	DMA_InitStruct.DMA_Channel=DMA_Channel_4;
	DMA_InitStruct.DMA_PeripheralBaseAddr=(uint32_t)&USART1->DR;
	DMA_InitStruct.DMA_Memory0BaseAddr=(uint32_t)Usart1_Rx_Data;
	DMA_InitStruct.DMA_DIR=DMA_DIR_PeripheralToMemory;//接收是外设到内存	
	DMA_InitStruct.DMA_BufferSize=Rx_Buf_Size;
	DMA_InitStruct.DMA_MemoryInc=DMA_MemoryInc_Enable;//内存地址递增
	DMA_InitStruct.DMA_Mode=DMA_Mode_Circular;//循环寻址
	DMA_InitStruct.DMA_Priority=DMA_Priority_High;
	DMA_Init(DMA2_Stream5, &DMA_InitStruct);
	DMA_Cmd(DMA2_Stream5, ENABLE);//启用DMA数据流
	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
}
/************************************************************************************
*RS232_2_Init_DMA_Rx															*
*初始化串口2（RS232）的DMA接收功能。												*
*USART3_Rx-DMA1_Stream1_Channel4													*
************************************************************************************/
void RS232_2_Init_DMA_Rx(void)	
{
	DMA_InitTypeDef  DMA_InitStruct;
	
	DMA_DeInit(DMA1_Stream1);		//禁用数据流，进入设置状态
	DMA_StructInit(&DMA_InitStruct);//缺省值填充初始化参数
	DMA_InitStruct.DMA_Channel=DMA_Channel_4;
	DMA_InitStruct.DMA_PeripheralBaseAddr=(uint32_t)&USART3->DR;
	DMA_InitStruct.DMA_Memory0BaseAddr=(uint32_t)Usart2_Rx_Data;
	DMA_InitStruct.DMA_DIR=DMA_DIR_PeripheralToMemory;//接收是外设到内存	
	DMA_InitStruct.DMA_BufferSize=Rx_Buf_Size;
	DMA_InitStruct.DMA_Mode=DMA_Mode_Circular;//循环寻址
	DMA_InitStruct.DMA_MemoryInc=DMA_MemoryInc_Enable;//内存地址递增
	DMA_InitStruct.DMA_Priority=DMA_Priority_High;
	DMA_Init(DMA1_Stream1, &DMA_InitStruct);
	DMA_Cmd(DMA1_Stream1, ENABLE);//启用DMA数据流
	USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
}
/************************************************************************************
*RS485_Init_DMA																		*
*初始化RS485的DMA发送功能。															*
*USART2_Tx-DMA1_Stream6_Channel4													*
************************************************************************************/
void RS485_Init_DMA_Tx(void)	
{
	DMA_InitTypeDef  DMA_InitStruct;
	
	DMA_DeInit(DMA1_Stream6);		//禁用数据流，进入设置状态
	DMA_StructInit(&DMA_InitStruct);//缺省值填充初始化参数
	DMA_InitStruct.DMA_Channel=DMA_Channel_4;
	DMA_InitStruct.DMA_PeripheralBaseAddr=(uint32_t)&USART2->DR;
	DMA_InitStruct.DMA_Memory0BaseAddr=(uint32_t)Usart3_Tx_Data;
	DMA_InitStruct.DMA_DIR=DMA_DIR_MemoryToPeripheral;//发送是内存到外设
	DMA_InitStruct.DMA_BufferSize=RS485_Tx_Num;
	DMA_InitStruct.DMA_PeripheralInc=DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_MemoryInc=DMA_MemoryInc_Enable;//内存地址递增
	DMA_InitStruct.DMA_PeripheralDataSize =DMA_PeripheralDataSize_Byte;
	DMA_InitStruct.DMA_MemoryDataSize=DMA_MemoryDataSize_Byte;
	DMA_InitStruct.DMA_Mode=DMA_Mode_Normal;
	DMA_InitStruct.DMA_Priority=DMA_Priority_High;
	DMA_InitStruct.DMA_FIFOMode= DMA_FIFOMode_Disable;
	DMA_InitStruct.DMA_FIFOThreshold= DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStruct.DMA_MemoryBurst = DMA_PeripheralBurst_Single;

	DMA_Init(DMA1_Stream6, &DMA_InitStruct);
	DMA_Cmd(DMA1_Stream6, ENABLE);
}

/************************************************************************************
*RS485_Init_DMA_Rx																	*
*初始化RS485的DMA接收功能。															*
*USART2_Rx-DMA1_Stream5_Channel4													*
************************************************************************************/
void RS485_Init_DMA_Rx(void)	
{
	DMA_InitTypeDef  DMA_InitStruct;
	
	DMA_DeInit(DMA1_Stream5);		//禁用数据流，进入设置状态
	DMA_StructInit(&DMA_InitStruct);//缺省值填充初始化参数
	DMA_InitStruct.DMA_Channel=DMA_Channel_4;
	DMA_InitStruct.DMA_PeripheralBaseAddr=(uint32_t)&USART2->DR;
	DMA_InitStruct.DMA_Memory0BaseAddr=(uint32_t)Usart3_Rx_Data;
	DMA_InitStruct.DMA_DIR=DMA_DIR_PeripheralToMemory;//接收是外设到内存	
	DMA_InitStruct.DMA_BufferSize=Rx_Buf_Size;
	DMA_InitStruct.DMA_Mode=DMA_Mode_Circular;//循环寻址
	DMA_InitStruct.DMA_MemoryInc=DMA_MemoryInc_Enable;//内存地址递增
	DMA_InitStruct.DMA_Priority=DMA_Priority_High;
	DMA_Init(DMA1_Stream5, &DMA_InitStruct);
	DMA_Cmd(DMA1_Stream5, ENABLE);//启用DMA数据流
	USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
}

/************************************************************************************
*RS232_1_Init_DMA																		*
*初始化串口1（RS232)的DMA功能。																*
*USART1_Tx-DMA2_Stream7_Channel4, USART1_Rx-DMA2_Stream5_Channel4					*
************************************************************************************/
void RS232_1_Init_DMA(void)	
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);//启用DMA2时钟
	RS232_1_Init_DMA_Tx();
	RS232_1_Init_DMA_Rx();	
}

/************************************************************************************
*RS232_2_Init_DMA																	*
*初始化串口2（RS232)的DMA功能。														*
*USART3_Tx-DMA1_Stream6_Channel4, USART2_Rx-DMA1_Stream5_Channel4				*
************************************************************************************/
void RS232_2_Init_DMA(void)	
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);//启用DMA1时钟
	RS232_2_Init_DMA_Tx();
	RS232_2_Init_DMA_Rx();	
}
/************************************************************************************
*RS485_Init_DMA																		*
*初始化RS485的DMA功能。																*
*USART2_Tx-DMA1_Stream6_Channel4, USART2_Rx-DMA1_Stream5_Channel4					*
************************************************************************************/
void RS485_Init_DMA(void)	
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);//启用DMA1时钟
	RS485_Init_DMA_Tx();
	RS485_Init_DMA_Rx(); 	
}

/************************************************************************************
*RS232_1_Tx																			*
*串口1发送数据区内全部数据。														*
************************************************************************************/
uint8_t RS232_1_Tx(uint8_t tx_data[], uint16_t len)	
{
	uint16_t i;
	uint8_t ret = 0;
	if (DMA_GetFlagStatus(DMA2_Stream7,DMA_FLAG_TCIF7)==SET)//发送完成
	{
		DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7);	//清除标志
		RS232_1_Tx_Enable_Flag=1;	  	//通知外部函数可以修改数据区并启动发送
		ret = 1;
	}
	if (RS232_1_Tx_Enable_Flag)
	{
		
		RS232_1_Tx_Enable_Flag=0;//清除发送允许标志
		DMA2_Stream7->NDTR = len;
		for (i = 0; i < len; i++)		//
		{
			Usart1_Tx_Data[i] = tx_data[i];
		}
		DMA_Cmd(DMA2_Stream7, ENABLE);	//重启DMA
		USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);//启动发送
	}

	//容错处理
	if (DMA_GetFlagStatus(DMA2_Stream7,DMA_FLAG_TEIF7)==SET)//发送错误
	{
		DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TEIF7);	//清除标志
		RS232_1_Init_DMA_Tx(); 		//重启DMA
		RS232_1_Tx_Enable_Flag=1;
	}
	if (DMA_GetFlagStatus(DMA2_Stream7,DMA_FLAG_DMEIF7)==SET)//直接模式错误
	{
		DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_DMEIF7);	//清除标志
		RS232_1_Init_DMA_Tx(); 		//重启DMA
		RS232_1_Tx_Enable_Flag=1;
	}
	return ret;
}

/************************************************************************************
*RS232_2_Tx																			*
*串口2发送数据区内全部数据。														*
************************************************************************************/
uint8_t RS232_2_Tx(uint8_t tx_data[], uint16_t len)	
{
	uint16_t i;
	uint8_t ret = 0;
	if (DMA_GetFlagStatus(DMA1_Stream3,DMA_FLAG_TCIF3)==SET)//发送完成
	{
		DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF3);	//清除标志
		RS232_2_Tx_Enable_Flag=1;	  	//通知外部函数可以修改数据区并启动发送
		ret = 1;
	}

	if (RS232_2_Tx_Enable_Flag)
	{
		RS232_2_Tx_Enable_Flag=0;
		DMA1_Stream3->NDTR = len;
		for (i = 0; i < len; i++)		//
		{
			Usart2_Tx_Data[i] = tx_data[i];
		}
		DMA_Cmd(DMA1_Stream3, ENABLE);				//重启DMA
		USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);//启动发送
	}
	//容错处理
	if (DMA_GetFlagStatus(DMA1_Stream3,DMA_FLAG_TEIF3)==SET)//发送错误
	{
		DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TEIF3);	//清除标志
		RS232_2_Init_DMA_Tx(); 		//重启DMA
		RS232_2_Tx_Enable_Flag=1;
	}
	if (DMA_GetFlagStatus(DMA1_Stream3,DMA_FLAG_DMEIF3)==SET)//直接模式错误
	{
		DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_DMEIF3);	//清除标志
		RS232_2_Init_DMA_Tx(); 		//重启DMA
		RS232_2_Tx_Enable_Flag=1;
	}
	return ret;
}

/************************************************************************************
*RS232_1_Rx																			*
*串口1(RS232)接收数据。																			*
************************************************************************************/
uint16_t RS232_1_Rx(uint8_t* RxData)	
{
	int i;
	uint16_t RxNum;
	uint16_t len = 0;

	RxNum=Rx_Buf_Size-DMA_GetCurrDataCounter(DMA2_Stream5); //计算已接收到的数据数量
	if (RxNum<RS232_1_Rx_Count)
	{
		RxNum+=Rx_Buf_Size;
	}
	if (RxNum-RS232_1_Rx_Count > 0)//命令长度为8字节
	{
		len = RxNum-RS232_1_Rx_Count;
		for (i=0;i<len;i++)
		{
			*RxData++=Usart1_Rx_Data[RS232_1_Rx_Count++];//读取数据，等待后续处理
			if (RS232_1_Rx_Count>=Rx_Buf_Size)
			{
				RS232_1_Rx_Count-=Rx_Buf_Size;
			}
		}
	}
	//容错处理
	if (DMA_GetFlagStatus(DMA2_Stream5,DMA_FLAG_TEIF5)==SET)//发送错误
	{
		DMA_ClearFlag(DMA2_Stream5,DMA_FLAG_TEIF5);	//清除标志
		RS232_1_Init_DMA_Rx(); 		//重启DMA
		len = 0;
	}
	if (DMA_GetFlagStatus(DMA2_Stream7,DMA_FLAG_DMEIF7)==SET)//直接模式错误
	{
		DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_DMEIF7);	//清除标志
		RS232_1_Init_DMA_Rx(); 		//重启DMA
		len = 0;
	}
	return len;
}

/************************************************************************************
*RS232_2_Rx																			*
*串口2(RS232)接收数据。																			*
************************************************************************************/
uint16_t RS232_2_Rx(uint8_t* RxData)	
{
	int i;
	uint16_t RxNum;
	uint16_t len = 0;
	
	RxNum=Rx_Buf_Size-DMA_GetCurrDataCounter(DMA1_Stream1); //计算已接收到的数据数量
	
	if (RxNum<RS232_2_Rx_Count)
	{
		RxNum+=Rx_Buf_Size;
	}
	if(RxNum-RS232_2_Rx_Count > 0)    						//有数据接收
	{
		len = RxNum-RS232_2_Rx_Count;
		for (i=0;i<len;i++)
		{
			*RxData++=Usart2_Rx_Data[RS232_2_Rx_Count++];//读取数据，数据指针递增，等待后续处理
			if (RS232_2_Rx_Count>=Rx_Buf_Size)
			{
				RS232_2_Rx_Count-=Rx_Buf_Size;
			}
		}
	}	
// 	if (RxNum-RS232_2_Rx_Count>=RS232_2_Rx_Threshold)//数据长度为Threshold
// 	{
// 		Rx_Flag=SET;
// 		for (i=0;i<RS232_2_Rx_Threshold;i++)
// 		{
// 			*RxData++=Usart2_Rx_Data[RS232_2_Rx_Count++];//读取数据，数据指针递增，等待后续处理
// 			if (RS232_2_Rx_Count>=Rx_Buf_Size)
// 			{
// 				RS232_2_Rx_Count-=Rx_Buf_Size;
// 			}
// 		}
// 	}
	//容错处理
	if (DMA_GetFlagStatus(DMA1_Stream1,DMA_FLAG_TEIF1)==SET)//发送错误
	{
		DMA_ClearFlag(DMA1_Stream1,DMA_FLAG_TEIF1);	//清除标志
		RS232_2_Init_DMA_Rx(); 		//重启DMA
		len = 0;
	}
	if (DMA_GetFlagStatus(DMA1_Stream1,DMA_FLAG_DMEIF1)==SET)//直接模式错误
	{
		DMA_ClearFlag(DMA1_Stream1,DMA_FLAG_DMEIF1);	//清除标志
		RS232_2_Init_DMA_Rx(); 		//重启DMA
		len = 0;
	}
	return len;
}
/************************************************************************************
*RS485_Tx																			*
*发送数据区内全部数据。																*
************************************************************************************/
void RS485_Tx(void)	
{	
	if (DMA_GetFlagStatus(DMA1_Stream6,DMA_FLAG_TCIF6)==SET)//发送完成
	{
		DMA_ClearFlag(DMA1_Stream6,DMA_FLAG_TCIF6);	//清除标志
		RS485_Tx_Enable_Flag=1;	  	//通知外部函数可以修改数据区并启动发送
	}

	if (RS485_Tx_Flag && RS485_Tx_Enable_Flag)
	{
		RS485_Tx_Flag=0;
		RS485_Tx_Enable_Flag=0;
		DMA_Cmd(DMA1_Stream6, ENABLE);				//重启DMA
		USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);//启动发送
	}
	//容错处理
	if (DMA_GetFlagStatus(DMA1_Stream6,DMA_FLAG_TEIF6)==SET)//发送错误
	{
		DMA_ClearFlag(DMA1_Stream6,DMA_FLAG_TEIF6);	//清除标志
		RS485_Init_DMA_Tx(); 		//重启DMA
		RS485_Tx_Enable_Flag=1;
	}
	if (DMA_GetFlagStatus(DMA1_Stream6,DMA_FLAG_DMEIF6)==SET)//直接模式错误
	{
		DMA_ClearFlag(DMA1_Stream6,DMA_FLAG_DMEIF6);	//清除标志
		RS485_Init_DMA_Tx(); 		//重启DMA
		RS485_Tx_Enable_Flag=1;
	}
}

/************************************************************************************
*RS485_Rx																			*
*接收数据。																			*
************************************************************************************/
FlagStatus RS485_Rx(uint8_t* RxData)	
{
	int i;
	uint32_t RxNum;
	FlagStatus Rx_Flag=RESET;

	RxNum=Rx_Buf_Size-DMA_GetCurrDataCounter(DMA1_Stream5); //计算已接收到的数据数量
	if (RxNum<RS485_Rx_Count)
	{
		RxNum+=Rx_Buf_Size;
	}
	if (RxNum-RS485_Rx_Count>=RS485_Rx_Threshold)//数据长度为Threshold
	{
		Rx_Flag=SET;
		for (i=0;i<RS485_Rx_Threshold;i++)
		{
			*RxData++=Usart3_Rx_Data[RS485_Rx_Count++];//读取数据，数据指针递增，等待后续处理
			if (RS485_Rx_Count>=Rx_Buf_Size)
			{
				RS485_Rx_Count-=Rx_Buf_Size;
			}
		}
	}
	//容错处理
	if (DMA_GetFlagStatus(DMA1_Stream5,DMA_FLAG_TEIF5)==SET)//发送错误
	{
		DMA_ClearFlag(DMA1_Stream5,DMA_FLAG_TEIF5);	//清除标志
		RS485_Init_DMA_Rx(); 		//重启DMA
	}
	if (DMA_GetFlagStatus(DMA1_Stream5,DMA_FLAG_DMEIF5)==SET)//直接模式错误
	{
		DMA_ClearFlag(DMA1_Stream5,DMA_FLAG_DMEIF5);	//清除标志
		RS485_Init_DMA_Rx(); 		//重启DMA
	}
	return Rx_Flag;
}

void Init_RS232_2_NVIC_Cofig(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3); 
	
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
	NVIC_Init(&NVIC_InitStructure);
	
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); 

}

/************************************************************************************
*Init_RS232_1																			*
*初始化串口1。																		*
************************************************************************************/
void Init_RS232_1(uint32_t baudrate,uint16_t parity,uint16_t wordlength,uint16_t stopbits)	
{
	RS232_1_Init_Para();	//数据区初始化
	RS232_1_Init_Pin();	//引脚及时钟初始化
	RS232_1_Init_Port(baudrate,parity,wordlength,stopbits);
	RS232_1_Init_DMA();
}
/************************************************************************************
*Init_RS232_2																			*
*初始化串口2,DMA方式																	*
************************************************************************************/
void Init_RS232_2(uint32_t baudrate,uint16_t parity,uint16_t wordlength,uint16_t stopbits)	
{
	RS232_2_Init_Para();	//数据区初始化
	RS232_2_Init_Pin();	//引脚及时钟初始化
	RS232_2_Init_Port(baudrate,parity,wordlength,stopbits);
	RS232_2_Init_DMA();
	
}
/************************************************************************************
*Init_RS232_2																			*
*初始化串口2，中断接收方式 																	*
************************************************************************************/
void Init_RS232_2_NVIC(uint32_t baudrate,uint16_t parity,uint16_t wordlength,uint16_t stopbits)	
{
	RS232_2_Init_Para();	//数据区初始化
	RS232_2_Init_Pin();	//引脚及时钟初始化
	RS232_2_Init_Port(baudrate,parity,wordlength,stopbits);
	Init_RS232_2_NVIC_Cofig();
	
}
/************************************************************************************
*Init_RS485																			*
*初始化串口2。																		*
************************************************************************************/
void Init_RS485(uint32_t BaudRate, uint32_t Tx_Num, uint32_t Rx_Threshold)	
{
	RS485_Init_Para(Tx_Num, Rx_Threshold);	//数据区初始化
	RS485_Init_Pin();	//引脚及时钟初始化
	RS485_Init_Port(BaudRate);//9600, n, 8, 1
	RS485_Init_DMA();
}


void USART3_IRQHandler(void)
{
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        //
        /* Read one byte from the receive data register */
        //unsigned char ch = USART_ReceiveData(USART6);
       // ch = USART_ReceiveData(USART6);
       // Uart_CharRxReady(ch);
            
        USART_ClearITPendingBit(USART3,USART_IT_RXNE);
    }
}

//===========================================================================
// End of file.
//===========================================================================
