
/************************************************************************************
*USARTx.c                                                                           *
*串口驱动                                                                           *
*                                                                                   *
*初始化函数                                                                         *
*串口1：Init_RS232(波特率，发送字节，触发接收标志的字节数)                          *
*串口2：Init_RS485(波特率，发送字节，触发接收标志的字节数)                          *
*波特率不得大于7500000，发送字节＜512，接收数据<256。                               *
*                                                                                   *
*接收函数                                                                           *
*串口1：FlagStatus RS232_Rx(unsigned char* RxData)                                  *
*串口2：FlagStatus RS485_Rx(unsigned char* RxData)                                  *
*接收缓冲中数据＞触发事件的字节则返回1， RxData是用户的接收缓冲。                   *
*                                                                                   *
*串口1：void RS232_Tx(void)                                                         *
*串口2：void RS485_Tx(void)                                                         *
*只写全局变量RS232_Tx_Flag/RS485_Tx_Flag，用户置位后启动发送，启动后自动清零。      *
*只读全局变量RS232_Tx_Enable_Flag/RS485_Tx_Enable_Flag，启动发送后自动清零，发送完成*
*后置位。为0时禁止修改发送数据区。Usart1_Tx_Data/Usart2_Tx_Data是发送数据区。       *
************************************************************************************/
//#include "main.h"
#include "stm32f4xx.h"                  // Device header

#define Rx_Buf_Size 512
#define Tx_Buf_Size 512

#define Rx_Delay_max 10

unsigned char Usart1_Tx_Data[Tx_Buf_Size];
unsigned char Usart1_Rx_Data[Tx_Buf_Size];
unsigned char Usart2_Tx_Data[Rx_Buf_Size];
unsigned char Usart2_Rx_Data[Rx_Buf_Size];
unsigned char RS232_DMA_Tx_Flag;
unsigned char RS485_DMA_Tx_Flag;
unsigned short int RS232_Rx_Count;
unsigned short int RS485_Rx_Count;
unsigned char RxData[8];
unsigned char RxData2[8];
unsigned int RS232_Tx_Num;
unsigned int RS232_Rx_Num_Old;
unsigned int RS232_Rx_Threshold;
unsigned int RS485_Tx_Num;
unsigned int RS485_Rx_Num_Old;
unsigned int RS485_Rx_Threshold;
//unsigned char RS232_Tx_Flag;
unsigned char RS232_Tx_Enable_Flag;
//unsigned char RS485_Tx_Flag;
unsigned char RS485_Tx_Enable_Flag;
int RS485_Delay;
int RS232_Delay;

/********************************************************************************
*Rx_Check                                                                       *
*接收数据帧是否破碎: 接收缓冲是否存在数据且长度很久未变.                        *
********************************************************************************/
unsigned char Rx_Check(unsigned int Rx_Num, unsigned int* Rx_Old, int* Rx_Delay)
{
	unsigned char Err_Flag=0;
	
	if (Rx_Num!=*Rx_Old)
	{
		*Rx_Old=Rx_Num;
		*Rx_Delay=0;
	}
	else if (Rx_Num>0)
	{
		*Rx_Delay=1+(*Rx_Delay);
		if (*Rx_Delay>Rx_Delay_max)
		{
			Rx_Delay=0;
			Err_Flag=1;
		}
	}
	
	return Err_Flag;
}

/********************************************************************************
*RS232_Init_Para                                                                *
*初始化COM1数据区和相关参数。                                                   *
********************************************************************************/
void RS232_Init_Para(unsigned int Tx_Num, unsigned int Rx_Threshold)
{
	unsigned int i;

	for (i=0;i<Tx_Buf_Size;i++)
	{
		Usart1_Tx_Data[i]=(unsigned char)(i & 0xff);
	}
	for (i=0;i<Rx_Buf_Size;i++)
	{
		Usart1_Rx_Data[i]=0;
		Usart2_Rx_Data[i]=0;
	}
	for (i=0;i<8;i++)
	{
		RxData[i]=0;
		RxData2[i]=0;
	}
	RS232_DMA_Tx_Flag=1;
	RS232_Rx_Count=0;
	//RS232_Tx_Flag=0;
	RS232_Tx_Enable_Flag=1;
	RS232_Tx_Num=Tx_Num;
	RS232_Rx_Threshold=Rx_Threshold;
	RS232_Delay=0;
	RS232_Rx_Num_Old=0;
}
/********************************************************************************
*RS485_Init_Para                                                                *
*初始化COM2数据区和相关参数。                                                   *
********************************************************************************/
void RS485_Init_Para(unsigned int Tx_Num, unsigned int Rx_Threshold)
{
	unsigned int i;

	for (i=0;i<Tx_Buf_Size;i++)
	{
		Usart2_Tx_Data[i]=(unsigned char)(i & 0xff);
	}
	for (i=0;i<Rx_Buf_Size;i++)
	{
		Usart2_Rx_Data[i]=0;
	}
	RS485_DMA_Tx_Flag=1;
	RS485_Rx_Count=0;//数据区已处理数据的指针
	//RS485_Tx_Flag=0;
	RS485_Tx_Enable_Flag=1;
	RS485_Tx_Num=Tx_Num;
	RS485_Rx_Threshold=Rx_Threshold;
	RS485_Delay=0;
	RS485_Rx_Num_Old=0;
}

/********************************************************************************
*RS232_Init_Pin                                                                 *
*初始化RS232相关引脚。                                                          *
*USART1, Tx-PA9, Rx-PA10。                                                      *
********************************************************************************/
void RS232_Init_Pin()
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

/********************************************************************************
*RS485_Init_Pin                                                                 *
*初始化RS485相关引脚。                                                          *
*USART2, Tx-PA2, Rx-PD6。                                                       *
********************************************************************************/
void RS485_Init_Pin(void)
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
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);//PD6指定给USART2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOB, &GPIO_InitStructure);		
}
				
/********************************************************************************
*RS232_Init_Port                                                                *
*初始化COM1，设置波特率。                                                       *
*********************************************************************************
*BaudRate：0~7,500,000                                                          *
*Parity: USART_Parity_No, USART_Parity_Even, USART_Parity_Odd                   *
*WORD_LENGTH: USART_WordLength_8b, USART_WordLength_9b                          *
*StopBits: USART_StopBits_1/USART_StopBits_0_5/USART_StopBits_2                 *
* /USART_StopBits_1_5                                                           *
********************************************************************************/
void RS232_Init_Port(unsigned int BaudRate)
{
	USART_InitTypeDef USART_InitStructure;

	USART_InitStructure.USART_BaudRate = BaudRate;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//禁用硬件流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
	USART_Cmd(USART1,ENABLE);  //打开串口
}

/********************************************************************************
*RS485_Init_Port                                                                *
*初始化COM2，设置波特率。                                                       *
*********************************************************************************
*BaudRate：0~7,500,000                                                          *
*Parity: USART_Parity_No, USART_Parity_Even, USART_Parity_Odd                   *
*WORD_LENGTH: USART_WordLength_8b, USART_WordLength_9b                          *
*StopBits: USART_StopBits_1/USART_StopBits_0_5/USART_StopBits_2                 *
* /USART_StopBits_1_5                                                           *
********************************************************************************/
void RS485_Init_Port(unsigned int BaudRate)
{
	USART_InitTypeDef USART_InitStructure;

	USART_InitStructure.USART_BaudRate = BaudRate;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//禁用硬件流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);
	USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
	USART_Cmd(USART3,ENABLE);  //打开串口
}

/********************************************************************************
*RS232_Init_DMA                                                                 *
*初始化RS232的DMA发送功能。                                                     *
*USART1_Tx-DMA2_Stream7_Channel4                                                *
********************************************************************************/
void RS232_Init_DMA_Tx(void)	
{
	DMA_InitTypeDef  DMA_InitStruct;
	
	//DMA_DeInit(DMA2_Stream7);		//禁用数据流，进入设置状态
	DMA_StructInit(&DMA_InitStruct);//缺省值填充初始化参数
	DMA_InitStruct.DMA_Channel=DMA_Channel_4;
	DMA_InitStruct.DMA_PeripheralBaseAddr=(unsigned int)&USART1->DR;
	DMA_InitStruct.DMA_Memory0BaseAddr=(unsigned int)Usart1_Tx_Data;
	DMA_InitStruct.DMA_DIR=DMA_DIR_MemoryToPeripheral;//发送是内存到外设
	DMA_InitStruct.DMA_BufferSize=RS232_Tx_Num;
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
	DMA_Cmd(DMA2_Stream7, ENABLE);	
}

/********************************************************************************
*RS232_Init_DMA_Rx                                                              *
*初始化RS232的DMA接收功能。                                                     *
*USART1_Rx-DMA2_Stream5_Channel4                                                *
********************************************************************************/
void RS232_Init_DMA_Rx(void)	
{
	DMA_InitTypeDef  DMA_InitStruct;
	
	DMA_DeInit(DMA2_Stream5);		//禁用数据流，进入设置状态
	DMA_StructInit(&DMA_InitStruct);//缺省值填充初始化参数
	DMA_InitStruct.DMA_Channel=DMA_Channel_4;
	DMA_InitStruct.DMA_PeripheralBaseAddr=(unsigned int)&USART1->DR;
	DMA_InitStruct.DMA_Memory0BaseAddr=(unsigned int)Usart1_Rx_Data;
	DMA_InitStruct.DMA_DIR=DMA_DIR_PeripheralToMemory;//接收是外设到内存	
	DMA_InitStruct.DMA_BufferSize=Rx_Buf_Size;
	DMA_InitStruct.DMA_MemoryInc=DMA_MemoryInc_Enable;//内存地址递增
	DMA_InitStruct.DMA_Mode=DMA_Mode_Circular;//循环寻址
	DMA_InitStruct.DMA_Priority=DMA_Priority_High;
	DMA_Init(DMA2_Stream5, &DMA_InitStruct);
	DMA_Cmd(DMA2_Stream5, ENABLE);//启用DMA数据流
	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
}

/********************************************************************************
*RS485_Init_DMA                                                                 *
*初始化RS485的DMA发送功能。                                                     *
*USART3_Tx-DMA1_Stream3_Channel4                                                *
********************************************************************************/
void RS485_Init_DMA_Tx(void)	
{
	DMA_InitTypeDef  DMA_InitStruct;
	
	DMA_DeInit(DMA1_Stream3);		//禁用数据流，进入设置状态
	DMA_StructInit(&DMA_InitStruct);//缺省值填充初始化参数
	DMA_InitStruct.DMA_Channel=DMA_Channel_4;
	DMA_InitStruct.DMA_PeripheralBaseAddr=(unsigned int)&USART3->DR;
	DMA_InitStruct.DMA_Memory0BaseAddr=(unsigned int)Usart2_Tx_Data;
	DMA_InitStruct.DMA_DIR=DMA_DIR_MemoryToPeripheral;//发送是内存到外设
	DMA_InitStruct.DMA_BufferSize=RS485_Tx_Num;
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

/********************************************************************************
*RS485_Init_DMA_Rx                                                              *
*初始化RS485的DMA接收功能。                                                     *
*USART3_Rx-DMA1_Stream1_Channel4                                                *
********************************************************************************/
void RS485_Init_DMA_Rx(void)	
{
	DMA_InitTypeDef  DMA_InitStruct;
	
	DMA_DeInit(DMA1_Stream1);		//禁用数据流，进入设置状态
	DMA_StructInit(&DMA_InitStruct);//缺省值填充初始化参数
	DMA_InitStruct.DMA_Channel=DMA_Channel_4;
	DMA_InitStruct.DMA_PeripheralBaseAddr=(unsigned int)&USART3->DR;
	DMA_InitStruct.DMA_Memory0BaseAddr=(unsigned int)Usart2_Rx_Data;
	DMA_InitStruct.DMA_DIR=DMA_DIR_PeripheralToMemory;//接收是外设到内存	
	DMA_InitStruct.DMA_BufferSize=Rx_Buf_Size;
	DMA_InitStruct.DMA_Mode=DMA_Mode_Circular;//循环寻址
	DMA_InitStruct.DMA_MemoryInc=DMA_MemoryInc_Enable;//内存地址递增
	DMA_InitStruct.DMA_Priority=DMA_Priority_High;
	DMA_Init(DMA1_Stream1, &DMA_InitStruct);
	DMA_Cmd(DMA1_Stream1, ENABLE);//启用DMA数据流
	USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
}

/********************************************************************************
*RS232_Init_DMA                                                                 *
*初始化RS232的DMA功能。                                                         *
*USART1_Tx-DMA2_Stream7_Channel4, USART1_Rx-DMA2_Stream5_Channel4               *
********************************************************************************/
void RS232_Init_DMA(void)	
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);//启用DMA2时钟
	RS232_Init_DMA_Tx();
	RS232_Init_DMA_Rx();	
}

/********************************************************************************
*RS485_Init_DMA                                                                 *
*初始化RS485的DMA功能。                                                         *
*USART3_Tx-DMA1_Stream3_Channel4, USART3_Rx-DMA1_Stream1_Channel4               *
********************************************************************************/
void RS485_Init_DMA(void)	
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);//启用DMA1时钟
	RS485_Init_DMA_Tx();
	RS485_Init_DMA_Rx(); 	
}

/********************************************************************************
*RS232_Tx                                                                       *
*发送数据区内全部数据。                                                         *
********************************************************************************/
void RS232_Tx(unsigned char* RS232_Tx_Flag, unsigned char* RS232_Tx_Data)	
{
	int i;

	if (*RS232_Tx_Flag && RS232_Tx_Enable_Flag)
	{
		for (i=0;i<RS232_Tx_Num;i++)//将发送数据从外部数据区复制到内部数据区
		{
			Usart1_Tx_Data[i]=*RS232_Tx_Data++;
		}
		*RS232_Tx_Flag=0;//允许修改外部数据区
		RS232_Tx_Enable_Flag=0;//清除发送允许标志
		
		DMA_Cmd(DMA2_Stream7, ENABLE);//重启DMA
		USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);//启动发送
	}
	if (DMA_GetFlagStatus(DMA2_Stream7,DMA_FLAG_TCIF7)==SET)//发送完成
	{
		DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7);	//清除标志
		RS232_Tx_Enable_Flag=1;		//通知外部函数可以修改数据区并启动发送
	}
	//容错处理
	if (DMA_GetFlagStatus(DMA2_Stream7,DMA_FLAG_TEIF7)==SET)//发送错误
	{
		DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TEIF7);	//清除标志
		RS232_Init_DMA_Tx(); 		//重启DMA
		RS232_Tx_Enable_Flag=1;
	}
	if (DMA_GetFlagStatus(DMA2_Stream7,DMA_FLAG_DMEIF7)==SET)//直接模式错误
	{
		DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_DMEIF7);	//清除标志
		RS232_Init_DMA_Tx(); 		//重启DMA
		RS232_Tx_Enable_Flag=1;
	}
}

/********************************************************************************
*RS232_Rx                                                                       *
*接收数据。                                                                     *
********************************************************************************/
unsigned char RS232_Rx(unsigned char* RxData)	
{
	int i;
	int RxNum;
	unsigned char Rx_Flag=0;

	RxNum=Rx_Buf_Size-DMA_GetCurrDataCounter(DMA2_Stream5); //计算已接收到的数据数量
	if (RxNum<RS232_Rx_Count)
	{
		RxNum+=Rx_Buf_Size;
	}
	if (RxNum-RS232_Rx_Count>=RS232_Rx_Threshold)//命令长度为8字节
	{
		Rx_Flag=1;
		for (i=0;i<RS232_Rx_Threshold;i++)
		{
			*RxData++=Usart1_Rx_Data[RS232_Rx_Count++];//读取数据，等待后续处理
			if (RS232_Rx_Count>=Rx_Buf_Size)
			{
				RS232_Rx_Count-=Rx_Buf_Size;
			}
		}
	}
	//容错处理
	if (DMA_GetFlagStatus(DMA2_Stream5,DMA_FLAG_TEIF5)==SET)//发送错误
	{
		DMA_ClearFlag(DMA2_Stream5,DMA_FLAG_TEIF5);	//清除标志
		RS232_Init_DMA_Rx(); 		//重启DMA
	}
	if (DMA_GetFlagStatus(DMA2_Stream7,DMA_FLAG_DMEIF7)==SET)//直接模式错误
	{
		DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_DMEIF7);	//清除标志
		RS232_Init_DMA_Rx(); 		//重启DMA
	}
	return Rx_Flag;
}

/********************************************************************************
*RS485_Tx                                                                       *
*发送数据区内全部数据。                                                         *
********************************************************************************/
void RS485_Tx(unsigned char* RS485_Tx_Flag, unsigned char* RS485_Tx_Data)	
{	
	unsigned short i;	
	
	if (DMA_GetFlagStatus(DMA1_Stream3,DMA_FLAG_TCIF3)==SET)//发送完成
	{
		DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF3);	//清除标志
		RS485_Tx_Enable_Flag=1;		//通知外部函数可以修改数据区并启动发送
	}

	if (*RS485_Tx_Flag && RS485_Tx_Enable_Flag)
	{
		for (i=0;i<RS485_Tx_Num;i++)//将发送数据从外部数据区复制到内部数据区
		{
			Usart2_Tx_Data[i]=*RS485_Tx_Data++;
		}
		*RS485_Tx_Flag=0;
		RS485_Tx_Enable_Flag=0;
		DMA_Cmd(DMA1_Stream3, ENABLE);//重启DMA
		USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);//启动发送
	}

	

	//容错处理
	if (DMA_GetFlagStatus(DMA1_Stream3,DMA_FLAG_TEIF3)==SET)//发送错误
	{
		DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TEIF3);	//清除标志
		RS485_Init_DMA_Tx(); 		//重启DMA
		RS485_Tx_Enable_Flag=1;
	}
	if (DMA_GetFlagStatus(DMA1_Stream3,DMA_FLAG_DMEIF3)==SET)//直接模式错误
	{
		DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_DMEIF3);	//清除标志
		RS485_Init_DMA_Tx(); 		//重启DMA
		RS485_Tx_Enable_Flag=1;
	}
}

/********************************************************************************
*RS485_Rx                                                                       *
*接收数据。                                                                     *
********************************************************************************/
unsigned char RS485_Rx(unsigned char* RxData)	
{
	int i;
	unsigned int RxNum;
	unsigned char Rx_Flag=0;

	RxNum=Rx_Buf_Size-DMA_GetCurrDataCounter(DMA1_Stream1); //计算已接收到的数据数量
	if (RxNum<RS485_Rx_Count)
	{
		RxNum+=Rx_Buf_Size;
	}
	if (RxNum-RS485_Rx_Count>=RS485_Rx_Threshold)//数据长度为Threshold
	{
		Rx_Flag=1;
		for (i=0;i<RS485_Rx_Threshold;i++)
		{
			*RxData++=Usart2_Rx_Data[RS485_Rx_Count++];//读取数据，数据指针递增，等待后续处理
			if (RS485_Rx_Count>=Rx_Buf_Size)
			{
				RS485_Rx_Count-=Rx_Buf_Size;
			}
		}
	}
	
//	if (Rx_Check(RxNum, &RS485_Rx_Num_Old, &RS485_Delay))
//	{
//		Rx_Flag=1;
//		for (i=0;i<RxNum;i++)
//		{
//			*RxData++=Usart2_Rx_Data[RS485_Rx_Count++];
//			
//			if (RS485_Rx_Count>=Rx_Buf_Size)
//			{
//				RS485_Rx_Count-=Rx_Buf_Size;
//			}
//		}
//		for (i=RxNum;i<RS485_Rx_Threshold;i++)
//		{
//			*RxData++=0;
//		}
//	}
		
	//容错处理
	if (DMA_GetFlagStatus(DMA1_Stream1,DMA_FLAG_TEIF1)==SET)//发送错误
	{
		DMA_ClearFlag(DMA1_Stream1,DMA_FLAG_TEIF1);	//清除标志
		RS485_Init_DMA_Rx(); 		//重启DMA
	}
	if (DMA_GetFlagStatus(DMA1_Stream1,DMA_FLAG_DMEIF1)==SET)//直接模式错误
	{
		DMA_ClearFlag(DMA1_Stream1,DMA_FLAG_DMEIF1);	//清除标志
		RS485_Init_DMA_Rx(); 		//重启DMA
	}
	return Rx_Flag;
}

/********************************************************************************
*Init_RS232                                                                     *
*初始化串口1。                                                                  *
********************************************************************************/
void Init_RS232(unsigned int BaudRate, unsigned int Tx_Num, unsigned int Rx_Threshold)	
{
	RS232_Init_Para(Tx_Num, Rx_Threshold);	//数据区初始化
	RS232_Init_Pin();	//引脚及时钟初始化
	RS232_Init_Port(BaudRate);//9600, n, 8, 1
	RS232_Init_DMA();
}

/********************************************************************************
*Init_RS485                                                                     *
*初始化串口2。                                                                  *
********************************************************************************/
void Init_RS485(unsigned int BaudRate, unsigned int Tx_Num, unsigned int Rx_Threshold)	
{
	RS485_Init_Para(Tx_Num, Rx_Threshold);	//数据区初始化
	RS485_Init_Pin();	//引脚及时钟初始化
	RS485_Init_Port(BaudRate);//9600, n, 8, 1
	RS485_Init_DMA();
}

//===========================================================================
// End of file.
//===========================================================================
