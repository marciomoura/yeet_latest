/**************************************************************************************************
* @file    w5200.c                                                                                *
* @author  能事达电气                                                                             *
* @version V1.0.0                                                                                 *
* @date    2016.8.10.                                                                             *
* @brief   w5200网口驱动                                                                          *
**************************************************************************************************/
#include <stm32f4xx.h>
#include "w5200Reg.h"

extern unsigned char uchNodeNum;

void Network_Init(SPI_TypeDef *SPIx, unsigned char* Self_IP, unsigned char* Socket_Enable, unsigned short int* Self_Port, 
unsigned char* Socket_mode, unsigned char* Dest_IP, unsigned short int* Dest_Port, int DMA_Size);

/********************************************************************
*Soft_Delay                                                         *
*软件延时。                                                         *
********************************************************************/
void Soft_Delay(unsigned int x)
{
	unsigned int i;
	for(i=0;i<x;i++);
}

/********************************************************************
*Net_Para_Init                                                      *
*初始化Net数据区和相关参数。                                        *
********************************************************************/
void Net_Para_Init(int Index)
{
	unsigned char i;

	Net_State[Index]=0; 					 							//状态变量
	Net_Rx_size[Index]=0;	
	
   	for(i= 0; i < 2; i++)
	{
		Net_Rx_Addr_Offset[Index][i] = 0;
		Net_Tx_Addr_Offset[Index][i] = 0;
		Net_Socket_Rx_Addr[Index][i]=Net_Rx_Addr[i];
		Net_Socket_Tx_Addr[Index][i]=Net_Tx_Addr[i];
		Net_Socket_State[Index][i] = 0;									//网口1(Socket0...Socket7)状态记录初始化
		Net_Socket_Data[Index][i] = 0;									//网口1(Socket0...Socket7)接收和发送数据的状态初始化
		Net_Tx_Flag[Index][i]=0;
		Net_Rx_Flag[Index][i]=0;
	}
	Net_Receiving_Socket[Index]=8;//Socket not exist
	Net_Sending_Socket[Index]=8;
	Net_Rx_Delay[Index]=0;
	Net_Tx_Delay[Index]=0;
}

/********************************************************************
*SPI2_Init                                                          *
*SPI2硬件初始化                                                     *
*SCK:PB13,MOSI:PB15,MISO:PB14,NSS:PB12, Reset:PA12, CS:PA11         *
********************************************************************/
void SPI2_Init(void)
{
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
																												
	// Enable the SPI clock			 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

	// Enable GPIO clocks 					
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	// Connect SPI pins to AF5
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;

	// SPI SCK pin configuration 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// SPI  MISO pin configuration 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// SPI  MOSI pin configuration 
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_15;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//NSS
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

 	//CS
 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIOA->BSRRH =  GPIO_Pin_11;//CS low

 	//Reset
 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIOA->BSRRH =  GPIO_Pin_12;//Reset off
	
	// SPI configuration 
	SPI_I2S_DeInit(SPI2);	 //必须先禁能,才能改变MODE 
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;//两线全双工 
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;//2分频 
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;

	// Initializes the SPI communication 
 	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_Init(SPI2, &SPI_InitStructure);

	//The Data transfer is performed in the SPI interrupt routine 
	SPI_Cmd(SPI2, DISABLE);
	SPI_SSOutputCmd (SPI2, ENABLE);
}

/********************************************************************
*SPI3_Init                                                          *
*SPI3硬件初始化                                                     *
*SCK:PC10,MOSI:PC12,MISO:PC11,NSS:PA15,Reset:PB4, CS:PB3            *	
********************************************************************/ 
void SPI3_Init(void)
{
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	// Enable the SPI clock			 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);

	// Enable GPIO clocks 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	// Connect SPI pins to AF5  
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_SPI3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_SPI3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_SPI3);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_SPI3);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;

	// SPI SCK pin configuration 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// SPI  MISO pin configuration 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// SPI  MOSI pin configuration
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// SPI  NSS pin configuration 
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//CS
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIOB->BSRRH =  GPIO_Pin_3;//CS low
	
	//Reset
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIOB->BSRRH =  GPIO_Pin_4;//Reset off
 
	// SPI configuration
	SPI_I2S_DeInit(SPI3);	 											//必须先禁能,才能改变MODE 
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;	//两线全双工 
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;//2分频 
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;

	// Initializes the SPI communication
 	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_Init(SPI3, &SPI_InitStructure);

	SPI_Cmd(SPI3, DISABLE);
	SPI_SSOutputCmd (SPI3, ENABLE);	   
}

/********************************************************************
*SPI2_DMA_Config                                                    *
*SPI2数据收发的DMA初始化                                            *
********************************************************************/
void SPI2_DMA_Config(void)
{
	DMA_InitTypeDef  DMA_InitStructureTx,DMA_InitStructureRx;

	//启动DMA时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);	 
	
	//发送通道
	DMA_DeInit(DMA1_Stream4);
	DMA_InitStructureTx.DMA_Channel = DMA_Channel_0;   							//选择通道0	
	DMA_InitStructureTx.DMA_PeripheralBaseAddr =(unsigned int)&SPI2->DR;			//SPI1的DR寄存器地址：SPI data register
	DMA_InitStructureTx.DMA_Memory0BaseAddr = (unsigned int)SPI_Tx_Buf[0];			//(unsigned int)SPI_Tx_Buf[0]：自定义的一个变量;
	DMA_InitStructureTx.DMA_DIR = DMA_DIR_MemoryToPeripheral;  					//设置DMA的传输方向：内存到外围设备
	DMA_InitStructureTx.DMA_BufferSize = DMA_Tx_Size[0]+4;									//是设置DMA在传输时缓冲区的长度											 
	DMA_InitStructureTx.DMA_PeripheralInc = DMA_PeripheralInc_Disable;			//置DMA的外设递增模式，如果DMA选用的通道（CHx）有多个外设连接，需要使用外设递增模式：
	DMA_InitStructureTx.DMA_MemoryInc = DMA_MemoryInc_Enable;  					//DMA内存地址自动增加模式，，DMA访问多个内存参数时，需要使用DMA_MemoryInc_Enable
	DMA_InitStructureTx.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//设置DMA在访问时每次操作的数据长度
	DMA_InitStructureTx.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;		
	DMA_InitStructureTx.DMA_Mode = DMA_Mode_Normal;    	 						//设置DMA的传输模式：通用模式	
	DMA_InitStructureTx.DMA_Priority = DMA_Priority_Low;						//设置DMA的优先级别：共4个级别
	DMA_InitStructureTx.DMA_FIFOMode = DMA_FIFOMode_Disable;					//
	DMA_InitStructureTx.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructureTx.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructureTx.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;        
	DMA_Init(DMA1_Stream4, &DMA_InitStructureTx);

	//使能SPI2的DMA收发功能
	SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, ENABLE);
	DMA_Cmd(DMA1_Stream4,DISABLE); 	
			
	//接收通道
	DMA_DeInit(DMA1_Stream3);
	DMA_StructInit(&DMA_InitStructureRx);						//缺省值填充初始化参数
	DMA_InitStructureRx.DMA_Channel = DMA_Channel_0;
	DMA_InitStructureRx.DMA_PeripheralBaseAddr =(unsigned int)&SPI2->DR;//SPI2的DR寄存器地址
	DMA_InitStructureRx.DMA_Memory0BaseAddr = (unsigned int)SPI_Rx_Buf[0];
	DMA_InitStructureRx.DMA_DIR = DMA_DIR_PeripheralToMemory;  
	DMA_InitStructureRx.DMA_BufferSize = DMA_Tx_Size[0]+4;
	DMA_InitStructureRx.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructureRx.DMA_MemoryInc = DMA_MemoryInc_Enable;  //DMA内存地址自动增加模式
	DMA_InitStructureRx.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructureRx.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructureRx.DMA_Mode = DMA_Mode_Circular;    
	DMA_InitStructureRx.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructureRx.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructureRx.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructureRx.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructureRx.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;                
	DMA_Init(DMA1_Stream3, &DMA_InitStructureRx);		 

	DMA_Cmd(DMA1_Stream3, ENABLE);
}

/********************************************************************
*SPI3_DMA_Config                                                    *
*SPI3数据收发的DMA初始化                                            *
********************************************************************/
void SPI3_DMA_Config(void)
{
	DMA_InitTypeDef  DMA_InitStructureTx,DMA_InitStructureRx;

	//启动DMA时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);	 
	
	//发送通道
	DMA_DeInit(DMA1_Stream7);
	DMA_InitStructureTx.DMA_Channel = DMA_Channel_0;   							//选择通道0	
	DMA_InitStructureTx.DMA_PeripheralBaseAddr =(unsigned int)&SPI3->DR;			//SPI1的DR寄存器地址：SPI data register
	DMA_InitStructureTx.DMA_Memory0BaseAddr = (unsigned int)SPI_Tx_Buf[1];			
	DMA_InitStructureTx.DMA_DIR = DMA_DIR_MemoryToPeripheral;  					//设置DMA的传输方向：内存到外围设备
	DMA_InitStructureTx.DMA_BufferSize = DMA_Tx_Size[1]+4;									//是设置DMA在传输时缓冲区的长度											 
	DMA_InitStructureTx.DMA_PeripheralInc = DMA_PeripheralInc_Disable;			//置DMA的外设递增模式，如果DMA选用的通道（CHx）有多个外设连接，需要使用外设递增模式：
	DMA_InitStructureTx.DMA_MemoryInc = DMA_MemoryInc_Enable;  					//DMA内存地址自动增加模式，，DMA访问多个内存参数时，需要使用DMA_MemoryInc_Enable
	DMA_InitStructureTx.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//设置DMA在访问时每次操作的数据长度
	DMA_InitStructureTx.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;		
	DMA_InitStructureTx.DMA_Mode = DMA_Mode_Normal;    	 						//设置DMA的传输模式：通用模式	
	DMA_InitStructureTx.DMA_Priority = DMA_Priority_Low;						//设置DMA的优先级别：共4个级别
	DMA_InitStructureTx.DMA_FIFOMode = DMA_FIFOMode_Disable;					//
	DMA_InitStructureTx.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructureTx.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructureTx.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;        
	DMA_Init(DMA1_Stream7, &DMA_InitStructureTx);

	//使能SPI3的DMA发送功能
	SPI_I2S_DMACmd(SPI3, SPI_I2S_DMAReq_Tx, ENABLE);
	DMA_Cmd(DMA1_Stream7,DISABLE); 		
			
	//接收通道
	DMA_DeInit(DMA1_Stream2);
	DMA_StructInit(&DMA_InitStructureRx);						//缺省值填充初始化参数
	DMA_InitStructureRx.DMA_Channel = DMA_Channel_0;
	DMA_InitStructureRx.DMA_PeripheralBaseAddr =(unsigned int)&SPI3->DR;//SPI2的DR寄存器地址
	DMA_InitStructureRx.DMA_Memory0BaseAddr = (unsigned int)SPI_Rx_Buf[1];
	DMA_InitStructureRx.DMA_DIR = DMA_DIR_PeripheralToMemory;  
	DMA_InitStructureRx.DMA_BufferSize = DMA_Tx_Size[1]+4;
	DMA_InitStructureRx.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructureRx.DMA_MemoryInc = DMA_MemoryInc_Enable;  //DMA内存地址自动增加模式
	DMA_InitStructureRx.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructureRx.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructureRx.DMA_Mode = DMA_Mode_Circular;    
	DMA_InitStructureRx.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructureRx.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructureRx.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructureRx.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructureRx.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;                
	DMA_Init(DMA1_Stream2, &DMA_InitStructureRx);		 

	DMA_Cmd(DMA1_Stream2, ENABLE);
}

/********************************************************************
*RST_On                                                             *
*网口复位引脚置高                                                   *
********************************************************************/
void RST_On(int Index)
{
	if (Index)
	{
		GPIOB->BSRRL =  GPIO_Pin_4; 
	}
	else
	{
		GPIOA->BSRRL =  GPIO_Pin_12;  
	}
}

/********************************************************************
*RST_Off                                                            *
*网口复位引脚置低                                                   *
********************************************************************/
void RST_Off(int Index)
{
   	if (Index)
	{
		GPIOB->BSRRH =  GPIO_Pin_4; 
	}
	else
	{
		GPIOA->BSRRH =  GPIO_Pin_12;  
	}
}

/********************************************************************
*Net_Write_Byte                                                     *
*向指定地址写单字节到W5200。                                        *
********************************************************************/
void Net_Write_Byte(SPI_TypeDef *SPIx, unsigned short int addr, unsigned char dat)
{
	int i=0;

	SPI_Cmd(SPIx, ENABLE);
	
	// Send W5200 Register address
	SPI_I2S_SendData(SPIx, (unsigned char)(addr/256));
	while((SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET) && i++<SPI_Delay);
	SPI_I2S_SendData(SPIx, (unsigned char)addr);
	i=0;
	while((SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET) && (i++<SPI_Delay));

	// Send WRITE Command 
	SPI_I2S_SendData(SPIx, 0x80);
	i=0;
	while((SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET) && (i++<SPI_Delay));
	SPI_I2S_SendData(SPIx, 0x01);
	i=0;
	while((SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET) && (i++<SPI_Delay));

	// Write data to W5200 register
	SPI_I2S_SendData(SPIx, dat);
	i=0;
	while((SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET) && (i++<SPI_Delay));

	SPI_Cmd(SPIx, DISABLE);
	Soft_Delay(2);
}

/********************************************************************
*Net_Write                                                          *
*向指定地址写入指定长度的字符串到W5200                              *
********************************************************************/
void Net_Write(SPI_TypeDef *SPIx, unsigned short int Reg_Addr, unsigned char *Data_ptr, unsigned short int Data_Size)
{
	unsigned short i,j=0;
	// Set W5200 CS Low 
	SPI_Cmd(SPIx, ENABLE);
	
	SPI_I2S_SendData(SPIx, Reg_Addr/256);
	while((SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET) && (j++<SPI_Delay));
	SPI_I2S_SendData(SPIx, Reg_Addr);
	j=0;
	while((SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET) && (j++<SPI_Delay));

	//Send WRITE Command, send write data size 
	i = Data_Size | 0x8000;
	SPI_I2S_SendData(SPIx, i/256);
	j=0;
	while((SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET) && (j++<SPI_Delay));
	SPI_I2S_SendData(SPIx, i);
	j=0;
	while((SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET) && (j++<SPI_Delay));

	// Write data to W5200 register, size=Data_Size 
	for(i = 0; i < Data_Size; i++)
	{
		SPI_I2S_SendData(SPIx, *Data_ptr);
		j=0;
		while((SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET) && (j++<SPI_Delay));
		Data_ptr++;
	}

	// Set W5200 CS HIGH 
	SPI_Cmd(SPIx, DISABLE);
	Soft_Delay(2);
}

/********************************************************************
*Net_Read_Byte                                                      *
*向W5200的指定地址读取单个字符串                                    *
********************************************************************/
unsigned char Net_Read_Byte(SPI_TypeDef *SPIx, unsigned short int Reg_Addr)
{
	unsigned char i;
	int j=0;
	SPI_Cmd(SPIx, ENABLE);
	// Send W5200 Register address 
	SPI_I2S_SendData(SPIx, (unsigned char)(Reg_Addr/256));
	while((SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET) && (j++<SPI_Delay));
	SPI_I2S_SendData(SPIx, (unsigned char)Reg_Addr);
	j=0;
	while((SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET) && (j++<SPI_Delay));
	// Send READ Command, send read data size 
	SPI_I2S_SendData(SPIx, 0x00);
	j=0;
	while((SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET) && (j++<SPI_Delay));
	SPI_I2S_SendData(SPIx, 0x01);
	j=0;
	while((SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET) && (j++<SPI_Delay));
	j=0;
	while((SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET) && (j++<SPI_Delay));
  
	i=SPI_I2S_ReceiveData(SPIx);
	// Read 1 byte from W5200 register
	SPI_I2S_SendData(SPIx, 0x00);
	j=0;
	while((SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET) && (j++<SPI_Delay));
	i = SPI_I2S_ReceiveData(SPIx);

	SPI_Cmd(SPIx, DISABLE);
	Soft_Delay(2);
	return i;
}

/********************************************************************
*Net_Read                                                           *
*向W5200的指定地址读取指定长度的字符串                              *
********************************************************************/
void Net_Read(SPI_TypeDef *SPIx, unsigned short int Reg_Addr, unsigned char *Data_ptr, unsigned short int Data_Size)
{
	unsigned short int i=0, j=0;
	
	SPI_Cmd(SPIx, ENABLE);

	// Send W5200 Register address 
	SPI_I2S_SendData(SPIx, Reg_Addr/256);
	while((SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET) && (i++<SPI_Delay));
	SPI_I2S_SendData(SPIx, Reg_Addr);
	i=0;
	while((SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET) && (i++<SPI_Delay));

	// Send READ Command, send read data size 
	i = Data_Size & 0x7fff;
	SPI_I2S_SendData(SPIx, i/256);
	while((SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET) && (j++<SPI_Delay));
	SPI_I2S_SendData(SPIx, i);
	j=0;
	while((SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET) && (j++<SPI_Delay));
	j=0;
	while((SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET) && (j++<SPI_Delay));
		
	i = SPI_I2S_ReceiveData(SPIx);
	// Read data from W5200 register, size=Data_Size
	for(i=0;i<Data_Size;i++)
	{
		SPI_I2S_SendData(SPIx, 0x00);
		j=0;
		while((SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET) && (j++<SPI_Delay));
		*Data_ptr=SPI_I2S_ReceiveData(SPIx);
		Data_ptr++;
	}
	// Set W5200 CS as HIGH
	SPI_Cmd(SPIx, DISABLE);
	Soft_Delay(2);
}

/********************************************************************
*函数名：Net_Config                                                 *
*功能：配置网口的W5200，设置发送缓冲区和接收缓冲区的大小，设置重试  *
*时间，设置重试次数。	                                              *
********************************************************************/
void Net_Config(SPI_TypeDef *SPIx)
{
	unsigned char i;

	for(i = 0; i < 2; i++)
	{
		Net_Write_Byte(SPIx, Sn_RXMEM_SIZE(i),0x08);	//Socket Rx memory size=8k
		Net_Write_Byte(SPIx, Sn_TXMEM_SIZE(i),0x08);	//Socket Tx memory size=8k
	}	
	Net_Write_Byte(SPIx, RTR_W, 0xf);//400ms
	Net_Write_Byte(SPIx, RTR_W+1, 0xa0);
}

/********************************************************************
*函数名: Socket_Mode                                                *
*功能：	作为服务器时打开网口, 开始侦听.                             *
*       作为客户端时打开网口, 开始连接.                             *
*参数: s : SOCKET号                                                 *
*返回: 成功则返回TRUE, 否则返回FALSE                                *
********************************************************************/
unsigned char Socket_Mode(SPI_TypeDef *SPIx, unsigned char s,unsigned char Mode)
{
	unsigned char r=TRUE;
	
	Net_Write_Byte(SPIx, Sn_MR(s), Sn_MR_TCP);			// Set Socket in TCP Mode 
	Net_Write_Byte(SPIx, Sn_CR(s), Sn_CR_OPEN);			// Open Socket 

	if(Net_Read_Byte(SPIx, Sn_SR(s))!=SOCK_INIT)
	{
		Net_Write_Byte(SPIx, Sn_CR(s),Sn_CR_CLOSE);		// Failed to open Socket, Close it and retuen false 
		r=FALSE;
	}
	else
	{
		if (Mode==Server)
		{
			Net_Write_Byte(SPIx, Sn_CR(s), Sn_CR_LISTEN);			// Set Socket in TCP Server Mode 

			if(Net_Read_Byte(SPIx, Sn_SR(s))!=SOCK_LISTEN)
			{
				Net_Write_Byte(SPIx, Sn_CR(s), Sn_CR_CLOSE);		// Failed to set in TCP Server Mode, Close Socket and return false 
				r=FALSE;
			}
		}
		else
		{
			Net_Write_Byte(SPIx, Sn_CR(s), Sn_CR_CONNECT);
		}
	}
	
	return r;
}

/********************************************************************
*Socket_Correct_Addr                                                *
*当网口使用DMA方式进行数据发送时，需对W5200的发送缓冲区地址进行校对 *
*，则引用该函数                                                     *
********************************************************************/
void Socket_Correct_Addr(SPI_TypeDef *SPIx, unsigned char s)
{
	unsigned short int i;
	unsigned short int tx_offset,tx_offset1;
	int Index;
	
	Index=(SPIx==SPI3);
		
	//读取发送缓冲区的偏移量
	tx_offset = Net_Read_Byte(SPIx, Sn_TX_WR0(s));
	tx_offset *= 256;
	tx_offset += Net_Read_Byte(SPIx, Sn_TX_WR0(s)+1);
	tx_offset1 = tx_offset;
	
	i = tx_offset / S_BUF_SIZE;					//计算实际的物理偏移量，S0_TX_SIZE需要在前面#define中定义*/
												//注意S0_TX_SIZE的值在W5200_Init()函数的W5200_TMSR中确定*/
	tx_offset = tx_offset - i * S_BUF_SIZE;
	i = tx_offset % DMA_Tx_Size[Index]; 
	i = DMA_Tx_Size[Index] - i;					//要补偿的数据长度
	
	Net_Write(SPIx, (TX_MEM + s * S_BUF_SIZE + tx_offset),SPI_Tx_Buf[0],i);
	
	//计算下一次的偏移量
	tx_offset1 += i;
	Net_Tx_Addr_Offset[Index][s] = tx_offset1;
	Net_Write_Byte(SPIx, Sn_TX_WR0(s),(tx_offset1 / 256));
	Net_Write_Byte(SPIx, (Sn_TX_WR0(s)+1), tx_offset1);
	Net_Write_Byte(SPIx, Sn_CR(s), Sn_CR_SEND);			// Send SEND command, start send process 
	
	Net_Socket_Tx_Addr[Index][s] += (tx_offset + i);
	if(Net_Socket_Tx_Addr[Index][s] >= (0x8000 + s * S_BUF_SIZE + S_BUF_SIZE))
	{
		Net_Socket_Tx_Addr[Index][s] = 0x8000 + s * S_BUF_SIZE;
	}
}

/********************************************************************************
*函数名: Network_Init                                                           *
*功能：	网口初始化函数                                                          *
*W5200各个Socket初始化参数：自身IP、网关、是否启用、端口、C/S、目标IP、目标端口.*
********************************************************************************/
void Network_Init(SPI_TypeDef *SPIx, unsigned char* Self_IP, unsigned char* Socket_Enable, unsigned short int* Self_Port, 
unsigned char* Socket_mode, unsigned char* Dest_IP, unsigned short int* Dest_Port, int DMA_Size)
{
	unsigned short int i, j;
	unsigned char ucC[2];
	unsigned char* ucPt;
	unsigned char Gate_Way[4];
	int Index;
	
	Index=(SPIx==SPI3);
	if (Index)
	{
		SPI3_Init();//包含SPI初始化，引脚初始化，复位引脚初始化及复位引脚输出低电平
	}
	else
	{
		SPI2_Init();
	}
	
	Soft_Delay(50000);
	RST_On(Index);
	for(i = 0 ;i < 400;i++)						//延时至少150ms
	{
		Soft_Delay(60000);		
	}
	
	DMA_Tx_Size[Index]=0;
	if (DMA_Size==DMA_SIZE_128 || DMA_Size==DMA_SIZE_256 || DMA_Size==DMA_SIZE_512 || 
		DMA_Size==DMA_SIZE_1K || DMA_Size==DMA_SIZE_2K || DMA_Size==DMA_SIZE_4K || DMA_Size==DMA_SIZE_8K)
	{
		DMA_Tx_Size[Index]=DMA_Size;
		if (Index)
		{
			SPI3_DMA_Config();							//DMA初始化
		}
		else
		{
			SPI2_DMA_Config();
		}
	}
	Net_Config(SPIx); //设置各个连接的缓冲大小 
	
	if(uchNodeNum)								//B网客户端
	{
		//设置MAC地址
		Network_Phy_Addr[0] = 0x34;
		Network_Phy_Addr[1] = 0x24;
		Network_Phy_Addr[2] = 0xE8;
		Network_Phy_Addr[3] = 0xB9;
		Network_Phy_Addr[4] = 0x8B+uchNodeNum;
		Network_Phy_Addr[5] = 0x31+Index;
	}
	else										//A网客户端								
	{
		//设置MAC地址
		Network_Phy_Addr[0] = 0x34;
		Network_Phy_Addr[1] = 0x24;
		Network_Phy_Addr[2] = 0xE8;
		Network_Phy_Addr[3] = 0xB9+uchNodeNum;
		Network_Phy_Addr[4] = 0x8B;
		Network_Phy_Addr[5] = 0x33+Index;
	}
	ucPt=Self_IP;
	Gate_Way[0]=*ucPt++;
	Gate_Way[1]=*ucPt++;
	Gate_Way[2]=*ucPt++;
	Gate_Way[3]=1;
	Net_Write(SPIx, GAR0, Gate_Way, 4);				//设置网关IP地址 
	Net_Write(SPIx, SUBR0,Network_Sub_Mask,4);		//设置子码掩码 
	Net_Write(SPIx, SHAR0,Network_Phy_Addr,6);		//设置MAC地址
	Net_Write(SPIx, SIPR0,Self_IP,4);				//设置本地IP地址
	i=4000;
	ucC[0]=i>>8;
	ucC[1]=i & 0xff;
	Net_Write(SPIx, RTR_W,ucC,2);
	ucC[0]=1;
	Net_Write(SPIx, RCR,ucC,1);
	
	//--Socket0...Socket7	本地端口号、目的地端口号和目的地IP地址的设置
	for(i = 0 ;i < 2;i++)
	{
		Net_Enable[Index][i]=Socket_Enable[i];
		Net_Mode[Index][i]=Socket_mode[i];//模式: 服务器/客户端 
		
		//自身端口号
		j=Self_Port[i];
		ucC[0]=j>>8;
		ucC[1]=j & 0xff;
		Net_Write(SPIx, Sn_PORT0(i),ucC,2);				//设置本地端口号 
		
		//目标IP
		Net_Write(SPIx, Sn_DIPR0(i),Dest_IP+i*4,4);
		//Dest_IP+=4;
		
		//目标端口号
		j=Dest_Port[i];
		ucC[0]=j>>8;
		ucC[1]=j & 0xff;
		Net_Write(SPIx, Sn_DPORT0(i),ucC,2);	
	}
   
	Net_Para_Init(Index);										//数据区初始化
	
	//记录参数, 用于网口重启 
	for (i=0;i<4;i++)
	{
		Para_Self_IP[Index][i]=Self_IP[i];
		Para_Gateway[Index][i]=Gate_Way[i];
		Para_Des_IP[Index][i]=Dest_IP[i];
		Para_Des_IP[Index][i+4]=Dest_IP[i+4];
	}
	for(i = 0 ;i < 2;i++)
	{
		Para_Socket_Enable[Index][i]=Socket_Enable[i];
		Para_Self_Port[Index][i]=Self_Port[i];
		Para_Socket_Mode[Index][i]=Socket_mode[i];
		Para_Des_Port[Index][i]=Dest_Port[i];		
	}
	Para_DMA_Size[Index]=DMA_Size;
	for (i=0;i<8;i++)
	{
		Net_Port[Index][i]=Self_Port[i];
	}
}

/********************************************************************
*Net_Interrupt_Process                                              *
*网口的状态读取及处理过程                                           *
********************************************************************/
void Net_Interrupt_Process(SPI_TypeDef *SPIx)
{
	unsigned char i,j,k=1,n;
	int Index;
	unsigned char ucC[2];
	
	Index=(SPIx==SPI3);
	i = Net_Read_Byte(SPIx, IR2);								//读取Socket中断
	for (n=0;n<2;n++)
	{
		if (i & k) 				
		{
			j = Net_Read_Byte(SPIx, Sn_IR(n));
			Net_Write_Byte(SPIx, Sn_IR(n),j);			   //回写清中断标志
	
			if(j & S_IR_CON) // 在TCP模式下,Socket成功连接 
			{
				if(Net_Mode[Index][n] == Server)
				{
					Net_Socket_State[Index][n] |= S_CONN;
				}
				else if(Net_Mode[Index][n] == Client)
				{
					Net_Socket_State[Index][n] |= (S_CONN | S_OK);
				}
			}
			if(j & S_IR_DISCON) // 在TCP模式下Socket断开连接处理 
			{
				Net_Write_Byte(SPIx, Sn_CR(n),Sn_CR_DISCON);		// Disconnect socket 
				Net_Write_Byte(SPIx, Sn_CR(n),Sn_CR_CLOSE);		// Close socket 
				Net_Socket_Rx_Addr[Index][n] = Net_Rx_Addr[n];
				Net_Socket_Tx_Addr[Index][n] = Net_Tx_Addr[n];
				Net_Rx_Addr_Offset[Index][n] = 0;
	
				Net_Socket_State[Index][n] = 0;
			}
			if(j & S_IR_SENDOK) // Socket数据发送完成，可以再次启动S_tx_process()函数发送数据 
			{
				Net_Socket_Data[Index][n] |= S_TRANSMITOK;
			}
			if(j & S_IR_RECV) //Socket接收到数据，可以启动S_rx_process()函数 
			{
				Net_Socket_Data[Index][n] |= S_RECEIVE;
			}
			if(j & S_IR_TIMEOUT) // Socket连接或数据传输超时处理 
			{
				if (Net_Socket_State[Index][n]==7)
				{
					Net_Write_Byte(SPIx, Sn_CR(n),Sn_CR_CLOSE);		// Close socket 
					Net_Socket_State[Index][n] = 0;
					Net_Socket_Rx_Addr[Index][n] = Net_Rx_Addr[n];
					Net_Socket_Tx_Addr[Index][n] = Net_Tx_Addr[n];
					Net_Rx_Addr_Offset[Index][n] = 0;
					ucC[0]=Net_Port[Index][n]>>8;
					ucC[1]=Net_Port[Index][n] & 0xff;
					Net_Write(SPIx, Sn_PORT0(i),ucC,2);	
				}
			}
		}
		k<<=1;
	}	
}

/********************************************************************
*Net_DMA_rx_process                                                 *
*网口采用DMA方式进行数据接收的过程，数据接收长度为128               *
********************************************************************/
unsigned char Net_DMA_rx_process(SPI_TypeDef *SPIx, unsigned char s)
{
	unsigned char i = Failure;
	int j;
	int Index;
	
	Index=(SPIx==SPI3);
	if(Net_State[Index] == S_IDLING) 
	{
		Net_State[Index] = S_RECEIVING;									//进入接收状态
			
		SPI_Tx_Buf[Index][0] = (unsigned char)(Net_Socket_Rx_Addr[Index][s] >> 8);			//接收缓冲区地址：每个端口的地址是不一样的
		SPI_Tx_Buf[Index][1] = (unsigned char)Net_Socket_Rx_Addr[Index][s];
		SPI_Tx_Buf[Index][2] = DMA_Tx_Size[Index]>>8;														//模式：接收，长度：8192
		SPI_Tx_Buf[Index][3] = DMA_Tx_Size[Index] & 0xff;	
															//模式：接收，长度：8192
		// Received Data Size
		Net_Rx_size[Index] = Net_Read_Byte(SPIx, Sn_RX_RSR0(s));
		Net_Rx_size[Index] *= 256;
		Net_Rx_size[Index] +=Net_Read_Byte(SPIx, Sn_RX_RSR0(s)+1);
	
		Net_Socket_Rx_Addr[Index][s] += Net_Rx_size[Index];
		
		if(Net_Socket_Rx_Addr[Index][s] >= Net_Rx_Addr[s]+S_BUF_SIZE)
		{
			Net_Socket_Rx_Addr[Index][s] = Net_Rx_Addr[s];
		}

		SPI_I2S_DMACmd(SPIx, SPI_I2S_DMAReq_Rx, ENABLE);       					//启动DMA读
		// Set W5200 CS as Low
		SPI_Cmd(SPIx, ENABLE);
		if (Index)//启动DMA发送
		{
			DMA_Cmd(DMA1_Stream7, ENABLE);  
		}
		else
		{
			DMA_Cmd(DMA1_Stream4, ENABLE);  											
		}
		
		i = Failure;
	}
	else if(Net_State[Index] == S_RECEIVING)
	{
		if (Index)
		{
			j=DMA_GetFlagStatus(DMA1_Stream7,DMA_FLAG_TCIF7);
		}
		else
		{
			j=DMA_GetFlagStatus(DMA1_Stream4,DMA_FLAG_TCIF4);
		}
		if(j == SET)
		{			
			if(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_BSY) == 0)
			{
	
				if (Index)
				{
					DMA_ClearFlag(DMA1_Stream7,DMA_FLAG_TCIF7);								//清除标志
				}
				else
				{
					DMA_ClearFlag(DMA1_Stream4,DMA_FLAG_TCIF4);			
				}
				
				SPI_Cmd(SPIx, DISABLE);
				SPI_I2S_DMACmd(SPIx, SPI_I2S_DMAReq_Rx, DISABLE);					   	//关闭DMA读
				for(j = 0; j < DMA_Tx_Size[Index]; j++)
				{
					Net_Rx_Buf[Index][j] = SPI_Rx_Buf[Index][j + 4];
				}
				Net_Rx_Addr_Offset[Index][s] += Net_Rx_size[Index];

				Net_Write_Byte(SPIx, Sn_RX_RD0(s), (Net_Rx_Addr_Offset[Index][s]/256));
				Net_Write_Byte(SPIx, (Sn_RX_RD0(s) + 1), Net_Rx_Addr_Offset[Index][s]);
	
				Net_Write_Byte(SPIx, Sn_CR(s), Sn_CR_RECV);			// Send RECV command
				
				Net_State[Index] = S_IDLING;
				Net_Socket_Data[Index][s] &= ~S_RECEIVE;
				i = Success; 
			}
		}
	 }
	return i;						
}

/********************************************************************************
*Net_Rx                                                                         *
*网口接收函数                                                                   *
********************************************************************************/
void Net_Rx(SPI_TypeDef *SPIx)
{
	unsigned short int i,j,k;
	unsigned short int rx_offset;
	int Index;
	
	Index=(SPIx==SPI3);
	
	for (i=0;i<2;i++)
	{
		if (Net_Enable[Index][i] && (Net_Socket_State[Index][i]==(S_INIT|S_CONN|S_OK)) && //启用了此网络套接字且连接成功 
			(Net_Socket_Data[Index][i] & S_RECEIVE))//收到数据
		{
			// Received Data Size
			Net_Rx_size[Index] = Net_Read_Byte(SPIx, Sn_RX_RSR0(i));
			Net_Rx_size[Index] *= 256;
			Net_Rx_size[Index] +=Net_Read_Byte(SPIx, Sn_RX_RSR0(i)+1);
			
			//rx offset
			rx_offset = Net_Read_Byte(SPIx, Sn_RX_RD0(i));
			rx_offset *= 256;
			rx_offset += Net_Read_Byte(SPIx, Sn_RX_RD0(i)+1);
			
			j=rx_offset/S_BUF_SIZE;
			k=rx_offset-j*S_BUF_SIZE;//相对偏移量 
			j=k+Net_Rx_size[Index];///数据尾部 
			
			if (j>S_BUF_SIZE)//超出尾部 
			{
				j=S_BUF_SIZE-k;
				Net_Read(SPIx, RX_MEM + i*S_BUF_SIZE + k, Net_Rx_Buf[Index], j);
				Net_Read(SPIx, RX_MEM + i*S_BUF_SIZE, Net_Rx_Buf[Index]+j, Net_Rx_size[Index]-j);
			}
			else
			{
				Net_Read(SPIx, RX_MEM + i*S_BUF_SIZE + k,Net_Rx_Buf[Index], Net_Rx_size[Index]);
			}
			
			//新的偏移地址 
			rx_offset+=Net_Rx_size[Index];
			Net_Write_Byte(SPIx, Sn_RX_RD0(i), (rx_offset/256));
			Net_Write_Byte(SPIx, Sn_RX_RD0(i)+1, rx_offset);
			Net_Write_Byte(SPIx, Sn_CR(i), Sn_CR_RECV);//启动接收 
			Net_Socket_Data[Index][i] &=~S_RECEIVE;
			Net_Rx_Flag[Index][i]=1;
		}
	}	
}

/********************************************************************************
*Net_Tx                                                                         *
*网口发送函数                                                                   *
********************************************************************************/
void Net_Tx(SPI_TypeDef *SPIx, unsigned char* TxData)
{
	unsigned short int i,j,k;
	unsigned short int tx_offset,tx_offset1;
	int Index;
	
	Index=(SPIx==SPI3);
	
	for (i=0;i<2;i++)
	{
		if (Net_Enable[Index][i] && Net_Tx_Flag[Index][i] //外部程序设置发送标志
			&& (Net_Socket_State[Index][i]==(S_INIT|S_CONN|S_OK)))//启用了此网络套接字且连接成功 			
		{		
			Net_Socket_Data[Index][i]&=~S_TRANSMITOK;//清除发送标志		
			
			j=Net_Read_Byte(SPIx, Sn_TX_FSR0(i));
			j*=256;
			j+=Net_Read_Byte(SPIx, Sn_TX_FSR0(i)+1);
			
			if (j>Net_Tx_size[Index][i])//剩余发送缓冲长度大于发送长度 
			{
			
				//读取发送缓冲区的偏移量
				tx_offset = Net_Read_Byte(SPIx, Sn_TX_WR0(i));
				tx_offset *= 256;
				tx_offset += Net_Read_Byte(SPIx, Sn_TX_WR0(i)+1);
				tx_offset1 = tx_offset;
				
				j=tx_offset/S_BUF_SIZE;
				k=tx_offset-j*S_BUF_SIZE;
				j=k+Net_Tx_size[Index][i];
				
				if (j>S_BUF_SIZE)//超出发送缓冲, 需要分两次写入 
				{
					j=S_BUF_SIZE-k;
					Net_Write(SPIx, TX_MEM + i * S_BUF_SIZE + k, TxData, j);
					Net_Write(SPIx, TX_MEM + i * S_BUF_SIZE, &TxData[j], Net_Tx_size[Index][i]-j);
				}
				else
				{
					Net_Write(SPIx, TX_MEM + i * S_BUF_SIZE + k, TxData, Net_Tx_size[Index][i]);
				}

				//计算下一次的偏移量
				tx_offset1 += Net_Tx_size[Index][i];
				Net_Tx_Addr_Offset[Index][i] = tx_offset1;
				Net_Write_Byte(SPIx, Sn_TX_WR0(i),(tx_offset1 / 256));
				Net_Write_Byte(SPIx, (Sn_TX_WR0(i)+1), tx_offset1);

				Net_Write_Byte(SPIx, Sn_CR(i), Sn_CR_SEND);			// Send SEND command, start send process 
				Net_Tx_Flag[Index][i]=0;
				
				//Net_tx_process(SPIx, i, Net_Tx_Buf[Index][i]);
			}
		}
	}
}

/********************************************************************
*Net_Mode_Select                                                    *
*网口工作模式的选择、服务器进行侦听和客户端进行连接的函数           *
********************************************************************/
void Net_Mode_Select(SPI_TypeDef *SPIx, unsigned char s)
{	
	int Index;
	
	Index=(SPIx==SPI3);
	if(Net_Socket_State[Index][s] == 0)
	{
		if(Net_State[Index] == S_IDLING)					//确定发送和接收都已经完成
		{
			Net_State[Index] = S_CHECKING;				//进入状态检测过程中
			if (Socket_Mode(SPIx, s,Net_Mode[Index][s]))//打开端口, 建立连接 
			{
				Net_Socket_State[Index][s] = S_INIT;
			}
			else
			{
				Net_Socket_State[Index][s] = 0;
			}			
			Net_State[Index] = S_IDLING;
		}
	}
	else if(Net_Socket_State[Index][s] == (S_INIT | S_CONN))
	{
		if(Net_Mode[Index][s] == Server)
		{
			if(Net_State[Index] == S_IDLING)
			{
				Net_State[Index] = S_CHECKING;
				if (DMA_Tx_Size[Index]==DMA_SIZE_128 || DMA_Tx_Size[Index]==DMA_SIZE_256|| DMA_Tx_Size[Index]==DMA_SIZE_512
					|| DMA_Tx_Size[Index]==DMA_SIZE_1K || DMA_Tx_Size[Index]==DMA_SIZE_2K 
					|| DMA_Tx_Size[Index]==DMA_SIZE_4K || DMA_Tx_Size[Index]==DMA_SIZE_8K)
				{
					Socket_Correct_Addr(SPIx, s);	 									//Socket0校正
				}
				Net_Socket_State[Index][s] = S_INIT | S_CONN | S_OK;	 			//地址校准完成
				Net_State[Index] = S_IDLING;
			}
		}
	}
}

/********************************************************************
*Net_State_Check                                                    *
*网口工作状态的读取及相关处理的函数                                 *
********************************************************************/
void Net_State_Check(SPI_TypeDef *SPIx)
{
	int Index;
	
	Index=(SPIx==SPI3);
	if(Net_State[Index] == S_IDLING)
	{
		Net_State[Index] = S_CHECKING;				//进入状态检测过程中
	   	if( (Net_Socket_State[Index][0] & S_INIT) == S_INIT | (Net_Socket_State[Index][1] & S_INIT) == S_INIT )
		{
		
			Net_Interrupt_Process(SPIx);											//事件处理过程
		}
		Net_State[Index] = S_IDLING;
	}
}

/********************************************************************
*Net_Cycle                                                          *
*网络通信循环函数                                                   *
********************************************************************/
void Net_Cycle(SPI_TypeDef *SPIx)
{
	unsigned char i;
	int Index;
	
	Index=(SPIx==SPI3);
	for (i=0;i<2;i++)
	{
		if (Net_Enable[Index][i])
		{
			Net_Mode_Select(SPIx, i);
		}
	}
	
	if((++Net_Check_Delay[Index] > 1200) && (Net_State[Index] == S_IDLING) )   //1.2s
	{
		Net_Check_Delay[Index] = 0;
		if (Net_State[Index] == S_IDLING)
		{
			Net_State[Index] = S_CHECKING;	
			
			for (i=0;i<2;i++)
			{
				if(Net_Enable[Index][i] && (Net_Socket_State[Index][i] & S_INIT))
				{
					Net_Write_Byte(SPIx, Sn_CR(i), S_CR_SEND_KEEP);
				}				
			}
			Net_State[Index] = S_IDLING;
		}
	}
	Net_State_Check(SPIx);
}

/********************************************************************************
*Net_DMA_Rx                                                                     *
*网口接收函数                                                                   *
********************************************************************************/
void Net_DMA_Rx(SPI_TypeDef *SPIx)
{
	unsigned char i;
	int Index;
	
	Index=(SPIx==SPI3);
	
	if (Net_State[Index]==S_RECEIVING)//有连接在接收数据 
	{
		if (Net_DMA_rx_process(SPIx, Net_Receiving_Socket[Index]))//数据存储在Net_Rx_Buf[Index][s] 
		{
			Net_Rx_Flag[Index][Net_Receiving_Socket[Index]]=1;//只置位，在数据处理中清除 
			Net_Receiving_Socket[Index]=8;//清除当前接收套接字序号，指向不存在的套接字 
			Net_Rx_Delay[Index]=0;
		}
		else
		{
			if (++Net_Rx_Delay[Index]>=100)//未完成DMA接收数据
			{
				Net_Rx_Delay[Index]=0;
				Net_Receiving_Socket[Index]=8;
				Net_State[Index]=S_IDLING;
				Net_Socket_State[Index][i]=0;//套接字连接复位
			}
		}
	}
	else
	{
		for (i=0;i<2;i++)
		{
			if (Net_Enable[Index][i] && (Net_Socket_State[Index][i]==(S_INIT|S_CONN|S_OK)) && //启用了此网络套接字且连接成功 
				(Net_Socket_Data[Index][i] & S_RECEIVE))//收到数据
			{
				Net_Receiving_Socket[Index]=i;//指向当前套接字
				Net_DMA_rx_process(SPIx, i);
				break;
			}
		}
	}
}

/********************************************************************
*Net_DMA_tx_process                                                 *
*网口采用DMA方式进行数据发送的过程，数据发送长度为128               *
********************************************************************/
unsigned char Net_DMA_tx_process(SPI_TypeDef *SPIx, unsigned char s,unsigned char* tx_buffer)
{
	unsigned char i;
	int j;							  
   	int Index;
	unsigned char Write_Command=0x80;
	
	Index=(SPIx==SPI3);
	
	i = Failure;
	
	if(Net_State[Index] == S_IDLING)															//如果空闲，就启动发送
	{
		Net_State[Index] = S_SENDING;															//进入发送状态
			
		SPI_Tx_Buf[Index][2] = DMA_Tx_Size[Index]>>8;													//长度 
		SPI_Tx_Buf[Index][3] = DMA_Tx_Size[Index] & 0xff;																
		SPI_Tx_Buf[Index][2]|= Write_Command;													//模式：发送 
		SPI_Tx_Buf[Index][0] = (unsigned char)(Net_Socket_Tx_Addr[Index][s] >> 8);						//发送缓冲区地址(高8位)
		SPI_Tx_Buf[Index][1] = (unsigned char)(Net_Socket_Tx_Addr[Index][s]);								//发送缓冲区地址(低8位)
		for(j = 0; j < DMA_Tx_Size[Index]; j++)
		{
			SPI_Tx_Buf[Index][j + 4] = tx_buffer[j];
		}
		Net_Socket_Tx_Addr[Index][s] += DMA_Tx_Size[Index];
		if(Net_Socket_Tx_Addr[Index][s] >= (0x8000 + s * S_BUF_SIZE + S_BUF_SIZE))
		{
			Net_Socket_Tx_Addr[Index][s] = 0x8000 + s * S_BUF_SIZE;
		}
		SPI_Cmd(SPIx, ENABLE);
		if (Index)
		{
			DMA_Cmd(DMA1_Stream7,ENABLE); 
		}
		else
		{
			DMA_Cmd(DMA1_Stream4,ENABLE); 
		}
		
		i =	Failure;
		
	}
	else if(Net_State[Index] == S_SENDING)
	{
		if (Index)
		{
			j=DMA_GetFlagStatus(DMA1_Stream7,DMA_FLAG_TCIF7);
		}
		else
		{
			j=DMA_GetFlagStatus(DMA1_Stream4,DMA_FLAG_TCIF4);
		}
		if(j == SET)
		{
			if(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_BSY) == 0)
			{
				if (Index)
				{
					DMA_ClearFlag(DMA1_Stream7,DMA_FLAG_TCIF7);								//清除标志
				}
				else
				{
					DMA_ClearFlag(DMA1_Stream4,DMA_FLAG_TCIF4);			
				}
				
				SPI_Cmd(SPIx, DISABLE);
		
				Net_Tx_Addr_Offset[Index][s] += DMA_Tx_Size[Index];
				Net_Write_Byte(SPIx, Sn_TX_WR0(s),(Net_Tx_Addr_Offset[Index][s] / 256));
				Net_Write_Byte(SPIx, (Sn_TX_WR0(s)+1), Net_Tx_Addr_Offset[Index][s]);
				Net_Write_Byte(SPIx, Sn_CR(s), Sn_CR_SEND);						// Send SEND command, start send process 

				Net_State[Index] = S_IDLING;								//发送完成，进入空闲状态
				Net_Tx_Flag[Index][Net_Sending_Socket[Index]]=0;
				Net_Sending_Socket[Index]=8;
				i =	Success;
			}
		}
	}
	return i;
}

/********************************************************************************
*Net_DMA_Tx                                                                     *
*网口发送函数                                                                   *
********************************************************************************/
void Net_DMA_Tx(SPI_TypeDef *SPIx, unsigned char* TxData)
{
	unsigned char i;
	int Index;
	
	Index=(SPIx==SPI3);
	
	if (Net_State[Index]==S_SENDING)//有连接在发送数据 
	{
		if (Net_DMA_tx_process(SPIx, Net_Sending_Socket[Index], TxData))//发送完成 , 1个网口仅使用一个socket
		{
			Net_Tx_Delay[Index]=0;							
		}
		else
		{
			if (++Net_Tx_Delay[Index]>=100)//DMA发送超时 
			{
				Net_Tx_Delay[Index]=0;
				Net_Sending_Socket[Index]=8;
				Net_Socket_State[Index][Net_Sending_Socket[Index]]=0;//套接字复位 
				Net_State[Index]=S_IDLING;
			}
		}					
	}
	else if(Net_State[Index] == S_IDLING)
	{
		for (i=0;i<2;i++)
		{
			if (Net_Enable[Index][i] && Net_Tx_Flag[Index][i] //外部程序设置发送标志
				&& (Net_Socket_State[Index][i]==(S_INIT|S_CONN|S_OK)))//启用了此网络套接字且连接成功 			
			{		
				Net_Sending_Socket[Index]=i;//指向当前套接字
				Net_Socket_Data[Index][i]&=~S_TRANSMITOK;//清除发送标志
				Net_DMA_tx_process(SPIx, Net_Sending_Socket[Index],TxData);//启动发送
				break;//一次仅能处理一个套接字发送 
			}
		}
	}
}

/********************************************************************************
*Net_Comm                                                                       *
*网口通信函数                                                                   *
********************************************************************************/
void Net_Comm(SPI_TypeDef *SPIx, unsigned char* TxData)
{
	Net_Cycle(SPIx);
	if (DMA_Tx_Size[SPIx==SPI3])//
	{
		Net_DMA_Tx(SPIx, TxData);
		Net_DMA_Rx(SPIx);
	}
	else
	{
		Net_Tx(SPIx, TxData);
		Net_Rx(SPIx);
	}
}
//===========================================================================
// End of file.
//===========================================================================
