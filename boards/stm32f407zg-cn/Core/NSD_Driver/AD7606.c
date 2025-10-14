/************************************************************************************
*AD7606驱动                                                                         *
*                                                                                   *
*双极性模拟量采样芯片,8通道16位精度                                                 *
************************************************************************************/

#include "stm32f4xx_gpio.h"

/********************************************************************************
*AD7606                                                                         *
*AD7606可以接入8路输入电压，16位测量精度。RANGE为高电平输入范围±10V，否则为±5V。*
*AD7606允许将所有模拟通道分为两组同时采样，CONVSTA/B各自驱动一组，可同时启动。	*
*AD7606与ARM之间使用16位数据总线连接，CS上升沿让数据总线处于高阻状态，下降沿让数*
*据总线脱离高阻状态，BUSY变为低电平后第一个RD下降沿读取V1的数据，下一个RD下降沿 *
*读取V2。这样依次向下，直到读取V8。                                             *
*********************************************************************************
*引脚分配                                                                       *
*BUSY: PC13，RST：PF10，RANGE：PF9，CONVSTA：PF6，CONVSTB：PF7。                *
********************************************************************************/
#define WRITE_READ_ADDR    0x800
#define Bank1_SRAM2_ADDR  ((uint32_t)0x64000000) 
#define ADC_CHANNNEL       0x08	

/********************************************************************************
*Delay_Nop                                                                      *
*延时函数                                                                       *
********************************************************************************/
void Delay_Nop(unsigned int x)
{
	unsigned int i;
	for(i=0;i<x;i++);
}

/********************************************************************************
*AD7606_Init_Pin                                                                *
*初始化AD7606相关引脚。                                                         *
*BUSY: PC13，RST：PF10，RANGE：PF9，CONVSTA：PF6，CONVSTB：PF7。                *
********************************************************************************/
void AD7606_Init_Pin(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	
	//BUSY 
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	//RST
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOF, &GPIO_InitStructure);

	//RANGE
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOF, &GPIO_InitStructure);

	//CONVSTA
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_Init(GPIOF, &GPIO_InitStructure);

	//CONVSTB
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_Init(GPIOF, &GPIO_InitStructure);
}

/********************************************************************************
*AD7606_RST_On                                                                  *
*AD7606的RST引脚输入高电平。                                                    *
********************************************************************************/
void AD7606_RST_On(void)
{
   GPIOF->BSRRL =  GPIO_Pin_10;  
}

/********************************************************************************
*AD7606_RST_Off                                                                 *
*AD7606的RST引脚输入低电平。                                                    *
********************************************************************************/
void AD7606_RST_Off(void)
{
   GPIOF->BSRRH =  GPIO_Pin_10;  
}

/********************************************************************************
*AD7606_RANGE_On                                                                *
*AD7606的RANGE引脚输入高电平。                                                  *
********************************************************************************/
void AD7606_RANGE_On(void)
{
   GPIOF->BSRRL =  GPIO_Pin_9;  
}

/********************************************************************************
*AD7606_RANGE_Off                                                               *
*AD7606的RANGE引脚输入低电平。                                                  *
********************************************************************************/
void AD7606_RANGE_Off(void)
{
   GPIOF->BSRRH =  GPIO_Pin_9;  
}

/********************************************************************************
*AD7606_CONVSTA_On                                                              *
*AD7606的CONVSTA引脚输入高电平。                                                *
********************************************************************************/
void AD7606_CONVSTA_On(void)
{
   GPIOF->BSRRL =  GPIO_Pin_6;  
}

/********************************************************************************
*AD7606_CONVSTA_Off                                                             *
*AD7606的CONVSTA引脚输入低电平。                                                *
********************************************************************************/
void AD7606_CONVSTA_Off(void)
{
   GPIOF->BSRRH =  GPIO_Pin_6;  
}

/********************************************************************************
*AD7606_CONVSTB_On                                                              *
*AD7606的CONVSTB引脚输入高电平。                                                *
********************************************************************************/
void AD7606_CONVSTB_On(void)
{
   GPIOF->BSRRL =  GPIO_Pin_7;  
}

/********************************************************************************
*AD7606_CONVSTB_Off                                                             *
*AD7606的CONVSTB引脚输入低电平。                                                *
********************************************************************************/
void AD7606_CONVSTB_Off(void)
{
   GPIOF->BSRRH =  GPIO_Pin_7;  
}

/********************************************************************************
*ConfigAD7606                                                                   *
*配置AD7606，测量范围限定为±10V。                                               *
********************************************************************************/
void ConfigAD7606(void)
{
	//AD7606复位
	AD7606_RST_Off();
	Delay_Nop(10);
	AD7606_RST_On();
	Delay_Nop(20);
	AD7606_RST_Off();
	AD7606_RANGE_On();  //±10v
//	AD7606_RANGE_Off();  //±5v
}

/********************************************************************************
*PSRAM_Init                                                                     *
*FSMC的配置和GPIO接口与外置AD7606。                                             *
*这个函数必须被调用在任何读AD7606之前。                                         *
********************************************************************************/
void PSRAM_Init(void)					 
{
	FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
	FSMC_NORSRAMTimingInitTypeDef  p;
	GPIO_InitTypeDef GPIO_InitStructure; 
  
	// Enable GPIOs clock 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOF |
                         RCC_AHB1Periph_GPIOG, ENABLE);

	// Enable FSMC clock 
	RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC, ENABLE); 
  
	// GPIOs Configuration 
	/*
	 +-------------------+--------------------+------------------+------------------+
	 +                       PSRAM pins assignment                                  +
	 +-------------------+--------------------+------------------+------------------+
	 | PD0  <-> FSMC_D2  | PE0  <-> FSMC_NBL0 | PF0  <-> FSMC_A0 | PG0 <-> FSMC_A10 |
	 | PD1  <-> FSMC_D3  | PE1  <-> FSMC_NBL1 | PF1  <-> FSMC_A1 | PG1 <-> FSMC_A11 |
	 | PD4  <-> FSMC_NOE | PE7  <-> FSMC_D4   | PF2  <-> FSMC_A2 | PG2 <-> FSMC_A12 |
	 | PD5  <-> FSMC_NWE | PE8  <-> FSMC_D5   | PF3  <-> FSMC_A3 | PG3 <-> FSMC_A13 |
	 | PD8  <-> FSMC_D13 | PE9  <-> FSMC_D6   | PF4  <-> FSMC_A4 | PG4 <-> FSMC_A14 |
	 | PD9  <-> FSMC_D14 | PE10 <-> FSMC_D7   | PF5  <-> FSMC_A5 | PG5 <-> FSMC_A15 |
	 | PD10 <-> FSMC_D15 | PE11 <-> FSMC_D8   | PF12 <-> FSMC_A6 | PG9 <-> FSMC_NE2 |
	 | PD11 <-> FSMC_A16 | PE12 <-> FSMC_D9   | PF13 <-> FSMC_A7 |------------------+
	 | PD12 <-> FSMC_A17 | PE13 <-> FSMC_D10  | PF14 <-> FSMC_A8 | 
	 | PD14 <-> FSMC_D0  | PE14 <-> FSMC_D11  | PF15 <-> FSMC_A9 | 
	 | PD15 <-> FSMC_D1  | PE15 <-> FSMC_D12  |------------------+
	 +-------------------+--------------------+
	*/
	// GPIOD configuration 
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource4, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource10, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource11, GPIO_AF_FSMC); 
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5 | 
                                GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 |
                                GPIO_Pin_12 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

	GPIO_Init(GPIOD, &GPIO_InitStructure);


	// GPIOE configuration 
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource0 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource1 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource7 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource8 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource9 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource10 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource11 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource12 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource13 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource14 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource15 , GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0  | GPIO_Pin_1 | GPIO_Pin_7 |
                                GPIO_Pin_8  | GPIO_Pin_9  | GPIO_Pin_10 | GPIO_Pin_11|
                                GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;

	GPIO_Init(GPIOE, &GPIO_InitStructure);


	// GPIOF configuration 
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource0 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource1 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource2 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource3 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource4 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource5 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource12 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource13 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource14 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource15 , GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0  | GPIO_Pin_1  | GPIO_Pin_2  | GPIO_Pin_3 | 
                              	GPIO_Pin_4  | GPIO_Pin_5  | 
                              	GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;      

	GPIO_Init(GPIOF, &GPIO_InitStructure);


	// GPIOG configuration 
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource0 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource1 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource2 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource3 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource4 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource5 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource9 , GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0  | GPIO_Pin_1  | GPIO_Pin_2  | GPIO_Pin_3 | 
                                GPIO_Pin_4  | GPIO_Pin_5  |GPIO_Pin_9; 

	GPIO_Init(GPIOG, &GPIO_InitStructure);

	//FSMC Configuration 
	p.FSMC_AddressSetupTime = 0x2;
	p.FSMC_AddressHoldTime = 0;
	p.FSMC_DataSetupTime = 0xB;
	p.FSMC_BusTurnAroundDuration = 1;
	p.FSMC_CLKDivision = 0;
	p.FSMC_DataLatency = 0;
	p.FSMC_AccessMode = FSMC_AccessMode_A;

	FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM2;
	FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
	FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_PSRAM;
	FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;
	FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;
	FSMC_NORSRAMInitStructure.FSMC_AsynchronousWait = FSMC_AsynchronousWait_Disable;  
	FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
	FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;
	FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
	FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
	FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
	FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable;
	FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;
	FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &p;
	FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &p;

	FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure); 

	//!< Enable FSMC Bank1_SRAM2 Bank 
	FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM2, ENABLE); 
}

/*********************************************************************************
*PSRAM_WriteBuffer                                                               *
*把长度为NumHalfwordToWrite的数据写入到PSRAM中，写入的初始地址为WriteAddr        *
*********************************************************************************/
void PSRAM_WriteBuffer(uint16_t* pBuffer, uint32_t WriteAddr, uint32_t NumHalfwordToWrite)
{
  for(; NumHalfwordToWrite != 0; NumHalfwordToWrite--) // while there is data to write 
  {
    // Transfer data to the memory 
    *(uint16_t *) (Bank1_SRAM2_ADDR + WriteAddr) = *pBuffer++;
    
    // Increment the address  
    WriteAddr += 2;
  } 
}

/********************************************************************************
*PSRAM_ReadBuffer                                                               *
*从SRAM中读取数据，读取的初始地址为ReadAddr，读取的长度为NumHalfwordToRead，    *
*读取的数据放入到pBuffer数组中                                                  *
********************************************************************************/
void PSRAM_ReadBuffer(uint16_t* pBuffer, uint32_t ReadAddr, uint32_t NumHalfwordToRead)
{
  for(; NumHalfwordToRead != 0; NumHalfwordToRead--) /* while there is data to read */
  {
    /* Read a half-word from the memory */
    *pBuffer++ = *(__IO uint16_t*) (Bank1_SRAM2_ADDR + ReadAddr);

    /* Increment the address*/  
    ReadAddr += 2;
  } 
}

/********************************************************************************
*Init_AD7606                                                                    *
*配置AD7606，测量范围限定为±10V。                                               *
********************************************************************************/
void Init_AD7606(void)
{
	AD7606_Init_Pin();
	ConfigAD7606();	//限定输入±10V，对应输出0~0xffff
	PSRAM_Init();	//并口初始化
}

/********************************************************************************
*Start_AD7606                                                                   *
*启动AD7606的AD转换。CONVSTA/B的上升沿启动转换。                                *
********************************************************************************/
void Start_AD7606(void)
{
	AD7606_CONVSTA_On();
	AD7606_CONVSTB_On();
	
	AD7606_CONVSTA_Off();
	AD7606_CONVSTB_Off();
	Delay_Nop(20);
	AD7606_CONVSTA_On();
	AD7606_CONVSTB_On();
}

/********************************************************************************
*Read_AD7606                                                                    *
*读取AD7606的AD转换结果。                                                       *
********************************************************************************/
void Read_AD7606(unsigned short int* pBuffer)
{
	Delay_Nop(500);
	PSRAM_ReadBuffer(pBuffer, WRITE_READ_ADDR, 8);	// 连续8次读取所有AD通道值 	
}

/********************************************************************************
*Start_Read_AD7606_Result                                                       *
*启动AD转换并等转换完成后读取转换结果，用于对实时性要求较高的应用，CPU占用率高。*
********************************************************************************/
void Start_Read_AD7606(unsigned short int* pBuffer)
{
	Start_AD7606();
	Read_AD7606(pBuffer);
}

/********************************************************************************
*Read_Start_AD7606                                                              *
*读取转换结果上一次的转换结果后启动AD采样供下一次读取，耗时短，CPU占用率低。    *
********************************************************************************/
void Read_Start_AD7606(unsigned short int* pBuffer)
{
	PSRAM_ReadBuffer(pBuffer, WRITE_READ_ADDR, 8);	// 连续8次读取所有AD通道值 
	Start_AD7606();
}

//===========================================================================
// End of file.
//===========================================================================

