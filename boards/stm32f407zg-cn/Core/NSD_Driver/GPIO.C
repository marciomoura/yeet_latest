
/************************************************************************************
*GPIO驱动                                                                           *
*                                                                                   *
*初始化函数Init_GPIO()                                                              *
*状态刷新函数GPIO()                                                                 *
*DIxx_Flag开入瞬时值，实时刷新，DIxx_Delayed_Flag，延时刷新，防抖。                 *
************************************************************************************/
#include "stm32f4xx_fsmc.h"

#define Bank1_SRAM4_ADDR  ((uint32_t)0x6C000000)
#define DI_Delay_C 2	//开入延时周期

unsigned char DI_Flag[8][8];			//开入字节
unsigned char DI_Delayed_Flag[8][8];	//防抖延时后的开入字节
unsigned char DO_Flag[8][8];			//开出字节
int DI_Delay[8][8];	//开入延时
unsigned char DO_LED_Run;	//运行指示灯(=1 闪烁,=0 灭)
unsigned char DO_LED_Fault;	//报错指示灯(=1 亮,=0 灭)
unsigned char DO_LED_alarm;	//告警指示灯(=1 亮,=0 灭)


/************************************************************************************
*Init_DIDO_Para                                                                     *
*初始化DI、DO参数。                                                                 *
************************************************************************************/
void Init_DIDO_Para(void)
{
	int i,j;

	for (i=0;i<8;i++)
	{
		for (j=0;j<8;j++)
		{
			DI_Flag[i][j]=0;	//开入字节
			DI_Delayed_Flag[i][j]=0;//防抖开入
			DI_Delay[i][j]=0;
			DO_Flag[i][j]=0;
		}
	}
	DO_LED_Run=1;//启动运行灯闪烁
	DO_LED_Fault=0;
	DO_LED_alarm=0;
}

/************************************************************************************
*GPIO_FSMC_Init                                                                     *
*GPIO使用的FSMC接口初始化。                                                         *
************************************************************************************/
void GPIO_FSMC_Init(void)
{
	FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
	FSMC_NORSRAMTimingInitTypeDef  p;
	GPIO_InitTypeDef GPIO_InitStructure; 
  
	// Enable GPIOs clock 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOF |
                         RCC_AHB1Periph_GPIOG, ENABLE);

	// Enable FSMC clock 
	RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC, ENABLE); 
  
	//-- GPIOs Configuration -----------------------------------------------------
	/*
	 +-------------------+--------------------+------------------+------------------+
	 +                       pins assignment                                  +
	 +-------------------+--------------------+------------------+------------------+
	 | PD0   <-> FSMC_D2  |(PE0  <-> FSMC_NBL0)| PF0  <-> FSMC_A0 | PG0 <-> FSMC_A10 |
	 | PD1   <-> FSMC_D3  |(PE1  <-> FSMC_NBL1)| PF1  <-> FSMC_A1 | PG1 <-> FSMC_A11 |
	 | PD4   <-> FSMC_NOE | PE7  <-> FSMC_D4   | PF2  <-> FSMC_A2 | PG2 <-> FSMC_A12 |
	 | PD5   <-> FSMC_NWE | PE8  <-> FSMC_D5   | PF3  <-> FSMC_A3 | PG3 <-> FSMC_A13 |
	 | PG12  <-> FSMC_NE4 | PE9  <-> FSMC_D6   | PF4  <-> FSMC_A4 | PG4 <-> FSMC_A14 |
	 | PD8   <-> FSMC_D13 | PE10 <-> FSMC_D7   | PF5  <-> FSMC_A5 | PG5 <-> FSMC_A15 |
	 | PD9   <-> FSMC_D14 | PE11 <-> FSMC_D8   | PF12 <-> FSMC_A6 |------------------+ 
	 | PD10  <-> FSMC_D15 | PE12 <-> FSMC_D9   | PF13 <-> FSMC_A7 | 
	 | PD11  <-> FSMC_A16 | PE13 <-> FSMC_D10  | PF14 <-> FSMC_A8 |  
	 | PD12  <-> FSMC_A17 | PE14 <-> FSMC_D11  | PF15 <-> FSMC_A9 | 
	 | PD14  <-> FSMC_D0  | PE15 <-> FSMC_D12  |------------------+
	 | PD15  <-> FSMC_D1  |										|
	 +-------------------+--------------------+
	*/
	// GPIOD configuration 
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource4, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_FSMC);
//	GPIO_PinAFConfig(GPIOD, GPIO_PinSource7, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource10, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource11, GPIO_AF_FSMC); 
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5 |/* GPIO_Pin_7 |*/
                                GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 |
                                GPIO_Pin_12 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

	GPIO_Init(GPIOD, &GPIO_InitStructure);


	// GPIOE configuration 
 //	GPIO_PinAFConfig(GPIOE, GPIO_PinSource0 , GPIO_AF_FSMC);
 //	GPIO_PinAFConfig(GPIOE, GPIO_PinSource1 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource7 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource8 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource9 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource10 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource11 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource12 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource13 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource14 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource15 , GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin = /*GPIO_Pin_0  | GPIO_Pin_1 |*/ GPIO_Pin_7 |
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
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource12 , GPIO_AF_FSMC);


	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0  | GPIO_Pin_1  | GPIO_Pin_2  | GPIO_Pin_3 | 
                                GPIO_Pin_4  | GPIO_Pin_5  |GPIO_Pin_12;      

	GPIO_Init(GPIOG, &GPIO_InitStructure);

	// FSMC Configuration 
	p.FSMC_AddressSetupTime = 6;
	p.FSMC_AddressHoldTime = 15;
	p.FSMC_DataSetupTime = 21;
	p.FSMC_BusTurnAroundDuration = 1;
	p.FSMC_CLKDivision = 1;
	p.FSMC_DataLatency = 0;
	p.FSMC_AccessMode = FSMC_AccessMode_A;

	FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM4;
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

	//!< Enable FSMC Bank1_SRAM4 Bank 
	FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM4, ENABLE); 	
}


/************************************************************************************
*DO16_Out                                                                           *
*开出16个DO值。                                                                     *
************************************************************************************/
void DO16_Out(uint16_t addr, unsigned char * DO_Flag)
{
	unsigned short int data = 0,i;

	for(i = 0; i < 16; i++)
	{
		data = data >> 1;
		if(*DO_Flag++)
		{
			data |= 0x8000;
		}
	}
	*(uint16_t *) (Bank1_SRAM4_ADDR + (addr * 2)) = (uint16_t)data;
	for(i = 0 ; i < 6; i++)
	{
		;
	}


}

/************************************************************************************
*DO_Out                                                                             *
*根据数据刷新DO值。                                                                 *
************************************************************************************/
void DO_Out(void)
{
	unsigned char i,DO_LED[16];
	
	DO_LED[0] = DO_LED_Run;						//运行指示灯(=1 闪烁,=0 灭)
	DO_LED[1] = DO_LED_Fault;					//报错指示灯(=1 亮,=0 灭)
	DO_LED[2] = DO_LED_alarm;					//告警指示灯(=1 亮,=0 灭)

	DO16_Out(3,&DO_Flag[0][0]);		   			//DO低16位
	DO16_Out(4,&DO_Flag[2][0]);		   			//DO中16位
	DO16_Out(5,&DO_Flag[4][0]);		  			//DO高16位

	*(uint16_t *) (Bank1_SRAM4_ADDR + 12) = 0;	//启动输出
	for(i = 0 ; i < 6; i++)
	{
		;
	}

	DO16_Out(7,DO_LED);							//LED指示灯

}

/************************************************************************************
*DI4_Input                                                                          *
*开入16个DI值。                                                                     *
************************************************************************************/
void DI16_Input(uint16_t addr, unsigned char* DI_Flag)
{
	unsigned short int data,i;

	data = *(__IO uint16_t*) (Bank1_SRAM4_ADDR + addr * 2);
	
	for(i = 0; i < 16; i ++)
	{
		if(data & 0x0001)
		{
			*DI_Flag++ = 1;
		}
		else
		{
			*DI_Flag++ = 0;
		
		}
		data = data >> 1;
	}
}

/************************************************************************************
*Refresh_DI                                                                         *
*刷新DI。                                                                           *
************************************************************************************/
void Refresh_DI(void)
{
	DI16_Input(201, &DI_Flag[0][0]);
	DI16_Input(202, &DI_Flag[2][0]);
	DI16_Input(203, &DI_Flag[4][0]);
}

/************************************************************************************
*Delayed_DI                                                                         *
*开入延时防抖。                                                                     *
************************************************************************************/
void Delayed_DI(unsigned char DI_t, uint8_t* DI_Old, int* Delay)
{
	if (*DI_Old!=DI_t)
	{
		*Delay=*Delay+1;
		if (*Delay>=DI_Delay_C)
		{
			*DI_Old=DI_t;
			*Delay=0;
		}
	}
	else
	{
		*Delay=0;
	}
}

/************************************************************************************
*Refresh_Delayed_DI                                                                 *
*刷新DI_Old, 延时防抖                                                               *
************************************************************************************/
void Refresh_Delayed_DI(void)
{
	int i,j;

	for (i=0;i<8;i++)
	{
		for (j=0;j<8;j++)
		{
			Delayed_DI(DI_Flag[i][j], &DI_Delayed_Flag[i][j], &DI_Delay[i][j]);
		}
	}		
}


/************************************************************************************
*Init_GPIO                                                                          *
*初始化DI、DO的引脚及数据区。                                                       *
************************************************************************************/
void Init_GPIO(void)
{
	Init_DIDO_Para();
	GPIO_FSMC_Init();
	DO_LED_Run=1;
	Refresh_DI();				//开入. 2015.7.3.
	Refresh_Delayed_DI();		//开入延时防抖. 2015.7.3.
	Refresh_Delayed_DI();		//初始化时强制DI_Flag与DI_Delayed_Flag一致. 2015.7.`3.	
}

/************************************************************************************
*GPIO                                                                               *
*刷新DI和DO。                                                                       *
************************************************************************************/
void GPIO(void)
{
	Refresh_DI();				//开入
	Refresh_Delayed_DI();		//开入延时防抖
	DO_Out();					//开出
}
//===========================================================================
// End of file.
//===========================================================================
