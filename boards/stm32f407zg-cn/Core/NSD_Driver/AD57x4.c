 /* Includes ------------------------------------------------------------------*/

#include "stm32f4xx_spi.h"

#define Read  0x800000
#define Write 0x000000

//Channel Select
#define DAC_Channel_A    0x000000
#define DAC_Channel_B    0x010000
#define DAC_Channel_C    0x020000
#define DAC_Channel_D    0x030000
#define DAC_Channel_ALL  0x040000

//Register Select
#define DAC_Register                  0x000000
#define Output_Range_Select_Register  0x080000
#define Power_Control_Register        0x100000
#define Control_Register              0x180000

// Output Range Config
//Range1: 0~5V
//Range2: 0~10V
//Range3: 0~10.8V
//Range4: -5~+5V
//Range5: -10~+10V
//Range6: -10.8~+10.8V

#define Range_0_5V 0x000000
#define Range_0_10V 0x000001
#define Range_0_11V 0x000002
#define Range_N5_5V 0x000003
#define Range_N10_10V 0x000004
#define Range_N11_11V 0x000005

// Power Control
#define PowerUp_ALL         0x00000F

#define PowerUp_Channel_A   0x000001
#define PowerDown_Channel_A 0xFFFFFE

#define PowerUp_Channel_B   0x000002
#define PowerDown_Channel_B 0xFFFFFD

#define PowerUp_Channel_C   0x000004
#define PowerDown_Channel_C 0xFFFFFB

#define PowerUp_Channel_D   0x000008
#define PowerDown_Channel_D 0xFFFFF7

#define PowerUp_REF         0x000010
#define PowerDown_REF       0xFFFFEF

#define Nop 0x180000 //for readback
#define Control        0x190000

#define TSD_Enable     0x000008
#define TSD_Disable    0x000000

#define Clamp_Enable   0x000004
#define TSD_Disable    0x000000
					  
#define Clear_Select0  0x000002
#define Clear_Select1  0x000000

#define SDO_Disable    0x000001
#define SDO_Enable     0x000000

#define Clear 0x1C0000
#define Load 0x1D0000


/****************************************************************************************
*SPI1_Init																				*
*初始化SPI1的NSS、MOSI、MISO、SCK引脚。													*
****************************************************************************************/
void SPI1_Init(void)
{
	SPI_InitTypeDef  SPI_InitStructure;
  	GPIO_InitTypeDef GPIO_InitStructure;

	//分别为SPI1和GPIOA启用时钟信号
  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
  	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	//将PA4~PA7引脚分配给SPI1
  	GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_SPI1);
  	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
  	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);

   //PA5~PA7：推拉输出、下拉，50MHz刷新频率
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;

  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);

  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);

  	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//PA4：推拉输出、上拉、50MHz刷新频率
  	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;  	
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
 	//SPI初始化，使用SPI连接DA芯片AD5724
  	SPI_I2S_DeInit(SPI1);	 //必须先禁用,才能改变MODE 
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;//两线全双工 
  	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  	SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
  	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;//4分频 
  	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//先发最高位
  	SPI_InitStructure.SPI_CRCPolynomial = 7; //CRC多项式

  	SPI_InitStructure.SPI_Mode = SPI_Mode_Master; //SPI主模式
  	SPI_Init(SPI1, &SPI_InitStructure);
	
	SPI_SSOutputCmd (SPI1, ENABLE);//启用SPI1
}

/****************************************************************************************
*AD5724_nLDAC_Init																		*
*初始化AD5724的LDAC引脚(PC4)。															*
****************************************************************************************/
void AD5724_nLDAC_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
  
  	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/****************************************************************************************
*AD5724_nCLR_Init																		*
*初始化AD5724的CLR引脚(PC5)。															*
****************************************************************************************/
void AD5724_nCLR_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
  
  	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/****************************************************************************************
*AD5724_nLDAC_On																		*
*在AD5724的LDAC引脚输出高电平。															*
****************************************************************************************/
void AD5724_nLDAC_On(void)
{
   GPIOC->BSRRL =  GPIO_Pin_4;  
}

/****************************************************************************************
*AD5724_nLDAC_Off																		*
*在AD5724的LDAC引脚输出低电平。															*
****************************************************************************************/
void AD5724_nLDAC_Off(void)
{
   GPIOC->BSRRH =  GPIO_Pin_4;  
}

/****************************************************************************************
*AD5724_nCLR_On																			*
*在AD5724的CLR引脚输出高电平。															*
****************************************************************************************/
void AD5724_nCLR_On(void)
{
   GPIOC->BSRRL =  GPIO_Pin_5;  
}

/****************************************************************************************
*AD5724_nCLR_Off																		*
*在AD5724的CLR引脚输出低电平。															*
****************************************************************************************/
void AD5724_nCLR_Off(void)
{
   GPIOC->BSRRH =  GPIO_Pin_5;  
}

/****************************************************************************************
*AD5724_Init																			*
*初始化AD5724引脚。																		*
****************************************************************************************/
void AD5724_Init(void)
{
	SPI1_Init();
	AD5724_nLDAC_Init();
	AD5724_nCLR_Init();	
	AD5724_nCLR_On();
	AD5724_nLDAC_Off();	
}

/****************************************************************************************
*WriteToAD5724RViaSpi																	*
*通过SPI1向AD5724写出数据。																*
****************************************************************************************/
void WriteToAD5724RViaSpi(uint32_t Data)
{
	uint8_t  temp,temp1;
	
	temp = 	Data >> 8;
	temp1 = Data >> 16;	 

	SPI_Cmd(SPI1, ENABLE);
	
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SPI1, (uint8_t)  temp1 );
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SPI1, (uint8_t)  temp);
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SPI1, (uint8_t)  Data);
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

	SPI_Cmd(SPI1, DISABLE);
	for (temp=0;temp<30;temp++);
}

/****************************************************************************************
*Delay																					*
*空等。																					*
****************************************************************************************/
void DelayT(unsigned int T)
{
	unsigned int i;
	
	for (i=0;i<T;i++);
}

/****************************************************************************************
*ConfigAD5724R																			*
*配置AD5724。																			*
****************************************************************************************/
void ConfigAD5724R(void)
{
	unsigned int i;

	i = Power_Control_Register | PowerUp_ALL ;
	WriteToAD5724RViaSpi(i);
	DelayT(4000);
	i = Output_Range_Select_Register | Range_N5_5V | DAC_Channel_ALL ;
	WriteToAD5724RViaSpi(i);
}

/****************************************************************************************
*Init_DA																				*
*初始化DA，将DA范围限定到±5V。															*
****************************************************************************************/
void Init_DA()
{
	int i;

	AD5724_Init();	 	//引脚初始化
	for (i=0;i<3;i++)	//AD5724配置一次可能不成功
	{
		ConfigAD5724R();//配置DA芯片: 4个DAC通道上电并设置输出范围为±5V
	}
}

/****************************************************************************************
*DA																						*
*在对应DA通道输出所需的电压值。															*
*nCh:通道选择, 0~3																		*
*DA_V:输出电压-5~5																		*
****************************************************************************************/
void DA(int nCh, float DA_V)
{
	uint32_t DA_Data;
	if (nCh>=0 && nCh<4 && DA_V>=-5 && DA_V<=5)//参数范围检测
	{
		DA_Data=(uint32_t)((DA_V+5)/(float)10.0*65535);  //电压转为DA数值
		DA_Data|=nCh<<16;  		
		WriteToAD5724RViaSpi(DA_Data);
	}	
}

//===========================================================================
// End of file.
//===========================================================================

