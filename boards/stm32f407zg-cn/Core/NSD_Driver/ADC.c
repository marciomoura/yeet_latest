/**
  ******************************************************************************
  * @file    ADC.c
  * @author  Hardware Development Team
  * @version V1.0.0
  * @date    5-Sep-2012
  * @brief   This file 
  *****************************************************************************/
#include "main.h"

extern void Delay_Nop(unsigned int x);

uint16_t ADC_Num=0;
unsigned int j;
unsigned short int AD_Result[12]={0};

/********************************************************************************
*片上AD																			*
*片上AD最多可以测量4个模拟信号，量程0~10V，输出对应为0~0xfff。					*
*AD通道使用引脚PC0~PC3接入模拟信号，然后输入ARM芯片的AD通道10~13。				*
*********************************************************************************
*初始化函数void Init_OnChip_AD(int AD_Num)										*
*其中AD_Num是启用的AD测量通道数，设置范围1~4。									*
*循环测量函数void Get_OnChip_AD_Result(unsigned short int *AD_Result)			*
*AD_Result是用户存储AD结果的数据区地址，启用了多少通道就返回多少数据。			*
********************************************************************************/

/********************************************************************************
*Init_AD_Pin																	*
*初始化ARM片上AD需要使用的引脚PC0~PC3 。										*
********************************************************************************/
void Init_AD_Pin(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
  
  	//启用GPIOC的时钟
  	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);//启动ADC1时钟
	
  	//GPIO引脚PC5初始化：输出、上拉，50MHz刷新频率
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOC, &GPIO_InitStructure);	
}
/********************************************************************************
*Init_AD_DMA																	*
*初始化ARM片上AD的DMA。ADC1使用DMA2Stream4Channel0								*
********************************************************************************/
void Init_AD_DMA(int nNum)
{	
	DMA_InitTypeDef  DMA_InitStruct;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	
	DMA_StructInit(&DMA_InitStruct);//缺省值填充初始化参数
	DMA_InitStruct.DMA_Channel=DMA_Channel_0;
	DMA_InitStruct.DMA_PeripheralBaseAddr=(uint32_t)&ADC1->DR;
	DMA_InitStruct.DMA_Memory0BaseAddr=(uint32_t)AD_Result;
	DMA_InitStruct.DMA_DIR=DMA_DIR_PeripheralToMemory;//发送是外设到内存
	DMA_InitStruct.DMA_BufferSize=nNum;
	DMA_InitStruct.DMA_PeripheralInc=DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_MemoryInc=DMA_MemoryInc_Enable;//内存地址递增
	DMA_InitStruct.DMA_PeripheralDataSize	= DMA_PeripheralDataSize_HalfWord; 
	DMA_InitStruct.DMA_MemoryDataSize=DMA_MemoryDataSize_HalfWord;
	DMA_InitStruct.DMA_Mode=DMA_Mode_Circular;//DMA_Mode_Normal;
	DMA_InitStruct.DMA_Priority=DMA_Priority_High;
	DMA_InitStruct.DMA_FIFOMode= DMA_FIFOMode_Disable;
	DMA_InitStruct.DMA_FIFOThreshold= DMA_FIFOThreshold_HalfFull;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStruct.DMA_MemoryBurst = DMA_PeripheralBurst_Single;
	
	DMA_Init(DMA2_Stream4, &DMA_InitStruct);
	DMA_Cmd(DMA2_Stream4, ENABLE);	
}

/********************************************************************************
*Init_AD_Mode																	*
*初始化ARM片上AD基本配置 。														*
********************************************************************************/
void Init_AD_Mode(int32_t AD_Num)
{
	ADC_InitTypeDef       	ADC_InitStructure;
  	ADC_CommonInitTypeDef 	ADC_CommonInitStructure;

	assert_param(AD_Num<5 && AD_Num>0);	//参数范围检测
	ADC_Num=AD_Num;				//保留初始化时的AD通道数，返回AD结果时使用
	ADC_StructInit(&ADC_InitStructure);//12位数据、单次转换、非连续、无外部触发、数据右对齐、转换数次1
	ADC_InitStructure.ADC_ScanConvMode=ENABLE;//启用扫描模式
	ADC_InitStructure.ADC_NbrOfConversion=ADC_Num;//常规转换序列的长度，即转换次数	
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; //非连续转换
	ADC_Init(ADC1, &ADC_InitStructure);//初始化ADC1转换器

	//常规转换序列的配置：顺序转换ADC_IN10~ADC_IN13
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10 , 1, ADC_SampleTime_28Cycles);
	if (AD_Num>1)
	{
		ADC_RegularChannelConfig(ADC1, ADC_Channel_11 , 2, ADC_SampleTime_28Cycles);
	}
	if (AD_Num>2)
	{
		ADC_RegularChannelConfig(ADC1, ADC_Channel_12 , 3, ADC_SampleTime_28Cycles);
	}
	if (AD_Num==4)
	{
		ADC_RegularChannelConfig(ADC1, ADC_Channel_13 , 4, ADC_SampleTime_28Cycles);
	}

	ADC_CommonStructInit(&ADC_CommonInitStructure);	//不与其他ADC转换器同步、时钟2分频、禁用DMA、5周期AD转换间隔
	ADC_CommonInitStructure.ADC_DMAAccessMode =ADC_DMAAccessMode_1; 
	ADC_CommonInit(&ADC_CommonInitStructure);
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
	ADC_DMACmd(ADC1, ENABLE);
	//ADC_EOCOnEachRegularChannelCmd(ADC1,ENABLE);	//每个AD通道转换完成都会产生EOC，不是序列完成才产生EOC
	ADC_Cmd(ADC1,ENABLE);//ADC1上电
	
}

/********************************************************************************
*Init_OnChip_AD																	*
*初始化片上AD 。																*
*参数AD_Num：需要启用的AD通道数量。												*
********************************************************************************/
void Init_OnChip_AD(int AD_Num)
{
	Init_AD_Pin();//引脚初始化
	Init_AD_DMA(AD_Num);
	Init_AD_Mode(AD_Num);//AD模式初始化
	ADC_SoftwareStartConv(ADC1);
}

/********************************************************************************
*Get_OnChip_AD_Result															*
*读取常规序列的转换结果 。														*
*参数AD_Result：存储返回的AD结果的数据区地址。									*
*输入信号幅值与对应的结果：输入0~10V对应输出0~4095								*
********************************************************************************/
void Get_OnChip_AD_Result(unsigned short int *ADC_Result)
{
	int i;
	
	for (i=0;i<ADC_Num;i++)
	{
		*ADC_Result++=AD_Result[i];		
	}
	//ADC_ClearFlag(ADC1, ADC_FLAG_STRT);	//清除启动位
	
	ADC_SoftwareStartConv(ADC1);  //软件启动AD转换		
	
	if (ADC_GetFlagStatus(ADC1, ADC_FLAG_OVR))//容错，过载时清除错误标志
	{
		ADC_ClearFlag(ADC1, ADC_FLAG_OVR);
		ADC_DeInit();//复位ADC
		Init_AD_DMA(ADC_Num);//DMA重启
		Init_AD_Mode(ADC_Num);//AD模式初始化
	}
}

//===========================================================================
// End of file.
//===========================================================================
