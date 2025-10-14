/**
  ******************************************************************************
  * @file    TIM_Init.c
  * @author  Hardware Development Team
  * @version V1.0.0
  * @date    5-Sep-2012
  * @brief   This file 
  *****************************************************************************/

#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "pub.h"
#include "AI.h"
#include "syn.h"
#include "optical.h"

#define Pulse_Zero 0x00
#define Pulse_All 0x3f
//#define Alpha_C 28.5f	//乌东德 
#define USE_TIM9 	1

unsigned int Syn_A_Old_t=0, Syn_B_Old_t=0, Syn_C_Old_t=0;
unsigned int Low_Level_Period;//脉冲低电平周期

int Syn_BA_Time;
int Syn_CB_Time;
int Syn_AC_Time;

unsigned int Rising_Edge_Time[7];//执行均流之后的6脉冲上升沿时刻 

unsigned int Syn_D_T=0;

 /******************************************************************************
  ** 定时器初始化
 *******************************************************************************/
 /*
  * @brief  TIM  configuration
  * @param  None
  * @retval None
 *******************************************************************************/


//3:-C+A 6:+B-C 0xc:-A+B 0x18:+C-A 0x30:-B+C 0x21:+A-B
unsigned char Pulse_Data[12]={3,6,0xc,0x18,0x30,0x21,3,6,0xc,0x18,0x30,0x21};

/************************************************************************
*Generate_Tx_Flag                                                       *
*生成光纤通信发送标志。                                                 *
************************************************************************/
void Generate_Tx_Flag(void)
{
	Comm_Count++;
	if (Comm_Count==1)
	{
		Optical_Tx_Flag=1;
	}

	if (Comm_Count>=4-Sec_A_Flag-Sec_B_Flag-Sec_C_Flag-Pulse_Low_F_Flag)//2019.1.22.
	{
		Comm_Count=0;
	}
}

/************************************************************************
*Generate_Pulse                                                         *
*生成脉冲。                                                             *
************************************************************************/
void Generate_Pulse(unsigned char Pulse)
{
	//2020.6.30. clear all pulse before generating new pulse
	GPIOC->BSRRH = GPIO_Pin_6;//+A, PC6
	GPIOB->BSRRH = GPIO_Pin_1;//-C, PB1
	GPIOC->BSRRH = GPIO_Pin_7;//+B, PC7
	GPIOC->BSRRH = GPIO_Pin_9;//-A, PC9
	GPIOC->BSRRH = GPIO_Pin_8;//+C, PC8
	GPIOB->BSRRH = GPIO_Pin_0;//-B, PB0
	
	if (Pulse & 1)	//+A, PC6
	{
		GPIOC->BSRRL = GPIO_Pin_6;
	}
	else
	{
		GPIOC->BSRRH = GPIO_Pin_6;
	}
	if (Pulse & 2)	//-C, PB1
	{
		GPIOB->BSRRL = GPIO_Pin_1;
	}
	else
	{
		GPIOB->BSRRH = GPIO_Pin_1;
	}
	if (Pulse & 4)	//+B, PC7
	{
		GPIOC->BSRRL = GPIO_Pin_7;
	}
	else
	{
		GPIOC->BSRRH = GPIO_Pin_7;
	}
	if (Pulse & 8)	//-A, PC9
	{
		GPIOC->BSRRL = GPIO_Pin_9;
	}
	else
	{
		GPIOC->BSRRH = GPIO_Pin_9;
	}
	if (Pulse & 16)	//+C, PC8
	{
		GPIOC->BSRRL = GPIO_Pin_8;
	}
	else
	{
		GPIOC->BSRRH = GPIO_Pin_8;
	}
	if (Pulse & 32)	//-B, PB0
	{
		GPIOB->BSRRL = GPIO_Pin_0;
	}
	else
	{
		GPIOB->BSRRH = GPIO_Pin_0;
	}
}

/************************************************************************
*Init_Pulse_Pin                                                         *
*初始化脉冲对应的引脚。                                                 *
************************************************************************/
void Init_Pulse_Pin(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;    	//					   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	Generate_Pulse(Pulse_Zero);
	Rising_Edge_Time[0]=0;
	Rising_Edge_Time[1]=0;
	Rising_Edge_Time[2]=0;
	Rising_Edge_Time[3]=0;
	Rising_Edge_Time[4]=0;
	Rising_Edge_Time[5]=0;
	Rising_Edge_Time[6]=0;
}

/************************************************************************
*First_Rising_Edge                                                      *
*α中断，生成脉冲序列第一组上升沿, 计算所有上升沿时刻。                  *
************************************************************************/
void First_Rising_Edge(void)
{
	//脉冲复位
	TIM_ClearFlag(TIM2, TIM_IT_Update);
	TIM_Cmd(TIM2, DISABLE);	
	TIM_Cmd(TIM4, DISABLE);	
	TIM_ITConfig(TIM4,TIM_IT_Update,DISABLE);
	TIM_ClearFlag(TIM4, TIM_IT_Update);
	
	Pulse_Pointer=0;	//脉冲序号归零
	
	//刷新同步中断时刻及脉冲相移
	if ((SynSource & 0xf)==2)//Syn_B_Work_Flag
	{
		Phase_Offset=2;				//同步相补偿仅与同步时刻一起刷新
	}
	else if ((SynSource & 0xf)==3)//Syn_C_Work_Flag
	{
		Phase_Offset=4;				//同步相补偿仅与同步时刻一起刷新
	}
	else
	{
		Phase_Offset=0;				//同步相补偿仅与同步时刻一起刷新
	}
	
	Rising_Edge_Time[1]=Period_av*0.0238095f;//   1/42=0.0238095
	Rising_Edge_Time[6]=Rising_Edge_Time[1]*2;//120°, 60°脉宽之后没有新的同步清除脉冲, 则持续约60°后停止 
	if (Rising_Edge_Time[1]>0xffff)
	{
		Rising_Edge_Time[1]=0xffff;
	}
	if (Rising_Edge_Time[1]<0x100)
	{
		Rising_Edge_Time[1]=0x100;
	}
	if (Rising_Edge_Time[6]>0xffff)
	{
		Rising_Edge_Time[6]=0xffff;
	}
	if (Rising_Edge_Time[6]<0x100)
	{
		Rising_Edge_Time[6]=0x100;
	}
	
	if ( DO_Lock_Flag==0 )	
	{
		if (Pulse_Test_Flag && Low_F_Flag && (Pulse_Enabled_Flag==0))
		{//测试时只发出测试部分的脉冲
			Generate_Pulse(Test_Pulse & Pulse_Data[Phase_Offset+Pulse_Pointer]);
		}
		else if (Pulse_Enabled_Flag && (Pulse_Low_F_Flag==0))
		{
			Current_Pulse=Pulse_Data[Phase_Offset+Pulse_Pointer];
			if (Alpha<5)
			{
				Generate_Pulse(Pulse_All);
			}
			else
			{
				Generate_Pulse(Current_Pulse);			
			}
		}
		else
		{
			Generate_Pulse(Pulse_Zero); 
		}
	}
	else
	{
		Generate_Pulse(Pulse_Zero); 
	}
	Pulse_Pointer++;
	TIM4->CNT=0;
	TIM4->ARR=Rising_Edge_Time[1];
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);	
	TIM_Cmd(TIM4, ENABLE);	
}

/************************************************************************
*Set_Alpha_Delay                                                        *
*设置α定时中断。                                                        *
************************************************************************/
void Set_Alpha_Delay(void)
{
	unsigned int dt;
	
	TIM_Cmd(TIM2, DISABLE);	
	TIM2->CNT=0;
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	
	dt=(Alpha+Alpha_C)*Period/360;//α*Period/360,即α折算成计数值

	TIM2->ARR=dt;
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM2, ENABLE);
}

/************************************************************************
*TIM5_Init                                                              *
*TIM5初始化，通道1、2、4对应三相同步捕捉，通道3作为脉冲同步中断。       *
************************************************************************/
void TIM5_Init(void)
{
	TIM_TimeBaseInitTypeDef 	TIM_TimeBaseStructure;	
	TIM_ICInitTypeDef   	  	TIM_IC;	
	NVIC_InitTypeDef      		NVIC_InitStructure;
	GPIO_InitTypeDef      		GPIO_InitStructure;

	TIM_DeInit(TIM5);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);   	//1. 打开定时器时钟源,84MHz
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;			//2. 设置GPIO引脚
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;			//   输入设置为下拉
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		//   同步信号捕捉,50MHz速度
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0,GPIO_AF_TIM5);	//3. 将引脚指定给T5 									   					
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1,GPIO_AF_TIM5);
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3,GPIO_AF_TIM5);

	//4. 输入捕捉设置：三相同步信号
	TIM_IC.TIM_Channel=TIM_Channel_1;						//通道1
	TIM_IC.TIM_ICPolarity=TIM_ICPolarity_Rising;			//捕捉上升沿
	TIM_IC.TIM_ICSelection=TIM_ICSelection_DirectTI;		//IC1连接到通道1
	TIM_IC.TIM_ICPrescaler=	TIM_ICPSC_DIV1;					//无预分频
	TIM_IC.TIM_ICFilter=0;									//无延时滤波
	TIM_ICInit(TIM5, &TIM_IC);
	TIM_IC.TIM_Channel=TIM_Channel_2;						//通道2
	TIM_ICInit(TIM5, &TIM_IC);
	TIM_IC.TIM_Channel=TIM_Channel_4;						//通道4
	TIM_ICInit(TIM5, &TIM_IC);															
	
	TIM_CCxCmd(TIM5,TIM_Channel_1,TIM_CCx_Enable);			//允许使用定时器3个通道
	TIM_CCxCmd(TIM5,TIM_Channel_2,TIM_CCx_Enable);
	TIM_CCxCmd(TIM5,TIM_Channel_3,TIM_CCx_Disable);			
	TIM_CCxCmd(TIM5,TIM_Channel_4,TIM_CCx_Enable);			

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);			//6.设置时间基值
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);			//7. 中断向量设置
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;			//中断服务程序入口
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;//次高中断优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	TIM_ClearFlag(TIM5, TIM_FLAG_Update);		            //清除溢出中断标志
	TIM_ARRPreloadConfig(TIM5, DISABLE);	                //禁止ARR预装载缓冲器，改写周期后新周期值下一刷新事件后有效
	TIM_ITConfig(TIM5,TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC4 ,ENABLE); //开启TIM5的中断
	DBGMCU_APB1PeriphConfig( DBGMCU_TIM5_STOP,ENABLE);		//调试中止程序时T5暂停
	TIM_Cmd(TIM5, ENABLE);									//8.启用TIM5，开始计数 	
}

/************************************************************************
*TIM2_Init                                                              *
*TIM2初始化，控制角中断。                                               *
************************************************************************/
void TIM2_Init(void)
{
	TIM_TimeBaseInitTypeDef   	TIM_TimeBaseStructure;	
	NVIC_InitTypeDef      		NVIC_InitStructure;
	
	TIM_DeInit(TIM2);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);   	//1. 打开定时器时钟源,84MHz
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);			//2.设置时间基值
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);			//3. 中断向量设置
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;			//中断服务程序入口
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//最高中断优先级，不允许嵌套
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
	
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);		            //清除溢出中断标志
	TIM_ARRPreloadConfig(TIM2, DISABLE);	                //禁止ARR预装载缓冲器，改写周期后新周期值下一刷新事件后有效
	DBGMCU_APB1PeriphConfig( DBGMCU_TIM2_STOP,ENABLE);		//调试中止程序时T2暂停
}

/************************************************************************
*TIM9_Init                                                              *
*TIM9初始化，脉冲驱动中断。                                             *
************************************************************************/
void TIM9_Init(void)
{
	TIM_TimeBaseInitTypeDef   	TIM_TimeBaseStructure;	
	NVIC_InitTypeDef      		NVIC_InitStructure;
	GPIO_InitTypeDef      		GPIO_InitStructure;
	TIM_ICInitTypeDef   	  	TIM_IC;	
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;			//2. 设置GPIO引脚
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;			//   输入设置为下拉
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		//   同步信号捕捉,50MHz速度
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM9);
	
	TIM_DeInit(TIM9);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);   	//1. 打开定时器时钟源,84MHz
	TIM_IC.TIM_Channel = TIM_Channel_1;						//通道3
	TIM_IC.TIM_ICPolarity = TIM_ICPolarity_Rising;			//捕捉上升沿
	TIM_IC.TIM_ICSelection = TIM_ICSelection_DirectTI;		//IC1连接到通道1
	TIM_IC.TIM_ICPrescaler =	TIM_ICPSC_DIV1;					//无预分频
	TIM_IC.TIM_ICFilter = 0;									//无延时滤波 
	TIM_ICInit(TIM9, &TIM_IC);
	TIM_CCxCmd(TIM9, TIM_Channel_1, TIM_CCx_Enable);	
	
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);			//2.设置时间基值
	TIM_TimeBaseStructure.TIM_Prescaler = 20;//21分频, 4MHz
	//TIM_TimeBaseStructure.TIM_Period = 0xFFFF;//5kHz
	TIM_TimeBaseInit(TIM9, &TIM_TimeBaseStructure);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);			//3. 中断向量设置
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_TIM9_IRQn;			//中断服务程序入口
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//最低中断优先级, 允许嵌套
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
	
	TIM_ClearFlag(TIM9, TIM_FLAG_Update);		            //清除溢出中断标志
	TIM_ARRPreloadConfig(TIM9, DISABLE);	                //禁止ARR预装载缓冲器，改写周期后新周期值下一刷新事件后有效
	TIM_ITConfig(TIM9, TIM_IT_CC1, ENABLE);
	DBGMCU_APB1PeriphConfig( DBGMCU_TIM9_STOP,ENABLE);		//调试中止程序时T2暂停 	
	TIM_Cmd(TIM9, ENABLE);
}


/************************************************************************
*TIM3_Init                                                              *
*TIM3初始化，定时AI。                                                   *
************************************************************************/
void TIM3_Init(void)
{
	TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
  	
	TIM_DeInit(TIM3);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);//开启定时器时钟
	TIM_InternalClockConfig(TIM3);
  	
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 0;//1分频, 84MHz
	TIM_TimeBaseStructure.TIM_Period = 16799;//5kHz
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); 
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);		                    	//清除溢出中断标志
	TIM_ARRPreloadConfig(TIM3, DISABLE);	                     	  	//禁止ARR预装载缓冲器
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);						  	//开启TIM3的中断
	TIM_Cmd(TIM3, ENABLE);												//初始化时启用, 不能终止 
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;			//抢断优先级配置（0为最高）
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;			    	//子优先级配置  （0为最高）
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
}

/************************************************************************
*TIM4_Init                                                              *
*TIM4初始化，脉冲中断。                                                 *
************************************************************************/
void TIM4_Init(void)
{
	TIM_TimeBaseInitTypeDef   	TIM_TimeBaseStructure;	
	NVIC_InitTypeDef      		NVIC_InitStructure;
	
	TIM_DeInit(TIM4);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);   	//1. 打开定时器时钟源,84MHz
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);			//2.设置时间基值
	TIM_TimeBaseStructure.TIM_Prescaler = 6;//7分频
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);			//3. 中断向量设置
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;			//中断服务程序入口
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//最高中断优先级，不允许嵌套
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;//次高响应优先级, 允许控制角中断优先响应 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
	
	TIM_ClearFlag(TIM4, TIM_FLAG_Update);		            //清除溢出中断标志
	TIM_ARRPreloadConfig(TIM4, DISABLE);	                //禁止ARR预装载缓冲器，改写周期后新周期值下一刷新事件后有效
	DBGMCU_APB1PeriphConfig( DBGMCU_TIM4_STOP,ENABLE);		//调试中止程序时T2暂停 	
}

/************************************************************************
*TIM5_CC1                                                               *
*A相同步中断功能。                                                      *
************************************************************************/
void TIM5_CC1()
{
	TIM_ClearITPendingBit(TIM5, TIM_IT_CC1);
	Syn_A_Int_Time=TIM5->CCR1;
	Syn_CA_Time=Phase_Difference(Syn_C_Int_Time,Syn_A_Int_Time);
	Cal_Period(Syn_A_Int_Time, &Syn_A_Old_t);
	Low_F_Flag=0;	//清除低频标志
	Low_F_Count=0;
	Syn_AC_Time=Period-Syn_CA_Time;
	Syn_Check(&Sec_A_Flag, &Sec_A_Count);//仅计算最近两相相差 	
	Generate_Tx_Flag();
}

/************************************************************************
*TIM5_CC2                                                               *
*B相同步中断功能。                                                      *
************************************************************************/
void TIM5_CC2()
{
	TIM_ClearITPendingBit(TIM5, TIM_IT_CC2);	 
	Syn_B_Int_Time=TIM5->CCR2;
	Syn_AB_Time=Phase_Difference(Syn_A_Int_Time,Syn_B_Int_Time);
	Cal_Period(Syn_B_Int_Time, &Syn_B_Old_t);//计算周期，并刷新前次本相中断时刻
	Low_F_Flag=0;	//清除低频标志
	Low_F_Count=0;
	Syn_BA_Time=Period-Syn_AB_Time; 
	Syn_Check(&Sec_B_Flag, &Sec_B_Count);
	Generate_Tx_Flag();
}

/************************************************************************
*TIM5_CC4                                                               *
*C相同步中断功能。                                                      *
************************************************************************/
void TIM5_CC4()
{
	TIM_ClearITPendingBit(TIM5, TIM_IT_CC4);
	Syn_C_Int_Time=TIM5->CCR4;
	Syn_BC_Time=Phase_Difference(Syn_B_Int_Time,Syn_C_Int_Time);

	Cal_Period(Syn_C_Int_Time, &Syn_C_Old_t);//计算周期，并刷新前次本相中断时刻
	Low_F_Flag=0;	//清除低频标志
	Low_F_Count=0;
	Syn_CB_Time=Period-Syn_BC_Time;
	Syn_Check(&Sec_C_Flag, &Sec_C_Count);
	Generate_Tx_Flag();
}
/************************************************************************
*TIM2_IRQHandler                                                        *
*控制角中断。                                                           *
************************************************************************/
void TIM2_IRQHandler()
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update))				//定时器通道1输入捕捉
	{
		First_Rising_Edge();
	}
}				

/************************************************************************
*TIM1_BRK_TIM9_IRQHandler                                               *
*FPGA软同步中断。                                                       *
************************************************************************/
void TIM1_BRK_TIM9_IRQHandler()
{
	unsigned int tSyn;
	unsigned int dt;
	unsigned int Period_min;
	
	if (TIM_GetITStatus(TIM9, TIM_IT_CC1))				//定时器通道1输入捕捉
	{
		TIM_ClearITPendingBit(TIM9, TIM_IT_CC1);		//清除定时中断标志
		tSyn = TIM5->CNT;

		dt=tSyn-Syn_D_T;
		Period_min=Period*0.95f;//2020.12.28.
		if (dt>Period_min)
		{
			Set_Alpha_Delay();			
			Syn_D_T=tSyn;
			
			if (Sec_A_Flag && Sec_B_Flag && Sec_C_Flag)
			{
				Cal_Period(tSyn, &Syn_Int_Time);
			}
			
			Syn_Int_Time=tSyn;
			
			Set_Alpha_Delay();
			Generate_Tx_Flag();
			Pulse_Syn_Error_Check();
			Pulse_Low_F_Count=0;
			Pulse_Low_F_Flag=0;
		}
	}	
}	

/************************************************************************
*TIM4_Update                                                            *
*脉冲中断功能。                                                         *
************************************************************************/
void TIM4_Update()
{
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);	 
	if (Pulse_Pointer<6)
	{
		if ( DO_Lock_Flag==0)	
		{
			if (Pulse_Test_Flag)
			{//测试时只发出测试部分的脉冲
				Generate_Pulse(Test_Pulse & Pulse_Data[Phase_Offset+Pulse_Pointer]);
			}
			else if (Pulse_Enabled_Flag)
			{
				if (Alpha>=5)
				{
					Current_Pulse=Pulse_Data[Phase_Offset+Pulse_Pointer];
					Generate_Pulse(Current_Pulse);
				}
				else//0°为起励时功率柜无控整流（6桥臂全开）专用
				{
					Generate_Pulse(Pulse_All);//整流桥全开
				}
			}			
		}
		Pulse_Pointer++;
		
		if (Pulse_Pointer==6)
		{
			TIM4->ARR=Rising_Edge_Time[6];
		}
		
	}//Pulse_Pointer<6	 
	else
	{
		Pulse_Pointer=6;//仅允许控制角中断复位脉冲指针 
		Generate_Pulse(Pulse_Zero);
	}
}

/************************************************************************
*TIM4_IRQHandler                                                        *
*脉冲中断。                                                             *
************************************************************************/
void TIM4_IRQHandler()
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update))				//定时器通道1输入捕捉
	{
		 TIM4_Update();
	}	
}
/************************************************************************
*TIM5_IRQHandler                                                        *
*TI5中断服务程序。                                                      *
************************************************************************/
void TIM5_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM5, TIM_IT_CC1))				//定时器通道1输入捕捉
	{
		 TIM5_CC1();
	}

	if (TIM_GetITStatus(TIM5, TIM_IT_CC2))				//定时器通道2输入捕捉
	{
		 TIM5_CC2();
	}

	if (TIM_GetITStatus(TIM5, TIM_IT_CC4))				//定时器通道4输入捕捉
	{
		 TIM5_CC4();
	}
}

/************************************************************************
*TIM3_IRQHandler                                                        *
*TI3中断服务程序：定时AI。                                              *
************************************************************************/
void TIM3_IRQHandler(void)
{
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	AD();//启动AD转换，获取AD结果
}

/************************************************************************
*TIMx_Init                                                              *
*定时器初始化。                                                         *
************************************************************************/
void TIMx_Init(void)								       
{	
	Init_Pulse_Pin();
	TIM5_Init();	//同步中断 
	TIM2_Init();  	//控制角驱动同步中断
	TIM3_Init();	//定时采样
	TIM4_Init();//脉冲中断
	TIM9_Init();
}

//===========================================================================
// End of file.
//===========================================================================
