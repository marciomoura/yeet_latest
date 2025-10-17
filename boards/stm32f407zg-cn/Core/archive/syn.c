/********************************************************************
*Syn.c                                                              *
*                                                                   *
*同步信号程序.                                                      *
*Copyright: 2002-2013 Nengshida Electric Ltd.                       *
*Author: Yu Xiang                                                   *
*Jun.2, 2013                                                       	*
********************************************************************/

#include "pub.h"
#include "optical.h"
#include"nandflash_sram.h"
#include "stm32f4xx_rtc.h"

#define Sec_Syn_Delay 4
#define Primary_Syn_Delay 4
#define Syn_Pulse_Delay 3

/************************************************************************
*Clear_Syn_Error                                                        *
*清除同步故障标志及相应的计数器。复位初始化。                           *
************************************************************************/
void Clear_Syn_Error(void)
{
	Sec_A_Count=0;
	Sec_B_Count=0;
	Sec_C_Count=0;
	
	Sec_A_Flag=0;
	Sec_B_Flag=0;
	Sec_C_Flag=0;
	
	Syn_A_Count=0;
	Syn_B_Count=0;
	Syn_C_Count=0;
	
	Syn_A_Err_Flag=0;
	Syn_B_Err_Flag=0;
	Syn_C_Err_Flag=0;
	
	Syn_Error_Flag=0;
}

/************************************************************************
*InitSynSelect                                                          *
*通知FPGA自身拨码, 明确设备ID。                                         *
************************************************************************/
void InitSynSelect(void)
{
	unsigned char* ucPt;
	
	ucPt=(unsigned char*)0x6C000010;
	*ucPt=uchNodeNum;
	ucPt=(unsigned char*)0x6C000012;
	*ucPt=0x11;
}

/************************************************************************
*Syn_Pulse_Init                                                         *
*同步、脉冲修改标志、变量的初始化。                                     *
************************************************************************/
void Syn_Pulse_Init(void)
{
	Syn_A_Work_Flag=1;
	Syn_B_Work_Flag=0;
	Syn_C_Work_Flag=0;

	Phase_Offset=0;
	Pulse_Pointer=0;
	Current_Pulse=3;

	Clear_Syn_Error();

	
	Period=1680000;//84MHz计数器对应的20ms计数值
	Period_av=Period;
	Period_Source[0]=Period;
	Period_Source[1]=Period;
	Period_Source[2]=Period;
	Period_Source[3]=Period;
	Period_Source[4]=Period;
	F=1.0;
	Fav=1.0;

	Syn_A_Int_Time=0;
	Syn_B_Int_Time=Period/3;
	Syn_C_Int_Time=Syn_B_Int_Time+Syn_B_Int_Time;
	Syn_P_Int_Time=0;  //脉冲驱动相中断时刻 
	
	Syn_AB_Time=Syn_B_Int_Time;
	Syn_BC_Time=Syn_B_Int_Time;
	Syn_CA_Time=Syn_B_Int_Time;

	Syn_Int_Time=0;
	Syn_F_Delay=50;

	Rising_Edge_Flag=1;
	Sec_A_Flag=1;
	Sec_B_Flag=1;
	Sec_C_Flag=1;
	Low_F_Flag=1;
	Pulse_Enabled_Flag=0;
	Alpha=15.0; 
	InitSynSelect();
	
	Pulse_Low_F_Flag=0;
	Pulse_Low_F_Count=0;
	Syn_Pulse_Count=0;
	
	Syn_Switch_Delay=0;
}

/************************************************************************
*SelectSyn                                                              *
*通知FPGA所选的驱动脉冲的同步相。                                       *
************************************************************************/
void SelectSyn(unsigned char ucSyn)
{
	unsigned short int* ucPt;
	
	ucPt=(unsigned short int*)0x6C000010;
	*ucPt=uchNodeNum;
	ucPt=(unsigned short int*)0x6C000012;
	*ucPt=ucSyn;
	
	Syn_A_Work_Flag=(ucSyn & 7)==1;
	Syn_B_Work_Flag=(ucSyn & 7)==2;
	Syn_C_Work_Flag=(ucSyn & 7)==3;
}

/************************************************************************
*Check_Syn_Error                                                        *
*根据原方和副方同步故障标志设置同步故障标志。                           *
************************************************************************/
void Check_Syn_Error()
{
	if (Syn_A_Err_Flag || Syn_B_Err_Flag || Syn_C_Err_Flag ||
		Sec_A_Flag || Sec_B_Flag ||Sec_C_Flag)
	{
		Syn_Error_Flag=1;		
	}
	else
	{
		Syn_Error_Flag=0;
	}
}

/************************************************************************
*Sec_Syn_Check                                                          *
*副方同步断线检测。                                                     *
************************************************************************/
void Sec_Syn_Check(unsigned char* Phase_Flag, int* Phase_Count)
{
	if (Phase_Flag!=&Sec_A_Flag)
	{
		if (++Sec_A_Count>=Sec_Syn_Delay)
		//if (++Sec_A_Count>=Sec_Syn_Delay-100*Pulse_Enabled_Flag)//Sec_Syn_Delay, 2015.9.1. 
		{
			Sec_A_Count=Sec_Syn_Delay;//Sec_Syn_Delay=7	
			Sec_A_Flag=1;
		}
	}
	if (Phase_Flag!=&Sec_B_Flag)
	{
		if (++Sec_B_Count>=Sec_Syn_Delay)
		//if (++Sec_B_Count>=Sec_Syn_Delay-100*Pulse_Enabled_Flag)//Sec_Syn_Delay, 2015.9.1. 
		{
			Sec_B_Count=Sec_Syn_Delay;	
			Sec_B_Flag=1;
		}
	}
	if (Phase_Flag!=&Sec_C_Flag)
	{
		if (++Sec_C_Count>=Sec_Syn_Delay)
		//if (++Sec_C_Count>=Sec_Syn_Delay-100*Pulse_Enabled_Flag)//Sec_Syn_Delay, 2015.9.1. 
		{
			Sec_C_Count=Sec_Syn_Delay;
			Sec_C_Flag=1;
		}
	}
	
	*Phase_Flag=0;//瞬时恢复
	*Phase_Count=0;
	
}

/************************************************************************
*Primary_Syn_Switch_Check                                               *
*原方同步断线是否需要切换同步信号源。                                   *
************************************************************************/
void Primary_Syn_Switch_Check(unsigned char* Syn_Flag, unsigned char Error_Syn_Phase)
{
	int n;
	
	n=SynSource>>4;
	
	if ((n==uchNodeNum) && (*Syn_Flag==0))//同步源来自本柜, 原方同步断线故障标志的边沿响应
	{
		ScrSynError[n+1] |=Error_Syn_Phase;
		SelectSynSource();
		SelectSyn(SynSource);
	}
}

/************************************************************************
*Primary_Syn_Check                                                      *
*原方同步断线检测, 每个同步中断执行一次。                               *
************************************************************************/
void Primary_Syn_Check(void)
{
 	unsigned int T_60, T_300;

 	if (Sec_A_Flag || Sec_B_Flag || Sec_C_Flag)//副方断线优先
 	{
 		Syn_A_Err_Flag=0;
 		Syn_B_Err_Flag=0;
 		Syn_C_Err_Flag=0;
 	}
 	else
 	{
 		T_60=Period/6;	 // 60度对应的计数值
 		T_300=Period-T_60;//300度对应的计数值
 		if (Syn_AB_Time<T_60 || Syn_AB_Time>T_300)
 		{
 			if (++Syn_C_Count>Primary_Syn_Delay)	//Primary_Syn_Delay=4
 			{
 				Primary_Syn_Switch_Check(&Syn_B_Err_Flag, 2);
				Syn_B_Err_Flag=1;
 				Syn_C_Count=100;
 			}
 		}
 		else
 		{
 			if (--Syn_C_Count<=0)
 			{
 				Syn_B_Err_Flag=0;
 				Syn_C_Count=0;
 			}
 		}
 		
 		if (Syn_BC_Time<T_60 || Syn_BC_Time>T_300)
 		{
 			if (++Syn_A_Count>Primary_Syn_Delay)
 			{
 				Primary_Syn_Switch_Check(&Syn_C_Err_Flag, 4);
				Syn_C_Err_Flag=1;
 				Syn_A_Count=100;
 			}
 		}
 		else
 		{
 			if (--Syn_A_Count<=0)
 			{
 				Syn_C_Err_Flag=0;
 				Syn_A_Count=0;
 			}
 		}
 		
 		if (Syn_CA_Time<T_60 || Syn_CA_Time>T_300)
 		{
 			if (++Syn_B_Count>Primary_Syn_Delay)
 			{
 				Primary_Syn_Switch_Check(&Syn_A_Err_Flag, 1);
				Syn_A_Err_Flag=1;
 				Syn_B_Count=100;
 			}
 		}
 		else
 		{
 			if (--Syn_B_Count<=0)
 			{
 				Syn_A_Err_Flag=0;
 				Syn_B_Count=0;
 			}
 		}
 	}
}

/************************************************************************
*Phase_Difference                                                       *
*计算两相之间的相角差。                                                 *
************************************************************************/
unsigned int Phase_Difference(unsigned int Former_Phase, unsigned int Current_Phase)
{
	unsigned int dt;

	dt=	Current_Phase-Former_Phase;
	if (dt>0x800000)//负数
	{
		dt=	Former_Phase-Current_Phase;
	}
	if (dt>Period)//丢失部分同步信号
	{
		dt%=Period;
	}
	return dt;
}

/************************************************************************
*Syn_Check                                                              *
*同步断线检测。                                                         *
************************************************************************/
void Syn_Check(unsigned char* Phase_Flag, int* Phase_Count)
{
 	if (Pulse_Enabled_Flag==0 )
 	{
 		Clear_Syn_Error();
 	}
 	else
 	{
		Sec_Syn_Check(Phase_Flag, Phase_Count);
		Primary_Syn_Check();
		Check_Syn_Error();//设置同步断线总标志 
 	}
}

/************************************************************************
*Cal_Period                                                             *
*计算周期。                                                             *
************************************************************************/
void Cal_Period(unsigned int t, unsigned int* tOld)
{
	unsigned int PeriodTemp[6];	//存储用于中值计算的周期数据的临时变量
	unsigned int Pt;	//交换周期值的中间变量
	int i,j;

	if (--Syn_F_Delay<=0)	//延时开启周期(频率)计算
	{		
		Syn_F_Delay=0;
		PeriodTemp[5]=t-*tOld;
		
		//周期限幅
		if (PeriodTemp[5]<420000)	//200Hz
		{
			PeriodTemp[5]=420000;
		}
		if (PeriodTemp[5]>84000000)	//1Hz
		{
			PeriodTemp[5]=84000000;
		}		
		
		//刷新存储的5个周期值, 加上刚刚得到的周期值进行中值计算
		for (i=0;i<4;i++)
		{
			PeriodTemp[i]=Period_Source[i];
			Period_Source[i]=Period_Source[i+1];
		}
		PeriodTemp[4]=Period_Source[4];
		Period_Source[4]=PeriodTemp[5];//新的周期
		
		for (i=0;i<4;i++)
		{
			for (j=i+1;j<6;j++)
			{
				if (PeriodTemp[i]<PeriodTemp[j])
				{
					Pt=PeriodTemp[i];
					PeriodTemp[i]=PeriodTemp[j];
					PeriodTemp[j]=Pt;
				}
			}
		}
		Period=(PeriodTemp[2]+PeriodTemp[3])>>1;
		F=(float)1680000.0/Period;
		
	}
	*tOld=t;//刷新中断时刻	
}

/************************************************************************
*Pulse_Syn_Low_F_Check                                                  *
*脉冲同步缺失检测。                                                     *
************************************************************************/
void Pulse_Syn_Low_F_Check(void)
{
	int i, n;
	
//	if (Vabc>0.3f)
//	{
//		if (++Pulse_Low_F_Count>100)//5周期延时 
		if (++Pulse_Low_F_Count>60)//3周期延时 
		{
			Pulse_Low_F_Count=0;
			Pulse_Low_F_Flag=1;
			if (Pulse_Enabled_Flag)
			{
				i=(SynSource>>4)-1;
				Scr_Pulse_Flag[i]=1;
				Scr_Pulse_Count[i]=1000;//delay to recover in 1s
				Syn_Switch_Delay=5;//5个周期内不再接受在线控制通道的同步信号源协调 
				n=SynSource>>4;
				i=SynSource & 3;
				//ScrSynError[n+1]=1<<(i-1);
				ScrSynError[n+1]|=1<<(i-1);
				SelectSynSource();
				SelectSyn(SynSource);
			}
		}
		
		for (i=0;i<5;i++)
		{
			if (Scr_Pulse_Flag[i])
			{
				if (--Scr_Pulse_Count[i]<=0)
				{
					Scr_Pulse_Flag[i]=0;
					Scr_Pulse_Count[i]=0;
				}
			}
		}	
//	}
//	else
//	{
//		Pulse_Low_F_Count=0;
//	}
}

/************************************************************************
*Pulse_Syn_Error_Check                                                  *
*脉冲同步偏移检测, 每个周期执行一次。                                   *
************************************************************************/
void Pulse_Syn_Error_Check(void)
{
	int i, n;
	unsigned int Local_Syn_Int_Time, dt, T_30, T_330;
	
	n=SynSource>>4;
	if (Pulse_Enabled_Flag && (Syn_Error_Flag==0) && (n!=uchNodeNum))// && Vabc>0.3f)
	{		
		i=SynSource & 3;
		Local_Syn_Int_Time=Syn_A_Int_Time;
		if (i==2)
		{
			Local_Syn_Int_Time=Syn_B_Int_Time;
		}
		else if (i==3)
		{
			Local_Syn_Int_Time=Syn_C_Int_Time;
		}
		dt=Syn_Int_Time-Local_Syn_Int_Time;
		T_30=Period/12;
		T_330=Period-T_30;
		
		if (dt>T_30 && dt<T_330)
		{
			if (++Syn_Pulse_Count>=Syn_Pulse_Delay)
			{
				Syn_Pulse_Count=0;
				Syn_Switch_Delay=5;
				ScrSynError[n+1]=1<<(i-1);//ScrSynError[0]-控制通道1, 1-控制通道2, 2~nScrNum+2为功率柜 
				SelectSynSource();
				SelectSyn(SynSource);
			}
		}
		else
		{
			if (--Syn_Pulse_Count<=0)
			{
				Syn_Pulse_Count=0;
			}
		}
	}
	else
	{
		Syn_Pulse_Count=0;
	}	
}
//===========================================================================
// End of file.
//===========================================================================
