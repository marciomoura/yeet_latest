/********************************************************************
*eIfd.c                                                             *
*均流控制，控制频率约10Hz，比较目标与当前值后修改均流角，限幅3°。   *
********************************************************************/
#include <stm32f4xx.h>
#include <math.h>
#include "pub.h"

#define PID_h (float)0.005
//#define PID_h (float)0.0005
#define pi (float)3.14159265
#define Vanode_V0_C 9

//#define Ref_Step (float)0.000007
#define Ref_Step_Limit (float)0.05
#define Ref_Max_C (float)1.1
#define Ref_Min_C (float)0.1
#define Stop_Machine_Ifd (float)0.02

extern unsigned short int Scr_Comm_OK_Num[8];//与各个控制模块通信正常的功率柜数量 

float Vamax;		//正向输出限幅
float Vamin;		//反向输出限幅
int PID_Cycle_C;	//PID计算延时

short int eIfd_C=0;

/********************************************************************
*Init_PID                                                           *
*PID参数初始化.                                                     *
********************************************************************/
void Init_PID(void)
{
	Vamax=Vanode_V0_C*cos(Alpha_Ref_Min/180.0*pi);
	Vamin=Vanode_V0_C*cos(Alpha_Ref_Max/180.0*pi);
	AVR_Flag=1;
	FCR_Flag=0;
	AER_Flag=0;
	Van_Vf0=Vanode_V0_C;
	
	Ref=1;
	Ref_Ctrl=1;
	Ifd_Sum=0;
	Ifd_av=0;
	Vav=0;
	
	//Stop
	Stop_Command=0;
	Stop_Command_Old=0;
	Stop_Machine_Flag=0;
	Stop_Delay_Count=0;
	Stop_Count=0;
	PID_Cycle_C=0;
}

/********************************************************************
*PID                                                                *
*PID algorithdm: K*(1+T1s)/(1+T2s)*(1+T3s)/(1+T4s).                 *
*串联PID算法：K×(1+T1s)/(1+T2s)×(1+T3s)/(1+T4s)。                   *
********************************************************************/
float PID(float dRef, float Kr, float* T, float* x)
{
	//(1+T1s)/(1+T2s)
	float k=(float)((*(T+1)-PID_h)/(*(T+1)+PID_h));
	float fMax=Vamax/Kr*(*(T+3))/(*(T+2));
	float fMin=Vamin/Kr*(*(T+3))/(*(T+2));
	float y[2];
	
	//消除静差
	dRef+=1.5f/Kr;
	
	//使用公共状态变量作为控制起点
	*x=PID_X[0];
	*(x+1)=PID_X[1];
	
	*x=*x*k+(1-k)*dRef;
	//内限幅
	if (*x>fMax)
	{
		*x=fMax;
	}
	if (*x<fMin)
	{
		*x=fMin;
	}
	k=*T/ *(T+1);
	y[0]=*x*(1-k)+k*dRef;
	
	//(1+T3s)/(1+T4s)
	fMax=Vamax/Kr;
	fMin=Vamin/Kr;
	k=(float)((*(T+3)-PID_h)/(*(T+3)+PID_h));
	*(x+1)=*(x+1)*k+(1-k)*(*y);
	//内限幅
	if (*(x+1)>fMax)
	{
		*(x+1)=fMax;
	}
	if (*(x+1)<fMin)
	{
		*(x+1)=fMin;
	}
	k=*(T+2)/(*(T+3));
	y[1]=*(x+1)*(1-k)+k*y[0];
	return y[1]*Kr;
}

/********************************************************************
*AVR                                                                *
*以阳极电压为控制对象的PID控制。                                    *
********************************************************************/
void AVR(void)
{
	float dRef;//,ft;
	float U;
	float Alpha_U;
	float X[2];
	
	if (++PID_Cycle_C>9)
	{
		PID_Cycle_C=0;
//		if (AVR_Flag)
//		{
			dRef=Ref_Ctrl-Vabc;
//			if (Over_Ex_Flag)
//			{
//				ft=(float)0.5*(Over_Ex_Ifd_Ref-Ifd_Sum);
//				if (ft<dRef)
//				{
//					dRef=ft;
//				}
//			}
//		}
//		else if (FCR_Flag)
//		{
//			dRef=(float)0.5*(Ref-Ifd_Sum);
//		}
//		else//AER
//		{
//			dRef=Ref_Ctrl-Vfd;
//		}
		
		U=PID(dRef, PID_K, PID_T, X);
		
		if (Online_Flag)
		{
			PID_X[0]=X[0];
			PID_X[1]=X[1];
			
			U/=(float)Vanode_V0_C;
			if (U>1)
			{
				U=1;
			}
			if (U<-1)
			{
				U=-1;
			}
			
			Alpha_U=(float)(acos(U)/pi*180);
		
			if (Alpha_U>Alpha_Ref_Max)
			{
				Alpha_U=Alpha_Ref_Max;
			}
			if (Alpha_U<Alpha_Ref_Min)
			{
				Alpha_U=Alpha_Ref_Min;
			}
			
			if (Stop_Machine_Flag)
			{
				Alpha_E=Alpha_Ref_Max;
			}
			else
			{
				Alpha_E=Alpha_U;
			}
		}
	}
}

//===========================================================================
// End of file.
//===========================================================================
