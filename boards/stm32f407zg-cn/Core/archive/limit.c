/********************************************************************
*limit.c                                                            *
*                                                                   *
*限制保护流程.                                                      *
*Copyright: 2002-2018 Nengshida Electric Ltd.                       *
*Author: Yu Xiang                                                   *
*Jan. 15, 2018                                                      *
********************************************************************/
#include "pub.h"
#include "mode.h"
#include "DIDO.h"
#include "syn.h"
#include<math.h>

//1.095f means 1.095 is a float
#define Scr_Exit_Ifd_1 2
#define Scr_Exit_Ifd_2 1.095f
#define Scr_Exit_Ifd_3 0.65f

#define Over_Ex_n 2
#define Over_Ex_k 34.5f
#define Over_Ex_Ifd_Hold 1.1f

extern unsigned short int Scr_Comm_OK_Num[8];//与各个控制模块通信正常的功率柜数量 

/********************************************************************
*PT check                                                           *
*Check if Pt's wire is broken                                       *
*Vmax=max(Va,Vb,Vc)                                                 *
*if |Vmax-Vmin|>Vmax/4, PT wire broken                              *
*只检查本通道使用的PT状态。                                         *
********************************************************************/
void PT_Check(void)
{
	float maxV, minV;
	float MaxErr;
	
	if (Vav<(float)0.3)
	{
		return;	//防止两相断线三相电压归零时误清除PT断线
	}

	if (Stop_Machine_Flag || Pulse_Enabled_Flag==0)
	{
		PT_Break_Flag=0;
		PT_Break_Count=0;
		return;
	}
	
	maxV=Va;
	minV=Vb;
	if(Va<Vb)
	{
		maxV=Vb;
		minV=Va;
	}
	if(maxV<Vc)
	{
		maxV=Vc;
	}
	if (minV>Vc)
	{
		minV=Vc;
	}
	MaxErr=maxV*(float)0.25;
	
	if(maxV-minV>MaxErr) //Single phase wire broken		
	{
		PT_Break_Count++;
		if(PT_Break_Count>3)
		{
			if (PT_Break_Flag==0)
			{
				PT_Break_Flag=1;				
			}
			
			if (++PT_Online_Delay>=4 && Online_Flag)
			{
				PT_Online_Delay=4;
				PT_Switch_Flag=1;
			}
			
			PT_Break_Count=500;
			if (FCR_Flag==0)
			{
				Work_Mode_Ifd(Ifd_av);
			}
		}
	}
	else
	{
		PT_Break_Count--;
		if (PT_Break_Count<=0)
		{
			PT_Break_Count=0;
			PT_Online_Delay=0;
			PT_Switch_Flag=0;
			if (PT_Break_Flag)
			{
				PT_Break_Flag=0;				
				Work_Mode_V(Vabc);//2015.1.8., 5s不足以让Vav≈Vabc
			}
		}
	}
}

/********************************************************************
*Over_Ex_Check                                                      *
*带反时限延时的强励限制.                                            *
********************************************************************/
void Over_Ex_Check(void)
{
	float t, dH,fRef;
	
	//t=Over_Ex_k/(pow(Ifd/Over_Ex_Ifd_Hold,Over_Ex_n)-1);
	//dH=(float)0.01/t;//累加的热量
	if (Ifd_Sum>Over_Ex_Ifd_Hold)//发热
	{
		dH=(pow(Ifd_Sum,Over_Ex_n)-1)/Over_Ex_k*(float)0.01;
	}
	else//散热
	{
		dH=(pow(Ifd_Sum/Over_Ex_Ifd_Hold,Over_Ex_n)-1)/Over_Ex_k*(float)0.01;
	}
	//dH=(pow(Ifd/Over_Ex_Ifd_Hold,Over_Ex_n)-1)/Over_Ex_k*(float)0.01;
	if (dH>1)
	{
		dH=1;
	}
	if (dH<-1)
	{
		dH=-1;
	}
	Over_Ex_Count+=dH;//每秒100次累加
	
	if (Over_Ex_Count>1)
	{
		Over_Ex_Count=1;
	}
	if (Over_Ex_Count<0)
	{
		Over_Ex_Count=0;
	}
	
	if (Over_Ex_Flag)
	{
		t=(Over_Ex_Count-(float)0.01)*1000000;
		Over_Ex_Flag=t>=1;
	}
	else
	{
		t=(Over_Ex_Count-(float)0.99)*1000000;
		if (t>=1)
		{
			Over_Ex_Flag=1;
		}
	}
	
	if (Over_Ex_Flag)
	{
		fRef=Over_Ex_Ifd_Hold-1/PID_K;	/*Ref=1.098*/						
	}
	else
	{
		fRef=Ifd_Max_Limit-1/PID_K;
	}
	
	Over_Ex_Ifd_Ref=Over_Ex_Ifd_Ref*(float)0.904762+fRef*(float)0.095238;// 1/(1+0.1s)
}

/********************************************************************
*Scr Exit                                                           *
*number of scr exited     Ifmax                                     *
*	0                       2.250                                     *
*	1                       2.000                                     *
*	2                       1.095                                     *
*	3                       0.650                                     *
*	4                       0.650                                     *
*	5                       0.650                                     *
********************************************************************/
void Scr_Exit_Check(void)
{
	float Ifd_Ref;
	int Num=0;	
	
	Num=nScrNum-Scr_Comm_OK_Num[uchNodeNum+1];
	
	if (Num)
	{
		Scr_Exit_Flag=1;
	}
	else
	{
		Scr_Exit_Flag=0;
	}
	
	if (((nScrNum==2) && Num>0)
	|| 	Num>1)	
	{
		if (nScrNum==2)
		{
			Ifd_Max_Limit=Scr_Exit_Ifd_2;
			Load_Limit_Flag=1;
		}
		else
		{
			switch (Num)
			{
				case 2:
					Ifd_Max_Limit=Scr_Exit_Ifd_2;
					break;
				case 3:
					Ifd_Max_Limit=Scr_Exit_Ifd_3;
					break;
				case 4:
					Ifd_Max_Limit=Scr_Exit_Ifd_3;
					break;
				case 5:
					Ifd_Max_Limit=Scr_Exit_Ifd_3;
					break;
			}
		}
		Scr_Exit_Delay=100;
		
		//Ref_Ctrl=min{Ifav,Ifd_Max_Limit}	(Not If Mode)
		//Ref_Ctrl=min{Ref,Ifd_Max_Limit}		(If Mode)		
		if (FCR_Flag)
		{
			Ifd_Ref=Ref_Ctrl;						
		}
		else
		{
			Ifd_Ref=Ifd_av;
		}

		if (Ifd_Ref>Ifd_Max_Limit)
		{
			Ifd_Ref=Ifd_Max_Limit;
			if (FCR_Flag)
			{
				Ref=Ifd_Max_Limit;
				Ref_Ctrl=Ifd_Max_Limit;
			}
		}		
		if ((Load_Limit_Flag==0) || (FCR_Flag==0))//2015.9.2., in case of not Ifd mode after Load limit has been triggered
		{
			Load_Limit_Flag=1;
			Work_Mode_Ifd(Ifd_Ref);	
		}		
	}
	else
	{
		if(Load_Limit_Flag)
		{			
			if(--Scr_Exit_Delay<=0)
			{
				Load_Limit_Flag=0;
				Work_Mode_V(Vav);
			}			
		}
		else
		{
			Ifd_Max_Limit=Scr_Exit_Ifd_1;			
		}		
	}
}

/********************************************************************
*Arm_Fuse check                                                     *
*Check if fuse of arms is broken                                    *
********************************************************************/
void Arm_Fuse_Check(void)
{
	int i;
	
	if (Pulse_Enabled_Flag && (Ifd_av>0.05f))
//	if (Ifd_av>0.05f)//test code
	{
		for (i=0;i<6;i++)
		{
			if (Iarm[i]<0.008f)
			{
				if (++Arm_Fuse_Delay[i]>5000)
				{
					Arm_Fuse_Delay[i]=5000;
					Arm_Fuse_Flag[i]=1;
				}
			}
			else
			{
				if (--Arm_Fuse_Delay[i]<=0)
				{
					Arm_Fuse_Delay[i]=0;
					Arm_Fuse_Flag[i]=0;
				}
			}
		}
	}
	else
	{
		for (i=0;i<6;i++)
		{
			Arm_Fuse_Flag[i]=0;
		}
	}
}

/********************************************************************
*Init_Limit                                                         *
*限制参数初始化.                                                    *
********************************************************************/
void Init_Limit(void)
{
	int i;
	
	//PT
	PT_Break_Flag=0;
	PT_Break_Count=0;
	PT_Online_Delay=0;
	PT_Switch_Flag=0;
	
	//Scr 
	Load_Limit_Flag=0;
	Scr_Exit_Delay=0;
	
	//OverEx
	Ifd_Max_Limit=2;
	Over_Ex_Ifd_Ref=2;
	Over_Ex_Count=0;
	Over_Ex_Flag=0;
	
	for (i=0;i<6;i++)
	{
		Arm_Fuse_Flag[i]=0;
		Arm_Fuse_Delay[i]=0;
	}
}

/********************************************************************
*Limit_Check                                                        *
*execute limit check selectly                                       *
********************************************************************/
void Limit_Check(void)
{	
	if (Pulse_Enabled_Flag && (Stop_Machine_Flag==0) && (Vav>(float)0.3))
	{
		Over_Ex_Check();		
//		Scr_Exit_Check();
//		PT_Check();		
	}
	
	Arm_Fuse_Check();
	Low_F_Check();	
	Pulse_Syn_Low_F_Check();
}



//===========================================================================
// End of file.
//===========================================================================
