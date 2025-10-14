/********************************************************************
*AI                                                                 *
*处理AD数据，计算V、I、Vfd、Ifd。                                   *
********************************************************************/
//#include <stm32f4xx.h>
//#include <stdint.h>
#include <math.h>

#include "pub.h"
#include "AD7606.h"
#include "w5200.h"

#define AD_Data_Num 300

int AI_Times;//采样记次
int ADC_Data[8][AD_Data_Num];
int V[4][AD_Data_Num];//三相电压的AD数据
int I[4][AD_Data_Num];//三相电流的AD数据
int AI_Count;
unsigned short int nI_Buf[3][2];

/********************************************************************
*Init_Avg                                                           *
*初始化积分数据。                                                   *
********************************************************************/
void Init_Avg(void)
{
	Vav=0.0;
	Iav=0.0;
	Ifd_av=0.0;
	Fav=1.0;
}

/********************************************************************
*Cal_Avg                                                            *
*integral values for control algorithms                             *
*状态量积分计算，作为运行方式切换的给定值。                         *
********************************************************************/
void Cal_Avg(void)
{
	Vav+=((Vabc-Vav) * Avg_Coef);
	Iav+=((Iabc-Iav) * Avg_Coef);
	Fav+=((F-Fav) * Avg_Ctrl_Coef);	
	//Period_av+=((Period-Period_av) * Avg_Ctrl_Coef);
	Period_av=Period;
	Ifd_av+=((Ifd-Ifd_av) * Avg_Coef);	
	Ifd_Sum_av+=(Ifd_Sum-Ifd_Sum_av)*Avg_Coef;//
	
}

/********************************************************************
*Init_AD_Data                                                       *
*初始化AD数据区。                                                   *
********************************************************************/
void Init_AD_Data(void)
{
	int i,j;
	unsigned short int* nPt;
	
	Init_Avg();

	for (i=0;i<AD_Data_Num;i++)
	{
		V[0][i]=0;
		V[1][i]=0;
		V[2][i]=0;
		V[3][i]=0;
		I[0][i]=0;
		I[1][i]=0;
		I[2][i]=0;
		I[3][i]=0;
	}
	for (i=0;i<8;i++)
	{
		for (j=0;j<AD_Data_Num;j++)
		{
			ADC_Data[i][j]=0;
		}
	}
	AI_Count=0;
	AI_Times=0;

	Va=0.0;
	Ia=0.0;
	
	Vb=0.0;
	Ib=0.0;
	
	Vc=0.0;
	Ic=0.0;
	
	Vabc=0.0;
	Iabc=0.0;

	Vfd=0.0;
	Ifd=0.0;

	Iap=0.0;
	Ian=0.0;
	Ibp=0.0;
	Ibn=0.0;
	Icp=0.0;
	Icn=0.0;
	for (i=0;i<6;i++)
	{
		Iarm[i]=0;
	}
	
	nPt=nI_Buf[0];
	for (i=0;i<6;i++)
	{
		*nPt++=0;
	}
	
//	Tap=20.0;
//	Tan=20.0;
//	Tbp=20.0;
//	Tbn=20.0;
//	Tcp=20.0;
//	Tcn=20.0;
//	Tscr=20.0;
//	Tin=20;
//	Tout=20;

	Iabc_Temp[0]=0;
	Iabc_Temp[1]=0;
	Vabc_Temp[0]=0;
	Vabc_Temp[1]=0;
	Ifd_Temp[0] =0;
	Ifd_Temp[1] =0;
	Vfd_Temp[0] =0;
	Vfd_Temp[1] =0;		
	Iout=0;
}

/********************************************************************
*Zero_Calibrate                                                     *
*静态计算AI零飘。                                                   *
********************************************************************/
void Zero_Calibrate(void)
{
	int i,j,nSum;
	
	if (Pulse_Enabled_Flag==0)
	{
		for (i=0;i<8;i++)
		{
			nSum=0;
			for (j=0;j<AD_Data_Num;j++)
			{
				nSum+=ADC_Data[i][j];
			}
			AI_Offset[i]-=nSum/AD_Data_Num;
		}
	}
}
/********************************************************************
*Get_Medium_n                                                       *
*取三个整数的中间值。                                               *
********************************************************************/
unsigned short int Get_Medium_n(unsigned short int a, unsigned short int* nPt)
{
	unsigned short int b, c, r;
	
	b=*nPt;
	c=*(nPt+1);
	
	if (a>b )
	{
		if (b>c)
		{
			r=b;
		}
		else if (a>c)
		{
			r=c;
		}
		else
		{
			r=a;
		}
	}
	else if (b>c)
	{
		if (c>a)
		{
			r=c;
		}
		else
		{
			r=a;
		}
	}
	else
	{
		r=b;
	}
	
	*nPt=c;
	*(nPt+1)=a;
	
	return r;
}

/********************************************************************
*Get_Medium_Value                                                   *
*取三个数的中间值。                                                 *
********************************************************************/
float Get_Medium_Value(float a, float b, float c)
{
	if (a>b )
	{
		if (b>c)
		{
			return b;
		}
		else if (a>c)
		{
			return c;
		}
		else
		{
			return a;
		}
	}
	else if (b>c)
	{
		if (c>a)
		{
			return c;
		}
		else
		{
			return a;
		}
	}
	else
	{
		return b;
	}
}

/********************************************************************
*RMS(root mean square)                                              *
*均方根值计算。                                                     *
********************************************************************/
float RMS(int* Data, float Kc, int Num)
{
	short int i;
	int n;
	float Sum=0;

	for (i=0;i<Num;i++)
	{
		n=*Data++;
		Sum+=n*n;
	}
	Sum/=Num;
	Sum=sqrt(Sum);	//均方根值
	return Sum*Kc;		//标幺化数据
}

/********************************************************************
*Shift_AD_Data                                                      *
*将AD数据移动到另一数据区供主循环计算，减轻中断服务程序负担。       *
********************************************************************/
void Shift_AD_Data(void)
{
	int i, j;
	int* nPtD, *nPtS;
	
	nPtS=ADC_Data[0];
	nPtD=V[0];
	j=4*AD_Data_Num;
	
	for (i=0;i<j;i++)
	{
		*nPtD++=*nPtS++;
	}
	
	nPtD=I[0];
	
	for (i=0;i<j;i++)
	{
		*nPtD++=*nPtS++;
	}
	
}

/********************************************************************
*Get_Monitor                                                        *
*在线监测数据存储。                                                 *
********************************************************************/
void Get_Monitor(unsigned short int* unData)
{
//	RTC_TimeTypeDef t;//时刻 
//	RTC_DateTypeDef date;//日期
	unsigned char* ucD, *ucS;
	int i;
	
	if (Net_Monitor_C==0)
	{//数据头、温度和起始时刻
		ucD=ucMornitor[1-Net_Monitor_Index];
		
		//MBAP
		*ucD++=0;//事务处理标志 
		*ucD++=0;
		*ucD++=0;//协议
		*ucD++=0;
		*ucD++=0x1f;//长度
		*ucD++=0xfa;
		*ucD++=0;//单元标识 
		*ucD++=3;//读响应 
		*ucD++=0x1f;//数据长度 
		*ucD++=0xf6;
		
		//起始时刻
//		RTC_GetTime(RTC_Format_BIN, &t);
//		RTC_GetDate(RTC_Format_BIN, &date);	
//		*ucD++=date.RTC_Year;
//		*ucD++=date.RTC_Month;
//		*ucD++=date.RTC_Date;
//		*ucD++=t.RTC_Hours;
//		*ucD++=t.RTC_Minutes;
//		*ucD++=t.RTC_Seconds;
//		*ucD++=ms_C>>8;
//		*ucD++=ms_C & 0xff;
		i=t_Op[0];
		i<<=8;
		i|=t_Op[1];
//		i-=2000;
		*ucD++=i;
		*ucD++=t_Op[2];
		*ucD++=t_Op[3];
		*ucD++=t_Op[4];
		*ucD++=t_Op[5];
		*ucD++=t_Op[6];
		*ucD++=t_Op[7];
		*ucD++=t_Op[8];		
		
		//温度 
		ucD=ucMornitor[1-Net_Monitor_Index]+8047;
		i=Tarm[0]*100;
		*ucD++=i>>8;
		*ucD++=i & 0xff;
		i=Tarm[1]*100;
		*ucD++=i>>8;
		*ucD++=i & 0xff;
		i=Tarm[2]*100;
		*ucD++=i>>8;
		*ucD++=i & 0xff;
		i=Tarm[3]*100;
		*ucD++=i>>8;
		*ucD++=i & 0xff;
		i=Tarm[4]*100;
		*ucD++=i>>8;
		*ucD++=i & 0xff;
		i=Tarm[5]*100;
		*ucD++=i>>8;
		*ucD++=i & 0xff;
	}
	
	ucS=(unsigned char*)unData;
	ucD=ucMornitor[1-Net_Monitor_Index]+18+Net_Monitor_C*2;
	
	
	for (i=0;i<8;i++)
	{
		*ucD++=*ucS++;
		*ucD++=*ucS++;
		ucD+=998;
	}
	if ((Net_Monitor_C%100)==0)//20ms
	{
		i=(int)(Alpha*100);
		ucD=ucMornitor[1-Net_Monitor_Index]+8018+(int)(Net_Monitor_C*0.02f);
		*ucD++=i>>8;
		*ucD++=i & 0xff;
		
		ucD+=8;
		*ucD++=DI_Data[0];
		*ucD++=DI_Data[1];
		
		ucD+=8;
		*ucD++=DO_Data[0];
		*ucD++=DO_Data[1];
	}
	
	if (++Net_Monitor_C>499)//100ms
	{
		Net_Monitor_C=0;
		if (Net_Monitor_Index)
		{
			Net_Monitor_Index=0;
		}
		else
		{
			Net_Monitor_Index=1;
		}
		
		if (Monitor_Upload_Flag)
		{
			Monitor_Upload_Flag=0;
			Net_Tx_Flag[1][0]=1;
		}
	}
}

/********************************************************************
*AD                                                                 *
*获取、处理AD结果。                                                 *
********************************************************************/
void AD(void)
{
	int i;
	unsigned short int AD_Data[8];
	
	Read_Start_AD7606(AD_Data);
	
	AD_Data[4]=Get_Medium_n(AD_Data[4], nI_Buf[0]);
	AD_Data[5]=Get_Medium_n(AD_Data[5], nI_Buf[1]);
	AD_Data[6]=Get_Medium_n(AD_Data[6], nI_Buf[2]);
	
	//8个AD输入依次为Va, Vb, Vc, Vfd, Ia, Ib, Ic, Ifd
	for (i=0;i<8;i++)
	{
		ADC_Data[i][AI_Times]=AD_Data[i]+AI_Offset[i];
		if (AD_Data[i] & 0x8000)//负数
		{
			ADC_Data[i][AI_Times]-=0x10000;
		} 
	}
	
	
	if (++AI_Times>=AD_Data_Num)//AD data in 60ms
	{
		AI_Times=0;
	}
	
//	if ((AI_Times % 60)==0)//10ms
//	{
//		AI_Flag=1;
//		Shift_AD_Data();
//	}
	
	Get_Monitor(AD_Data);
}

/********************************************************************
*Cal_Arm_I                                                          *
*计算桥臂电流，根据当前脉冲判断导通桥臂。                           *
********************************************************************/
void Cal_Arm_I(int* Data, float* fIp, float* fIn, float* fI, float AD_Coef)
{
	float fT=I_AD_Const*(float)0.6667;
	int i,j;
	int nP=0, nN=0;
	float fP=0, fN=0;
	
	for (i=0;i<AD_Data_Num;i++)
	{
		j=*Data++;
		if (j>0)
		{
			nP++;
			fP+=j*j;
		}
		else
		{
			nN++;
			fN+=j*j;
		}
	}
	
	if (nP>0)
	{
		*fIp=sqrt(fP/nP)*fT*Ifd_Const*AD_Coef;
	}
	else
	{
		*fIp=0;
	}
	
	if (nN>0)
	{
		*fIn=sqrt(fN/nN)*fT*Ifd_Const*AD_Coef;
	}
	else
	{
		*fIn=0;
	}
	*fI=sqrt((fN+fP)/AD_Data_Num)*fT*Ifd_Const*AD_Coef;	
}

/********************************************************************
*Cal_AD                                                             *
*处理AD结果.                                                        *
********************************************************************/
void Cal_AD(void)
{
	float fTemp;
	float k;
	
	AI_Flag=0;
	Cal_Arm_I(ADC_Data[4], &Iap, &Ian, &Ia, I_Const[0]);	
	Cal_Arm_I(ADC_Data[5], &Ibp, &Ibn, &Ib, I_Const[1]);	
	Cal_Arm_I(ADC_Data[6], &Icp, &Icn, &Ic, I_Const[2]);
	
//	if (Pulse_Enabled_Flag==0)//2020.5.7.
//	{
//		Iap=0;
//		Ian=0;
//		Ibp=0;
//		Ibn=0;
//		Icp=0;
//		Icn=0;
//	}
	
	Va=RMS(ADC_Data[0],V_AD_Const,AD_Data_Num)*V_Const;
	Vb=RMS(ADC_Data[1],V_AD_Const,AD_Data_Num)*V_Const;
	Vc=RMS(ADC_Data[2],V_AD_Const,AD_Data_Num)*V_Const;
	fTemp=(Va+Vb+Vc)/3.0f;
	Vabc=Get_Medium_Value(fTemp, Vabc_Temp[0],Vabc_Temp[1]);
	
	//test code
//	Vabc=Get_Medium_Value(fTemp, Vabc_Temp[0],Vabc_Temp[1])*15;
	
	Vabc_Temp[1]=Vabc_Temp[0];
	Vabc_Temp[0]=fTemp;
	
	fTemp=RMS(ADC_Data[3],Vfd_AD_Const,AD_Data_Num)*Vfd_Const;
	Vfd=Get_Medium_Value(fTemp, Vfd_Temp[0],Vfd_Temp[1]);
	Vfd_Temp[1]=Vfd_Temp[0];
	Vfd_Temp[0]=fTemp;
	
	//中值滤波 
	fTemp=(Ia+Ib+Ic)/3;
	Iabc=Get_Medium_Value(fTemp, Iabc_Temp[0],Iabc_Temp[1]);
	Iabc_Temp[1]=Iabc_Temp[0];
	Iabc_Temp[0]=fTemp;
	Iout=fTemp;
	
	//1/(1+Ts)
	if (T_Ifd>(float)0.001)//时间常数下限 
	{
		k=(T_Ifd-0.0005f)/(T_Ifd+0.0005f);
		Ifd=Ifd*k+(1-k)*Iabc;
		Iarm[0]=Iarm[0]*k+(1-k)*Iap;
		Iarm[1]=Iarm[1]*k+(1-k)*Ian;
		Iarm[2]=Iarm[2]*k+(1-k)*Ibp;
		Iarm[3]=Iarm[3]*k+(1-k)*Ibn;
		Iarm[4]=Iarm[4]*k+(1-k)*Icp;
		Iarm[5]=Iarm[5]*k+(1-k)*Icn;
	}
	else
	{
		Ifd=Iabc;
		Iarm[0]=Iap;
		Iarm[1]=Ian;
		Iarm[2]=Ibp;
		Iarm[3]=Ibn;
		Iarm[4]=Icp;
		Iarm[5]=Icn;
	}
}

//===========================================================================
// End of file.
//===========================================================================
