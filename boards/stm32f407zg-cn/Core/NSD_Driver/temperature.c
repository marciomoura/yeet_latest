#include <stm32f4xx.h>
#include "USARTx.h"
#include "pub.h"
#include "AI.h"
#include "optical.h"

#define dT 5

int Request_Delay;	//温度刷新的请求间隔延时
unsigned char uchT[36];//温度数据区, 供外部读取
unsigned char T_Tx_Flag;	//串口发送标志
unsigned char uchData[90];
unsigned char Init_T_Flag;
int Init_T_Delay;
float fT[15][10];//温度中值滤波的旧数据区 

/********************************************************************
*T_Filter                                                           *
*温度数据滤波。                                                     *
********************************************************************/
void T_Filter(float T_Data, float* Tout, float* T_buf)
{
	int i, n;
	float* fD, *fS;
	float fSum, fMax, fMin;
	
	//refresh buffer with latest data
	fD=T_buf;
	fS=fD+1;
	for (i=0;i<9;i++)
	{
		*fD++=*fS++;
	}
	*fD=T_Data;
	
	fD=T_buf;
	fSum=*Tout;
	fMax=fSum+dT;
	fMin=fSum-dT;
	
	//whether there's data in the range of [T-5, T+5]
	for (i=0;i<10;i++)
	{
		if (*fD<fMax && *fD>fMin)
		{
			break;
		}
		fD++;
	}
	
	if (i==10)//all data out of range
	{		
		fD=T_buf;
		fSum=*fD;
		fMax=*fD;
		fMin=*fD++;
		for (i=1;i<10;i++)
		{
			fSum+=*fD;
			if (fMax<*fD)
			{
				fMax=*fD;
			}
			if (fMin>*fD)
			{
				fMin=*fD;
			}			
		}
		fSum-=fMax;
		fSum-=fMin;
		*Tout=fSum/8;
	}
	else//there are valid data in buffer
	{
		n=0;
		fSum=0;
		fD=T_buf;
		for (i=0;i<10;i++)
		{
			if (*fD<fMax && *fD>fMin)
			{
				fSum+=*fD;
				n++;
			}
			fD++;
		}
		*Tout=fSum/n;
	}
}

/********************************************************************
*Set_Sensor                                                         *
*设置温度传感器间隔时间并启动上传温度。                             *
********************************************************************/
void Set_Sensor(void)
{
//	unsigned char start[11] = {0xbb, 0xf6, 0x05, 0x01, 0x01, 0, 0, 0, 0xfd,0xd,0xa};//start uploading T
//	unsigned char stop[11]  = {0xbb, 0xf6, 0x05, 0x01, 0x00, 0, 0, 0, 0xfc,0xd,0xa};
//	unsigned char interval[16] = {0xbb, 0x1f, 0x0a, 7, 0xd0, 7, 0xd0, 1, 0x2c, 1, 0x2c, 0, 0, 0x31,0xd, 0xa};//set work time for antennas: 1-2000ms, 2-2000ms, 3-30ms, 4-30ms
//	unsigned char antenna[7] = {0xbb, 8, 1, 3, 0xc, 0xd, 0xa};//open antenna 1 and antenna 2
//	unsigned char antena_power[15] = {0xbb, 0x8c, 9, 0, 0x1d, 0x1d, 0x1d, 0x1d, 5, 5, 5, 5, 0x91, 0xd, 0xa};//antenna power: 1-29db, 2-29db, 3-5db, 4-5db
//	
//	if (Init_T_Flag)
//	{
//		Init_T_Delay++;
//		
//		switch (Init_T_Delay)
//		{
//			case 1:
//				T_Tx_Flag=1;	
////				Init_RS485(115200, 11, 21);	
//				Init_RS485(19200, 11, 21);	
//				RS485_Tx(&T_Tx_Flag, stop);//stop uploading T as soon as USART being initialized
//				break;
//			case 2001:	
//				T_Tx_Flag=1;
////				Init_RS485(115200, 16, 21);	
//				Init_RS485(19200, 16, 21);	
//				RS485_Tx(&T_Tx_Flag, interval);//set interval for ant1
//				break;
//			case 4001:	
//				T_Tx_Flag=1;
////				Init_RS485(115200, 15, 21);	
//				Init_RS485(19200, 15, 21);	
//				RS485_Tx(&T_Tx_Flag, antena_power);//set interval for ant1
//				break;
//			case 6001:	
////				T_Tx_Flag=1;
////				Init_RS485(115200, 7, 21);	
//				Init_RS485(19200, 7, 21);	
//				RS485_Tx(&T_Tx_Flag, antenna);//set interval for ant1
//				break;
//			case 8001:
//				T_Tx_Flag=1;
////				Init_RS485(115200, 11, 21);	
//				Init_RS485(19200, 11, 21);	
//				RS485_Tx(&T_Tx_Flag, start);//start uploading T as soon as USART being initialized
//				Init_T_Delay=0;
//				Init_T_Flag=0;
//				Request_Delay=0;	
//				break;
//		}
//	}
	
}
	
/********************************************************************
*Init_T                                                             *
*初始化串口2及对应数据区。                                          *
********************************************************************/
void Init_T(void)
{
	int i;
	
	for (i=0; i<36; i++)
	{
		uchT[i]=0;
	}
	for (i=0; i<90; i++)
	{
		uchData[i]=0;
	}
	for (i=0;i<15;i++)
	{
		fT[i][0]=20;
		fT[i][1]=20;
	}
	for (i=0;i<6;i++)
	{
		Tarm[i]=20;
	}
	Init_T_Flag=1;
	Init_T_Delay=0;	
	Init_RS485(19200, 11, 23);	
}

/********************************************************************
*ID_To_No                                                           *
*根据ID返回序号。                                                   *
********************************************************************/
int ID_To_No(unsigned char* uchTData)
{
	int i;
	
	for (i=0;i<15;i++)
	{
		if ((Sensor_ID[i][2]==*uchTData) && (Sensor_ID[i][3]==*(uchTData+1)) ) 
		{
			break;
		}
	}
	
	return i;	
}

/********************************************************************
*Data_To_T                                                          *
*温度数据解析。                                                     *
********************************************************************/
float Data_To_T(unsigned char* uchTData)
{
	int Temp, P1, P2;
//	float C;
	float T;	
	
	P1=*(uchTData+5);
	P1<<=8;
	P1|=*(uchTData+6);
	
	P2=*(uchTData+7);
	P2<<=8;
	P2|=*(uchTData+8);
	if (P2 & 0x8000)
	{
		P2-=65536;
	}
	
	Temp=*(uchTData+9);
	Temp<<=8;
	Temp|=*(uchTData+10);
	Temp<<=8;
	Temp|=*(uchTData+11);
	Temp<<=8;
	Temp|=*(uchTData+12);
	
//	C=Temp;
//	C/=100;
//	C-=101;
//	
//	T=11984.47f/(21.25f+P1+(P2/2752.0f)+C)-301.57f;
	Temp&=0xffff;
	if (Temp & 0x8000)
	{
		Temp-=65536;
	}
	T=11109.6f/(24+(P2+Temp)/375.3f)-290;//LTU32
	if (T>125)
	{
		T=T*1.2f-25;
	}
	return T;
}

/********************************************************************
*Get_T                                                              *
*读取所有温度数据。                                                 *
********************************************************************/
void Get_T(void)
{
	unsigned short int i, j;
	float f;
	
	if (RS485_Rx(uchData+22))//收到数据
	{
		//搜索数据头
		for (i=0;i<23;i++)
		{
			if ((uchData[i]==0xbb) && (uchData[i+1]==0xf8) &&
				(uchData[i+21]==0xd) && (uchData[i+22]==0xa))
			{
				j=ID_To_No(uchData+i+15);
				if (j<15)// num of sensors is less than 16
				{
					T_c[j]=0;
					T_Flag[j]=0;
					f=Data_To_T(uchData+i);
					
//					Tarm[j]=Get_Medium_Value(f, fT[j][0], fT[j][1]);					
//					fT[j][0]=fT[j][1];
//					fT[j][1]=f;
					T_Filter(f, Tarm+j, fT[j]);
				}
				break;
			}
		}
		for (i=0;i<22;i++)
		{
			uchData[i]=uchData[i+23];
		}
	}
	
	//delay to warn
	j=0;
	for (i=0;i<15;i++)
	{
		if (++T_c[i]>=300000)//T must be refreshed in 300s
		{
			if ((Sensor_ID[i][0]==0) && (Sensor_ID[i][1]==0) && (Sensor_ID[i][2]==0) && (Sensor_ID[i][3]==0) )
			{
				T_c[i]=0;//not valid ID				
			}
			else
			{
				Tarm[i]=255;
				j=1;
			}
		}
	}
	
	if (j)//reinitialize sensors when losing touch with 1 sensor
	{
		for (i=0;i<15;i++)
		{
			T_c[i]=0;
			T_Flag[i]=1;			
		}
		Init_T_Flag=1;
		Init_T_Delay=0;	
	}
}


/********************************************************************
*Requst_T                                                           *
*读取所有温度数据。                                                 *
********************************************************************/
void Requst_T(void)
{
	Get_T();
	Set_Sensor();	
}
//===========================================================================
// End of file.
//===========================================================================

