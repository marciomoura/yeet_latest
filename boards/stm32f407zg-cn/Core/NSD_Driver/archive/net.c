/********************************************************************************
*Net.c                                                                          *
*网络通信和数据处理。                                                           *
********************************************************************************/

#include <stm32f4xx.h>
#include "w5200.h"
#include "pub.h"
extern void Byte_To_Float(float *data,unsigned char* b);

int Net_Tx_Count;

/********************************************************************
*Init_Net                                                           *
*网口初始化.                                                        *
********************************************************************/
void Init_Net(void)
{
	unsigned char Self_IP[4];
	unsigned char Socket_Enable[8];
	unsigned short int Self_Port[8];
	unsigned char Socket_Mode[8];
	unsigned char Des_IP[32];
	unsigned short int Des_Port[8];
	int i;
	unsigned char* ucP;
	
	Self_IP[0]=192;
	Self_IP[1]=168;
	Self_IP[2]=2;
	Self_IP[3]=100+uchNodeNum;
	
	Socket_Enable[0]=1;
	Socket_Enable[1]=0;
	Socket_Enable[2]=0;
	Socket_Enable[3]=0;
	Socket_Enable[4]=0;
	Socket_Enable[5]=0;
	Socket_Enable[6]=0;
	Socket_Enable[7]=0;
	
	Self_Port[0]=7000;
	Self_Port[1]=7001;
	Self_Port[2]=7002;
	Self_Port[3]=7003;
	Self_Port[4]=7004;
	Self_Port[5]=7005;
	Self_Port[6]=7006;
	Self_Port[7]=7007;
	
	Socket_Mode[0]=1;//1:Server, 0:Client
	Socket_Mode[1]=1;
	Socket_Mode[2]=1;
	Socket_Mode[3]=1;
	Socket_Mode[4]=1;
	Socket_Mode[5]=1;
	Socket_Mode[6]=1;
	Socket_Mode[7]=0;
	
	ucP=Des_IP;
	for (i=0;i<8;i++)
	{
		*ucP++=192;
		*ucP++=168;
		*ucP++=2;
		*ucP++=51+i;
		Des_Port[i]=502;
	}
	
 	Network_Init(SPI2, Self_IP, Socket_Enable, Self_Port, Socket_Mode, Des_IP, Des_Port, 0);//no DMA, SPI2's DMA Rx conflicts with USART3's DMA Rx
	
	Self_IP[0]=192;
	Self_IP[1]=168;
	Self_IP[2]=10;
	Self_IP[3]=31+uchNodeNum;
	
	Network_Init(SPI3, Self_IP, Socket_Enable, Self_Port, Socket_Mode, Des_IP, Des_Port, DMA_SIZE_8K);//8k DMA Rx/Tx 
	
	Net_Monitor_C=0;
	Net_Monitor_Index=0;
	ms_C=0;
	Monitor_Upload_Flag=0;
}


/************************************************************************
*Net_Order  															*
*向功率变送器发送数据刷新请求。             	 						*
************************************************************************/
void Net_Order(int Index)
{
	int i;
	unsigned char Tx[12]={0, 1, 2, 3, 4, 5, 6, 99,  0, 0, 0, 10};
	if (Net_Socket_State[Index][0]==7)//已连接 
	{	
		if(++Net_Tx_Count>39)//refresh Q in 20ms
		{
			Net_Tx_Count=0;
			Net_Tx_size[Index][0]=12;
			
			for (i=0;i<12;i++)
			{
				ucQTx[i]=Tx[i];
			}
			Net_Tx_Flag[Index][0]=1;
		}
	}
	else//Clear Q when losing touch with Power transmitter
	{
//		Q=0;
//		P=0;
	}
}

/************************************************************************
*Net_Check                                                              *
*维护通信连接，通信收发及数据处理。                                     *
************************************************************************/
void Net_Check(void)
{
//	float a, b;
//	int i;
//	unsigned char r[8];
	
	if (Net_Rx_Flag[0][0])
	{
		Net_Rx_Flag[0][0]=0;
		Byte_To_Float(&P,  Net_Rx_Buf[0]+14);
		Byte_To_Float(&Q,  Net_Rx_Buf[0]+18);
//		for (i=0;i<4;i++)
//		{
//			r[i]=Net_Rx_Buf[0][16-i];
//			r[i+4]=Net_Rx_Buf[0][20-i];
//		}
//		Byte_To_Float(&a, r);
//		Byte_To_Float(&b, r+4);
//		Q=a;
//		P=b;
		if (P>1 || P<-1)
		{
			P=0;
		}
		if (Q>1 || Q<-1)
		{
			Q=0;
		}
		Qav+=(Q-Qav)*0.03f;
		Pav+=(P-Pav)*0.03f;
	}
	
	Net_Order(0);//发给功率变送器的数据刷新命令 
	
	if  (Net_Rx_Flag[1][0])
	{
		Net_Rx_Flag[1][0]=0;
		Monitor_Upload_Flag=1;
	}
		
	if (++ms_C>999)
	{
		ms_C=0;
	}
	
	Net_Comm(SPI2, ucQTx);//功率变送器 	
	Net_Comm(SPI3, ucMornitor[Net_Monitor_Index]);//在线监测 
}


//===========================================================================
// End of file.
//===========================================================================
