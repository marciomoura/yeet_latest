/********************************************************************
 * COPYRIGHT --  
 ********************************************************************
 * Program: SCR
 * File: command.c
 * Author: Yu Xiang
 * Created: September 11, 2018
 *******************************************************************/
#include "pub.h"
#include "stm32f4xx.h"
#include "USARTx.h"
#include "optical.h"

/********************************************************************
*Init_USART_Data                                                    *
*清除开出测试标志及数据。                                           *
********************************************************************/
void Init_USART_Data(void)
{
	int i;
	unsigned char* ucD;
	
	Asyn_Tx_Flag=0;
	Asyn_Buf_Flag=0;
	Status_Delay=0;
	
	ucD=AsynTxData;	
	for (i=0;i<148;i++)
	{
		*ucD++=0;
	}
	
	UI_Ref_Up_Flag=0;
	UI_Ref_Down_Flag=0;
	UI_Stop_Flag=0;
}


/********************************************************************
*Status_Data                                                        *
*将状态数据写入串口发送数据区。                                     *
********************************************************************/
extern float Scr_Ifd_Data[8];
void Status_Data(void)
{
	int nTemp, i;
	
	unsigned char* ptAsyn, *ptCRC, *ptS;
	
	if (Asyn_Tx_Flag)
	{
		Asyn_Buf_Flag=1;
		ptAsyn=AsynTxBuf;
	}
	else
	{
		Asyn_Tx_Flag=1;
		ptAsyn=AsynTxData;
	}
	ptCRC=ptAsyn;
	*ptAsyn++=0x68;	
	*ptAsyn++=0x68;		
	*ptAsyn++=3;
	*ptAsyn++=0xc0+uchNodeNum;
	
	//DI
	for (i=0;i<8;i++)
	{
		*ptAsyn++=DI_Data[i];
	}	
	
	//DO
	for (i=0;i<8;i++)
	{
		*ptAsyn++=DO_Data[i];
	}
	
	//Analog In
	for (i=0;i<6;i++)
	{
		Float_To_Byte(Iarm[i], ptAsyn);
		ptAsyn+=4;
	}
	
	nTemp=Ia*10000;
	*ptAsyn++=nTemp>>8;
	*ptAsyn++=nTemp & 0xff;
	
	nTemp=Ib*10000;
	*ptAsyn++=nTemp>>8;
	*ptAsyn++=nTemp & 0xff;
	
	nTemp=Ic*10000;
	*ptAsyn++=nTemp>>8;
	*ptAsyn++=nTemp & 0xff;
	
	nTemp=Va*10000;
	*ptAsyn++=nTemp>>8;
	*ptAsyn++=nTemp & 0xff;
	
	nTemp=Vb*10000;
	*ptAsyn++=nTemp>>8;
	*ptAsyn++=nTemp & 0xff;
	
	nTemp=Vc*10000;
	*ptAsyn++=nTemp>>8;
	*ptAsyn++=nTemp & 0xff;
	
	nTemp=Vfd*10000;
	*ptAsyn++=nTemp>>8;
	*ptAsyn++=nTemp & 0xff;
	
	nTemp=Ifd*10000;
	*ptAsyn++=nTemp>>8;
	*ptAsyn++=nTemp & 0xff;
	
	nTemp=Alpha*100;
	*ptAsyn++=nTemp>>8;
	*ptAsyn++=nTemp & 0xff;
	
	nTemp=P*10000;
	*ptAsyn++=nTemp>>8;
	*ptAsyn++=nTemp & 0xff;
	
	nTemp=Q*10000;
	*ptAsyn++=nTemp>>8;
	*ptAsyn++=nTemp & 0xff;
	
	nTemp=Ref*10000;
	*ptAsyn++=nTemp>>8;
	*ptAsyn++=nTemp & 0xff;
	
	nTemp=Ref_Ctrl*10000;
	*ptAsyn++=nTemp>>8;
	*ptAsyn++=nTemp & 0xff;
	//
	/*
	Tarm[0] = IfdSumAvag * 100;
	Tarm[1] = Scr_Ifd_Data[2] * 100;
	Tarm[2] = Scr_Ifd_Data[3] * 100;	
	Tarm[3] = Scr_Ifd_Data[4] * 100;
	Tarm[4] = AlphaAddValue*100;
	Tarm[5] = Scr_Ifd_Data[5] * 100;
	//
	Tarm[6] = Scr_Ifd_Data[2] * 100;
	Tarm[7] = Scr_Ifd_Data[3] * 100;
	Tarm[8] = Scr_Ifd_Data[4] * 100;
	Tarm[9] = Scr_Ifd_Data[5] * 100;
	Tarm[10] = Scr_Ifd_Data[6] * 100;
	Tarm[11] = Scr_Ifd_Data[1] * 100;
	*/
	//
	for (i=0;i<12;i++)
	{
		nTemp=Tarm[i]*100;
		*ptAsyn++=nTemp>>8;
		*ptAsyn++=nTemp & 0xff;
	}
	
	ptS=Sensor_ID[6];
	for (i=0;i<24;i++)
	{
		*ptAsyn++=*ptS++;
	}
	
	for (i=0;i<16;i++)
	{
		*ptAsyn++=0;
	}
	
	Float_To_Byte(S_n, ptAsyn);
	ptAsyn+=4;
	
	Float_To_Byte(I_n, ptAsyn);
	ptAsyn+=4;
	
	Float_To_Byte(V_n, ptAsyn);
	ptAsyn+=4;
	
	nTemp=CRC16(ptCRC, 146);
	*ptAsyn++=(nTemp>>8) & 0xff;
	*ptAsyn++=nTemp & 0xff;	
}

/********************************************************************
*Monitor_Command                                                    *
*接收响应监控命令。                                                 *
********************************************************************/
void Monitor_Command(void)
{
	short int i,j,k;

	for (i=0;i<10;i++)//
	{//搜索命令头部特征字
		if ((AsynRxData[i]==0x68) && (AsynRxData[i+1]==0x68) && (AsynRxData[i+2]==0x10))
			//&& (AsynRxData[i+3]<3) && (AsynRxData[i+4]==0))
		{
			k=CRC16((unsigned char*)(AsynRxData+i),8);
			j=AsynRxData[i+8];
			j<<=8;
			j+=AsynRxData[i+9];
			if (k==j)
			{
				break;
			}
		}
	}
	
	if (i<10) //搜索到命令
	{
		switch (AsynRxData[i+3])
		{
			case 1://增磁 
				UI_Ref_Up_Flag=1;
				break;
			case 2://减磁 
				UI_Ref_Down_Flag=1;
				break;
			case 3://停机 
				UI_Stop_Flag=1;
				break;
			case 4://Sensor ID 7
			case 5://Sensor ID 8
			case 6://Sensor ID 9
			case 7://Sensor ID 10
			case 8://Sensor ID 11
			case 9://Sensor ID 12
			case 10://Sensor ID 13
			case 11://Sensor ID 14
			case 12://Sensor ID 15
				j=AsynRxData[i+3]+2;
				Sensor_ID[j][0]=AsynRxData[i+4];
				Sensor_ID[j][1]=AsynRxData[i+5];
				Sensor_ID[j][2]=AsynRxData[i+6];
				Sensor_ID[j][3]=AsynRxData[i+7];
				break;
			case 13://S_n
				Byte_To_Float(&S_n, AsynRxData+i+4);
				break;
			
		}
		
	}
	
	for (i=0;i<9;i++)//按照先进先出原则刷新命令缓冲
	{
		AsynRxData[i]=AsynRxData[i+10];
	}
	
}

/********************************************************************
*Init_RS232                                                         *
*串口初始化。                                                       *
********************************************************************/
void Init_RS232_Comm(void)
{
	Init_RS232(4800, 148,10);
	Init_USART_Data();
}

/********************************************************************
*RS232_Comm                                                         *
*串口通信。                                                         *
********************************************************************/
void RS232_Comm(void)
{
	int i;
	
	if (++Status_Delay>1000)
	{
		Status_Delay=0;
		Status_Data();
	}
	
	if (Asyn_Tx_Flag==0 && Asyn_Buf_Flag)
	{
		for (i=0;i<148;i++)
		{
			AsynTxData[i]=AsynTxBuf[i];
		}
		Asyn_Tx_Flag=1;
		Asyn_Buf_Flag=0;
	}
	
	RS232_Tx(&Asyn_Tx_Flag, AsynTxData);
	if (RS232_Rx(AsynRxData+9))
	{
		Monitor_Command();
	}
}

//===========================================================================
// End of file.
//===========================================================================

