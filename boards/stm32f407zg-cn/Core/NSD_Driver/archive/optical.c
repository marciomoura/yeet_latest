/****************************************************************************************
*文件名称 optical.c                                                                     *
*作者     Three Gorges Nengshida Electric Co.,Ltd                                       *
*版本     NSD_STM32F2_Driver_6.0                                                        *
*修订日期 2014-01-10                                                                    *
*备注     光纤通信                                                                      *
****************************************************************************************/

#include <stm32f4xx.h>
#include "Optical_fiber.h"
#include "pub.h"
#include "mode.h"
#include "AI.h"
#include "para.h"
#include "syn.h"

#define Optical_Len 128	//数据帧长度128字节
#define Optical_Comm_Err_Time 100	//双通道失去联系后延时100ms清除脉冲允许标志
#define Optical_Fail_Delay 5	//是否在100ms内连续与多个设备失去光纤联系

//extern void Feed_Dog(void);
//extern void Zero_Calibrate(void);
//extern void SelectSyn(unsigned char ucSyn);

//extern float eIfd_Ref;//均流给定值
//extern unsigned char Write_ROM(void);
//extern float Ref;
//extern unsigned char AVR_Flag; 
//extern unsigned char FCR_Flag;//电流方式 
//extern unsigned char AER_Flag;//励磁电压方式
//extern float Ifd_Sum;

//1:Ch1
//2:Ch2
//3:DeEx
//4:SCR1
//5:SCR2
//6:SCR3
//7:SCR4
//8:SCR5
unsigned char OpticalRxData[8][300];//8个光纤口接收缓冲	1连接控制通道1， 2连接控制通道2
unsigned char OpticalData[8][128];//8个光纤口接收数据区
unsigned char Online_Source[8];//各个模块在线标志, 
float Optical_PID_X[8][2];	//各个控制模块的PID中间状态量 
float Optical_Alpha[8];		//各个控制模块的控制角 
short int OpticalRxCount[8];//接收数据计数器								 
unsigned char OpticalTxData[Optical_Len];//用于广播的公共发送缓冲
unsigned short int Optical_Comm_Delay[8];//通信周期延时
unsigned short int Optical_Comm_Status[8];//当前通信状态, 0: 中断 1: 接收正常 3:收发正常
unsigned short int Scr_Comm_OK_Num[8];//与各个控制模块通信正常的功率柜数量 
unsigned char Optical_Rx_Flag[8];//接收标志
short int Optical_Remain_Device[8];	//与本模块保持联系的设备的序号
short int nOC[8]={0,0,0,0,0,0,0,0};
short int nChNum[8];//各个智能模块保持通信的控制通道数量 
unsigned char Optical_Pulse_Flag[8];//各个智能模块脉冲标志 
unsigned char Optical_Syn[8];//各个智能模块当前同步工作相 1:A, 2:B, 4:C
unsigned char Optical_Standby_Flag[8];//功率柜模块热备标志 
unsigned char Ch_Online_Flag[8]; //各个功率柜的在线控制模块是否是控制通道 
short int Ch_Online_Delay;	//有功率柜接受控制通道控制时放弃控制权的延时 
unsigned char Unique_Scr_Flag;//光纤网络中调节器仅剩自身这1个功率柜 
short int Unique_Scr_Delay;//调节柜退出后单功率柜标志维持的延时 
float Optical_Scr_Ifd[8];	//各个功率柜输出电流, 求和之后得到Ifd

unsigned char Optical_Tx_Flag;//发送标志
unsigned char ROM_Write_Result;	//参数固化的结果
unsigned char Result_Flag;//数据中加入固化结果的标志
unsigned char Optical_Comm_Err_Flag;//与两个控制通道同时失去联系
unsigned char Online_Comm_Err_Flag;	//与在线通道失去联系
short int Optical_Comm_Err_Delay;//与两个控制通道均失去联系的延时计数器
short int Optical_OK_Num;	//设备连续掉线前的通信正常设备数量
short int Standby_Scr_Comm_Err_Delay;//功率柜间通信故障时延时退出热备(如果功率柜通信故障判断存在时间差)
short int Standby_Online_Comm_Err_Delay;//与控制通道通信故障时延时退出热备(如果功率柜通信故障判断存在时间差) 
short int Standby_Delay;//功率柜热备延时 
short int Pulse_Off_Delay;//脉冲允许时与在线模块失联后延时清除脉冲允许标志 
int Reset_Delay;//复位延时, 双通道光纤通信失败60s后停止喂狗, 重启CPU

short int Regulator_Offline_Count;	//同时与双通道失去联系时清除脉冲允许标志的延时 
int nStandbyNo[8];
unsigned char ScrSynError[8];//各个功率柜同步信号故障状态 


int Comm_Count;
unsigned char OC_D;
int t_Refresh_Delay;//时钟刷新间隔 

unsigned char m_auchCRCHi[]={
    0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,
    0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
    0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,
    0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
    0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x00,0xC1,
    0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,
    0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x00,0xC1,
    0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
    0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,
    0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,
    0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,
    0x81,0x40,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
    0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,
    0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,
    0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,
    0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
    0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,
    0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
    0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,
    0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
    0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,
    0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,
    0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,
    0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
    0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,
    0x80,0x41,0x00,0xC1,0x81,0x40 
	};

unsigned char m_auchCRCLo[]={
    0x00,0xC0,0xC1,0x01,0xC3,0x03,0x02,0xC2,0xC6,0x06,
    0x07,0xC7,0x05,0xC5,0xC4,0x04,0xCC,0x0C,0x0D,0xCD,
    0x0F,0xCF,0xCE,0x0E,0x0A,0xCA,0xCB,0x0B,0xC9,0x09,
    0x08,0xC8,0xD8,0x18,0x19,0xD9,0x1B,0xDB,0xDA,0x1A,
    0x1E,0xDE,0xDF,0x1F,0xDD,0x1D,0x1C,0xDC,0x14,0xD4,
    0xD5,0x15,0xD7,0x17,0x16,0xD6,0xD2,0x12,0x13,0xD3,
    0x11,0xD1,0xD0,0x10,0xF0,0x30,0x31,0xF1,0x33,0xF3,
    0xF2,0x32,0x36,0xF6,0xF7,0x37,0xF5,0x35,0x34,0xF4,
    0x3C,0xFC,0xFD,0x3D,0xFF,0x3F,0x3E,0xFE,0xFA,0x3A,
    0x3B,0xFB,0x39,0xF9,0xF8,0x38,0x28,0xE8,0xE9,0x29,
    0xEB,0x2B,0x2A,0xEA,0xEE,0x2E,0x2F,0xEF,0x2D,0xED,
    0xEC,0x2C,0xE4,0x24,0x25,0xE5,0x27,0xE7,0xE6,0x26,
    0x22,0xE2,0xE3,0x23,0xE1,0x21,0x20,0xE0,0xA0,0x60,
    0x61,0xA1,0x63,0xA3,0xA2,0x62,0x66,0xA6,0xA7,0x67,
    0xA5,0x65,0x64,0xA4,0x6C,0xAC,0xAD,0x6D,0xAF,0x6F,
    0x6E,0xAE,0xAA,0x6A,0x6B,0xAB,0x69,0xA9,0xA8,0x68,
    0x78,0xB8,0xB9,0x79,0xBB,0x7B,0x7A,0xBA,0xBE,0x7E,
    0x7F,0xBF,0x7D,0xBD,0xBC,0x7C,0xB4,0x74,0x75,0xB5,
    0x77,0xB7,0xB6,0x76,0x72,0xB2,0xB3,0x73,0xB1,0x71,
    0x70,0xB0,0x50,0x90,0x91,0x51,0x93,0x53,0x52,0x92,
    0x96,0x56,0x57,0x97,0x55,0x95,0x94,0x54,0x9C,0x5C,
    0x5D,0x9D,0x5F,0x9F,0x9E,0x5E,0x5A,0x9A,0x9B,0x5B,
    0x99,0x59,0x58,0x98,0x88,0x48,0x49,0x89,0x4B,0x8B,
    0x8A,0x4A,0x4E,0x8E,0x8F,0x4F,0x8D,0x4D,0x4C,0x8C,
    0x44,0x84,0x85,0x45,0x87,0x47,0x46,0x86,0x82,0x42,
    0x43,0x83,0x41,0x81,0x80,0x40 
	};

/********************************************************************
*CRC16 FUCTION                                                      *
*校验子程序。                                                       *
********************************************************************/
short int CRC16(unsigned char *Data, int length)
{	
	unsigned char uchCRCHi=0xFF ;
    unsigned char uchCRCLo=0xFF ;
    unsigned char uIndex ;
    
    while(length--)
    {
        uIndex=uchCRCHi^*Data++;
        uchCRCHi=uchCRCLo^m_auchCRCHi[uIndex];
        uchCRCLo=m_auchCRCLo[uIndex];
    }
    return(uchCRCHi<<8|uchCRCLo);
}

/********************************************************************
*Float_To_Byte(float,char*,char*,char*,char*)                       *
*                                                                   *
*一个浮点数分解成四个字节，用于参数存储。                           *
********************************************************************/
void Float_To_Byte(float data,unsigned char *b)
{
	unsigned char* ptr;
	
	ptr = (unsigned char *)&data;
	*b++ = *ptr++ ;
	*b++ = *ptr++ ;
	*b++ = *ptr++;
	*b++ = *ptr ;
}

/********************************************************************
*Byte_To_Float(float*,char,char,char,char)                          *
*                                                                   *
*四个字节合成一个浮点数，用于读取存储于ROM的参数。                  *
********************************************************************/
void Byte_To_Float(float *data,unsigned char* b)
{
	unsigned char* ptr;
	
	ptr = (unsigned char *)data;
	*ptr++=*b++;
	*ptr++=*b++;
	*ptr++=*b++;
	*ptr++=*b++;	
}

/************************************************************************
*Init_Optical_Comm                                                      *
*初始化光纤通信标志和数据区。                                           *
************************************************************************/
void Init_Optical_Comm(void)
{
	short int i,j;

	for (i=0;i<8;i++)
	{
		for (j=0;j<300;j++)
		{
			OpticalRxData[i][j]=0;
		}
		for (j=0;j<128;j++)
		{
			OpticalData[i][j]=0;
		}
	}

	for (i=0;i<Optical_Len;i++)
	{
		OpticalTxData[i]=0;
		OpticalData[0][i]=0;
		OpticalData[1][i]=0;		
	}

	Optical_Tx_Flag=0;

	for (i=0;i<8;i++)
	{
		Optical_Rx_Flag[i]=0;//接收标志
		OpticalRxCount[i]=0;
		Optical_Comm_Delay[i]=0;
		Optical_Comm_Status[i]=0;
		Online_Source[i]=0;
		Optical_PID_X[i][0]=0;
		Optical_PID_X[i][1]=0;
		Optical_Alpha[i]=120;
		nChNum[i]=2;
		Optical_Pulse_Flag[i]=0;
		Optical_Syn[i]=(uchNodeNum<<4)+1;//本柜A相 
		Ch_Online_Flag[i]=0;
		nStandbyNo[i]=2;//各控制模块记录的热备功率柜, 初始化时指向功率柜1 
		Scr_Comm_OK_Num[i]=1;
	}
	
	
	for (i=0;i<8;i++)
	{
		Optical_Scr_Ifd[i]=0;		
		ScrSynError[i]=0;
	}
	
	for (i=0;i<5;i++)
	{
		Scr_Pulse_Flag[i]=0;
		Scr_Pulse_Count[i]=0;
	}
	
	for (i=0;i<9;i++)
	{
		t_Op[i]=0;
	}
	Online_Source[0]=1;//上电后默认ch1在线 
	Optical_Comm_Status[uchNodeNum+1]=3;//与自身始终通信正常 
	ROM_Write_Result=0;
	Result_Flag=0;
	Optical_Comm_Err_Delay=0;
	Optical_Comm_Err_Flag=0;
	Online_Comm_Err_Flag=0;
	Reset_Delay=0;
	Pulse_Off_Delay=0;
	Ch_Online_Delay=0;
	
	Optical_OK_Num=nScrNum+2;
	Optical_Remain_Device[0]=0;
	Optical_Remain_Device[1]=1;
	Regulator_Offline_Count=0;
	Standby_Scr_Comm_Err_Delay=0;
	Standby_Online_Comm_Err_Delay=0;
	Standby_Delay=0;
	Unique_Scr_Flag=0;
	Unique_Scr_Delay=0;
	SynSource=(uchNodeNum<<4)+1;//本柜A相同步 
	Comm_Count=0;
	OC_D=0;
	t_Refresh_Delay=0;
}

/************************************************************************
*VerifySynSource                                                        *
*验证当前的同步相是否有问题 。                                          *
************************************************************************/
unsigned char VerifySynSource(unsigned char ucSyn)
{
	unsigned char Syn_OK_Flag=1;
	int n;
	
	n=(ucSyn>>4)-1;
	if (n<0 || n>4 || ScrSynError[n+2] || Scr_Pulse_Flag[n])
	{
		Syn_OK_Flag=0;
	}
	return Syn_OK_Flag;
}

/************************************************************************
*SelectSynSource                                                        *
*根据当前状态选择驱动脉冲的同步相。                                     *
************************************************************************/
void SelectSynSource(void)
{
	int i, n; 	
	
	for (i=2;i<nScrNum+2;i++)//选择序号最低的无同步故障的功率柜 
	{
		if (ScrSynError[i]==0 && Scr_Pulse_Flag[i-2]==0)
		{
			break;
		}
	}
	
	if (i==nScrNum+2)//均出现同步故障 
	{
		for (i=2;i<nScrNum+2;i++)
		{
			if (//(Scr_Pulse_Flag[i-2]==0) && 
				((ScrSynError[i]==1) || (ScrSynError[i]==2) || (ScrSynError[i]==4)))//单相故障 
			{
				break;
			}
		}
		
		if (i==nScrNum+2)//均出现多相故障 
		{
			for (i=2;i<nScrNum+2;i++)
			{
				if (//(Scr_Pulse_Flag[i-2]==0) && 
					(((ScrSynError[i] & 1)==0) || ((ScrSynError[i] & 2)==0) || ((ScrSynError[i] & 4)==0)))//单相正常
				{
					break;
				}
			}
			
			if (i<nScrNum+2)//取序号最小功率柜的无故障相  
			{
				if ((ScrSynError[i] & 1)==0)
				{
					SynSource=((i-1)<<4)+1;
				}
				else if ((ScrSynError[i] & 2)==0)
				{
					SynSource=((i-1)<<4)+2;
				}
				else
				{
					SynSource=((i-1)<<4)+3;
				}
			}
		}//均出现多相故障 
		else
		{
			if (ScrSynError[i] & 1)//A相故障 
			{
				SynSource=((i-1)<<4)+2;//切到B相
			}
			else if (ScrSynError[i] & 2)//B相故障 
			{
				SynSource=((i-1)<<4)+3;//切到C相
			}
			else
			{
				SynSource=((i-1)<<4)+1;
			}
		}
	}//(i==nScrNum)//均出现同步故障 
	else
	{
		n=SynSource & 7;
		if ((n==1)  || (n==2) || (n==3) )//尽量保持同步相为相同相  
		{
			SynSource=((i-1)<<4)+n;
		}
		else
		{
			SynSource=((i-1)<<4)+1;//取序号最小无故障功率柜的A相同步 
		}		
	}
	i=SynSource>>4;
	i--;
	if (Alpha_C!=Scr_Alpha_C[i])
	{
		Alpha_C=Scr_Alpha_C[i];
	}
}

/************************************************************************
*SCR_Data_Optical                                                       *
*收集需要上传的数据。                                                   *
************************************************************************/
void SCR_Data_Optical(void)
{
	short int i, j;
	unsigned char* ptCRC,*ptData;

	ptCRC=OpticalTxData;
	ptData=ptCRC;
	
	if (Online_Flag)
	{
		Alpha=Alpha_E;
	}
	
	*ptData++=0xab;//数据头
	*ptData++=0xc0 + uchNodeNum;
	
//	for (i=0;i<8;i++)
//	{
//		*ptData++=DI_Data[i];
//	}
	for (i=0;i<2;i++)//2019.9.10., add extra temperature data here
	{
		*ptData++=DI_Data[i];
	}
	
	i=Iabc*10000;//2020.6.7.
//	i=Iout*10000;//2020.8.1.
	*ptData++=(unsigned char)(i>>8);
	*ptData++=(unsigned char)(i & 0xff);
	
	i=Tarm[8]*100;
	*ptData++=(unsigned char)(i>>8);
	*ptData++=(unsigned char)(i & 0xff);

	i=Tarm[9]*100;
	*ptData++=(unsigned char)(i>>8);
	*ptData++=(unsigned char)(i & 0xff);
	
	*ptData=DO_Data[0];
	if (Result_Flag)
	{
		Result_Flag=0;
		if (ROM_Write_Result)
		{
			*ptData |=0x80;
		}
		else
		{
			*ptData |=0x40;
		}
	}
	ptData++;
	for (i=1;i<8;i++)
	{
		*ptData++=DO_Data[i];
	}
	
	//Iarm
	for (i=0;i<6;i++)
	{
		Float_To_Byte(Iarm[i],ptData);
		ptData+=4;
	}
	
	//PID_X
	Float_To_Byte(PID_X[0],ptData);
	ptData+=4;
	Float_To_Byte(PID_X[1],ptData);
	ptData+=4;
	
	//V, I
	i=Ia*10000;
	*ptData++=(unsigned char)(i>>8);
	*ptData++=(unsigned char)(i & 0xff);

	i=Ib*10000;
	*ptData++=(unsigned char)(i>>8);
	*ptData++=(unsigned char)(i & 0xff);

	i=Ic*10000;
	*ptData++=(unsigned char)(i>>8);
	*ptData++=(unsigned char)(i & 0xff);

	i=Va*10000;
	*ptData++=(unsigned char)(i>>8);
	*ptData++=(unsigned char)(i & 0xff);

	i=Vb*10000;
	*ptData++=(unsigned char)(i>>8);
	*ptData++=(unsigned char)(i & 0xff);

	i=Vc*10000;
	*ptData++=(unsigned char)(i>>8);
	*ptData++=(unsigned char)(i & 0xff);

	i=Vfd*10000;
	*ptData++=(unsigned char)(i>>8);
	*ptData++=(unsigned char)(i & 0xff);
	
	i=Ifd*10000;
	*ptData++=(unsigned char)(i>>8);
	*ptData++=(unsigned char)(i & 0xff);
	
	i=Alpha*100;
	*ptData++=(unsigned char)(i>>8);
	*ptData++=(unsigned char)(i & 0xff);

	i=Tarm[0]*100;
	*ptData++=(unsigned char)(i>>8);
	*ptData++=(unsigned char)(i & 0xff);

	i=Tarm[1]*100;
	*ptData++=(unsigned char)(i>>8);
	*ptData++=(unsigned char)(i & 0xff);

	i=Tarm[2]*100;
	*ptData++=(unsigned char)(i>>8);
	*ptData++=(unsigned char)(i & 0xff);

	i=Tarm[3]*100;
	*ptData++=(unsigned char)(i>>8);
	*ptData++=(unsigned char)(i & 0xff);

	i=Tarm[4]*100;
	*ptData++=(unsigned char)(i>>8);
	*ptData++=(unsigned char)(i & 0xff);
	
	i=Tarm[5]*100;
	*ptData++=(unsigned char)(i>>8);
	*ptData++=(unsigned char)(i & 0xff);

	i=Tarm[6]*100;
	*ptData++=(unsigned char)(i>>8);
	*ptData++=(unsigned char)(i & 0xff);

	i=Tarm[7]*100;
	*ptData++=(unsigned char)(i>>8);
	*ptData++=(unsigned char)(i & 0xff);
	
//	i=Ifd_Const*10000;
	i=Scr_Alpha_C[uchNodeNum-1]*100;
	*ptData++=(unsigned char)(i>>8);
	*ptData++=(unsigned char)(i & 0xff);
	
	i=Vfd_Const*10000;
	*ptData++=(unsigned char)(i>>8);
	*ptData++=(unsigned char)(i & 0xff);
	
	i=V_Const*10000;
	*ptData++=(unsigned char)(i>>8);
	*ptData++=(unsigned char)(i & 0xff);
	
	i=PID_T[0]*100;
	*ptData++=(unsigned char)(i>>8);
	*ptData++=(unsigned char)(i & 0xff);
	
	i=PID_T[1]*100;
	*ptData++=(unsigned char)(i>>8);
	*ptData++=(unsigned char)(i & 0xff);
	
	i=PID_T[2]*100;
	*ptData++=(unsigned char)(i>>8);
	*ptData++=(unsigned char)(i & 0xff);
	
	i=PID_T[3]*100;
	*ptData++=(unsigned char)(i>>8);
	*ptData++=(unsigned char)(i & 0xff);
	
	i=PID_K;
	*ptData++=(unsigned char)(i>>8);
	*ptData++=(unsigned char)(i & 0xff);	
	
	i=Van_Vf0*1000;
	*ptData++=(unsigned char)(i>>8);
	*ptData++=(unsigned char)(i & 0xff);
	
	i=T_Ifd*1000;
	*ptData++=(unsigned char)(i>>8);
	*ptData++=(unsigned char)(i & 0xff);
	
	i=Kvrefq*10000;
	*ptData++=(unsigned char)(i>>8);
	*ptData++=(unsigned char)(i & 0xff);
	
	i=I_Const[0]*10000;
	*ptData++=(unsigned char)(i>>8);
	*ptData++=(unsigned char)(i & 0xff);
	
	i=I_Const[1]*10000;
	*ptData++=(unsigned char)(i>>8);
	*ptData++=(unsigned char)(i & 0xff);
	
	i=I_Const[2]*10000;
	*ptData++=(unsigned char)(i>>8);
	*ptData++=(unsigned char)(i & 0xff);
	
	*ptData++=nScrNum;
	for (j=0;j<6;j++)
	{
		i=eIfd_Count[j];
		*ptData++=(unsigned char)(i>>8);
		*ptData++=(unsigned char)(i & 0xff);
	}
	*ptData++=0;
	
	i=CRC16(ptCRC, Optical_Len-2);
	*ptData++=(i>>8) & 0xff;
	*ptData=i & 0xff;
}

/************************************************************************
*Pulse_Test                                                             *
*接收界面发送的脉冲测试数据。                                           *
************************************************************************/
void Pulse_Test(unsigned char uchPulse)
{
	//脉冲测试
	if (Pulse_Enabled_Flag || 
	Vav>(float)0.1 ||
	(uchPulse & 0x3f)==0 )
	{
		Pulse_Test_Flag=0;
		Test_Pulse=0;
	}
	else
	{
		Pulse_Test_Flag=1;
		Test_Pulse=uchPulse & 0x3f;
	} 
}

/************************************************************************
*Get_eIfd_Command                                                       *
*从励磁调节器数据中读取均流命令。                                       *
************************************************************************/
void Get_eIfd_Command(unsigned char* Data)
{
	int i, j, k;
	
	i=*(Data+114);	
	if (i)//递减 
	{					
		k=1;
		for (j=0;j<6;j++)
		{
			if (i & k)
			{
				eIfd_Count[j]--;
				if (eIfd_Count[j]<-300)//3度 
				{
					eIfd_Count[j]=-300;
				}
			}
			k<<=1;
		}
	}
	
	i=*(Data+115);	
	if (i)//递增 
	{					
		k=1;
		for (j=0;j<6;j++)
		{
			if (i & k)
			{
				eIfd_Count[j]++;
				if (eIfd_Count[j]>300)//3度 
				{
					eIfd_Count[j]=300;
				}
			}
			k<<=1;
		}
	}
}

/************************************************************************
*Get_Command                                                            *
*从励磁调节器数据中读取命令。                                           *
************************************************************************/
void Get_Command(unsigned char* Data, unsigned short int n)
{
	unsigned char uchT;
	int i;
	float ft;
	
	uchT=*(Data+112);
	if (uchT==uchNodeNum || uchT==0x0f)
	{
		uchT=*(Data+113);
		i=*(Data+114);
		i<<=8;
		i+=*(Data+115);
		if (i>32767)
		{
			i-=65536;
		}
		switch (uchT)
		{
			case 1://脉冲测试
				Pulse_Test(*(Data+115));
				break;
			case 2:
				if ((*(Data+115)==1) && (AVR_Flag==0))
				{
					Work_Mode_V(Vav);					
				}
				else if ((*(Data+115)==2) && (FCR_Flag==0))
				{
//					Work_Mode_Ifd(Ifd_Sum_av);
				}
				break;
			case 3://修改可控硅均流补偿相位			
				Get_eIfd_Command(Data);
				break;
			case 4://零飘校正 
				if	(*(Data+114))
				{
					Zero_Calibrate();
				}				
				break;
			case 5://固化参数
				if (Pulse_Enabled_Flag==0)
				{
					ROM_Write_Result=Write_ROM();
					Result_Flag=1;
				}
				break;
			case 6://PID_T[0]
				Byte_To_Float(PID_T, Data+114);
				break;
			case 7://PID_T[1]
				Byte_To_Float(PID_T+1, Data+114);
				break;
			case 8:	//PID_T[2]		
				Byte_To_Float(PID_T+2, Data+114);
				break;
			case 9://PID_T[3]
				Byte_To_Float(PID_T+3, Data+114);
				break;
			case 10://PID_K
				Byte_To_Float(&PID_K, Data+114);
				break;
			case 11://Van_Vf0
				Byte_To_Float(&Van_Vf0, Data+114);
				break;	
			case 12://T_Ifd
				Byte_To_Float(&T_Ifd, Data+114);
				break;
			case 13://Kvrefq
				Byte_To_Float(&Kvrefq, Data+114);
				break;
			case 14://not use
//				Byte_To_Float(&V_n, Data+114);
				break;
			case 15://not use
				
				break;
			case 16://number of SCR
				Byte_To_Float(&ft, Data+114);	
				if (ft>1)
				{
					nScrNum=(int)ft;
				}
				break;	
//			case 17://Ifd_Const	
//				Byte_To_Float(&Ifd_Const, Data+114);
//				break;
			case 17://Alpha_C
				Byte_To_Float(&Scr_Alpha_C[uchNodeNum-1], Data+114);
				i=SynSource>>4;
				if (i==uchNodeNum)
				{
					Alpha_C=Scr_Alpha_C[uchNodeNum-1];
				}
				break;
			case 18://Vfd_Const
				Byte_To_Float(&Vfd_Const, Data+114);
				break;
			case 19:	//V_Const		
				Byte_To_Float(&V_Const, Data+114);
				break;	
			case 20://Vfd_n
				Byte_To_Float(&Vfd_n, Data+114);
				break;
			case 21://Ifd_n
				Byte_To_Float(&I_n, Data+114);
				break;
			case 22://Vanode_n 
				Byte_To_Float(&V_n, Data+114);
				break;
			case 23://上传传感器ID
				Upload_Sensor_ID_Flag=1;
				Sensor_ID_Des_Ch=n;
				break;
//			case 24://synSource
//				SynSource=*(Data+115);
//				break;
			case 25://I_Const[0]
				Byte_To_Float(&I_Const[0], Data+114);
				break;
			case 26://I_Const[1]
				Byte_To_Float(&I_Const[1], Data+114);
				break;
			case 27://I_Const[2]
				Byte_To_Float(&I_Const[2], Data+114);
				break;
		}		
	}
}

/************************************************************************
*Refresh_Alpha_Optical                                                  *
*从在线控制模块数据中读取脉冲允许标志和控制角。                         *
************************************************************************/
float Scr_Ifd_Data[8];
unsigned int Scr_Ifd_RxCnt[8] = {0};
unsigned int Scr_Comm_Fault[18] = {0};
//
void Calc_Update_Flag(void)
{
	//2020-05-29
	Sec10Elapse = TIM5->CNT - Sec10Start;
	//
	if(Sec10Elapse > 210000000)//2.5s  1s = 84,000,000
	{
		Sec10Start = TIM5->CNT;
		if((Flag10Sec & 0x01) == 0x00)
		{
			Flag10Sec = 0x11;
		}
		else
		{
			Flag10Sec = 0x10;
		}
	}
	//2020-05-29
}
//
void Refresh_Alpha_Optical(void)
{
	int i;
	float f=0, ifdLocal = 0.0f;
	unsigned char SynSource_t, flagIfdMax = 0, flagIfdMin = 0;
	
	for (i=2;i<nScrNum+2;i++)
	{
		f+=Optical_Scr_Ifd[i];
	}
	
	
	f = 0;
	Scr_Ifd_Data[uchNodeNum+1] = Ifd;
	for (i=2;i<nScrNum+2;i++)
	{
		f+=Scr_Ifd_Data[i];
	}
	//总电流
	Ifd_Sum=f;
	//总电流平均值
	IfdSumAvag = f / nScrNum;
	//本柜电流 
	//ifdLocal = Optical_Scr_Ifd[uchNodeNum+1];
	//参照void Scr_Data_Optical(unsigned char* Data, unsigned short int n)中内容说明, 上述描述不对
	ifdLocal = Ifd;
	//本柜电流是否最大
	for (i=2;i<nScrNum+2;i++)
	{
		if(i != (uchNodeNum+1))
		{
			if(ifdLocal > Scr_Ifd_Data[i])
			{
				flagIfdMax = 1;////如果本柜电流出现比其他柜电流大，则置位，只有比其他所有柜电流大时，则保持置位
			}
			else
			{
				flagIfdMax = 0;//如果本柜电流出现比其他柜电流小，则复位
			}
		}
	}
	//本柜电流是否最小
	for (i=2;i<nScrNum+2;i++)
	{
		if(i != (uchNodeNum+1))
		{
			if(ifdLocal < Scr_Ifd_Data[i])
			{
				flagIfdMin = 1;////如果本柜电流出现比其他柜电流小，则置位，只有比其他所有柜电流小时，则保持置位
			}
			else
			{
				flagIfdMin = 0;//如果本柜电流出现比其他柜电流大，则复位
			}
		}
	}
	//
	
	for (i=0;i<8;i++)
	{
		if ((Optical_Comm_Status[i]==3) && Online_Source[i]) //If modules online are more than 1, the module with smaller number will be chosen
		{
			break;
		}
	}
	
	Scr_Online_Flag=((i>2) && (i<nScrNum+2));
	
	//When no module online or this is the module online, not refresh data
	if ((Online_Flag==0) && (i<nScrNum+2))
	{
		PID_X[0]=Optical_PID_X[i][0];
		PID_X[1]=Optical_PID_X[i][1];
		SynSource_t=Optical_Syn[i];
		Pulse_Enabled_Flag=Optical_Pulse_Flag[i];
		//Alpha=Optical_Alpha[i];
		if((Flag10Sec & 0x10) == 0x10)
		{
			Flag10Sec &= 0x0F;//清除定时标志
			IfdDifference = Ifd - IfdSumAvag;
			//最小 & 偶数 & 超过负门槛
			if((IfdDifference < -0.0025f) && (flagIfdMin == 1) && (((Flag10Sec & 0x01) == 0x00)))//IfdDiffThreshold)//
			{
				AlphaAddValue -= 0.01f;//AlphaAddStep;
				
				if(AlphaAddValue < -1.0f)//0.5f)//AlphaAddMax)
				{
					AlphaAddValue = -1.0f;//0.5f;//AlphaAddMax;
				}
			}
			else if((IfdDifference > 0.0025f) && (flagIfdMax == 1) && (((Flag10Sec & 0x01) == 0x01)))//IfdDiffThreshold)//最大 & 奇数 & 超过正门槛
			{
				AlphaAddValue += 0.01f;//AlphaAddStep;
				if(AlphaAddValue > 1.0f)//0.5f)//AlphaAddMax)
				{
					AlphaAddValue = 1.0f;//0.5f;//AlphaAddMax;
				}				
			}
//			if ((Pulse_Enabled_Flag==0)|| (Ifd_Sum<0.02f))
			if (Pulse_Enabled_Flag==0)
			{
				AlphaAddValue=0;
			}
		}
		Alpha=Optical_Alpha[i]+ AlphaAddValue;
		//Alpha += AlphaAddValue;
	}
	else
	{
		SynSource_t=SynSource;
	}
	
	if (--Syn_Switch_Delay<=0)
	{
		Syn_Switch_Delay=0;
		if (VerifySynSource(SynSource_t))
		{
			SynSource=SynSource_t;
		}
		else
		{
			SelectSynSource();
		}
	}
	
	SelectSyn(SynSource);
	i=SynSource>>4;
	i--;
	Alpha_C=Scr_Alpha_C[i];
}

/************************************************************************
*Refresh_Own_Data                                                       *
*将自身数据写入数据区，便于状态分析。                                   *
************************************************************************/
void Refresh_Own_Data(void)
{
	int i,j,m;
	
	m=uchNodeNum+1;
	if (m<8)
	{
		Optical_Comm_Status[m]=3;
		
		//How many SCRs keep in touch with the SCR
		j=0;
		for (i=2;i<nScrNum+2;i++)
		{
			if (Optical_Comm_Status[i]==3)
			{
				j++;
			}
		}
		Scr_Comm_OK_Num[m]=j;
		
		//How many Channels keep in touch with the SCR
		nChNum[m]=(Optical_Comm_Status[0]==3)+(Optical_Comm_Status[1]==3);
		
		//PID_X
		Optical_PID_X[m][0]=PID_X[0];
		Optical_PID_X[m][1]=PID_X[1];
		
		//Online_Flag
		Online_Source[m]=Online_Flag;
		Ch_Online_Flag[m]=( (Online_Source[0]==1) || (Online_Source[1]==1) );
		
		//standby flag
		Optical_Standby_Flag[m]=Standby_Flag;
		
		//Syn
		Optical_Syn[m]=SynSource;
		ScrSynError[m]=(Syn_A_Err_Flag | Sec_A_Flag) + ((Syn_B_Err_Flag | Sec_B_Flag)<<1) + ((Syn_C_Err_Flag | Sec_C_Flag)<<2);
		
		//Pulse_Enabled_Flag
		Optical_Pulse_Flag[m]=Pulse_Enabled_Flag;
		
		//Alpha
		Optical_Alpha[m]=Alpha;
		
		//Ifd
		Optical_Scr_Ifd[m]=Ifd;
	}
}

/************************************************************************
*Scr_Data_Optical                                                       *
*从功率柜数据中读取在线标志、通信状态、同步相、脉冲允许标志和控制角。   *
************************************************************************/

void Scr_Data_Optical(unsigned char* Data, unsigned short int n)
{
	int i,j,k;
	float ft;
	//
	//For SCR1, uchNodeNum=1, Optical fiber 3: SCR2, 4:SCR3, 5:SCR4, 6:SCR5, 7:SCR6  ,3,4,5
	//For SCR2, uchNodeNum=2, Optical fiber 3: SCR1, 4:SCR3, 5:SCR4, 6:SCR5, 7:SCR6 2, ,4,5
	//For SCR3, uchNodeNum=3, Optical fiber 3: SCR1, 4:SCR2, 5:SCR4, 6:SCR5, 7:SCR6 2,3, ,5
	//For SCR4, uchNodeNum=4, Optical fiber 3: SCR1, 4:SCR2, 5:SCR3, 6:SCR5, 7:SCR6 2,3,4,
	//For SCR5, uchNodeNum=5, Optical fiber 3: SCR1, 4:SCR2, 5:SCR3, 6:SCR4, 7:SCR6
	//For SCR6, uchNodeNum=6, Optical fiber 3: SCR1, 4:SCR2, 5:SCR3, 6:SCR4, 7:SCR5
	//n: 0-Ch1, 1-Ch2, 2-DeEx, 3-SCR1, 4-SCR2, 5-SCR3, 6-SCR4, 7-SCR5, 8-SCR6 
	i=*(Data+15);
	i<<=8;
	i+=*(Data+14);
	j=1<<(uchNodeNum+uchNodeNum-2);//此功率柜与本功率柜的通信状态位
	if (i & j)//接收状态
	{
		Optical_Comm_Status[n] |=2;		
	}
	else
	{
		Optical_Comm_Status[n] &=1;
	}

	//How many SCRs keep in touch with the SCR
	k=i;
	j=0;
	for(i=0;i<nScrNum;i++)
	{
		if ((k & 3)==3)
		{
			j++;
		}
		k>>=2;
	}
	Scr_Comm_OK_Num[n]=j;
	
	//How many Channels keep in touch with the SCR
	k=*(Data+12);
	j=0;
	if ((k & 3)==3)
	{
		j++;
	}
	if ((k & 12)==12)
	{
		j++;
	}
	nChNum[n]=j;
	
	//PID_X
	Byte_To_Float(Optical_PID_X[n], Data+42);
	Byte_To_Float(Optical_PID_X[n]+1, Data+46);
	
	//Pulse_Enabled_Flag
	Optical_Pulse_Flag[n]= *(Data+3) & 1;
	
	//Online_Flag
	if (*(Data+13) & (1<<n))
	{
		Online_Source[n]=1;
//		Pulse_Enabled_Flag=Optical_Pulse_Flag[n];//脉冲允许标志
//		Alpha=Optical_Alpha[n];
	}
	else
	{
		Online_Source[n]=0;
	}
	
	//Ch online
	Ch_Online_Flag[n]=(*(Data+13)==1) || (*(Data+13)==2)  ;//if *(Data+13)=1, Ch1 online; if *(Data+13)=2, Ch2 online 
	
	//standby flag
	Optical_Standby_Flag[n]= ((*(Data+12) & 0x80)==0x80);
	
	//Syn
	ScrSynError[n]=(*(Data+10)>>3) & 7;
	Optical_Syn[n]=*(Data+16);
	
	//Alpha
	i=*(Data+66);
	i<<=8;
	i+=*(Data+67);
	Optical_Alpha[n]=(float)(i/100.0);
	
	//Ifd
	i=*(Data+64);
	i<<=8;
	i+=*(Data+65);
	Scr_Ifd_Data[n] = Optical_Scr_Ifd[n]=(float)(i/10000.0);
	Scr_Ifd_RxCnt[n]++;
	i=*(Data+84);
	i<<=8;
	i|=*(Data+85);
	if (i & 0x8000)
	{
		i-=65536;
	}
	ft=i;
	ft*=0.01f;
	if (Scr_Alpha_C[n-2]!=ft)
	{
		Scr_Alpha_C[n-2]=ft;		
	}
	i=SynSource>>4;
	if ((n==(i+1)) && (Alpha_C!=ft))
	{
		Alpha_C=ft;
	}
}

/************************************************************************
*Pulse_Off_Check                                                        *
*功率柜与在线控制模块失联后延时清除脉冲允许标志。                       *
************************************************************************/
void Pulse_Off_Check(void)
{
	int i;
	
	for (i=0;i<8;i++)
	{
		if (Online_Source[i])
		{
			break;
		}
	}
			
	if (Pulse_Enabled_Flag && (i==8))
	{
		Pulse_Off_Delay++;
		//通信中断判断周期4, 即80ms, 光纤板损坏判断延时周期5, 即100ms, 才能清除此前的在线标志, 
		//在此基础之上再延时20周期, 即400ms清除脉冲允许标志 
		if (Pulse_Off_Delay>20)
		{
			Pulse_Off_Delay=0;
			Pulse_Enabled_Flag=0;
		}
	}
	else
	{
		Pulse_Off_Delay=0;
	}		
}

/************************************************************************
*Standby_Check                                                          *
*功率柜争夺热备权限。                                                   *
************************************************************************/
void Standby_Check(void)
{
	int i,j=0;	
	int nMaxScr=0;
	int nMaxNo;
	
	//统计热备的功率柜数量 
	//nStandbyNo[0]=2;//初始化
	for (i=2;i<nScrNum+2;i++)
	{
		if (Optical_Standby_Flag[i])
		{
			nStandbyNo[j++]=i;//记录热备功率柜序号 
		}
		
		if ((nMaxScr<Scr_Comm_OK_Num[i]) && (Scr_Comm_OK_Num[i]<=nScrNum))
		{
			nMaxScr=Scr_Comm_OK_Num[i];
			nMaxNo=i;
		}
	}
	
	if (Standby_Flag)
	{
		if (Scr_Comm_OK_Num[1+uchNodeNum]<nMaxScr || //所带功率柜较少 
			( (Scr_Comm_OK_Num[1+uchNodeNum]==nMaxScr) && (nMaxNo<1+uchNodeNum) && (Online_Flag==0) ) ||//序号较大时自动退出 
			(Scr_Online_Flag && (Online_Flag==0)))//功率柜在线且不是自身在线 
		{
			Standby_Flag=0;
		}
		else
		{		
			//与在线控制模块失去联系时退出 
			for (i=0;i<6;i++)
			{
				if (Online_Source[i])
				{
					break;
				}
			}
			if (i==6)
			{
				if (++Standby_Online_Comm_Err_Delay>40)
				{
					Standby_Online_Comm_Err_Delay=0;
					Standby_Flag=0;
				}
			}
			else
			{
				Standby_Online_Comm_Err_Delay=0;
			}
		}			
	}
	else if (DO_Lock_Flag==0)// 抢夺热备权限 
	{		
		if ( Online_Flag ||	//本功率柜在线 
		 (Scr_Comm_OK_Num[nStandbyNo[0]]<Scr_Comm_OK_Num[uchNodeNum+1]) )//热备功率柜所带功率柜数量比本柜少 		 
		{
			Standby_Flag=1;
			Standby_Delay=0;
		}		
		
		if (j)//有功率柜热备时清除热备延时
		{
			Standby_Delay=0;
		}
		else if (j==0)//无功率柜热备 
		{
			if (++Standby_Delay>10*uchNodeNum)
			{
				Standby_Flag=1;
				Standby_Delay=0;
			}
		}
	}
}

/************************************************************************
*Online_Check                                                           *
*功率柜争夺控制权限, 每个周期检测一次。                                 *
************************************************************************/
void Online_Check(void)
{
	int i,j,n;
	int nOnlineModule[8];
	
	if (DO_Lock_Flag==0)
	{
		for (i=0,n=0;i<8;i++)//统计在线模块数量
		{
			if (Online_Source[i])
			{
				nOnlineModule[n++]=i;//记录在线模块数量及序号 
			}
		}
		
		if (Online_Flag)
		{
			if (n>1)//多个控制模块同时在线
			{
				if ((nOnlineModule[0]==0) || (nOnlineModule[0]==1))//控制通道在线 
				{
					Scr_Offline_Delay=0;
					Online_Flag=0;
				}
				else//多个功率柜在线 
				{				
					for (i=0,j=0;i<n;i++)
					{
						if ((nOnlineModule[i]!=(uchNodeNum+1)) && (Scr_Comm_OK_Num[nOnlineModule[i]]>j))//其他在线功率柜所带功率柜最大数量 
						{
							j=Scr_Comm_OK_Num[nOnlineModule[i]];
						}
					}
					if (j>Scr_Comm_OK_Num[uchNodeNum+1])//所带功率柜数量少则立即放弃在线权 
					{
						Scr_Offline_Delay=0;
						Online_Flag=0;
					}
					else if (j==Scr_Comm_OK_Num[uchNodeNum+1])//所带功率柜数量一致则延时放弃, 拨码值低者后放弃 
					{
						Scr_Offline_Delay++;
						if (Scr_Offline_Delay>(30-5*uchNodeNum))
						{
							Scr_Offline_Delay=0;
							Online_Flag=0;
						}
					}
					else
					{
						Scr_Offline_Delay=0;
					}
				}
			}//n>1, more than 1 SCR online
			
			for (i=2;i<nScrNum+2;i++)
			{
				if (Ch_Online_Flag[i])
				{
					break;
				}
			}
			
			if (i<nScrNum+2)//There are other SCRs controlled by Ch 
			{
				if (++Ch_Online_Delay>5)//Delay to clear influence result from commuication delay
				{
					Ch_Online_Delay=0;
					Online_Flag=0;
				}
			}
			else
			{
				Ch_Online_Delay=0;
			}
			
			//作为在线控制通道协调同步信号源 
			if (Syn_Switch_Delay<=0)
			{
				SelectSynSource();
				SelectSyn(SynSource);//即时生效 
			}
		}//Online_Flag
		else// if (Pulse_Enabled_Flag)
		{//夺取在线控制权 
			for (i=2,j=0;i<nScrNum+2;i++)
			{
				if ((Optical_Comm_Status[i]==3) && nChNum[i])//功率柜是否仍然与控制通道保持通信 
				{
					j++;//记录仍然与调节柜保持联系的功率柜数量  
				}
			}
			
			if (j==0) //仅当所有功率柜与控制通道失去联系后才能争夺在线权  
			{
				if (n==0) //没有模块在线 
				{				
					if (Scr_Comm_OK_Num[uchNodeNum+1]>1)//其他功率柜中, 与其中至少一个仍然保持联系 
					{
						if (Standby_Flag)//热备功率柜无延时夺取在线权 
						{
							Online_Flag=1;
							Scr_Online_Delay=0;
						}
						else//非热备功率柜按照拨码延时夺取在线权 
						{
							if (++Scr_Online_Delay>5*uchNodeNum)
							{
								Online_Flag=1;
								Scr_Online_Delay=0;								
							}
						}
					}//光纤网络至少还有2个SCR
					else if (Unique_Scr_Flag)//此前是控制通道+单功率柜 
					{
						Online_Flag=1;
						Scr_Online_Delay=0;
					}
					else if (Qav-Q>0.02f || Vav-Vabc>0.02f)//无功功率跌落2%以上判在线功率柜退出运行 
					{
						Online_Flag=1;
						Scr_Online_Delay=0;
					}
					else
					{
						Pulse_Enabled_Flag=0;//Not online, and only 1 SCR left in networks, pulses are disabled
						Scr_Online_Delay=0;
					}
				}//没有模块在线 
				else//当前有功率柜在线 
				{
					if (Scr_Comm_OK_Num[nOnlineModule[0]]<Scr_Comm_OK_Num[uchNodeNum+1])//所带功率柜数量多于在线功率柜 
					{
						if (++Scr_Online_Delay>5*uchNodeNum)
						{
							Online_Flag=1;
							Scr_Online_Delay=0;
						}
					}
					else
					{
						Scr_Online_Delay=0;
					}
				}				
			}
		}
		
		if (Online_Flag && Alpha_Mode_Flag)//2019.9.23., clear pulse flag in alpha mode
		{
			Pulse_Enabled_Flag=0;
		}
	}//DO_Lock_Flag==0
}

/************************************************************************
*Collect_Sensor_ID                                                      *
*上传传感器ID。                                                         *
************************************************************************/
void Collect_Sensor_ID(void)
{
	int i;
	unsigned char *ucS, *ucD;
	
	ucS=Sensor_ID[0];
	ucD=OpticalTxData;
	
	*ucD++=0xab;
	*ucD++=0xc0 | uchNodeNum;
	for (i=0;i<24;i++)
	{
		*ucD++=*ucS++;
	}
	for (i=0;i<99;i++)
	{
		*ucD++=0;
	}
	*ucD++=1;//data ID
	i=CRC16(OpticalTxData, Optical_Len-2);
	*ucD++=(i>>8) & 0xff;
	*ucD=i & 0xff;
}

/************************************************************************
*Ex_Data_Sensor_ID                                                      *
*从励磁调节器数据中读取传感器ID。                                       *
************************************************************************/
void Ex_Data_Sensor_ID(unsigned char* Data, unsigned short int n)
{
	int i;
	unsigned char *ucS, *ucD;
	
	ucD=Sensor_ID[0];
	ucS=Data+uchNodeNum*24-20;
	for (i=0;i<24;i++)
	{
		*ucD++=*ucS++;
	}
	Upload_Sensor_ID_Flag=1;
	Sensor_ID_Des_Ch=n;
}

/************************************************************************
*Ex_Data_Optical                                                        *
*从励磁调节器数据中读取在线标志、同步相、脉冲允许标志和控制角。         *
************************************************************************/
void Ex_Data_Optical(unsigned char* Data, unsigned short int n)
{
	RTC_TimeTypeDef t;//时刻 
	RTC_DateTypeDef date;//日期
	int i,j,k;

	i=1<<(uchNodeNum+uchNodeNum-2);//功率柜对应的通信状态位
	j=*(Data+8);
	j&=0xf;
	j<<=8;
	j+=*(Data+7);
	if (j & i)//控制通道接收状态 
	{
		Optical_Comm_Status[n] |=2;		
	}
	else
	{
		Optical_Comm_Status[n] &=1;
	}

	//How many SCRs keep in touch with the channel
	k=j;
	j=0;
	for(i=0;i<6;i++)
	{
		if ((k & 3)==3)
		{
			j++;
		}
		k>>=2;
	}
	Scr_Comm_OK_Num[n]=j;
	
	//How many channels keep in touch with the channel
	k=*(Data+8);
	if ((k & 0xc0)==0xc0)
	{
		nChNum[n]=2;
	}
	else
	{
		nChNum[n]=1;
	}
	
	//PID_X
	Byte_To_Float(Optical_PID_X[n], Data+72);
	Byte_To_Float(Optical_PID_X[n]+1, Data+76);
	
	//Pulse_Enabled_Flag
	//Optical_Pulse_Flag[n]=((*(Data+17) & 8) ==8);
	Optical_Pulse_Flag[n]=((*(Data+12) & 1) ==1);
	
	//Alpha
	i=*(Data+48);
	i<<=8;
	i+=*(Data+49);
	Optical_Alpha[n]=i/100.0;
	
	//Online_Flag
	if (*(Data+9) & (1<<n))
	{
		Online_Source[n]=1;
		Pulse_Enabled_Flag=Optical_Pulse_Flag[n];//脉冲允许标志
		//Alpha=Optical_Alpha[n];
		Alpha=Optical_Alpha[n]+ AlphaAddValue;
		//Alpha mode
		if (*(Data+6) & 0x10) //from online channel 
		{
			Alpha_Mode_Flag=1;
		}
		else
		{
			Alpha_Mode_Flag=0;
		}
	}
	else
	{
		Online_Source[n]=0;
	}	
	
	if (*(Data+10) & 2)//通信在线
	{
		if (++t_Refresh_Delay>1000)//10s
		{
			t_Refresh_Delay=0;
			date.RTC_Year=*(Data+82);
			date.RTC_Month=*(Data+83);
			date.RTC_Date=*(Data+84);
			if (date.RTC_Year>18)
			{
				RTC_SetDate(RTC_Format_BIN, &date);	
				t.RTC_Hours=*(Data+86);
				t.RTC_Minutes=*(Data+87);
				t.RTC_Seconds=*(Data+88);
				RTC_SetTime(RTC_Format_BIN, &t);
				i=*(Data+89);
				i<<=8;
				i|=*(Data+90);
				ms_C=i;
			}
		}
	}
	//Syn
 	Optical_Syn[n]=*(Data+80);
	Get_Command(Data, n);
	Optical_Scr_Ifd[n]=0;
}

/************************************************************************
*Search_Command                                                         *
*从接收的数据搜索命令。                                                 *
************************************************************************/
void Search_Command(unsigned short int n)
{
	unsigned short int i,j;
	unsigned short int unCRC;
	unsigned char* chPtD, *chPtS;

	if (OpticalRxCount[n]>=Optical_Len)//Optical_Len=128
	{
		for (j=0;j<OpticalRxCount[n]-Optical_Len+1;j++)
		{
			if (OpticalRxData[n][j]==0xab && ((OpticalRxData[n][j+1] & 0xf0)==0xc0))//数据头
			{
				unCRC=CRC16(&OpticalRxData[n][j],Optical_Len-2);
				i=OpticalRxData[n][j+Optical_Len-2];
				i<<=8;
				i+=OpticalRxData[n][j+Optical_Len-1];
				if (i==unCRC)//校验通过
				{
					break;
				}
			}
		}
		if (j!=OpticalRxCount[n]-Optical_Len+1)//验证通过
		{
			
			Optical_Comm_Status[n] |=1;//接收正常
			//nOpticalDelay[n][nOC[n]++]=Optical_Comm_Delay[n];
			if (nOC[n]>=500)
			{
				nOC[n]=0;
			}
			Optical_Comm_Delay[n]=0;//延时清零
			Optical_Rx_Flag[n]=1;
			chPtS=&OpticalRxData[n][j];//使用指针以减少计算量
			chPtD=OpticalData[n];
			for (i=0;i<Optical_Len;i++)
			{
				*chPtD++=*chPtS++;
			}

			//剩余数据前移
			if (OpticalRxCount[n]>j+Optical_Len-1)//收到的数据大于帧长
			{
				chPtS=&OpticalRxData[n][j+Optical_Len];//使用指针以减少计算量
				chPtD=OpticalRxData[n];
				for (i=0;i<OpticalRxCount[n]-j-Optical_Len;i++)
				{
					*chPtD++=*chPtS++;
				}
			}
			OpticalRxCount[n]-=j+Optical_Len;//计数值减去帧长
			if (n<2)//控制通道 
			{
				if ((OpticalData[n][1]==0xc1) && (OpticalData[n][2]==0xff) && (OpticalData[n][3]==0xff))//sensor ID
				{
					Ex_Data_Sensor_ID(OpticalData[n],n);
				}
				else
				{
					Ex_Data_Optical(OpticalData[n],n);//数据解析		
				}
			}
//			else if (n==5)//对时数据, 葛洲坝专用  
//			{
//				for (i=0;i<9;i++)
//				{
//					t_Op[i]=OpticalData[n][i+2];
//				}
//			}
			else
			{				
				Scr_Data_Optical(OpticalData[n],n);//数据解析		
			}
		}
		else //去掉没有通过校验的OpticalRxCount[n]-Optical_Len+1个数据, 保留可能通过校验的Optical_Len-1的后来数据
		{
			j=OpticalRxCount[n]-Optical_Len+1;
			chPtS=&OpticalRxData[n][j];
			chPtD=OpticalRxData[n];
			for (i=0;i<Optical_Len-1;i++)
			{
				*chPtD++=*chPtS++;
			}
			OpticalRxCount[n]-=j;//2014.10.29., 清除多余数据后在接收数据数量中减去对应的长度			
		}
	}
}

/************************************************************************
*Optical_Error_Reset                                                    *
*光纤模块故障响应, 清除自身脉冲允许、在线、热备, 设置故障标志。         *
************************************************************************/
void Optical_Error_Reset(void)
{
	int i=uchNodeNum+1;
	
	Optical_Comm_Err_Flag=1;
	Pulse_Enabled_Flag=0;
	Online_Flag=0;
	Standby_Flag=0;
	nChNum[i]=0;
	Scr_Comm_OK_Num[i]=1;
	Online_Source[i]=0;
	Optical_Standby_Flag[i]=0;
	Optical_Scr_Ifd[i]=0;
	Optical_Pulse_Flag[i]=0;
	Optical_Alpha[i]=120;
	Optical_PID_X[i][0]=0;
	Optical_PID_X[i][1]=0;
}

/************************************************************************
*Clear_Comm_Err_Module                                                  *
*复位失联模块状态 。                                                    *
************************************************************************/
void Clear_Comm_Err_Module(int i)
{
	Online_Source[i]=0;
	Optical_PID_X[i][0]=0;
	Optical_PID_X[i][1]=0;
	Optical_Alpha[i]=120;
	Optical_Pulse_Flag[i]=0;
	Optical_Syn[i]=0x11;
	Optical_Standby_Flag[i]=0;
	Scr_Comm_OK_Num[i]=0;
	Ch_Online_Flag[i]=0;
	if (i>1)//功率柜 
	{
		ScrSynError[i]=7;
	}
	nChNum[i]=0;
	if (i>1)
	{
		Optical_Scr_Ifd[i-2]=0;
	}
}

/************************************************************************
*Unique_Scr_Check                                                       *
光纤网络是否仅剩控制通道与自身这一个SCR.                                *
************************************************************************/
void Unique_Scr_Check(void)
{
	if (((Optical_Comm_Status[0]==3) && (Scr_Comm_OK_Num[0]==1) && (Online_Source[0]==1)) ||
		((Optical_Comm_Status[1]==3) && (Scr_Comm_OK_Num[1]==1) && (Online_Source[1]==1)))
	{
		Unique_Scr_Flag=1;
	}
	else if ((Optical_Comm_Status[0]!=3) && (Optical_Comm_Status[1]!=3) && Unique_Scr_Flag)
	{
		if (++Unique_Scr_Delay>20)
		{
			Unique_Scr_Flag=0;
			Unique_Scr_Delay=0;
		}
	}
	else
	{
		Unique_Scr_Flag=0;
		Unique_Scr_Delay=0;
	}
}

/************************************************************************
*Optical_Comm_Check                                                     *
*光纤通信状态检测。                                                     *
************************************************************************/
void Optical_Comm_Check(void)
{
	Refresh_Own_Data();//自身数据写入数据区，便于集中分析 
	Unique_Scr_Check();//单功率柜检测 
	Online_Check();//在线检测 
	Standby_Check();//热备检测 
	//Pulse_Off_Check();//与在线控制模块失联后自动延时清除脉冲允许标志 
}

/************************************************************************
*Optical_Comm                                                           *
*光纤通信。                                                             *
************************************************************************/
void Optical_Comm(void)
{
	short int i,j,n;
	
	for (i=0;i<8;i++)//依次查询6个光纤串口数据接收, 不接收灭磁柜数据(i=2) 
	{
		if (i<2)
		{
			n=i;
		}
		else if(i>2)//from other SCR
		{
			n=i-1;
			if (i>uchNodeNum+1)
			{
				n=i;
			}			
		}
		
		if (i!=2)
		{
			j=Rx_Buffer(i+1, OpticalRxData[n]+OpticalRxCount[n]);
			OpticalRxCount[n]+=j;
			Search_Command(n);
			if (++Optical_Comm_Delay[n] > 80)//最长4周期延时
			{
				Optical_Comm_Status[n]=0;
				Optical_Rx_Flag[n]=0;
				Optical_Comm_Delay[n]=80;//延时不得无限增大 
				Clear_Comm_Err_Module(n);
				if(Scr_Comm_Fault[n + 8] == 0)
				{
					Scr_Comm_Fault[n + 8] = 1;
					Scr_Comm_Fault[n]++;
				}
			}
			else
			{
				Scr_Comm_Fault[n + 8] = 0;
			}
		}
	}

	if (Optical_Tx_Flag)
	{
		++OC_D;
		if (OC_D>=2)//延时2ms, 保证FPGA完成同步时刻转发 
		{
			OC_D=0;
			Optical_Tx_Flag=0;
			
			SCR_Data_Optical();
			Refresh_Alpha_Optical();//获取在线控制模块的控制角、脉冲允许标志和PID中间状态量 
			Optical_Comm_Check();//通信状态检测, 是否需要抢夺在线权、热备权或清除脉冲允许标志 
			Tx_Buffer(0,OpticalTxData,Optical_Len);//广播模式发送给控制通道及其他功率柜 
			if (Upload_Sensor_ID_Flag)
			{
				Collect_Sensor_ID();
				Upload_Sensor_ID_Flag=0;
				Tx_Buffer(Sensor_ID_Des_Ch+1,OpticalTxData,Optical_Len);//点对点发送 
			}
		}		
	}
	
	
	if (++ms_C>999)//毫秒计数器
	{
		ms_C=0;
	}
}
//===========================================================================
// End of file.
//===========================================================================
