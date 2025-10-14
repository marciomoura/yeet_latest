/********************************************************************
*pub.c                                                              *
*全局变量定义。                                                     *
********************************************************************/ 


short int Online_Delay;	//连续收到非在线数据的记次，防止两个通道均不在线时间过长 

unsigned char DO_Lock_Flag;//上电开出闭锁标志
unsigned short int DO_Lock_Delay;//上电闭锁延时

unsigned char Current_Pulse;//当前脉冲，根据当前判断导通的桥臂，对桥臂电流赋值

unsigned char Scr_Fan_Err_Flag;//功率柜风机停运标志
unsigned char Scr_Exit_Flag;

unsigned char SynSource;//所选脉冲信号同步源: 高四位是功率柜拨码, 第四位是同步相(A:1, B:2, C:3) 

float Ifd;//励磁电流
float Vfd;//励磁电压
float Alpha;//控制角 
float Q;//无功功率 
float P;//有功功率
float F;//频率
unsigned long Period;//周期 
unsigned long Period_Source[5];//待滤波的周期测量值 
float Ifd_av;//励磁电流平均值 
float Vfd_av;//励磁电压平均值 
float Qav;
float Pav;
float Fav;//频率平均值，用于伏赫限制等
long Period_av;//周期平均值

float Avg_Ctrl_Coef;//Vf, F, P的积分系数
float Avg_Coef;		//V, I, Ifd, Q的积分系数
float Kvrefq;

//unsigned char Write_ROM_Flag;	//参数固化标志
unsigned Over_Temperature_Flag;	//过热标志
//*****************************************************************************************************************

int Syn_F_Delay;	//出现同步信号后到开始频率计算的延迟次数

unsigned char Low_F_Flag;		//低频标志
int Low_F_Count;				//低频计数器
int Low_F_Trig_Delay;			//低频时替代同步触发通信测量的主循环触发延时
unsigned int Syn_A_Int_Time;	//A相中断时刻
unsigned int Syn_B_Int_Time;	//B相中断时刻
unsigned int Syn_C_Int_Time;	//C相中断时刻 
unsigned int Syn_P_Int_Time;  //脉冲驱动相中断时刻 
unsigned int Syn_AB_Time;	//AB相中断时刻差
unsigned int Syn_BC_Time;	//BC相中断时刻差
unsigned int Syn_CA_Time;	//CA相中断时刻差
unsigned int Syn_Int_Time;		//当前工作相中断时刻

unsigned char Syn_A_Work_Flag;	//A相同步工作标志
unsigned char Syn_B_Work_Flag;	//B相同步工作标志
unsigned char Syn_C_Work_Flag;	//C相同步工作标志
unsigned char Syn_A_Err_Flag;	//A相同步原方断线标志
unsigned char Syn_B_Err_Flag;	//B相同步原方断线标志
unsigned char Syn_C_Err_Flag;	//C相同步原方断线标志
unsigned char Sec_A_Flag;	//A相同步副方断线标志
unsigned char Sec_B_Flag;	//B相同步副方断线标志
unsigned char Sec_C_Flag;	//C相同步副方断线标志
unsigned char Syn_Error_Flag;		//同步故障标志 

int Sec_A_Count;	//A相同步副方延迟计数器
int Sec_B_Count;	//B相同步副方延迟计数器
int Sec_C_Count;	//C相同步副方延迟计数器
int Syn_A_Count;	//A相同步原方恢复延迟计数器
int Syn_B_Count;	//B相同步原方恢复延迟计数器
int Syn_C_Count;	//C相同步原方恢复延迟计数器 
int Syn_Pulse_Count;//同步驱动相恢复延时计数器 

unsigned char DI_Data[8];
unsigned char DO_Data[8];
unsigned int Running_Count;		//运行指示灯闪烁计数器，10ms计数1次
unsigned int Running_Flag;	//运行指示灯状态，1：亮，2：灭

//ad.c 
float Vfd_AD_Const;	//励磁电压标幺化系数
float V_AD_Const;		//阳极电压标幺化系数
float I_AD_Const;		//阳极电流标幺化系数
float Vfd_n;//励磁电压额定值  
float V_n;//阳极电压额定值 
float I_n;//电流额定值 
float S_n;//视在功率额定值 

unsigned char AD_Err_Flag;			//AD芯片故障
unsigned char SCR_Error_Flag;		//功率柜故障标志
// unsigned char Comm_Err_Flag[2];		//通信失败标志 
// unsigned char Net_Tx_Err_Flag[2];	//网络发送超时标志 
// unsigned char Net_Rx_Err_Flag[2];	//网络接收超时标志

unsigned char Fuse_Flag;		//快熔熔断标志
unsigned char Fuse_A_Flag;		//快熔A+熔断标志
unsigned char Fuse_B_Flag;		//快熔B+熔断标志
unsigned char Fuse_C_Flag;		//快熔C+熔断标志
unsigned char Fuse_An_Flag;		//快熔A-熔断标志
unsigned char Fuse_Bn_Flag;		//快熔B-熔断标志
unsigned char Fuse_Cn_Flag;		//快熔C-熔断标志

unsigned char Phase_A_Flag;	//A相桥臂短路标志
unsigned char Phase_B_Flag;	//B相桥臂短路标志
unsigned char Phase_C_Flag;	//C相桥臂短路标志

short int Phase_A_Delay;	//A相桥臂短路延时 
short int Phase_B_Delay;	//B相桥臂短路延时
short int Phase_C_Delay;	//C相桥臂短路延时

unsigned char Fan_Flag;			//风机已投入标志
unsigned char Fan_On_Flag;		//风机投入命令

//临时数组：保留上三次采样结果，滤波用
//float Vfd_Data[3];	//励磁电压	
//float Ifd_Data[3];	//励磁电流

unsigned short int Pulse_Pointer;//脉冲序号
unsigned short int Phase_Offset;//同步信号引起的脉冲序号偏差补偿
unsigned int Rising_Edge_Flag;	//脉冲上升沿标志					 
unsigned char Pulse_Enabled_Flag;//脉冲允许标志

unsigned char chSCR[76];		//功率柜、灭磁柜数据
unsigned char chEx[2][76];		//来自控制通道的数据

unsigned char Pulse_Test_Flag;	//脉冲测试标志
unsigned char Test_Pulse;		//待测试的脉冲

float Va;	//a相阳极电压
float Vb;	//b相阳极电压
float Vc;	//c相阳极电压
float Vabc;	//三相阳极电压平均值
float Vav;	//阳极电压积分值

float Ia;	//a相阳极电流
float Ib;	//b相阳极电流
float Ic;	//c相阳极电流
float Iabc;	//三相阳极电流平均值
float Iav;	//阳极电流积分值

float Iap;	//可控硅A+桥臂电流
float Ibp;	//可控硅B+桥臂电流
float Icp;	//可控硅C+桥臂电流
float Ian;	//可控硅A-桥臂电流
float Ibn;	//可控硅B-桥臂电流
float Icn;	//可控硅C-桥臂电流
float Iarm[6];//滤波后的桥臂电流 

//float Tap;	//可控硅A+温度
//float Tbp;	//可控硅B+温度
//float Tcp;	//可控硅C+温度
//float Tan;	//可控硅A-温度
//float Tbn;	//可控硅B-温度
//float Tcn;	//可控硅C-温度
//float Tscr;	//柜内温度
//float Tin;	//进风口温度
//float Tout; //出风口温度

float Vabc_Temp[2];//用于中值滤波的临时变量
float Iabc_Temp[2];
float Vfd_Temp[2];
float Ifd_Temp[2];

unsigned char uchNodeNum;//拨码数值

unsigned char Scr_FCR_Flag;//发脉冲时与调节柜通信中断时独立运行标志
unsigned char AI_Flag;//脉冲下降沿采样完成后通知主循环计算采样结果
unsigned char Ref_Up_Flag;//调节柜增磁令
unsigned char Ref_Down_Flag;//调节柜减磁令
unsigned char DeEx_Flag;//调节柜逆变令

//PID
float Ref;//给定值 
float Ref_Ctrl;//调差后的给定值 
float Ref_Step_Up_Adder;//连续增磁
float Ref_Step_Down_Adder;//连续减磁 
float Ref_Step;//给定值步长 
float Ref_Step_Limit;//连续增减磁时允许调节的给定值幅值
float Ref_Max;//允许通过增磁达到的给定值最大值
float Ref_Min;//允许通过减磁达到的给定值最小值
float Alpha_E;	//在线控制时计算得到的控制角 
float PID_X[2];
float PID_T[4];
float PID_K;

short int Syn_T[3];	//同步偏差时刻
short int Ifd_Index;//Ifd算法序号
float T_Ifd;//Ifd计算的积分时间常数
unsigned char Scr_Control_Enabled_Flag;//独立运行标志
unsigned char Alpha_Ifdav_Enabled_Flag;//初始均流角允许标志
unsigned char Scr_Ifdav_Enabled_Flag;//均流控制允许标志
unsigned char ROM_Flag;	//从ROM读取的数据已通过CRC校验

float Ifd_Const;//功率柜单柜输出电流校正系数 
float Vfd_Const;//功率柜单柜输出电压校正系数 
float V_Const;	//功率柜阳极电压校正系数 
float I_Const[3];//相电流校正系数
int AI_Offset[8];//8个AI采样零飘校正 

//float Ifdn;//励磁电流额定值 
unsigned char CT_n;	//功率柜CT变比 
//float Vanode_n;	//阳极电压额定值 
unsigned char V_Range;	//功率柜电压量程 
float Alpha_H;	//补偿硬件相移的补偿角 

unsigned char Scr_Control_Enabled_Flag;//独立运行允许标志
unsigned char Alpha_Ifdav_Enabled_Flag;//初始均流角允许标志
unsigned char Scr_Ifdav_Enabled_Flag;//均流控制允许标志
unsigned char ROM_Flag;	//从ROM读取的数据已通过CRC校验

unsigned char Scr_Online_Flag;//功率柜备选标志：独立运行时优先获取控制权
unsigned char Bak_Online_Flag;//其余功率柜备选标志
unsigned char Scr_Control_Flag;//功率柜独立运行标志
unsigned char Scr_Switch_Flag;//功率柜备选切换标志
int Scr_Switch_Delay;	//功率柜备选切换延时
int Scr_Offline_Delay;		//功率柜退出在线控制的延时
int Scr_Online_Delay;		//功率柜设置在线控制的延时
unsigned char Control_Source[7];//控制信号源, 0: Ch1, 1:Ch2, 2:Scr1, 3:Scr2, 4:Scr3, 5:备用, 6:备用 
unsigned char Control_Switch_Flag;//控制源切换标志
short int nScrNum;	//功率柜数量

float PID_X[2];
float PID_T[4];
float PID_K;
float Van_Vf0;//1.35*Vanode/(0.9*Vf0)

unsigned char Online_Flag;//在线控制标志 
unsigned char Online_Old_Flag;//此前本柜的在线控制标志 
unsigned char Standby_Flag;//热备标志
unsigned char Bak_Standby_Flag;//其余功率柜处于热备状态   
int Standby_Offline_Delay;		//功率柜退出备选的延时
int Standby_Online_Delay;		//功率柜设置备选的延时

//start & stop
unsigned char Start_Ex_Flag;//起励标志
unsigned char Start_Count;//起励令自保持时间，保证有电气制动时可以保持起励令到所有开关动作完毕
int Stop_Delay_Count;//逆变完成后(If<0.02)延时进入停机等待
int Init_Ex_Time_Count;//初励计时器（投初励不得超过10s）
float Soft_Ref;//软起励控制的给定值
unsigned char Stop_Machine_Flag;//内部停机令
unsigned char Auto_Ex_Fail_Flag;

//均流 
short int eIfd_Count[6];//6个可控硅均流调相幅值 

//控制方式
unsigned char AVR_Flag;//电压方式 
unsigned char FCR_Flag;//电流方式 
unsigned char AER_Flag;//励磁电压方式 

unsigned char Breaker_Flag;//断路器令 
unsigned char Breaker_Old_Flag;
unsigned char Stop_Machine_Flag;//逆变令 

//限制 
int PT_Break_Count;
unsigned char PT_Break_Flag;//阳极PT断线 
int PT_Online_Delay;
unsigned char PT_Switch_Flag;
unsigned char Load_Limit_Flag;//功率柜退出导致负荷限制 
unsigned char Over_Ex_Flag;//强励限制 
int Over_Ex_Count;
float Over_Ex_Ifd_Ref;
int Scr_Exit_Delay;

float Ifd_Max_Limit;


float Ifd_Sum;
float Ifd_Sum_av;
short int Stop_Count;
float eIfd_Ref;//均流给定值 

unsigned char Stop_Command;//开入的逆变令  
unsigned char Stop_Command_Old;//开入的逆变令  

unsigned char Pulse_Low_F_Flag;//驱动脉冲的同步信号消失 
volatile int Pulse_Low_F_Count;//延时计数器 
unsigned char Scr_Pulse_Flag[5];//源于此功率柜同步信号消失 
int Scr_Pulse_Count[5];//功率柜同步信号消失的自恢复延时 

unsigned char AsynTxData[148];
unsigned char AsynTxBuf[148];
unsigned char AsynRxData[20];
unsigned char Asyn_Tx_Flag;
unsigned char Asyn_Buf_Flag;
int Status_Delay;//状态刷新延时 

unsigned char EE_Data[256];

unsigned char UI_Ref_Up_Flag; //界面增磁令 
unsigned char UI_Ref_Down_Flag;//界面减磁令 
unsigned char UI_Stop_Flag;//界面逆变令 

int Net_Monitor_C;//在线监测通信延时 
unsigned char Net_Monitor_Index;//在线监测通信数据区索引 
int ms_C;//毫秒 

unsigned char ucMornitor[2][8192];//在线监测数据区 
unsigned char ucQTx[128]; //功率变送器通信的发送数据区 
unsigned char Monitor_Upload_Flag;//在线监测数据上传标志 
unsigned char Sensor_ID[15][4];//温度传感器ID 
unsigned char Upload_Sensor_ID_Flag;//上传温度传感器ID标志 
unsigned char Sensor_ID_Des_Ch;//温度传感器ID上传到哪个控制通道 0-ch1, 1-ch2 
float Tarm[15];//可控硅温度 
int T_c[15];//通信延时计数器 
unsigned char T_Flag[15];//温度传感器通信故障标志 


int Syn_Switch_Delay;//Switching synchronous source is forbidden in delay 

unsigned char t_Op[9];//来自光纤的B吗对时数据 

unsigned char Arm_Fuse_Flag[6];
int Arm_Fuse_Delay[6];

unsigned char Pulse_Power_Flag;//脉放电源故障 
unsigned char Alpha_Mode_Flag; //调节柜控制角方式 

float Alpha_C;//同步补偿角 
float Scr_Alpha_C[5];//各功率柜柜同步补偿角 

float Iout;//不经过中值滤波的单柜输出电流瞬时值 

//2020-05-29
volatile unsigned int Sec10Start = 0, Sec10Elapse = 0;
volatile unsigned char Flag10Sec = 0;
volatile float IfdSumAvag = 0.0f;
//10/2500=0.004
volatile float AlphaAddStep = 0.05, AlphaAddValue = 0.00, AlphaAddMax = 0.5, IfdDiffThreshold = 0.005;
volatile float IfdDifference = 0.0f;
volatile signed char FlagDirectionCnt = 0;
//



//===========================================================================
// End of file.
//===========================================================================

