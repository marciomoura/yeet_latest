/********************************************************************
*pub.h																*
*全局变量声明。														*
********************************************************************/ 


#define Alpha_Ref_Max 120			//控制角减磁下限
#define Alpha_Ref_Min 15			//控制角增磁上限

extern short int Online_Delay;	//连续收到非在线数据的记次，防止两个通道均不在线时间过长

extern unsigned char DO_Lock_Flag;//上电开出闭锁标志
extern unsigned short int DO_Lock_Delay;//上电闭锁延时

extern unsigned char Current_Pulse;//当前脉冲，根据当前判断导通的桥臂，对桥臂电流赋值

extern unsigned char Scr_Fan_Err_Flag;//功率柜风机停运标志
extern unsigned char Scr_Exit_Flag;

extern unsigned char SynSource;//所选脉冲信号同步源: 高四位是功率柜拨码, 第四位是同步相(A:1, B:2, C:3) 

extern float Ifd;//励磁电流
extern float Vfd;//励磁电压
extern float Alpha;//控制角
extern float F;//频率 
extern float Q;//无功功率 
extern float P;//有功功率
extern unsigned long Period;//周期 
extern unsigned long Period_Source[5];//待滤波的周期测量值 
extern float Ifd_av;//励磁电流平均值 
extern float Vfd_av;//励磁电压平均值 
extern float Qav;
extern float Pav;
extern float Fav;//频率平均值，用于伏赫限制等
extern long Period_av;//周期平均值

extern float Avg_Ctrl_Coef;//Vfd, F, P的积分系数
extern float Avg_Coef;		//V, I, Ifd, Q的积分系数 
extern float Kvrefq;

//extern unsigned char Write_ROM_Flag;	//参数固化标志
extern unsigned Over_Temperature_Flag;	//过热标志
//*****************************************************************************************************************

extern int Syn_F_Delay;	//出现同步信号后到开始频率计算的延迟次数

extern unsigned char Low_F_Flag;	//低频标志
extern int Low_F_Count;				//低频计数器
extern int Low_F_Trig_Delay;		//低频时替代同步触发通信测量的主循环触发延时
extern unsigned int Syn_A_Int_Time;	//A相中断时刻
extern unsigned int Syn_B_Int_Time;	//B相中断时刻
extern unsigned int Syn_C_Int_Time;	//C相中断时刻 
extern unsigned int Syn_P_Int_Time;  //脉冲驱动相中断时刻 
extern unsigned int Syn_AB_Time;	//AB相中断时刻差
extern unsigned int Syn_BC_Time;	//BC相中断时刻差
extern unsigned int Syn_CA_Time;	//CA相中断时刻差
extern unsigned int Syn_Int_Time;	//当前工作相中断时刻

extern unsigned char Syn_A_Work_Flag;	//A相同步工作标志
extern unsigned char Syn_B_Work_Flag;	//B相同步工作标志
extern unsigned char Syn_C_Work_Flag;	//C相同步工作标志
extern unsigned char Syn_A_Err_Flag;	//A相同步原方断线标志
extern unsigned char Syn_B_Err_Flag;	//B相同步原方断线标志
extern unsigned char Syn_C_Err_Flag;	//C相同步原方断线标志
extern unsigned char Sec_A_Flag;	//A相同步副方断线标志
extern unsigned char Sec_B_Flag;	//B相同步副方断线标志
extern unsigned char Sec_C_Flag;	//C相同步副方断线标志
extern unsigned char Syn_Error_Flag;		//同步故障标志 

extern int Sec_A_Count;	//A相同步副方延迟计数器
extern int Sec_B_Count;	//B相同步副方延迟计数器
extern int Sec_C_Count;	//C相同步副方延迟计数器
extern int Syn_A_Count;	//A相同步原方恢复延迟计数器
extern int Syn_B_Count;	//B相同步原方恢复延迟计数器
extern int Syn_C_Count;	//C相同步原方恢复延迟计数器 
extern int Syn_Pulse_Count;//同步驱动相恢复延时计数器 

extern unsigned char DI_Data[8];
extern unsigned char DO_Data[8];
extern unsigned int Running_Count;		//运行指示灯闪烁计数器，10ms计数1次
extern unsigned int Running_Flag;	//运行指示灯状态，1：亮，2：灭


//ad.c 
extern float Vfd_AD_Const;	//励磁电压校正系数
extern float V_AD_Const;	//阳极电压标幺化系数
extern float I_AD_Const;	//阳极电流标幺化系数 
extern float Vfd_n;//励磁电压额定值
extern float V_n;//阳极电压额定值 
extern float I_n;//电流额定值 
extern float S_n;//视在功率额定值 


extern unsigned char AD_Err_Flag;		//AD芯片故障
extern unsigned char SCR_Error_Flag;	//功率柜故障标志
// extern unsigned char Comm_Err_Flag[2];	//通信失败标志
// extern unsigned char Net_Tx_Err_Flag[2];//网络发送超时标志 
// extern unsigned char Net_Rx_Err_Flag[2];//网络接收超时标志

extern unsigned char Fuse_Flag;			//快熔熔断标志
extern unsigned char Fuse_A_Flag;		//快熔A+熔断标志
extern unsigned char Fuse_B_Flag;		//快熔B+熔断标志
extern unsigned char Fuse_C_Flag;		//快熔C+熔断标志
extern unsigned char Fuse_An_Flag;		//快熔A-熔断标志
extern unsigned char Fuse_Bn_Flag;		//快熔B-熔断标志
extern unsigned char Fuse_Cn_Flag;		//快熔C-熔断标志

extern unsigned char Phase_A_Flag;	//A相桥臂短路标志
extern unsigned char Phase_B_Flag;	//B相桥臂短路标志
extern unsigned char Phase_C_Flag;	//C相桥臂短路标志

extern short int Phase_A_Delay;	//A相桥臂短路延时 
extern short int Phase_B_Delay;	//B相桥臂短路延时
extern short int Phase_C_Delay;	//C相桥臂短路延时

extern unsigned char Fan_Flag;			//风机已投入标志
extern unsigned char Fan_On_Flag;		//风机投入命令

//临时数组：保留上三次采样结果，滤波用
extern float Vfd_Data[3];	//励磁电压	
extern float Ifd_Data[3];	//励磁电流

extern unsigned short int Pulse_Pointer;//脉冲序号
extern unsigned short int Phase_Offset;//同步信号引起的脉冲序号偏差补偿
extern unsigned int Rising_Edge_Flag;	//脉冲上升沿标志
extern unsigned char Pulse_Enabled_Flag;//脉冲允许标志

extern unsigned char chSCR[76];		//功率柜、灭磁柜数据
extern unsigned char chEx[2][76];		//来自控制通道的数据

extern unsigned char Pulse_Test_Flag;//脉冲测试标志
extern unsigned char Test_Pulse;		//待测试的脉冲

extern float Va;	//a相阳极电压
extern float Vb;	//b相阳极电压
extern float Vc;	//c相阳极电压
extern float Vabc;	//三相阳极电压平均值
extern float Vav;	//阳极电压积分值

extern float Ia;	//a相阳极电流
extern float Ib;	//b相阳极电流
extern float Ic;	//c相阳极电流
extern float Iabc;	//三相阳极电流平均值
extern float Iav;	//阳极电流积分值

extern float Iap;	//可控硅A+桥臂电流
extern float Ibp;	//可控硅B+桥臂电流
extern float Icp;	//可控硅C+桥臂电流
extern float Ian;	//可控硅A-桥臂电流
extern float Ibn;	//可控硅B-桥臂电流
extern float Icn;	//可控硅C-桥臂电流
extern float Iarm[6];//滤波后的桥臂电流 

//extern float Tap;	//可控硅A+温度
//extern float Tbp;	//可控硅B+温度
//extern float Tcp;	//可控硅C+温度
//extern float Tan;	//可控硅A-温度
//extern float Tbn;	//可控硅B-温度
//extern float Tcn;	//可控硅C-温度
//extern float Tscr;	//柜内温度
//extern float Tin;	//进风口温度
//extern float Tout;  //出风口温度

extern float Vabc_Temp[2];//用于中值滤波的临时变量
extern float Iabc_Temp[2];
extern float Vfd_Temp[2];
extern float Ifd_Temp[2];

extern unsigned char uchNodeNum;//拨码数值

extern unsigned char Scr_FCR_Flag;//发脉冲时与调节柜通信中断时独立运行标志
extern unsigned char AI_Flag;//脉冲下降沿采样完成后通知主循环计算采样结果
extern unsigned char Ref_Up_Flag;//调节柜增磁令
extern unsigned char Ref_Down_Flag;//调节柜减磁令
extern unsigned char DeEx_Flag;//调节柜逆变令
extern float Ref;//给定值 
extern float Ref_Ctrl;//调差后的给定值 
extern float Ref_Step_Up_Adder;//连续增磁
extern float Ref_Step_Down_Adder;//连续减磁 
extern float Ref_Step;//给定值步长 
extern float Ref_Step_Limit;//连续增减磁时允许调节的给定值幅值
extern float Ref_Max;//允许通过增磁达到的给定值最大值
extern float Ref_Min;//允许通过减磁达到的给定值最小值

extern float Alpha_E;	//在线控制时计算得到的控制角 
extern float PID_X[2];
extern float PID_T[4];
extern float PID_K;

extern short int Syn_T[3];	//同步偏差时刻
extern short int Ifd_Index;//Ifd算法序号
extern float T_Ifd;//Ifd计算的积分时间常数
extern unsigned char Scr_Control_Enabled_Flag;//独立运行标志
extern unsigned char Alpha_Ifdav_Enabled_Flag;//初始均流角允许标志
extern unsigned char Scr_Ifdav_Enabled_Flag;//均流控制允许标志
extern unsigned char ROM_Flag;	//从ROM读取的数据已通过CRC校验

extern float Ifd_Const;//功率柜单柜输出电流校正系数  
extern float Vfd_Const;//功率柜单柜输出电压校正系数 
extern float V_Const;	//功率柜阳极电压校正系数 
extern float I_Const[3];//相电流校正系数
extern int AI_Offset[8];//8个AI采样零飘校正 

//extern float Ifdn;//励磁电流额定值 
extern unsigned char CT_n;	//功率柜CT变比 
//extern float Vanode_n;	//阳极电压额定值 
extern unsigned char V_Range;	//功率柜电压量程 
extern float Alpha_H;	//补偿硬件相移的补偿角 

extern unsigned char Scr_Control_Enabled_Flag;//独立运行允许标志
extern unsigned char Alpha_Ifdav_Enabled_Flag;//初始均流角允许标志
extern unsigned char Scr_Ifdav_Enabled_Flag;//均流控制允许标志
extern unsigned char ROM_Flag;	//从ROM读取的数据已通过CRC校验

extern unsigned char Scr_Online_Flag;//功率柜备选标志：独立运行时优先获取控制权
extern unsigned char Bak_Online_Flag;//其余功率柜备选标志
extern unsigned char Scr_Control_Flag;//功率柜独立运行标志
extern unsigned char Scr_Switch_Flag;//功率柜备选切换标志
extern int Scr_Switch_Delay;	//功率柜备选切换延时
extern int Scr_Offline_Delay;		//功率柜退出备选的延时
extern int Scr_Online_Delay;		//功率柜设置备选的延时
extern unsigned char Control_Source[7];//控制信号源, 0: Ch1, 1:Ch2, 2:Scr1, 3:Scr2, 4:Scr3, 5:备用, 6:备用 
extern unsigned char Control_Switch_Flag;//控制源切换标志
extern short int nScrNum;	//功率柜数量

extern float PID_X[2];
extern float PID_T[4];
extern float PID_K;
extern float Van_Vf0;//1.35*Vanode/(0.9*Vf0)

extern unsigned char Online_Flag;//在线控制标志
extern unsigned char Online_Old_Flag;//此前本柜的在线控制标志 
extern unsigned char Standby_Flag;//热备标志
extern unsigned char Bak_Standby_Flag;//其余功率柜处于热备状态   
extern int Standby_Offline_Delay;		//功率柜退出备选的延时
extern int Standby_Online_Delay;		//功率柜设置备选的延时

//start & stop
extern unsigned char Start_Ex_Flag;//起励标志
extern unsigned char Start_Count;//起励令自保持时间，保证有电气制动时可以保持起励令到所有开关动作完毕
extern int Stop_Delay_Count;//逆变完成后(If<0.02)延时进入停机等待
extern int Init_Ex_Time_Count;//初励计时器（投初励不得超过10s）
extern float Soft_Ref;//软起励控制的给定值
extern unsigned char Stop_Machine_Flag;//内部停机令
extern unsigned char Auto_Ex_Fail_Flag;//起励失败 

//均流 
extern short int eIfd_Count[6];//6个可控硅均流调相幅值 

//控制方式
extern unsigned char AVR_Flag;//电压方式 
extern unsigned char FCR_Flag;//电流方式 
extern unsigned char AER_Flag;//励磁电压方式 

extern unsigned char Breaker_Flag;//断路器令 
extern unsigned char Stop_Machine_Flag;//逆变令 

//限制 
extern int PT_Break_Count;
extern unsigned char PT_Break_Flag;//阳极PT断线 
extern int PT_Online_Delay;
extern unsigned char PT_Switch_Flag;
extern unsigned char Load_Limit_Flag;//功率柜退出导致负荷限制 
extern unsigned char Over_Ex_Flag;//强励限制 
extern int Over_Ex_Count;
extern float Over_Ex_Ifd_Ref;
extern int Scr_Exit_Delay;

extern float Ifd_Max_Limit; 

extern float Ifd_Sum;
extern float Ifd_Sum_av;
extern short int Stop_Count;
extern float eIfd_Ref;//均流给定值 

extern unsigned char Stop_Command;//开入的逆变令  
extern unsigned char Stop_Command_Old;//开入的逆变令  

extern unsigned char Pulse_Low_F_Flag;//驱动脉冲的同步信号消失 
extern volatile int Pulse_Low_F_Count;//延时计数器 
extern unsigned char Scr_Pulse_Flag[5];//源于此功率柜同步信号消失 
extern int Scr_Pulse_Count[5];//功率柜同步信号消失的自恢复延时 

extern unsigned char AsynTxData[148];
extern unsigned char AsynTxBuf[148];
extern unsigned char AsynRxData[20];
extern unsigned char Asyn_Tx_Flag;
extern unsigned char Asyn_Buf_Flag;
extern int Status_Delay;//状态刷新延时 

extern unsigned char EE_Data[256]; 

extern unsigned char UI_Ref_Up_Flag; //界面增磁令 
extern unsigned char UI_Ref_Down_Flag;//界面减磁令 
extern unsigned char UI_Stop_Flag;//界面逆变令 

extern int Net_Monitor_C;//在线监测通信延时 
extern unsigned char Net_Monitor_Index;//在线监测通信数据区索引 
extern int ms_C;//毫秒 

extern unsigned char ucMornitor[2][8192];//在线监测数据区 
extern unsigned char ucQTx[128]; //功率变送器通信的发送数据区 
extern unsigned char Monitor_Upload_Flag;//在线监测数据上传标志 
extern unsigned char Sensor_ID[15][4];//温度传感器ID
extern unsigned char Upload_Sensor_ID_Flag;//上传温度传感器ID标志 
extern unsigned char Sensor_ID_Des_Ch;//温度传感器ID上传到哪个控制通道 0-ch1, 1-ch2
extern float Tarm[15];//可控硅温度 
extern int T_c[15];//通信延时计数器 
extern unsigned char T_Flag[15];//温度传感器通信故障标志 


extern int Syn_Switch_Delay;//Switching synchronous source is forbidden in delay

extern unsigned char t_Op[9];//来自光纤的B吗对时数据 

extern unsigned char Arm_Fuse_Flag[6];
extern int Arm_Fuse_Delay[6];

extern unsigned char Pulse_Power_Flag;//脉放电源故障 
extern unsigned char Alpha_Mode_Flag; //调节柜控制角方式 

extern float Alpha_C;//同步补偿角 
extern float Scr_Alpha_C[5];//各功率柜柜同步补偿角 

extern float Iout;//不经过中值滤波的单柜输出电流瞬时值 

//2020-05-29
extern unsigned int Sec10Start, Sec10Elapse;
extern unsigned char Flag10Sec;
extern float IfdSumAvag;
extern float AlphaAddStep, AlphaAddValue;//, AlphaAddMax, IfdDiffThreshold;
extern float IfdDifference;
//

//===========================================================================
// End of file.
//===========================================================================
