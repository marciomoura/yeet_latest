/************************************************************************
 *DIDO                                                                   *
 *�����������������������ռ���                                           *
 ************************************************************************/
#include <math.h>
#include <stm32f4xx.h>

#include "GPIO.h"
#include "pub.h"

#define Period_25Hz 3360000  // 84*40*1000

extern void Set_Alpha_Delay(void);
extern void Syn_Pulse_Init(void);
extern unsigned short int Optical_Comm_Status[8];  // ��ǰͨ��״̬, 0: �ж� 1: �������� 3:�շ�����
extern unsigned char Optical_Comm_Err_Flag;
extern unsigned char Online_Comm_Err_Flag;  // ������ͨ��ʧȥ��ϵ
extern unsigned char Online_Source[8];      // ����ģ�����߱�־,
extern unsigned char SynSource;
extern unsigned char Optical_Tx_Flag;  // ���ͱ�־

/************************************************************************
 *Running_Signal                                                         *
 *����ָʾ�Ƶ���˸��                                                     *
 ************************************************************************/
void Running_Signal(void)
{
    if (++Running_Count >= 100) {
        Running_Count = 0;
        Running_Flag = ~Running_Flag;
    }
}
/************************************************************************
 *SCR_Check                                                              *
 *���ʹ�״̬��顣                                                       *
 ************************************************************************/
void SCR_Check(void)
{
    if ((DO_Lock_Flag == 0 && Vabc > 0.1f) &&  // 2018.3.12.
        //		((Pulse_Enabled_Flag && Low_F_Flag) || Over_Temperature_Flag || Optical_Comm_Err_Flag ||
        //Online_Comm_Err_Flag) )//2019.1.16., ��ͣ����״̬���
        ((Fuse_Flag || (Pulse_Enabled_Flag && Low_F_Flag) || Over_Temperature_Flag || Optical_Comm_Err_Flag ||
             Online_Comm_Err_Flag) ||
            Pulse_Power_Flag))  // 2019.9.3., �������ŵ�Դ���ϱ�־
    {
        SCR_Error_Flag = 1;
    }
    else {
        SCR_Error_Flag = 0;
    }
}

/************************************************************************
 *Breaker_Check                                                          *
 *���ݹ��ʱ��������͵Ĺ��ʼ���·��״̬��                               *
 ************************************************************************/
void Breaker_Check(void)
{
    double S;

    S = sqrt(P * P + Q * Q);

    if (S > 0.11) {
        Breaker_Flag = 1;
    }
    if (S < 0.09) {
        Breaker_Flag = 0;
    }
}

/************************************************************************
 *DO_Lock_Check                                                          *
 *����������⡣                                                         *
 ************************************************************************/
void DO_Lock_Check(void)
{
    if (--DO_Lock_Delay <= 0) {
        DO_Lock_Delay = 0;
        DO_Lock_Flag = 0;
    }
}

/************************************************************************
 *Low_F_Trigger                                                          *
 *��Ƶʱ����ѭ������ͬ���ź�����AI������ͨ�š�                           *
 ************************************************************************/
void Low_F_Trigger(void)
{
    if (Low_F_Flag) {
        if (++Low_F_Trig_Delay >= 40) {
            Low_F_Trig_Delay = 0;
            if (Pulse_Low_F_Flag) {
                Optical_Tx_Flag = 1;
                if (Pulse_Enabled_Flag == 0) {
                    Set_Alpha_Delay();  // ʹ�õ�ǰ���ƽ�
                }
            }
        }
    }
    else {
        Low_F_Trig_Delay = 0;
    }
}

/************************************************************************
 *Low_F_Check                                                            *
 *���Ƶ���Ƿ�ܵͣ�����ѭ����������������ͬ���ж������                 *
 ************************************************************************/
void Low_F_Check(void)
{
    if (++Low_F_Count >= 200) {
        Low_F_Count = 200;
        if (Low_F_Flag == 0) {
            Syn_Pulse_Init();
        }
    }
    else {
        Low_F_Flag = 0;
    }
    Low_F_Trigger();
}

/************************************************************************
 *Refresh_DO_Flag                                                        *
 *                                                                       *
 *ˢ�¿�����־��                                                         *
 ************************************************************************/
void Refresh_DO_Flag(void)
{
    Running_Signal();
    DO_Lock_Check();
    SCR_Check();
    Low_F_Check();
}

/************************************************************************
 *Get_DI1                                                                *
 *ʹ�þ�����ʱ�����Ŀ�����1ˢ�¶�Ӧ�ı�־��                              *
 ************************************************************************/
void Get_DI1(void)
{
    unsigned char DI_t;
    int i;

    //	�����۶�
    //	�¶�Խ��
    //	���Ͷ��
    //	���Ŵ�
    //	�տ�δ��
    //	����
    //	����
    //	���
    //	Fuse_Flag=DI_Delayed_Flag[4][0];
    Fuse_Flag =
        Arm_Fuse_Flag[0] | Arm_Fuse_Flag[1] | Arm_Fuse_Flag[2] | Arm_Fuse_Flag[3] | Arm_Fuse_Flag[4] | Arm_Fuse_Flag[5];
    if (DI_Delayed_Flag[4][1]) {
        Over_Temperature_Flag = 0;
    }
    else {
        Over_Temperature_Flag = 1;
    }

    Fan_Flag = DI_Delayed_Flag[4][2];
    Ref_Up_Flag = 1 - DI_Delayed_Flag[4][5];
    Ref_Down_Flag = 1 - DI_Delayed_Flag[4][6];
    Stop_Command = 1 - DI_Delayed_Flag[4][7];

    //	DI_t=DI_Delayed_Flag[4][0];
    DI_t = Fuse_Flag;
    for (i = 1; i < 8; i++) {
        if (DI_Delayed_Flag[4][i]) {
            DI_t += 1 << i;
        }
    }
    DI_Data[0] = DI_t;
}

/************************************************************************
 *Get_DI2                                                                *
 *ʹ�þ�����ʱ�����Ŀ�����2ˢ�¶�Ӧ�ı�־��                              *
 ************************************************************************/
void Get_DI2(void)
{
    unsigned char DI_t;
    int i;

    // ��������
    // ��·����
    // ���ŵ�Դ����
    // ����
    // ����
    // ����
    // ����
    // ����
    // Breaker_Flag=DI_Delayed_Flag[5][1];
    Breaker_Check();
    Pulse_Power_Flag = DI_Delayed_Flag[5][2];
    // DI_t=DI_Delayed_Flag[5][0];
    DI_t = Pulse_Enabled_Flag;
    for (i = 1; i < 8; i++) {
        if (DI_Delayed_Flag[5][i]) {
            DI_t += 1 << i;
        }
    }

    DI_Data[1] = DI_t;
}

/************************************************************************
 *DI                                                                     *
 *ʹ�þ�����ʱ�����Ŀ�����ˢ�¶�Ӧ�ı�־��                               *
 ************************************************************************/
void DI(void)
{
    Refresh_DI();
    Refresh_Delayed_DI();
    Get_DI1();
    Get_DI2();
}

/************************************************************************
 *DO1                                                                    *
 *                                                                       *
 *������һ�������ֽڡ�                                                   *
 ************************************************************************/
void DO1(void)
{
    unsigned char DO_Temp;

    // ���ʹ����
    // �����۶�
    // ���Ͷ��
    // A��ͬ������
    // B��ͬ������
    // C��ͬ������
    // ROMд��ɹ� (�ڹ���ͨ������λ)
    // ROMд��ʧ��
    DO_Temp = SCR_Error_Flag;  // �ռ�DO���ݹ�ͨ��ʹ��

    if (Fuse_Flag) {
        DO_Temp |= 2;
    }

    Fan_On_Flag = Pulse_Enabled_Flag;
    //	Fan_On_Flag=Over_Temperature_Flag;
    if (Fan_On_Flag) {
        DO_Temp |= 4;
    }

    if (Sec_A_Flag || Syn_A_Err_Flag) {
        DO_Temp |= 8;
    }

    if (Sec_B_Flag || Syn_B_Err_Flag) {
        DO_Temp |= 16;
    }

    if (Sec_C_Flag || Syn_C_Err_Flag) {
        DO_Temp |= 32;
    }

    // 	if (Syn_A_Work_Flag)
    // 	{
    // 		DO_Temp|=64;
    // 	}

    // 	if (Syn_B_Work_Flag)
    // 	{
    // 		DO_Temp|=128;
    // 	}
    DO_Data[0] = DO_Temp;

    // ���ڲ���־ӳ�䵽���
    DO_Flag[0][0] = SCR_Error_Flag;
    DO_Flag[0][1] = Fuse_Flag;
    DO_Flag[0][2] = Fan_On_Flag;
    DO_Flag[0][3] = Sec_A_Flag || Syn_A_Err_Flag;
    DO_Flag[0][4] = Sec_B_Flag || Syn_B_Err_Flag;
    DO_Flag[0][5] = Sec_C_Flag || Syn_C_Err_Flag;
    DO_Flag[0][6] = 0;
    DO_Flag[0][7] = 0;

    // ��չ����
    DO_Flag[4][0] = DO_Flag[0][0];
    DO_Flag[4][1] = DO_Flag[0][1];
    DO_Flag[4][2] = DO_Flag[0][2];
    DO_Flag[4][3] = DO_Flag[0][3];
    DO_Flag[4][4] = DO_Flag[0][4];
    DO_Flag[4][5] = DO_Flag[0][5];
    DO_Flag[4][6] = DO_Flag[0][6];
    DO_Flag[4][7] = DO_Flag[0][7];
}

/************************************************************************
 *DO2                                                                    *
 *                                                                       *
 *�����ڶ��������ֽڡ�                                                   *
 ************************************************************************/
void DO2(void)
{
    unsigned char DO_Temp;

    // ROM����
    // �����۶�-C
    // �����۶�+C
    // �����۶�-B
    // �����۶�+B
    // �����۶�-A
    // �����۶�+A
    // ���ʹ��˳�
    DO_Temp = ROM_Flag;  // �ռ�DO���ݹ�ͨ��ʹ��

    //	if (Fuse_Cn_Flag)
    //	{
    //		DO_Temp|=2;
    //	}

    //	if (Fuse_C_Flag)
    //	{
    //		DO_Temp|=4;
    //	}

    //	if (Fuse_Bn_Flag)
    //	{
    //		DO_Temp|=8;
    //	}

    //	if (Fuse_B_Flag)
    //	{
    //		DO_Temp|=16;
    //	}

    //	if (Fuse_An_Flag)
    //	{
    //		DO_Temp|=32;
    //	}

    //	if (Fuse_A_Flag)
    //	{
    //		DO_Temp|=64;
    //	}

    //	if (Scr_Exit_Flag)
    //	{
    //		DO_Temp|=128;
    //	}
    //
    //	DO_Data[1]=DO_Temp;

    //	//���ڲ���־ӳ�䵽���
    //	DO_Flag[1][0]=ROM_Flag;
    //	DO_Flag[1][1]=Fuse_Cn_Flag;
    //	DO_Flag[1][2]=Fuse_C_Flag;
    //	DO_Flag[1][3]=Fuse_Bn_Flag;
    //	DO_Flag[1][4]=Fuse_B_Flag;
    //	DO_Flag[1][5]=Fuse_An_Flag;
    //	DO_Flag[1][6]=Fuse_A_Flag;
    //	DO_Flag[1][7]=Scr_Exit_Flag;

    if (Arm_Fuse_Flag[5]) {
        DO_Temp |= 2;
    }

    if (Arm_Fuse_Flag[4]) {
        DO_Temp |= 4;
    }

    if (Arm_Fuse_Flag[3]) {
        DO_Temp |= 8;
    }

    if (Arm_Fuse_Flag[2]) {
        DO_Temp |= 16;
    }

    if (Arm_Fuse_Flag[1]) {
        DO_Temp |= 32;
    }

    if (Arm_Fuse_Flag[0]) {
        DO_Temp |= 64;
    }

    if (Scr_Exit_Flag) {
        DO_Temp |= 128;
    }

    DO_Data[1] = DO_Temp;

    // ���ڲ���־ӳ�䵽���
    DO_Flag[1][0] = ROM_Flag;
    DO_Flag[1][1] = Arm_Fuse_Flag[5];
    DO_Flag[1][2] = Arm_Fuse_Flag[4];
    DO_Flag[1][3] = Arm_Fuse_Flag[3];
    DO_Flag[1][4] = Arm_Fuse_Flag[2];
    DO_Flag[1][5] = Arm_Fuse_Flag[1];
    DO_Flag[1][6] = Arm_Fuse_Flag[0];
    DO_Flag[1][7] = Scr_Exit_Flag;
}

/************************************************************************
 *DO3                                                                    *
 *                                                                       *
 *���������������ֽڡ�                                                   *
 ************************************************************************/
void DO3(void)
{
    unsigned char DO_Temp;
    // ����1���ͳ�ʱ
    // ����1���ճ�ʱ
    // ����2���ͳ�ʱ
    // ����2���ճ�ʱ
    // ��Ƶ
    // ǿ��
    // PT����
    // �ȱ�

    DO_Temp = Optical_Comm_Status[0];  // �ռ�DO���ݹ�ͨ��ʹ��
    DO_Temp |= Optical_Comm_Status[1] << 2;

    if (Low_F_Flag) {
        DO_Temp |= 16;
    }
    if (Over_Ex_Flag) {
        DO_Temp |= 32;
    }
    if (PT_Break_Flag) {
        DO_Temp |= 64;
    }
    if (Standby_Flag) {
        DO_Temp |= 128;
    }

    DO_Data[2] = DO_Temp;

    // ���ڲ���־ӳ�䵽���
    DO_Flag[2][0] = Optical_Comm_Status[0] & 1;
    DO_Flag[2][1] = Optical_Comm_Status[0] >> 1;
    DO_Flag[2][2] = Optical_Comm_Status[1] & 1;
    DO_Flag[2][3] = Optical_Comm_Status[1] >> 1;
    DO_Flag[2][4] = Low_F_Flag;
}

/************************************************************************
 *DO4                                                                    *
 *                                                                       *
 *������4�������ֽڡ�                                                    *
 ************************************************************************/
void DO4(void)
{
    unsigned char DO_Temp = 0;
    int i, j;
    // Ch1����
    // Ch2����
    // Scr1����
    // Scr2����
    // Scr3����
    // Scr4����
    // ����
    // ����

    j = 1;
    if (Online_Flag) {
        j = 1 << (uchNodeNum + 1);
    }
    else {
        for (i = 0; i < 6; i++) {
            if (Online_Source[i]) {
                break;
            }
            j <<= 1;
        }
        if (i == 6) {
            j = 0;
        }
    }
    DO_Temp = j;

    DO_Data[3] = DO_Temp;
}

/************************************************************************
 *DO5                                                                    *
 *                                                                       *
 *������5�������ֽڡ�                                                    *
 ************************************************************************/
void DO5(void)
{
    unsigned char DO_Temp = 0;

    // Scr1����
    // Scr1����
    // Scr2����
    // Scr2����
    // Scr3����
    // Scr3����
    // Scr4����
    // Scr4����
    DO_Temp = Optical_Comm_Status[2];
    DO_Temp += Optical_Comm_Status[3] << 2;
    DO_Temp += Optical_Comm_Status[4] << 4;
    DO_Temp += Optical_Comm_Status[5] << 6;

    DO_Data[4] = DO_Temp;
}

/************************************************************************
 *DO6                                                                    *
 *                                                                       *
 *������6�������ֽڡ�                                                    *
 ************************************************************************/
void DO6(void)
{
    unsigned char DO_Temp = 0;

    // Scr5����
    // Scr5����
    // AVR
    // FCR
    // AER
    // ����
    // ����
    // ����
    DO_Temp = Optical_Comm_Status[6];
    if (AVR_Flag) {
        DO_Temp |= 4;
    }
    if (FCR_Flag) {
        DO_Temp |= 8;
    }
    if (AER_Flag) {
        DO_Temp |= 0x10;
    }

    DO_Data[5] = DO_Temp;
}

/************************************************************************
 *DO8                                                                    *
 *                                                                       *
 *������8�������ֽڡ�                                                    *
 ************************************************************************/
void DO8(void)
{
    unsigned char DO_Temp = 0;
    int i, j;

    //+A fuse broken
    //-A fuse broken
    //+B fuse broken
    //-B fuse broken
    //+C fuse broken
    //-C fuse broken
    // ����
    // ����

    DO_Temp = Arm_Fuse_Flag[0];

    j = 1;
    for (i = 1; i < 6; i++) {
        j <<= 1;
        if (Arm_Fuse_Flag[i]) {
            DO_Temp |= j;
        }
    }
    DO_Data[7] = DO_Temp;
}

/************************************************************************
 *DO                                                                     *
 *���ڲ���־ӳ�䵽���������ռ��������ݡ�                                 *
 ************************************************************************/
void DO(void)
{
    Refresh_DO_Flag();  // ˢ�²��ֿ�����־
    DO1();              // ӳ���ڲ���־������
    DO2();
    DO3();
    DO4();
    DO5();
    DO6();
    DO8();
    DO_Data[6] = SynSource;
    DO_Out();  // ���������
}

/************************************************************************
 *DIDO                                                                   *
 *ˢ��DI��DO��                                                           *
 ************************************************************************/
void DIDO(void)
{
    DI();
    DO();
}
//===========================================================================
// End of file.
//===========================================================================
