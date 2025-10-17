/********************************************************************
 *ref.c                                                              *
 *                                                                   *
 *Increase/Decrease excitation, stop excitation, reactive current    *
 *compensation.                                                      *
 *������, ��估���                                               *
 *Copyright: 2002-2018 Nengshida Electric Ltd.                       *
 *Author: Yu Xiang                                                   *
 *Jan.15, 2018                                                       *
 ********************************************************************/

#include <math.h>

#include "pub.h"

// #define PID_h (float)0.0005
#define Stop_Machine_Ifd 0.02f

/********************************************************************
 *Ref_Step_Up                                                        *
 *add ref value step by step                                         *
 *����                                                               *
 ********************************************************************/
void Ref_Step_Up(void)
{
    if ((Ref_Up_Flag && Online_Flag) || UI_Ref_Up_Flag) {
        UI_Ref_Up_Flag = 0;

        if (Load_Limit_Flag == 0) {
            Ref_Step_Up_Adder += Ref_Step;
            if (Ref_Step_Up_Adder < Ref_Step_Limit) {
                if (AER_Flag || FCR_Flag) {
                    Ref += Ref_Step;

                    if (Ref_Ctrl > Ref_Max) {
                        Ref = Ref_Max;
                    }
                    Ref_Ctrl = Ref;
                }
                else  // ��ѹ��ʽ
                {
                    Ref += Ref_Step;
                    if (Ref > Ref_Max) {
                        Ref = Ref_Max;
                    }

                    if (Breaker_Flag) {
                        Ref_Ctrl =
                            Ref - Kvrefq * Qav;  // ���ݸ���ֵ�����޵���ʱ�ĸ���ֵ
                        if (Ref_Ctrl > Ref_Max) {
                            Ref_Ctrl = Ref_Max;
                        }
                    }
                    else {
                        Ref_Ctrl = Ref;
                    }
                }
            }
            else {
                Ref_Step_Up_Adder = Ref_Step_Limit;
            }  // ������������δ������
        }  // ����������������
    }  // ��������
    else {
        Ref_Step_Up_Adder = 0;
    }
}

/********************************************************************
 *Ref_Step_Down                                                      *
 *sub ref value step by step                                         *
 *����                                                               *
 ********************************************************************/
void Ref_Step_Down(void)
{
    if (Ref_Down_Flag || UI_Ref_Down_Flag) {
        UI_Ref_Down_Flag = 0;

        Ref_Step_Down_Adder += Ref_Step;
        if (Ref_Step_Down_Adder < Ref_Step_Limit) {
            if (FCR_Flag || AER_Flag) {
                Ref -= Ref_Step;

                if (Ref_Ctrl < Ref_Min) {
                    Ref = Ref_Min;
                }
                Ref_Ctrl = Ref;
            }
            else  // ��ѹ��ʽ
            {
                Ref -= Ref_Step;
                if (Ref < Ref_Min) {
                    Ref = Ref_Min;
                }

                if (Breaker_Flag) {
                    Ref_Ctrl = Ref - Kvrefq * Qav;  // ���ݸ���ֵ�����޵���ʱ�ĸ���ֵ
                    if (Ref_Ctrl < Ref_Min) {
                        Ref_Ctrl = Ref_Min;
                    }
                }
                else {
                    Ref_Ctrl = Ref;
                }
            }
        }
        else  // Ref_Step_Down_Adder>Ref_Step_Limit
        {
            Ref_Step_Down_Adder = Ref_Step_Limit;
        }
    }
    else {
        Ref_Step_Down_Adder = 0;
    }
}

/********************************************************************
 * tracking between mode v, q & ifd                                  *
 * channel online --> out                                            *
 * channel not online--> ref tracking                                *
 *����ͨ����ֹ����ֵԽ�ޡ�                                           *
 ********************************************************************/
void Ref_Tracking(void)
{
    if (Online_Flag == 0 && PT_Break_Flag == 0 && Pulse_Enabled_Flag) {
        //		if (AVR_Flag)
        //		{
        Ref_Ctrl = Vav;
        Ref = Ref_Ctrl;
        //			if (Breaker_Flag)
        //			{
        //				Ref+=Qav*Kvrefq;	//Ref=Ref_Ctrl+Qav*Kvrefq
        //			}
        //			else
        //			{
        Ref = Vav;
        //			}
        //		}
        //		else if (FCR_Flag)
        //		{
        //			Ref_Ctrl=Ifd_av;
        //			Ref=Ref_Ctrl;
        //		}
        //		else if (AER_Flag)
        //		{
        //			Ref_Ctrl=Vfd_av;
        //			Ref=Ref_Ctrl;
        //		}
    }
}

/********************************************************************
 *V_Ref_Q                                                            *
 *1. adjust ref_ctrl according to Q                                  *
 *2. if Vs start, tracking Vs                                        *
 *3. if main ex absent,set alpha and ref                             *
 *�����������                                                     *
 ********************************************************************/
void V_Ref_Q(void)
{
    if (AVR_Flag && Online_Flag && Breaker_Flag) {
        Ref_Ctrl = Ref - Qav * Kvrefq;
    }
    else {
        Ref_Ctrl = Ref;
    }  // ��ѹ��ʽ����
}

/********************************************************************
 *Stop_Machine_Check                                                 *
 *Check and respond stop machine command and startup Ex command      *
 ********************************************************************/
void Stop_Machine_Check(void)
{
    if ((Stop_Command || UI_Stop_Flag) && Online_Flag && Pulse_Enabled_Flag && (Stop_Machine_Flag == 0))  // ������Ӧ
    {
        Stop_Machine_Flag = 1;
        Stop_Count = 0;
        UI_Stop_Flag = 0;
    }
    //	else
    //	{
    //		Stop_Machine_Flag=0;
    //	}

    if (Online_Flag && Stop_Machine_Flag) {
        if (Ifd_Sum < Stop_Machine_Ifd)  // If<0.02
        {
            if (++Stop_Delay_Count >= 1500) {
                Stop_Machine_Flag = 0;  // 2025.01.12�����޸���ͣ�����bug
                Pulse_Enabled_Flag = 0;
                Stop_Delay_Count = 0;
            }
        }
        else  // If>0.02
        {
            Stop_Delay_Count = 0;
        }

        if (++Stop_Count > 20000)  // �����ʱ20s��ɣ���ʱ������
        {
            Stop_Machine_Flag = 0;  // 2025.01.12�����޸���ͣ�����bug
            Pulse_Enabled_Flag = 0;
            Stop_Count = 0;
        }
    }
}

/********************************************************************
 *Ref_COM                                                            *
 *�����š�������                                                 *
 ********************************************************************/
void Ref_Com(void)
{
    Ref_Step_Down();
    Ref_Step_Up();
    //	V_Ref_Q();
    Ref_Tracking();
    Stop_Machine_Check();
}

//===========================================================================
// End of file.
//===========================================================================
