/********************************************************************************
 *Para                                                                           *
 *²ÎÊý³õÊ¼»¯Á÷³Ì¡£                                                               *
 ********************************************************************************/
#include <stm32f4xx.h>

#include "AD7606.h"
#include "AI.h"
#include "DIDO.h"
#include "GPIO.h"
#include "M25P32.h"
#include "TIM.h"
#include "WatchDog.h"
#include "command.h"
#include "eIfd.h"
#include "limit.h"
#include "mode.h"
#include "nandflash_sram.h"
#include "net.h"
#include "optical.h"
#include "optical_fiber.h"
#include "pub.h"
#include "syn.h"
#include "temperature.h"
#include "w5200.h"

/********************************************************************
 *intToChar                                                          *
 *½«Ò»¸ö16Î»ÕûÐÎ×ª»»³ÉÁ½¸öÎÞ·ûºÅ×Ö½Ú¡£                               *
 ********************************************************************/
void intToChar(short int n, unsigned char* Data)
{
    *Data++ = (n >> 8) & 0xff;
    *Data = n & 0xff;
}

/********************************************************************
 *charToInt                                                          *
 *½«Á½¸öÎÞ·ûºÅ×Ö½Ú×ª»»³ÉÒ»¸ö16Î»ÕûÐÎ¡£                               *
 ********************************************************************/
short int charToInt(unsigned char* Data)
{
    short int i;
    i = *Data++;
    i <<= 8;
    return i | *Data;
}

/********************************************************************
 *DIP_Switch_Init                                                    *
 *²¦Âë¿ª¹Ø³õÊ¼»¯£¬ÓÃÀ´ÉèÖÃÉè±¸µÄIPµØÖ·¡£                             *
 *Ó²¼þÅäÖÃ£ºPA8¡¢PB2¡¢PF11                                           *
 ********************************************************************/
void DIP_Switch_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // Enable the Clock
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);

    // Configure W5100_INT pin as input
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_Init(GPIOF, &GPIO_InitStructure);
}

/********************************************************************
 *Read_DIP_Switch                                                    *
 *¶ÁÈ¡²¦Âë¿ª¹ØµÄ×´Ì¬Öµ                                               *
 ********************************************************************/
unsigned char Read_DIP_Switch(void)
{
    unsigned char state = 0x00;
    //------²¦Âë¿ª¹Ø3-------
    if (GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_11)) {
        state |= 0x04;
    }

    //------²¦Âë¿ª¹Ø2-------
    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_2)) {
        state |= 0x02;
    }

    //------²¦Âë¿ª¹Ø1-------
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8)) {
        state |= 0x01;
    }

    return state;
}

/********************************************************************
 *Write_ROM                                                          *
 *Write all the control parameters into flash memory                 *
 ********************************************************************/
unsigned char Write_ROM(void)
{
    int i;
    unsigned char *ucD, *ucS;

    Float_To_Byte(PID_T[0], EE_Data);
    Float_To_Byte(PID_T[1], EE_Data + 4);
    Float_To_Byte(PID_T[2], EE_Data + 8);
    Float_To_Byte(PID_T[3], EE_Data + 12);
    Float_To_Byte(PID_K, EE_Data + 16);
    Float_To_Byte(Van_Vf0, EE_Data + 20);
    Float_To_Byte(T_Ifd, EE_Data + 24);
    Float_To_Byte(Kvrefq, EE_Data + 28);
    Float_To_Byte(V_n, EE_Data + 32);
    Float_To_Byte(Vfd_n, EE_Data + 36);
    Float_To_Byte(I_n, EE_Data + 40);
    Float_To_Byte(S_n, EE_Data + 44);
    EE_Data[48] = nScrNum;
    Float_To_Byte(Scr_Alpha_C[uchNodeNum - 1], EE_Data + 52);
    Float_To_Byte(Vfd_Const, EE_Data + 56);
    Float_To_Byte(V_Const, EE_Data + 60);

    for (i = 0; i < 8; i++) {
        intToChar(AI_Offset[i], EE_Data + 64 + i + i);
    }
    ucD = EE_Data + 80;
    ucS = Sensor_ID[0];
    for (i = 0; i < 60; i++) {
        *ucD++ = *ucS++;
    }
    Float_To_Byte(I_Const[0], EE_Data + 140);
    Float_To_Byte(I_Const[1], EE_Data + 144);
    Float_To_Byte(I_Const[2], EE_Data + 148);
    ucD += 4;
    for (i = 152; i < 198; i++) {
        EE_Data[i] = 0;
    }
    i = CRC16(EE_Data, 198);
    intToChar(i, EE_Data + 198);

    return EEROM_Write(0, EE_Data, 200);  // ·µ»ØROMÐ´ÈëµÄ½á¹û
}

/********************************************************************
 *Read_ROM                                                           *
 *Read control parameters from EEROM.                                *
 ********************************************************************/
unsigned char Read_ROM(void)
{
    short int i, j;
    unsigned char Result = 0;
    unsigned char *ucD, *ucS;

    sFLASH_ReadBuffer(EE_Data, 0, 200);
    i = CRC16(EE_Data, 198);
    j = EE_Data[198];
    j = j * 256 + EE_Data[199];
    if (i == j) {
        Byte_To_Float(&PID_T[0], EE_Data);
        Byte_To_Float(&PID_T[1], EE_Data + 4);
        Byte_To_Float(&PID_T[2], EE_Data + 8);
        Byte_To_Float(&PID_T[3], EE_Data + 12);
        Byte_To_Float(&PID_K, EE_Data + 16);
        Byte_To_Float(&Van_Vf0, EE_Data + 20);
        Byte_To_Float(&T_Ifd, EE_Data + 24);
        Byte_To_Float(&Kvrefq, EE_Data + 28);
        Byte_To_Float(&V_n, EE_Data + 32);
        Byte_To_Float(&Vfd_n, EE_Data + 36);
        Byte_To_Float(&I_n, EE_Data + 40);
        Byte_To_Float(&S_n, EE_Data + 44);
        nScrNum = EE_Data[48];
        Byte_To_Float(&Scr_Alpha_C[uchNodeNum - 1], EE_Data + 52);
        Byte_To_Float(&Vfd_Const, EE_Data + 56);
        Byte_To_Float(&V_Const, EE_Data + 60);

        for (i = 0; i < 8; i++) {
            AI_Offset[i] = charToInt(EE_Data + 64 + i + i);
        }

        ucS = EE_Data + 80;
        ucD = Sensor_ID[0];
        for (i = 0; i < 60; i++) {
            *ucD++ = *ucS++;
        }
        Byte_To_Float(&I_Const[0], EE_Data + 140);
        Byte_To_Float(&I_Const[1], EE_Data + 144);
        Byte_To_Float(&I_Const[2], EE_Data + 148);
    }
    else {
        Result = 1;  // Ð£Ñé´íÎóºó²»Ë¢ÐÂ²ÎÊý
    }
    return Result;
}

/********************************************************************
 *Init_Coef                                                          *
 *                                                                   *
 *Initialize coeffcients. Call only once on being electrified.       *
 *ÏµÊý³õÊ¼»¯£¬½öÉÏµçÊ±µ÷ÓÃ¡£                                         *
 ********************************************************************/
void Init_Coef(void)
{
    int i;

    Avg_Coef = 0.003;
    Avg_Ctrl_Coef = 0.003;
    // Ifd_Const=6.43087e-5;//¶î¶¨Àø´ÅµçÁ÷1590A¶ÔÓ¦µÄAD¶ÁÊý15550, 6.43087e-5=1/15550
    // Vfd_AD_Const=1.4337e-4;//¶î¶¨Àø´ÅµçÑ¹461V¶ÔÓ¦ADÊäÈë2.1286V, AD²ÉÑùÊýÖµ6975
    // I_AD_Const=1.85e-4;//¼ÆËã½»Á÷²àÀø´ÅµçÁ÷, 1590AÊäÈë¶ÔÓ¦AIÊäÈëÎª3.55V, AD¶ÁÊýÎª11633, 8.596e-5=1/11633
    // V_AD_Const=7.3719e-5; //400VÏßµçÑ¹/231VÏàµçÑ¹ÊäÈëÊ±ADÐ¾Æ¬ÊäÈë4.14V£¬AD²ÉÑùÖµ0x34FD, 1/0x34fd=7.3719e-5
    V_AD_Const = 1.71e-4;  // ÑÎ¹øÏ¿²ÎÊý
    // I_AD_Const=7.2e-5;//1.168e-4;//»Æ½ðÆº²ÎÊý, ½»Á÷ÐÅºÅÐ£Õý
    I_AD_Const = 2.728e-4;
    Ifd_Const = 1;
    Vfd_Const = 1;
    V_Const = 1;
    Vfd_n = 1;
    V_n = 1;
    I_n = 1;
    S_n = 1;
    I_Const[0] = 1;
    I_Const[1] = 1;
    I_Const[2] = 1;

    // Vfd_AD_Const=3.15414e-4;//»Æ½ðÆº²ÎÊý
    Vfd_AD_Const = 1.577e-4;  // ¸ðÖÞ°Ó²ÎÊý
    CT_n = 2;
    V_Range = 2;
    for (i = 0; i < 5; i++) {
        Scr_Alpha_C[i] = 32;
    }
    Alpha_C = 32;
    //	Ifdn=1900;
    Breaker_Flag = 0;
    Ref_Up_Flag = 0;
    Ref_Down_Flag = 0;
    Stop_Machine_Flag = 0;

    Syn_T[0] = 0;
    Syn_T[1] = 0;
    Syn_T[2] = 0;
    Scr_Control_Enabled_Flag = 0;
    Alpha_Ifdav_Enabled_Flag = 0;
    Scr_Ifdav_Enabled_Flag = 0;
    Alpha_E = 15;
    Ifd_Index = 0;
    T_Ifd = 0;

    // 	V_Range=1;
    // 	CT_n=1;
    //	Ifdn=1900;
    //	Vanode_n=715;

    // 8¸öAI²ÉÑùÁãÆ®Ð£Õý
    AI_Offset[0] = 0;
    AI_Offset[1] = 0;
    AI_Offset[2] = 0;
    AI_Offset[3] = 0;
    AI_Offset[4] = 0;
    AI_Offset[5] = 0;
    AI_Offset[6] = 0;
    AI_Offset[7] = 0;

    // PID
    PID_X[0] = 0;
    PID_X[1] = 0;
    PID_T[0] = (float)1.5;
    PID_T[1] = 15;
    PID_T[2] = 1;
    PID_T[3] = 1;
    PID_K = 500;

    eIfd_Count[0] = 0;
    eIfd_Count[1] = 0;
    eIfd_Count[2] = 0;
    eIfd_Count[3] = 0;
    eIfd_Count[4] = 0;
    eIfd_Count[5] = 0;

    nScrNum = 3;
    Kvrefq = -0.05f;

    for (i = 0; i < 15; i++) {
        Sensor_ID[i][0] = 0;
        Sensor_ID[i][1] = 0;
        Sensor_ID[i][2] = 0;
        Sensor_ID[i][3] = 0;
    }

    ROM_Flag = Read_ROM();  // ¶ÁÈ¡¹Ì»¯²ÎÊý£¬Ê§°ÜÔòÊ¹ÓÃÈ±Ê¡²ÎÊý
}

/********************************************************************
 *Init_DIDO                                                          *
 *                                                                   *
 *Initialize DI/DO. Call only once on being electrified              *
 *¿ª¹ØÁ¿³õÊ¼»¯£¬½öÔÚÉÏµçÊ±µ÷ÓÃ¡£                                     *
 ********************************************************************/
void Init_DIDO(void)
{
    short int i;

    // 	Net_Status[0]=0; //³õÊ¼»¯Îª²»Õý³£
    // 	Net_Status[1]=0;
    // 	Net_Recv_Delay[0]=0;
    // 	Net_Recv_Delay[1]=0;
    // 	Net_Tx_Delay[0]=0;
    // 	Net_Tx_Delay[1]=0;
    // 	Net_Rx_Flag[0]=0;
    // 	Net_Rx_Flag[1]=0;
    // 	Net_Tx_Flag[0]=0;
    // 	Net_Tx_Flag[1]=0;

    DO_Lock_Flag = 1;
    DO_Lock_Delay = 8000;  // ¿ª³ö±ÕËø8Ãë

    AD_Err_Flag = 0;
    SCR_Error_Flag = 0;  // ²»Ó°ÏìÔËÐÐµÄ¹ÊÕÏ£¬ÈçÍ¬²½¶ÏÏß¡¢AD¹ÊÕÏµÈµÈ
    // 	Comm_Err_Flag[0]=0;
    // 	Comm_Err_Flag[1]=0;
    // 	Net_Tx_Err_Flag[0]=0;
    // 	Net_Tx_Err_Flag[1]=0;
    // 	Net_Rx_Err_Flag[0]=0;
    // 	Net_Rx_Err_Flag[1]=0;

    Fuse_Flag = 0;
    Fuse_A_Flag = 0;
    Fuse_B_Flag = 0;
    Fuse_C_Flag = 0;
    Fuse_An_Flag = 0;
    Fuse_Bn_Flag = 0;
    Fuse_Cn_Flag = 0;

    Phase_A_Flag = 0;
    Phase_B_Flag = 0;
    Phase_C_Flag = 0;

    Phase_A_Delay = 0;
    Phase_B_Delay = 0;
    Phase_C_Delay = 0;

    Fan_Flag = 0;
    Fan_On_Flag = 0;
    Alpha_Mode_Flag = 0;

    Pulse_Enabled_Flag = 0;
    Pulse_Test_Flag = 0;
    Pulse_Power_Flag = 0;
    Test_Pulse = 0;  // ´ý²âÊÔµÄÂö³å
    Running_Flag = 0;
    Running_Count = 0;
    Scr_FCR_Flag = 0;  // ·¢Âö³åÊ±Óëµ÷½Ú¹ñÍ¨ÐÅÖÐ¶ÏÊ±¶ÀÁ¢ÔËÐÐ±êÖ¾
    AI_Flag = 0;
    Ref_Up_Flag = 0;
    Ref_Down_Flag = 0;
    DeEx_Flag = 0;
    Ref_Step_Up_Adder = 0;
    Ref_Step_Down_Adder = 0;

    Over_Temperature_Flag = 0;
    Low_F_Flag = 0;
    Low_F_Count = 0;
    Low_F_Trig_Delay = 0;
    Online_Flag = 0;
    Online_Old_Flag = 0;
    Standby_Flag = 0;
    Upload_Sensor_ID_Flag = 0;
    Sensor_ID_Des_Ch = 0;

    for (i = 0; i < 8; i++) {
        DI_Data[i] = 0;
        DO_Data[i] = 0;
    }

    for (i = 0; i < 6; i++) {
        Tarm[i] = 20;
        T_c[i] = 0;
        T_Flag[i] = 0;
    }
}

/********************************************************************
 *Init_RTC                                                           *
 *ARMÆ¬ÉÏÊµÊ±Ê±ÖÓ³õÊ¼»¯£¬½öÔÚÉÏµçÊ±µ÷ÓÃ¡£                            *
 ********************************************************************/
void Init_RTC(void)
{
    int i = 0;
    RTC_InitTypeDef RTC_Init_Para;
    RTC_TimeTypeDef t;
    RTC_DateTypeDef date;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
    PWR_BackupAccessCmd(ENABLE);
    RCC_LSEConfig(RCC_LSE_ON);
    while ((RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET) && ++i < 100000) {
        Soft_Delay(100000);
    }

    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
    RCC_RTCCLKCmd(ENABLE);
    RTC_WaitForSynchro();

    RTC_WriteProtectionCmd(DISABLE);
    RTC_EnterInitMode();
    RTC_Init_Para.RTC_HourFormat = RTC_HourFormat_24;     // 24Ð¡Ê±ÖÆ
    RTC_Init_Para.RTC_AsynchPrediv = (unsigned int)0x7F;  // 127
    RTC_Init_Para.RTC_SynchPrediv = (unsigned int)0xFF;   // 255¡£Á½¸ö·ÖÆµÏµÊýÓëLSEÊ±ÖÓÔ´Æ¥Åä
    RTC_Init(&RTC_Init_Para);

    date.RTC_WeekDay = 0;
    date.RTC_Year = 19;
    date.RTC_Month = 3;
    date.RTC_Date = 2;
    RTC_SetDate(RTC_Format_BIN, &date);

    t.RTC_Hours = 13;
    t.RTC_Minutes = 31;
    t.RTC_Seconds = 11;
    RTC_SetTime(RTC_Format_BIN, &t);
    RTC_ExitInitMode();
    RTC_WriteProtectionCmd(ENABLE);

    ms_C = 0;
}

/********************************************************************
 *Init_Hardware                                                      *
 *Ó²¼þ³õÊ¼»¯¡£                                                       *
 ********************************************************************/
void Init_Hardware(void)
{
    Optical_fiber_Init(691200);
    DIP_Switch_Init();
    uchNodeNum = Read_DIP_Switch();
    Init_Net();
    Init_GPIO();    // ¿ª¹ØÁ¿¶ÔÓ¦µÄÒý½Å
    SRAM_Init();    // Íâ²¿SRAM³õÊ¼»¯
    Init_AD7606();  // Íâ²¿AD(AD7606)³õÊ¼»¯
    // sFLASH_Init();	//M25P32³õÊ¼»¯
    // TIMx_Init();	                                    //¶¨Ê±Æ÷³õÊ¼»¯
    // Init_T();//´®¿Ú2Á¬½ÓÎÂ¶ÈÑ²¼ìÒÇ
    // Init_RS232_Comm();//´®¿Ú1Á¬½Ó¹¦ÂÊ¹ñÏÔÊ¾ÆÁ
    // WatchDog_Init(200);//¿´ÃÅ¹·¶¯×÷Ê±¼ä200ms
    // Init_RTC();
}