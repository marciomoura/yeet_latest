/********************************************************************
 *pub.c - Hardware Status Variable Definitions                       *
 *                                                                    *
 *Global variables for hardware status monitoring.                   *
 *Application developers should initialize these in their main loop. *
 ********************************************************************/

#include "pub.h"

// Hardware status monitoring variables
unsigned char DI_Data[8] = {0};
unsigned char DO_Data[8] = {0};

unsigned int Running_Count = 0;
unsigned int Running_Flag = 0;

unsigned char DO_Lock_Flag = 0;
unsigned short int DO_Lock_Delay = 0;

unsigned char SCR_Error_Flag = 0;
unsigned char Fuse_Flag = 0;
unsigned char Arm_Fuse_Flag[6] = {0};
unsigned char Over_Temperature_Flag = 0;
unsigned char Fan_Flag = 0;
unsigned char Fan_On_Flag = 0;

unsigned char Pulse_Enabled_Flag = 0;
unsigned char Pulse_Power_Flag = 0;
unsigned char Low_F_Flag = 0;
int Low_F_Count = 0;
unsigned char Breaker_Flag = 0;

unsigned char Ref_Up_Flag = 0;
unsigned char Ref_Down_Flag = 0;
unsigned char Stop_Command = 0;

unsigned char Sec_A_Flag = 0;
unsigned char Sec_B_Flag = 0;
unsigned char Sec_C_Flag = 0;
unsigned char Syn_A_Err_Flag = 0;
unsigned char Syn_B_Err_Flag = 0;
unsigned char Syn_C_Err_Flag = 0;

unsigned char ROM_Flag = 0;
unsigned char Scr_Exit_Flag = 0;

unsigned char AVR_Flag = 0;
unsigned char FCR_Flag = 0;
unsigned char AER_Flag = 0;
unsigned char PT_Break_Flag = 0;
unsigned char Standby_Flag = 0;
unsigned char Over_Ex_Flag = 0;

unsigned char Online_Flag = 0;
// Note: uchNodeNum is defined in para.c (DIP switch value)

float Vabc = 0.0f;
float P = 0.0f;
float Q = 0.0f;

//===========================================================================
// End of file.
//===========================================================================
