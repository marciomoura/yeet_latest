/********************************************************************
*syn.h                                                              *
*                                                                   *
*Copyright: 2002-2018 Nengshida Electric Ltd.                       *
*Author: Yu Xiang                                                   *
*July.12, 2018                                                      *
********************************************************************/
void Syn_Pulse_Init(void);

void Pulse_Syn_Low_F_Check(void);

void Syn_Check(unsigned char* Phase_Flag, int* Phase_Count);

void Pulse_Syn_Error_Check(void);

void Cal_Period(unsigned int t, unsigned int* tOld);

unsigned int Phase_Difference(unsigned int Former_Phase, unsigned int Current_Phase);

void SelectSyn(unsigned char ucSyn);
//===========================================================================
// End of file.
//===========================================================================
