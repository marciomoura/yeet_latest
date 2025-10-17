/****************************************************************************************
*文件名称   optical.h                                                                   *
*作者       Three Gorges Nengshida Electric Co.,Ltd.                                    *
*版本       NSD_STM32F2_Driver_6.0                                                      *
*修订日期   2014-01-10                                                                  *
*备注       光纤通信                                                                    *
****************************************************************************************/
extern int Comm_Count;

extern unsigned char Optical_Tx_Flag;//光纤通信发送标志; 

extern unsigned char ScrSynError[8];

void Init_Optical_Comm(void);

void Optical_Comm(void);

void SelectSynSource(void);

short int CRC16(unsigned char *Data, int length);

void Float_To_Byte(float data,unsigned char *b);

void Byte_To_Float(float *data,unsigned char* b);

extern void Calc_Update_Flag(void);
//===========================================================================
// End of file.
//===========================================================================
