/********************************************************************
 *AI.h - Analog Input Driver (Hardware Layer Only)                  *
 *                                                                   *
 *Provides access to AD7606 ADC data                                 *
 ********************************************************************/
#ifndef __AI_H
#define __AI_H

#define AD_Data_Num 300

// External declarations for ADC data buffers
extern int AI_Times;                     // Sample counter
extern int ADC_Data[8][AD_Data_Num];     // Raw ADC data [channel][sample]
extern int AI_Count;                     // Current buffer index
extern unsigned short int nI_Buf[3][2];  // Median filter buffers

// Function prototypes
void Init_AD_Data(void);
void Zero_Calibrate(void);
int Get_Medium_n(int a, int b, int c);
float Get_Medium_Value(float a, float b, float c);
void AD(void);      // Main ADC read function
void Cal_AD(void);  // Placeholder for application processing

#endif  // __AI_H

//===========================================================================
// End of file.
//===========================================================================
