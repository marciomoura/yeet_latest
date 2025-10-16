/********************************************************************
 *AI.c - Analog Input Driver (Hardware Layer Only)                  *
 *                                                                   *
 *Reads AD7606 ADC data and provides basic utilities                *
 *Application control logic removed - see archive/ directory         *
 ********************************************************************/
#include <math.h>

#include "AD7606.h"
#include "AI.h"

#define AD_Data_Num 300

// Raw ADC data buffers
int AI_Times;                     // Sample counter
int ADC_Data[8][AD_Data_Num];     // Raw ADC data [channel][sample]
int AI_Count;                     // General counter
unsigned short int nI_Buf[3][2];  // Median filter buffers

/********************************************************************
 *Init_AD_Data                                                       *
 *Initialize ADC data buffers                                        *
 ********************************************************************/
void Init_AD_Data(void)
{
    int i, j;

    // Clear ADC buffers
    for (i = 0; i < 8; i++) {
        for (j = 0; j < AD_Data_Num; j++) {
            ADC_Data[i][j] = 0;
        }
    }

    AI_Count = 0;
    AI_Times = 0;
}

/********************************************************************
 *Zero_Calibrate                                                     *
 *DC offset calibration for ADC channels                             *
 ********************************************************************/
void Zero_Calibrate(void)
{
    int i, j;
    long int nSum;
    unsigned short int nBuffer[8];

    for (i = 0; i < 8; i++) {
        nSum = 0;
        for (j = 0; j < 100; j++) {
            Start_Read_AD7606(nBuffer);
            nSum += (long int)nBuffer[i];
        }
        ADC_Data[i][0] = (int)(nSum / 100);  // Store calibration offset
    }
}

/********************************************************************
 *Get_Medium_n                                                       *
 *Median filter for noise rejection (integer version)                *
 ********************************************************************/
int Get_Medium_n(int a, int b, int c)
{
    if (a > b) {
        if (b > c)
            return b;
        else if (a > c)
            return c;
        else
            return a;
    }
    else  // a <= b
    {
        if (a > c)
            return a;
        else if (b > c)
            return c;
        else
            return b;
    }
}

/********************************************************************
 *Get_Medium_Value                                                   *
 *Median filter for noise rejection (float version)                  *
 ********************************************************************/
float Get_Medium_Value(float a, float b, float c)
{
    if (a > b) {
        if (b > c)
            return b;
        else if (a > c)
            return c;
        else
            return a;
    }
    else  // a <= b
    {
        if (a > c)
            return a;
        else if (b > c)
            return c;
        else
            return b;
    }
}

/********************************************************************
 *AD                                                                 *
 *Read ADC with median filtering                                     *
 *This is the main hardware interface - keep this function           *
 ********************************************************************/
void AD(void)
{
    unsigned short int nBuffer[8];
    int i;

    // Read all 8 ADC channels
    Start_Read_AD7606(nBuffer);

    // Apply median filter to reduce noise
    for (i = 0; i < 8; i++) {
        nI_Buf[i % 3][0] = nI_Buf[i % 3][1];
        nI_Buf[i % 3][1] = nBuffer[i];

        // Store median-filtered value
        ADC_Data[i][AI_Count] = Get_Medium_n(nI_Buf[i % 3][0], nI_Buf[i % 3][1], nBuffer[i]);
    }

    // Increment circular buffer pointer
    if (++AI_Count >= AD_Data_Num) {
        AI_Count = 0;
    }

    AI_Times++;
}

/********************************************************************
 *Cal_AD                                                             *
 *Placeholder for application-specific ADC processing                *
 *Users should implement their own processing in their application   *
 ********************************************************************/
void Cal_AD(void)
{
    // Empty - application control logic removed
    // User application can process ADC_Data[8][300] buffer directly
    // Or implement custom calculations here
}

//===========================================================================
// End of file.
//===========================================================================
