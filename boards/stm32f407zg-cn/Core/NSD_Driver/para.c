/********************************************************************************
 *para.c - Hardware Initialization (Application Logic Removed)                  *
 *                                                                               *
 *Provides hardware peripheral initialization only                               *
 *Application control initialization removed - see archive/ directory            *
 ********************************************************************************/
#include <stm32f4xx.h>

#include "AD7606.h"
#include "AI.h"
#include "DIDO.h"
#include "GPIO.h"
#include "M25P32.h"
#include "TIM.h"
#include "WatchDog.h"
#include "nandflash_sram.h"
#include "temperature.h"
#include "w5200.h"

// Note: Removed includes for archived files:
// command.h, eIfd.h, limit.h, mode.h, net.h, optical.h, optical_fiber.h, pub.h, syn.h

// Node ID from DIP switches
unsigned char uchNodeNum = 0;

/********************************************************************
 *intToChar                                                          *
 *Convert 16-bit integer to two unsigned bytes                       *
 ********************************************************************/
void intToChar(short int n, unsigned char* Data)
{
    *Data++ = (n >> 8) & 0xff;
    *Data = n & 0xff;
}

/********************************************************************
 *charToInt                                                          *
 *Convert two unsigned bytes to 16-bit integer                       *
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
 *Initialize DIP switch GPIO pins                                    *
 ********************************************************************/
void DIP_Switch_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // Enable GPIOC clock
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    // Configure PC6, PC7, PC8, PC9 as inputs with pull-ups
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/********************************************************************
 *Read_DIP_Switch                                                    *
 *Read node ID from DIP switches (4-bit value)                       *
 ********************************************************************/
unsigned char Read_DIP_Switch(void)
{
    unsigned char i = 0;

    // Read 4 bits from PC6-PC9 (inverted logic)
    if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_6) == 0) {
        i |= 0x01;
    }
    if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7) == 0) {
        i |= 0x02;
    }
    if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_8) == 0) {
        i |= 0x04;
    }
    if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_9) == 0) {
        i |= 0x08;
    }

    return i;
}

/********************************************************************
 *Init_Hardware                                                      *
 *Initialize all hardware peripherals                                *
 *Application-specific init removed (see Init_Data in archive)       *
 ********************************************************************/
void Init_Hardware(void)
{
    // NOTE: Optical_fiber_Init() removed - was in archived optical_fiber.c
    // NOTE: Init_Net() removed - was in archived net.c

    // Read board node ID from DIP switches
    DIP_Switch_Init();
    uchNodeNum = Read_DIP_Switch();

    // Hardware peripherals
    Init_GPIO();         // GPIO configuration
    SRAM_Init();         // External SRAM initialization
    Init_AD7606();       // External ADC (AD7606) initialization
    sFLASH_Init();       // M25P32 Flash memory initialization
    TIMx_Init();         // Timer initialization
    Init_T();            // Timer 2 for temperature polling
    WatchDog_Init(200);  // Watchdog with 200ms timeout

    // NOTE: Init_RTC() was here but depends on removed code
    // User should implement RTC init if needed
}

/********************************************************************
 *Init_Data                                                          *
 *Initialize data structures - minimal version                       *
 *                                                                   *
 *Original version initialized control algorithms - see archive/      *
 *User applications should implement their own initialization        *
 ********************************************************************/
void Init_Data(void)
{
    // NOTE: Original called many archived functions:
    // - Syn_Pulse_Init() - from syn.c (archived)
    // - Init_Optical_Comm() - from optical.c (archived)
    // - Init_DIDO() - had control logic mixed in
    // - Init_Coef() - control coefficients (archived)
    // - Init_PID() - from eIfd.c (archived)
    // - Init_Limit() - from limit.c (archived)
    // - Work_Mode_V(1) - from mode.c (archived)

    // Initialize ADC buffers (hardware only)
    Init_AD_Data();

    // User application should add their own initialization here:
    // - Control algorithm initialization
    // - State observer initialization
    // - Communication protocol setup
    // - etc.
}

//===========================================================================
// End of file.
//===========================================================================
