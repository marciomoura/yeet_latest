/***************************************************************************************
 * GPIO.C - Digital I/O Driver Implementation
 ***************************************************************************************
 * Company:     Three Gorges Nengshida Electric Co., Ltd
 * Version:     NSD_STM32F4_Driver_2.0 (Cleaned)
 * Date:        2013-08-22 (Updated 2025-10-16)
 * Description: FSMC-based digital I/O driver with debounce filtering
 ***************************************************************************************/

#include "GPIO.H"
#include "stm32f4xx_fsmc.h"

//=======================================================================================
// Hardware Configuration
//=======================================================================================

/** FSMC Bank 4 base address for GPIO expansion hardware */
#define BANK1_SRAM4_ADDR ((uint32_t)0x6C000000)

/** Debounce filter delay (number of consecutive stable reads required) */
#define DI_DEBOUNCE_CYCLES 2

//=======================================================================================
// Global Variables
//=======================================================================================

/** Raw digital input state (real-time) */
unsigned char DI_Flag[8][8];

/** Debounced digital input state (filtered) */
unsigned char DI_Delayed_Flag[8][8];

/** Digital output state buffer */
unsigned char DO_Flag[8][8];

/** Debounce delay counters */
static int DI_Delay[8][8];

/** LED indicator states */
unsigned char DO_LED_Run = 0;
unsigned char DO_LED_Fault = 0;
unsigned char DO_LED_alarm = 0;

//=======================================================================================
// Private Function Prototypes
//=======================================================================================

static void Init_DIDO_Para(void);
static void GPIO_FSMC_Init(void);
static void DI16_Input(uint16_t addr, unsigned char* DI_Flag);
static void DO16_Out(uint16_t addr, unsigned char* DO_Flag);
static void Delayed_DI(unsigned char DI_current, uint8_t* DI_filtered, int* delay_counter);

//=======================================================================================
// Initialization Functions
//=======================================================================================

/**
 * @brief Initialize DI/DO data structures
 *
 * Clears all input/output buffers and sets default LED states.
 */
static void Init_DIDO_Para(void)
{
    int i, j;

    // Clear all DI/DO buffers and delay counters
    for (i = 0; i < 8; i++) {
        for (j = 0; j < 8; j++) {
            DI_Flag[i][j] = 0;
            DI_Delayed_Flag[i][j] = 0;
            DI_Delay[i][j] = 0;
            DO_Flag[i][j] = 0;
        }
    }

    // Set default LED states
    DO_LED_Run = 1;    // Running LED enabled (will blink)
    DO_LED_Fault = 0;  // Fault LED off
    DO_LED_alarm = 0;  // Alarm LED off
}

/**
 * @brief Initialize FSMC interface for GPIO expansion hardware
 *
 * Configures GPIO pins (PD, PE, PF, PG) for FSMC alternate function and
 * initializes FSMC Bank 4 for PSRAM mode with appropriate timing parameters.
 */
static void GPIO_FSMC_Init(void)
{
    FSMC_NORSRAMInitTypeDef FSMC_NORSRAMInitStructure;
    FSMC_NORSRAMTimingInitTypeDef timing;
    GPIO_InitTypeDef GPIO_InitStructure;

    // Enable GPIO clocks
    RCC_AHB1PeriphClockCmd(
        RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOF | RCC_AHB1Periph_GPIOG, ENABLE);

    // Enable FSMC clock
    RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC, ENABLE);

    //-----------------------------------------------------------------------------------
    // GPIO Configuration for FSMC
    //-----------------------------------------------------------------------------------
    // Pin Assignment:
    // GPIOD: D0-D3(PD14,15,0,1), NOE(PD4), NWE(PD5), D13-D15(PD8-10), A16-A17(PD11-12)
    // GPIOE: D4-D12(PE7-15)
    // GPIOF: A0-A5(PF0-5), A6-A9(PF12-15)
    // GPIOG: A10-A15(PG0-5), NE4(PG12)
    //-----------------------------------------------------------------------------------

    // Common GPIO settings for FSMC pins
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

    // GPIOD configuration
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource4, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource10, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource11, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_FSMC);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_8 | GPIO_Pin_9 |
                                  GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    // GPIOE configuration
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource7, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource8, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource10, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource12, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource15, GPIO_AF_FSMC);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 |
                                  GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    // GPIOF configuration
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource0, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource1, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource2, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource3, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource4, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource5, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource12, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource13, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource14, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource15, GPIO_AF_FSMC);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 |
                                  GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOF, &GPIO_InitStructure);

    // GPIOG configuration
    GPIO_PinAFConfig(GPIOG, GPIO_PinSource0, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOG, GPIO_PinSource1, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOG, GPIO_PinSource2, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOG, GPIO_PinSource3, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOG, GPIO_PinSource4, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOG, GPIO_PinSource5, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOG, GPIO_PinSource12, GPIO_AF_FSMC);

    GPIO_InitStructure.GPIO_Pin =
        GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_12;
    GPIO_Init(GPIOG, &GPIO_InitStructure);

    //-----------------------------------------------------------------------------------
    // FSMC Timing Configuration
    //-----------------------------------------------------------------------------------
    timing.FSMC_AddressSetupTime = 6;
    timing.FSMC_AddressHoldTime = 15;
    timing.FSMC_DataSetupTime = 21;
    timing.FSMC_BusTurnAroundDuration = 1;
    timing.FSMC_CLKDivision = 1;
    timing.FSMC_DataLatency = 0;
    timing.FSMC_AccessMode = FSMC_AccessMode_A;

    //-----------------------------------------------------------------------------------
    // FSMC Bank 4 Configuration (PSRAM mode)
    //-----------------------------------------------------------------------------------
    FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM4;
    FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
    FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_PSRAM;
    FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;
    FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;
    FSMC_NORSRAMInitStructure.FSMC_AsynchronousWait = FSMC_AsynchronousWait_Disable;
    FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
    FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;
    FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
    FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
    FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
    FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable;
    FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;
    FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &timing;
    FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &timing;

    FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure);

    // Enable FSMC Bank 4
    FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM4, ENABLE);
}

/**
 * @brief Initialize GPIO driver
 *
 * Complete initialization sequence: hardware setup, data structures,
 * and initial input read to synchronize DI_Flag with DI_Delayed_Flag.
 */
void Init_GPIO(void)
{
    Init_DIDO_Para();
    GPIO_FSMC_Init();

    DO_LED_Run = 1;

    // Initial input read and double-filter to sync raw and debounced states
    Refresh_DI();
    Refresh_Delayed_DI();
    Refresh_Delayed_DI();  // Force DI_Flag == DI_Delayed_Flag at startup
}

//=======================================================================================
// Digital Input Functions
//=======================================================================================

/**
 * @brief Read 16 digital inputs from hardware
 *
 * @param addr   FSMC address offset (word address)
 * @param DI_ptr Pointer to destination buffer (16 bytes for 16 bits)
 */
static void DI16_Input(uint16_t addr, unsigned char* DI_ptr)
{
    uint16_t data;
    int i;

    // Read 16-bit word from FSMC address
    data = *(__IO uint16_t*)(BANK1_SRAM4_ADDR + addr * 2);

    // Unpack bits into byte array
    for (i = 0; i < 16; i++) {
        *DI_ptr++ = (data & 0x0001) ? 1 : 0;
        data >>= 1;
    }
}

/**
 * @brief Read all digital inputs from hardware
 *
 * Updates DI_Flag array with current hardware state (48 inputs across 3 words).
 */
void Refresh_DI(void)
{
    DI16_Input(201, &DI_Flag[0][0]);  // DI00..DI15
    DI16_Input(202, &DI_Flag[2][0]);  // DI16..DI31
    DI16_Input(203, &DI_Flag[4][0]);  // DI32..DI47
}

/**
 * @brief Apply debounce filter to a single input
 *
 * @param DI_current     Current raw input state
 * @param DI_filtered    Pointer to filtered output
 * @param delay_counter  Pointer to delay counter for this input
 */
static void Delayed_DI(unsigned char DI_current, uint8_t* DI_filtered, int* delay_counter)
{
    if (*DI_filtered != DI_current) {
        // Input has changed - increment delay counter
        (*delay_counter)++;

        if (*delay_counter >= DI_DEBOUNCE_CYCLES) {
            // Input stable for required cycles - accept change
            *DI_filtered = DI_current;
            *delay_counter = 0;
        }
    }
    else {
        // Input matches filtered state - reset counter
        *delay_counter = 0;
    }
}

/**
 * @brief Apply debounce filter to all digital inputs
 *
 * Updates DI_Delayed_Flag array with filtered input states.
 * Call after Refresh_DI() in each scan cycle.
 */
void Refresh_Delayed_DI(void)
{
    int i, j;

    for (i = 0; i < 8; i++) {
        for (j = 0; j < 8; j++) {
            Delayed_DI(DI_Flag[i][j], &DI_Delayed_Flag[i][j], &DI_Delay[i][j]);
        }
    }
}

//=======================================================================================
// Digital Output Functions
//=======================================================================================

/**
 * @brief Write 16 digital outputs to hardware
 *
 * @param addr   FSMC address offset (word address)
 * @param DO_ptr Pointer to source buffer (16 bytes for 16 bits)
 */
static void DO16_Out(uint16_t addr, unsigned char* DO_ptr)
{
    uint16_t data = 0;
    int i;

    // Pack byte array into 16-bit word (LSB first)
    for (i = 0; i < 16; i++) {
        data >>= 1;
        if (*DO_ptr++) {
            data |= 0x8000;
        }
    }

    // Write to FSMC address
    *(uint16_t*)(BANK1_SRAM4_ADDR + (addr * 2)) = data;

    // Small delay for hardware settling (6 CPU cycles @ 168MHz = ~36ns)
    for (i = 0; i < 6; i++) {
        __NOP();
    }
}

/**
 * @brief Write all digital outputs to hardware
 *
 * Writes DO_Flag array and LED indicators to hardware GPIO.
 */
void DO_Out(void)
{
    unsigned char DO_LED[16] = {0};

    // Prepare LED indicator word
    DO_LED[0] = DO_LED_Run;
    DO_LED[1] = DO_LED_Fault;
    DO_LED[2] = DO_LED_alarm;

    // Write digital outputs (48 outputs across 3 words)
    DO16_Out(3, &DO_Flag[0][0]);  // DO00..DO15
    DO16_Out(4, &DO_Flag[2][0]);  // DO16..DO31
    DO16_Out(5, &DO_Flag[4][0]);  // DO32..DO47

    // Clear extended outputs (if hardware supports)
    *(uint16_t*)(BANK1_SRAM4_ADDR + 12) = 0;
    for (int i = 0; i < 6; i++) {
        __NOP();
    }

    // Write LED indicators
    DO16_Out(7, DO_LED);
}

//=======================================================================================
// Combined I/O Update Function
//=======================================================================================

/**
 * @brief Refresh both inputs and outputs
 *
 * Convenience function for simple periodic I/O update.
 */
void GPIO(void)
{
    Refresh_DI();
    Refresh_Delayed_DI();
    DO_Out();
}

//===========================================================================
// End of file
//===========================================================================
