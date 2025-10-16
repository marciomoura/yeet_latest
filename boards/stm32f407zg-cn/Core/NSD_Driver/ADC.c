/**
 ******************************************************************************
 * @file    adc.c
 * @brief   AD7606 8-channel ADC driver implementation
 * @date    2025
 ******************************************************************************
 */

#include "adc.h"

#include <string.h>

#include "stm32f4xx_gpio.h"

/* ============================================================================
 * Private Constants
 * ============================================================================ */

#define DELAY_CYCLES_PER_US 42 /**< Approximate delay loop cycles (168MHz / 4) */

/* ============================================================================
 * Private Function Prototypes
 * ============================================================================ */

static void AD7606_DelayUs(uint32_t us);
static void AD7606_InitPins(void);
static void AD7606_InitFSMC(void);
static void FSMC_ReadBuffer(uint16_t* pBuffer, uint32_t read_addr, uint32_t num_words);

/* ============================================================================
 * Public Function Implementations - Initialization
 * ============================================================================ */

/**
 * @brief Initialize AD7606 driver instance
 */
bool AD7606_Init(ad7606_instance_t* adc)
{
    if (adc == NULL) {
        return false;
    }

    // Clear instance structure
    memset(adc, 0, sizeof(ad7606_instance_t));

    // Initialize hardware
    AD7606_InitPins();
    AD7606_InitFSMC();

    // Reset and configure AD7606
    AD7606_Reset(adc);
    AD7606_SetRange(adc, AD7606_RANGE_10V);

    adc->initialized = true;
    adc->sample_count = 0;

    return true;
}

/**
 * @brief Configure AD7606 voltage range
 */
void AD7606_SetRange(ad7606_instance_t* adc, ad7606_range_t range)
{
    if (adc == NULL) {
        return;
    }

    adc->range = range;

    if (range == AD7606_RANGE_10V) {
        GPIOF->BSRRL = GPIO_Pin_9;  // RANGE pin HIGH = ±10V
    }
    else {
        GPIOF->BSRRH = GPIO_Pin_9;  // RANGE pin LOW = ±5V
    }

    AD7606_DelayUs(1);
}

/**
 * @brief Reset AD7606 ADC
 */
void AD7606_Reset(ad7606_instance_t* adc)
{
    if (adc == NULL) {
        return;
    }

    // Reset sequence: LOW → HIGH → LOW
    GPIOF->BSRRH = GPIO_Pin_10;  // RST LOW
    AD7606_DelayUs(1);

    GPIOF->BSRRL = GPIO_Pin_10;  // RST HIGH
    AD7606_DelayUs(1);

    GPIOF->BSRRH = GPIO_Pin_10;  // RST LOW (idle)
    AD7606_DelayUs(1);
}

/* ============================================================================
 * Public Function Implementations - Data Acquisition
 * ============================================================================ */

/**
 * @brief Start AD7606 conversion
 */
void AD7606_StartConversion(ad7606_instance_t* adc)
{
    if (adc == NULL) {
        return;
    }

    // Trigger conversion: HIGH → LOW → HIGH
    GPIOF->BSRRL = GPIO_Pin_6 | GPIO_Pin_7;  // CONVSTA/B HIGH

    GPIOF->BSRRH = GPIO_Pin_6 | GPIO_Pin_7;  // CONVSTA/B LOW
    for (volatile int i = 0; i < 20; i++) {
        __NOP();  // >50ns pulse
    }

    GPIOF->BSRRL = GPIO_Pin_6 | GPIO_Pin_7;  // CONVSTA/B HIGH
}

/**
 * @brief Check if conversion is complete
 */
bool AD7606_IsConversionReady(const ad7606_instance_t* adc)
{
    if (adc == NULL) {
        return false;
    }

    // BUSY pin: LOW = conversion complete, HIGH = busy
    return (GPIOD->IDR & GPIO_Pin_13) == 0;
}

/**
 * @brief Wait for conversion to complete with timeout
 */
bool AD7606_WaitConversion(const ad7606_instance_t* adc, uint32_t timeout_us)
{
    if (adc == NULL) {
        return false;
    }

    uint32_t timeout = timeout_us * DELAY_CYCLES_PER_US;

    // Wait for BUSY pin to go LOW
    while ((GPIOD->IDR & GPIO_Pin_13) && timeout > 0) {
        timeout--;
        __NOP();
    }

    return timeout > 0;
}

/**
 * @brief Read all 8 channels from AD7606
 */
bool AD7606_Read(ad7606_instance_t* adc, ad7606_reading_t* reading)
{
    if (adc == NULL || reading == NULL) {
        return false;
    }

    // Wait for conversion to complete
    if (!AD7606_WaitConversion(adc, 100)) {
        reading->valid = false;
        return false;
    }

    // Read all channels via FSMC
    FSMC_ReadBuffer(reading->raw, AD7606_DATA_OFFSET, AD7606_NUM_CHANNELS);

    reading->valid = true;
    adc->sample_count++;

    return true;
}

/**
 * @brief Start conversion and read previous results (pipeline mode)
 */
bool AD7606_ReadAndStart(ad7606_instance_t* adc, ad7606_reading_t* reading)
{
    if (adc == NULL || reading == NULL) {
        return false;
    }

    bool success = AD7606_Read(adc, reading);
    AD7606_StartConversion(adc);
    return success;
}

/**
 * @brief Convert raw ADC value to voltage
 */
float AD7606_RawToVoltage(uint16_t raw_value, ad7606_range_t range)
{
    int16_t signed_value = (int16_t)raw_value;
    float voltage_range = (range == AD7606_RANGE_10V) ? 10.0f : 5.0f;

    // Convert to voltage: (raw / 32768) * range
    return ((float)signed_value / 32768.0f) * voltage_range;
}

/* ============================================================================
 * Public Function Implementations - On-Chip ADC
 * ============================================================================ */

/**
 * @brief Initialize on-chip ADC
 */
bool OnChipADC_Init(onchip_adc_t* adc, uint16_t num_channels)
{
    if (adc == NULL || num_channels == 0 || num_channels > 4) {
        return false;
    }

    memset(adc, 0, sizeof(onchip_adc_t));
    adc->num_channels = num_channels;

    // Initialize hardware (calling legacy functions)
    extern void Init_AD_Pin(void);
    extern void Init_AD_DMA(int nNum);
    extern void Init_AD_Mode(int32_t AD_Num);

    Init_AD_Pin();
    Init_AD_DMA((int)num_channels);
    Init_AD_Mode((int32_t)num_channels);

    adc->initialized = true;
    return true;
}

/**
 * @brief Get on-chip ADC results
 */
void OnChipADC_GetResults(onchip_adc_t* adc, uint16_t* results)
{
    if (adc == NULL || results == NULL) {
        return;
    }

    extern unsigned short int AD_Result[12];

    // Copy results
    for (uint16_t i = 0; i < adc->num_channels && i < 12; i++) {
        results[i] = AD_Result[i];
    }
}

/* ============================================================================
 * Private Function Implementations - Hardware Layer
 * ============================================================================ */

/**
 * @brief Microsecond delay
 */
static void AD7606_DelayUs(uint32_t us)
{
    uint32_t loops = us * DELAY_CYCLES_PER_US;
    for (volatile uint32_t i = 0; i < loops; i++) {
        __NOP();
    }
}

/**
 * @brief Initialize AD7606 control pins
 */
static void AD7606_InitPins(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOF, ENABLE);

    // BUSY (PD13) - Input
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    // Control pins (PF6,7,9,10) - Output
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOF, &GPIO_InitStructure);
}

/**
 * @brief Initialize FSMC for AD7606
 * @note Calls legacy PSRAM_Init implementation
 */
static void AD7606_InitFSMC(void)
{
    extern void PSRAM_Init(void);
    PSRAM_Init();
}

/**
 * @brief Read buffer from FSMC memory
 */
static void FSMC_ReadBuffer(uint16_t* pBuffer, uint32_t read_addr, uint32_t num_words)
{
    if (pBuffer == NULL) {
        return;
    }

    for (uint32_t i = 0; i < num_words; i++) {
        *pBuffer++ = *(volatile uint16_t*)(AD7606_BASE_ADDR + read_addr);
        read_addr += 2;
    }
}

/* ============================================================================
 * Legacy Function Implementations (preserved from original code)
 * ============================================================================ */

// The following functions preserve the original PSRAM and legacy ADC implementations
// They are kept for backward compatibility with existing code

/********************************************************************************
 * PSRAM_Init - Initialize FSMC for external SRAM/AD7606
 ********************************************************************************/
void PSRAM_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    FSMC_NORSRAMInitTypeDef FSMC_NORSRAMInitStructure;
    FSMC_NORSRAMTimingInitTypeDef p;

    // Enable clocks
    RCC_AHB1PeriphClockCmd(
        RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOF | RCC_AHB1Periph_GPIOG, ENABLE);
    RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC, ENABLE);

    // Configure GPIO for FSMC
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

    // Data lines D0-D15 (PD14,15,0,1, PE7-15)
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_FSMC);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

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

    // Address lines A0-A11 (PF0-5,12-15)
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

    // Address lines A12-A17 (PG0-5)
    GPIO_PinAFConfig(GPIOG, GPIO_PinSource0, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOG, GPIO_PinSource1, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOG, GPIO_PinSource2, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOG, GPIO_PinSource3, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOG, GPIO_PinSource4, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOG, GPIO_PinSource5, GPIO_AF_FSMC);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_Init(GPIOG, &GPIO_InitStructure);

    // Control lines NOE, NWE, NE2 (PD4,5, PG9)
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource4, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_FSMC);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOG, GPIO_PinSource9, GPIO_AF_FSMC);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_Init(GPIOG, &GPIO_InitStructure);

    // NBL0, NBL1 (PE0,1)
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource0, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource1, GPIO_AF_FSMC);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    // FSMC timing configuration
    p.FSMC_AddressSetupTime = 0x01;
    p.FSMC_AddressHoldTime = 0x00;
    p.FSMC_DataSetupTime = 0x02;
    p.FSMC_BusTurnAroundDuration = 0x00;
    p.FSMC_CLKDivision = 0x00;
    p.FSMC_DataLatency = 0x00;
    p.FSMC_AccessMode = FSMC_AccessMode_A;

    // FSMC NOR/SRAM configuration
    FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM2;
    FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
    FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_SRAM;
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
    FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &p;
    FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &p;

    FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure);
    FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM2, ENABLE);
}
