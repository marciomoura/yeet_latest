/**
 ******************************************************************************
 * @file    dac.c
 * @brief   AD5724 4-channel DAC driver implementation
 * @date    2025
 ******************************************************************************
 */

#include "dac.h"

#include <string.h>

#include "stm32f4xx_spi.h"
#include "tasks.h"  // For delay_ms() timer-based delays

/* ============================================================================
 * Private Constants - Register Definitions
 * ============================================================================ */

// Command field bits
#define AD5724_CMD_READ  0x800000
#define AD5724_CMD_WRITE 0x000000

// Channel select bits
#define AD5724_CH_SEL_A   0x000000
#define AD5724_CH_SEL_B   0x010000
#define AD5724_CH_SEL_C   0x020000
#define AD5724_CH_SEL_D   0x030000
#define AD5724_CH_SEL_ALL 0x040000

// Register select bits
#define AD5724_REG_DAC           0x000000
#define AD5724_REG_OUTPUT_RANGE  0x080000
#define AD5724_REG_POWER_CONTROL 0x100000
#define AD5724_REG_CONTROL       0x180000

// Power control bits
#define AD5724_PWR_UP_ALL    0x00000F
#define AD5724_PWR_UP_CH_A   0x000001
#define AD5724_PWR_DOWN_CH_A 0xFFFFFE
#define AD5724_PWR_UP_CH_B   0x000002
#define AD5724_PWR_DOWN_CH_B 0xFFFFFD
#define AD5724_PWR_UP_CH_C   0x000004
#define AD5724_PWR_DOWN_CH_C 0xFFFFFB
#define AD5724_PWR_UP_CH_D   0x000008
#define AD5724_PWR_DOWN_CH_D 0xFFFFF7
#define AD5724_PWR_UP_REF    0x000010
#define AD5724_PWR_DOWN_REF  0xFFFFEF

// Control register commands
#define AD5724_CMD_NOP     0x180000
#define AD5724_CMD_CONTROL 0x190000
#define AD5724_CMD_CLEAR   0x1C0000
#define AD5724_CMD_LOAD    0x1D0000

// Control register bits
#define AD5724_CTRL_TSD_ENABLE    0x000008
#define AD5724_CTRL_TSD_DISABLE   0x000000
#define AD5724_CTRL_CLAMP_ENABLE  0x000004
#define AD5724_CTRL_CLAMP_DISABLE 0x000000
#define AD5724_CTRL_CLEAR_SEL0    0x000002
#define AD5724_CTRL_CLEAR_SEL1    0x000000
#define AD5724_CTRL_SDO_DISABLE   0x000001
#define AD5724_CTRL_SDO_ENABLE    0x000000

/* ============================================================================
 * Private Function Prototypes
 * ============================================================================ */

static void AD5724_InitSPI(void);
static void AD5724_InitPins(void);
static void AD5724_WriteRegister(uint32_t data);
static void AD5724_DrainRxFifo(void);
static uint32_t AD5724_GetChannelBits(ad5724_channel_t channel);
static void AD5724_GetRangeMinMax(ad5724_range_t range, float* min_v, float* max_v);

/* ============================================================================
 * Public Function Implementations - Initialization
 * ============================================================================ */

/**
 * @brief Initialize AD5724 driver instance
 */
bool AD5724_Init(ad5724_instance_t* dac)
{
    if (dac == NULL) {
        return false;
    }

    // Clear instance structure
    memset(dac, 0, sizeof(ad5724_instance_t));

    // Initialize hardware
    AD5724_InitSPI();
    AD5724_InitPins();

    // Hardware reset sequence
    AD5724_Reset(dac);

    // Configure AD5724 (may need multiple attempts)
    for (int i = 0; i < 3; i++) {
        // Power up all channels and reference
        uint32_t power_cmd = AD5724_REG_POWER_CONTROL | AD5724_PWR_UP_ALL;
        AD5724_WriteRegister(power_cmd);
        delay_ms(1);  // Use timer-based delay instead of busy-wait

        // Set default range to ±5V for all channels
        uint32_t range_cmd = AD5724_REG_OUTPUT_RANGE | AD5724_RANGE_N5_5V | AD5724_CH_SEL_ALL;
        AD5724_WriteRegister(range_cmd);
    }

    // Initialize default ranges
    for (int ch = 0; ch < AD5724_NUM_CHANNELS; ch++) {
        dac->range[ch] = AD5724_RANGE_N5_5V;
        dac->last_voltage[ch] = 0.0f;
    }

    dac->initialized = true;
    dac->write_count = 0;

    return true;
}

/**
 * @brief Configure AD5724 voltage range for a channel
 */
bool AD5724_SetRange(ad5724_instance_t* dac, ad5724_channel_t channel, ad5724_range_t range)
{
    if (dac == NULL || (uint32_t)channel >= AD5724_NUM_CHANNELS || range > AD5724_RANGE_N11_11V) {
        return false;
    }

    dac->range[channel] = range;

    uint32_t range_cmd = AD5724_REG_OUTPUT_RANGE | range | AD5724_GetChannelBits(channel);
    AD5724_WriteRegister(range_cmd);

    return true;
}

/**
 * @brief Configure AD5724 voltage range for all channels
 */
void AD5724_SetRangeAll(ad5724_instance_t* dac, ad5724_range_t range)
{
    if (dac == NULL || range > AD5724_RANGE_N11_11V) {
        return;
    }

    for (int ch = 0; ch < AD5724_NUM_CHANNELS; ch++) {
        dac->range[ch] = range;
    }

    uint32_t range_cmd = AD5724_REG_OUTPUT_RANGE | range | AD5724_CH_SEL_ALL;
    AD5724_WriteRegister(range_cmd);
}

/**
 * @brief Reset AD5724 DAC (clear all outputs)
 */
void AD5724_Reset(ad5724_instance_t* dac)
{
    if (dac == NULL) {
        return;
    }

    // Pull CLR pin low then high
    GPIOC->BSRRH = GPIO_Pin_5;  // CLR LOW
    delay_ms(1);                // Use timer-based delay for reset pulse
    GPIOC->BSRRL = GPIO_Pin_5;  // CLR HIGH

    // Pull LDAC pin low (default state)
    GPIOC->BSRRH = GPIO_Pin_4;  // LDAC LOW

    // Reset tracking
    for (int ch = 0; ch < AD5724_NUM_CHANNELS; ch++) {
        dac->last_voltage[ch] = 0.0f;
    }
}

/* ============================================================================
 * Public Function Implementations - Output Control
 * ============================================================================ */

/**
 * @brief Set voltage output on a specific channel
 */
bool AD5724_SetVoltage(ad5724_instance_t* dac, ad5724_channel_t channel, float voltage_v)
{
    if (dac == NULL || (uint32_t)channel >= AD5724_NUM_CHANNELS) {
        return false;
    }

    // Check if voltage is within range
    if (!AD5724_IsVoltageInRange(voltage_v, dac->range[channel])) {
        return false;
    }

    // Convert voltage to DAC code
    uint16_t code = AD5724_VoltageToCode(voltage_v, dac->range[channel]);

    // Update tracking
    dac->last_voltage[channel] = voltage_v;

    // Write to DAC register
    uint32_t dac_cmd = AD5724_REG_DAC | AD5724_GetChannelBits(channel) | code;
    AD5724_WriteRegister(dac_cmd);

    dac->write_count++;
    return true;
}

/**
 * @brief Set raw DAC code on a specific channel
 */
bool AD5724_SetCode(ad5724_instance_t* dac, ad5724_channel_t channel, uint16_t code)
{
    if (dac == NULL || (uint32_t)channel >= AD5724_NUM_CHANNELS) {
        return false;
    }

    // Write to DAC register
    uint32_t dac_cmd = AD5724_REG_DAC | AD5724_GetChannelBits(channel) | code;
    AD5724_WriteRegister(dac_cmd);

    dac->write_count++;
    return true;
}

/**
 * @brief Power up a specific channel
 */
void AD5724_PowerUpChannel(ad5724_instance_t* dac, ad5724_channel_t channel)
{
    if (dac == NULL || (uint32_t)channel >= AD5724_NUM_CHANNELS) {
        return;
    }

    uint32_t power_bit = (1U << channel);
    uint32_t power_cmd = AD5724_REG_POWER_CONTROL | power_bit;
    AD5724_WriteRegister(power_cmd);
}

/**
 * @brief Power down a specific channel
 */
void AD5724_PowerDownChannel(ad5724_instance_t* dac, ad5724_channel_t channel)
{
    if (dac == NULL || (uint32_t)channel >= AD5724_NUM_CHANNELS) {
        return;
    }

    uint32_t power_mask = ~(1U << channel);
    uint32_t power_cmd = AD5724_REG_POWER_CONTROL | (power_mask & 0x0F);
    AD5724_WriteRegister(power_cmd);
}

/**
 * @brief Power up all channels
 */
void AD5724_PowerUpAll(ad5724_instance_t* dac)
{
    if (dac == NULL) {
        return;
    }

    uint32_t power_cmd = AD5724_REG_POWER_CONTROL | AD5724_PWR_UP_ALL;
    AD5724_WriteRegister(power_cmd);
}

/* ============================================================================
 * Public Function Implementations - Data Conversion
 * ============================================================================ */

/**
 * @brief Convert voltage to DAC code based on range
 */
uint16_t AD5724_VoltageToCode(float voltage_v, ad5724_range_t range)
{
    float min_v, max_v;
    AD5724_GetRangeMinMax(range, &min_v, &max_v);

    // Clamp voltage to range
    if (voltage_v < min_v) {
        voltage_v = min_v;
    }
    if (voltage_v > max_v) {
        voltage_v = max_v;
    }

    // Convert to code: code = (voltage - min) / (max - min) * 65535
    float normalized = (voltage_v - min_v) / (max_v - min_v);
    uint32_t code = (uint32_t)(normalized * 65535.0f);

    return (uint16_t)code;
}

/**
 * @brief Check if voltage is within valid range
 */
bool AD5724_IsVoltageInRange(float voltage_v, ad5724_range_t range)
{
    float min_v, max_v;
    AD5724_GetRangeMinMax(range, &min_v, &max_v);

    return (voltage_v >= min_v && voltage_v <= max_v);
}

/* ============================================================================
 * Private Function Implementations - Hardware Layer
 * ============================================================================ */

/**
 * @brief Initialize SPI1 for AD5724
 */
static void AD5724_InitSPI(void)
{
    SPI_InitTypeDef SPI_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    // Enable clocks
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    // Configure alternate functions for PA4-PA7
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_SPI1);  // NSS
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);  // SCK
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);  // MISO
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);  // MOSI

    // Configure GPIO pins PA5-PA7 (SCK, MISO, MOSI)
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Configure GPIO pin PA4 (NSS)
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Configure SPI1
    SPI_I2S_DeInit(SPI1);
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_Init(SPI1, &SPI_InitStructure);

    SPI_SSOutputCmd(SPI1, ENABLE);
}

/**
 * @brief Initialize AD5724 control pins (LDAC, CLR)
 */
static void AD5724_InitPins(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    // Configure PC4 (LDAC) and PC5 (CLR) as outputs
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // Set initial states
    GPIOC->BSRRL = GPIO_Pin_5;  // CLR HIGH (inactive)
    GPIOC->BSRRH = GPIO_Pin_4;  // LDAC LOW (active)
}

/**
 * @brief Write 24-bit command/data to AD5724 via SPI
 */
static void AD5724_WriteRegister(uint32_t data)
{
    uint8_t tx_bytes[3] = {
        (uint8_t)(data >> 16),
        (uint8_t)(data >> 8),
        (uint8_t)data,
    };

    AD5724_DrainRxFifo();

    SPI_Cmd(SPI1, ENABLE);

    for (int i = 0; i < 3; i++) {
        while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) {
            __NOP();
        }
        SPI_I2S_SendData(SPI1, tx_bytes[i]);

        while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET) {
            if (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == RESET) {
                break;
            }
            __NOP();
        }

        if (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == SET) {
            (void)SPI_I2S_ReceiveData(SPI1);  // Drain dummy data to prevent OVR
        }
    }

    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET) {
        __NOP();
    }

    SPI_Cmd(SPI1, DISABLE);

    AD5724_DrainRxFifo();

    for (volatile int i = 0; i < 30; i++) {
        __NOP();
    }
}

static void AD5724_DrainRxFifo(void)
{
    if (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_OVR) == SET) {
        (void)SPI1->DR;
        (void)SPI1->SR;
    }

    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == SET) {
        (void)SPI_I2S_ReceiveData(SPI1);
    }
}

/**
 * @brief Get channel select bits for register command
 */
static uint32_t AD5724_GetChannelBits(ad5724_channel_t channel)
{
    switch (channel) {
        case AD5724_CH_A:
            return AD5724_CH_SEL_A;
        case AD5724_CH_B:
            return AD5724_CH_SEL_B;
        case AD5724_CH_C:
            return AD5724_CH_SEL_C;
        case AD5724_CH_D:
            return AD5724_CH_SEL_D;
        default:
            return AD5724_CH_SEL_A;
    }
}

/**
 * @brief Get voltage range min/max values
 */
static void AD5724_GetRangeMinMax(ad5724_range_t range, float* min_v, float* max_v)
{
    switch (range) {
        case AD5724_RANGE_0_5V:
            *min_v = 0.0f;
            *max_v = 5.0f;
            break;
        case AD5724_RANGE_0_10V:
            *min_v = 0.0f;
            *max_v = 10.0f;
            break;
        case AD5724_RANGE_0_11V:
            *min_v = 0.0f;
            *max_v = 10.8f;
            break;
        case AD5724_RANGE_N5_5V:
            *min_v = -5.0f;
            *max_v = 5.0f;
            break;
        case AD5724_RANGE_N10_10V:
            *min_v = -10.0f;
            *max_v = 10.0f;
            break;
        case AD5724_RANGE_N11_11V:
            *min_v = -10.8f;
            *max_v = 10.8f;
            break;
        default:
            *min_v = -5.0f;
            *max_v = 5.0f;
            break;
    }
}
