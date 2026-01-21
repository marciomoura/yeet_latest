/**
 ******************************************************************************
 * @file    dac.c (LEGACY RESTORED VERSION)
 * @brief   Restored legacy-style AD5724 driver (simplified) plus thin wrappers
 *          to keep existing higher-level calls working.
 * @date    2025
 *
 * This file intentionally reverts to the original minimal sequence used in
 * `legacy/AD57x4.c` so you can validate basic hardware behaviour first.
 * Once validated on the oscilloscope, you can iterate forward again.
 *
 * Key decisions:
 *  - Only the ±5 V bipolar range (legacy Range_N5_5V) is configured.
 *  - No per‑channel power management / range caching logic.
 *  - Each write enables SPI, shifts 24 bits MSB→LSB, then disables SPI (as legacy).
 *  - Wrapper functions `AD5724_Init`, `AD5724_SetVoltage`, etc. map to the legacy
 *    primitives so the rest of the firmware does not need to change.
 */

#include "dac.h"

#include <string.h>

#include "stm32f4xx_spi.h"

/* ========================= Legacy Macro Set (ported) ===================== */

#define AD5724_READ  0x800000UL
#define AD5724_WRITE 0x000000UL

/* Channel Select */
#define LEGACY_DAC_CH_A   0x000000UL
#define LEGACY_DAC_CH_B   0x010000UL
#define LEGACY_DAC_CH_C   0x020000UL
#define LEGACY_DAC_CH_D   0x030000UL
#define LEGACY_DAC_CH_ALL 0x040000UL

/* Register Select */
#define LEGACY_REG_DAC           0x000000UL
#define LEGACY_REG_OUTPUT_RANGE  0x080000UL
#define LEGACY_REG_POWER_CONTROL 0x100000UL
#define LEGACY_REG_CONTROL       0x180000UL

/* Output Range Codes (legacy names -> here we only use ±5 V) */
#define LEGACY_RANGE_N5_P5V 0x000003UL /* -5 .. +5 V */

/* Power Control */
#define LEGACY_POWER_UP_ALL 0x00000FUL  // Only channels, NOT reference (legacy may use external ref)

/* Control helper commands */
#define LEGACY_CMD_CLEAR 0x1C0000UL

/* ========================= Static Helpers (legacy style) ================= */
static void legacy_SPI1_Init(void)
{
    SPI_InitTypeDef SPI_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

    // Configure SPI pins (PA5-PA7) as alternate function
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);  // SCK
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);  // MISO
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);  // MOSI

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

    // Configure NSS (PE6 and PE5) and PA4 as GPIO outputs (legacy uses PE6 for CS)
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_5;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Set CS pins HIGH (inactive)
    GPIO_SetBits(GPIOE, GPIO_Pin_6 | GPIO_Pin_5);
    GPIO_SetBits(GPIOA, GPIO_Pin_4);

    SPI_I2S_DeInit(SPI1);
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;    // Clock idle LOW (Mode 0)
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;  // Sample on first edge (Mode 0)
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;     // Software NSS control
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_Init(SPI1, &SPI_InitStructure);

    SPI_Cmd(SPI1, ENABLE);  // Enable once and keep enabled (legacy style)
}

static void legacy_LDAC_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;  // LDAC
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // Legacy default: LDAC low (transparent update) or high? Original set high then toggled.
    GPIOC->BSRRH = GPIO_Pin_4;  // Drive low (allows immediate update on write)
}

static void legacy_CLR_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;  // CLR
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIOC->BSRRL = GPIO_Pin_5;  // CLR high (inactive)
}

static void legacy_Write24(uint32_t word)
{
    uint8_t b2 = (uint8_t)(word >> 16);
    uint8_t b1 = (uint8_t)(word >> 8);
    uint8_t b0 = (uint8_t)(word);

    // CS LOW (PE6) - start transaction
    GPIO_ResetBits(GPIOE, GPIO_Pin_6);
    for (volatile int i = 0; i < 20; i++) __NOP();  // Setup delay

    // Send 3 bytes MSB first
    while (!(SPI1->SR & SPI_I2S_FLAG_TXE));
    SPI_I2S_SendData(SPI1, b2);
    while (!(SPI1->SR & SPI_I2S_FLAG_TXE));
    SPI_I2S_SendData(SPI1, b1);
    while (!(SPI1->SR & SPI_I2S_FLAG_TXE));
    SPI_I2S_SendData(SPI1, b0);
    while (!(SPI1->SR & SPI_I2S_FLAG_TXE));

    for (volatile int i = 0; i < 20; i++) __NOP();  // Hold delay

    // CS HIGH (PE6) - end transaction
    GPIO_SetBits(GPIOE, GPIO_Pin_6);

    for (volatile int d = 0; d < 30; d++) __NOP();  // Inter-frame delay
}

static void legacy_Config(void)
{
    // Power up all four channels + internal reference
    legacy_Write24(LEGACY_REG_POWER_CONTROL | LEGACY_POWER_UP_ALL);
    for (volatile int d = 0; d < 4000; ++d) __NOP();
    // Set all channels to ±5 V (bipolar) range
    legacy_Write24(LEGACY_REG_OUTPUT_RANGE | LEGACY_RANGE_N5_P5V | LEGACY_DAC_CH_ALL);
}

/* ========================= Wrapper API (expected by current code) ======== */

bool AD5724_Init(ad5724_instance_t* dac)
{
    if (!dac) return false;
    memset(dac, 0, sizeof(*dac));
    legacy_SPI1_Init();
    legacy_LDAC_Init();
    legacy_CLR_Init();
    // Configure device (repeat a few times like legacy robustness approach)
    for (int i = 0; i < 3; i++) legacy_Config();

    // Record state
    for (int ch = 0; ch < AD5724_NUM_CHANNELS; ++ch) {
        dac->range[ch] = AD5724_RANGE_0_10V;  // map to legacy ±5 V
        dac->last_voltage[ch] = 0.0f;
    }
    dac->initialized = true;
    return true;
}

void AD5724_Reset(ad5724_instance_t* dac)
{
    (void)dac;
    legacy_Config();
}

bool AD5724_SetRange(ad5724_instance_t* dac, ad5724_channel_t channel, ad5724_range_t range)
{
    // Legacy baseline only supports ±5 V right now. Accept but ignore others.
    if (!dac || channel >= AD5724_NUM_CHANNELS) return false;
    dac->range[channel] = AD5724_RANGE_N5_5V;  // force
    return true;
}

void AD5724_SetRangeAll(ad5724_instance_t* dac, ad5724_range_t range)
{
    (void)range;
    if (!dac) return;
    for (int ch = 0; ch < AD5724_NUM_CHANNELS; ++ch) dac->range[ch] = AD5724_RANGE_N5_5V;
}

static inline uint32_t legacy_channel_bits(ad5724_channel_t ch)
{
    switch (ch) {
        case AD5724_CH_A:
            return LEGACY_DAC_CH_A;
        case AD5724_CH_B:
            return LEGACY_DAC_CH_B;
        case AD5724_CH_C:
            return LEGACY_DAC_CH_C;
        case AD5724_CH_D:
            return LEGACY_DAC_CH_D;
        default:
            return LEGACY_DAC_CH_A;
    }
}

// Legacy voltage conversion for ±5 V: code = (V + 5)/10 * 65535
static uint16_t legacy_voltage_to_code(float v)
{
    if (v < -5.0f) v = -5.0f;
    if (v > 5.0f) v = 5.0f;
    float norm = (v + 5.0f) * (1.0f / 10.0f);  // 0..1
    uint32_t code = (uint32_t)(norm * 65535.0f + 0.5f);
    if (code > 65535U) code = 65535U;
    return (uint16_t)code;
}

bool AD5724_SetVoltage(ad5724_instance_t* dac, ad5724_channel_t channel, float voltage_v)
{
    if (!dac || channel >= AD5724_NUM_CHANNELS) return false;
    uint16_t code = legacy_voltage_to_code(voltage_v);
    dac->last_voltage[channel] = voltage_v;
    legacy_Write24(LEGACY_REG_DAC | legacy_channel_bits(channel) | code);
    return true;
}

bool AD5724_SetCode(ad5724_instance_t* dac, ad5724_channel_t channel, uint16_t code)
{
    if (!dac || channel >= AD5724_NUM_CHANNELS) return false;
    legacy_Write24(LEGACY_REG_DAC | legacy_channel_bits(channel) | code);
    return true;
}

void AD5724_PowerUpChannel(ad5724_instance_t* dac, ad5724_channel_t channel)
{
    (void)dac;
    (void)channel;
}
void AD5724_PowerDownChannel(ad5724_instance_t* dac, ad5724_channel_t channel)
{
    (void)dac;
    (void)channel;
}
void AD5724_PowerUpAll(ad5724_instance_t* dac)
{
    (void)dac;
    legacy_Write24(LEGACY_REG_POWER_CONTROL | LEGACY_POWER_UP_ALL);
}

uint16_t AD5724_VoltageToCode(float voltage_v, ad5724_range_t range)
{
    (void)range;
    return legacy_voltage_to_code(voltage_v);
}

bool AD5724_IsVoltageInRange(float voltage_v, ad5724_range_t range)
{
    (void)range;
    return (voltage_v >= -5.0f && voltage_v <= 5.0f);
}

/* ========================= End legacy restored implementation ============ */
