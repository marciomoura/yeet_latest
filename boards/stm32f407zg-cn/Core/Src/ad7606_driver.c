/**
 ******************************************************************************
 * @file    ad7606_driver.c
 * @brief   AD7606 8-channel ADC driver implementation
 * @date    2025
 ******************************************************************************
 */

#include "ad7606_driver.h"

/* ============================================================================
 * Private Variables
 * ============================================================================ */

/** @brief Current voltage range setting */
static ad7606_range_t s_current_range = AD7606_RANGE_10V;

/* ============================================================================
 * Private Function Prototypes
 * ============================================================================ */

static void AD7606_DelayUs(uint32_t us);

/* ============================================================================
 * Public Function Implementations
 * ============================================================================ */

/**
 * @brief Initialize AD7606 ADC
 */
void AD7606_Init(void)
{
    // Perform hardware reset
    AD7606_Reset();

    // Set default range to ±10V
    AD7606_SetRange(AD7606_RANGE_10V);

    // Set CONVSTA/B to idle state (HIGH)
    HAL_GPIO_WritePin(AD7606_CONVST_PORT, AD7606_CONVST_PINS, GPIO_PIN_SET);

    // Small delay to stabilize
    AD7606_DelayUs(10);
}

/**
 * @brief Configure AD7606 voltage range
 */
void AD7606_SetRange(ad7606_range_t range)
{
    s_current_range = range;

    if (range == AD7606_RANGE_10V) {
        // RANGE pin HIGH = ±10V
        HAL_GPIO_WritePin(AD7606_RANGE_PORT, AD7606_RANGE_PIN, GPIO_PIN_SET);
    }
    else {
        // RANGE pin LOW = ±5V
        HAL_GPIO_WritePin(AD7606_RANGE_PORT, AD7606_RANGE_PIN, GPIO_PIN_RESET);
    }

    // Allow range change to settle
    AD7606_DelayUs(1);
}

/**
 * @brief Reset AD7606 ADC
 */
void AD7606_Reset(void)
{
    // Reset sequence: LOW → HIGH → LOW
    // Hold reset LOW for at least 50ns
    HAL_GPIO_WritePin(AD7606_RESET_PORT, AD7606_RESET_PIN, GPIO_PIN_RESET);
    AD7606_DelayUs(1);

    // Pulse HIGH for at least 50ns
    HAL_GPIO_WritePin(AD7606_RESET_PORT, AD7606_RESET_PIN, GPIO_PIN_SET);
    AD7606_DelayUs(1);

    // Return to LOW (idle state)
    HAL_GPIO_WritePin(AD7606_RESET_PORT, AD7606_RESET_PIN, GPIO_PIN_RESET);
    AD7606_DelayUs(1);
}

/**
 * @brief Start AD7606 conversion
 */
void AD7606_StartConversion(void)
{
    // Conversion trigger: HIGH → LOW → HIGH (minimum 50ns pulse)
    // Start high (idle)
    HAL_GPIO_WritePin(AD7606_CONVST_PORT, AD7606_CONVST_PINS, GPIO_PIN_SET);

    // Pulse LOW to trigger conversion (>50ns)
    HAL_GPIO_WritePin(AD7606_CONVST_PORT, AD7606_CONVST_PINS, GPIO_PIN_RESET);

    // Small delay (at 168 MHz, few cycles = >50ns)
    for (volatile int i = 0; i < 20; i++) {
        __NOP();
    }

    // Return HIGH to complete pulse
    HAL_GPIO_WritePin(AD7606_CONVST_PORT, AD7606_CONVST_PINS, GPIO_PIN_SET);
}

/**
 * @brief Check if AD7606 conversion is complete
 */
bool AD7606_IsConversionReady(void)
{
    // BUSY pin: LOW = conversion complete, HIGH = busy
    return (HAL_GPIO_ReadPin(AD7606_BUSY_PORT, AD7606_BUSY_PIN) == GPIO_PIN_RESET);
}

/**
 * @brief Wait for conversion to complete (blocking)
 */
bool AD7606_WaitConversion(uint32_t timeout_us)
{
    uint32_t start_tick = HAL_GetTick();
    uint32_t timeout_ms = (timeout_us + 999) / 1000;  // Convert µs to ms (round up)

    if (timeout_us == 0) {
        // No timeout, wait forever
        while (!AD7606_IsConversionReady()) {
            __NOP();
        }
        return true;
    }

    // Wait with timeout
    while (!AD7606_IsConversionReady()) {
        if ((HAL_GetTick() - start_tick) > timeout_ms) {
            return false;  // Timeout
        }
    }

    return true;  // Conversion complete
}

/**
 * @brief Read all 8 channels from AD7606
 */
void AD7606_Read(ad7606_data_t* data)
{
    if (data == NULL) {
        return;
    }

    // Wait for conversion to complete (with timeout)
    if (!AD7606_WaitConversion(100)) {
        // Timeout - mark data as invalid
        data->conversion_ready = false;
        return;
    }

    // Access FSMC data register
    volatile uint16_t* pData = (volatile uint16_t*)AD7606_DATA_ADDR;

    // Read all 8 channels sequentially
    // Each read automatically advances to next channel
    for (int i = 0; i < AD7606_NUM_CHANNELS; i++) {
        data->raw[i] = *pData;
        data->voltage[i] = AD7606_RawToVoltage(data->raw[i], s_current_range);
    }

    // Update metadata
    data->range = s_current_range;
    data->conversion_ready = true;
    data->conversion_count++;
}

/**
 * @brief Start conversion and read previous results (pipeline mode)
 */
void AD7606_ReadAndStart(ad7606_data_t* data)
{
    // Read previous conversion results
    AD7606_Read(data);

    // Immediately start next conversion
    AD7606_StartConversion();
}

/**
 * @brief Convert raw ADC value to voltage
 */
float AD7606_RawToVoltage(uint16_t raw_value, ad7606_range_t range)
{
    int16_t signed_value;
    float voltage_range;

    // Determine voltage range
    voltage_range = (range == AD7606_RANGE_10V) ? AD7606_VOLTAGE_RANGE_10V : AD7606_VOLTAGE_RANGE_5V;

    // Convert unsigned 16-bit to signed 16-bit (two's complement)
    if (raw_value & 0x8000) {
        // Negative value
        signed_value = (int16_t)(raw_value - 0x10000);
    }
    else {
        // Positive value
        signed_value = (int16_t)raw_value;
    }

    // Convert to voltage: (signed_value / 32768) * voltage_range
    return ((float)signed_value * voltage_range) / (float)AD7606_MAX_COUNT;
}

/* ============================================================================
 * Private Function Implementations
 * ============================================================================ */

/**
 * @brief Simple microsecond delay
 * @param us Delay in microseconds
 * @note Uses busy-wait loop, not accurate for short delays
 * @note At 168 MHz: 1 µs ≈ 168 cycles
 */
static void AD7606_DelayUs(uint32_t us)
{
    // Approximate: 168 MHz / 4 cycles per loop ≈ 42 loops per µs
    uint32_t loops = us * 42;

    for (volatile uint32_t i = 0; i < loops; i++) {
        __NOP();
    }
}
