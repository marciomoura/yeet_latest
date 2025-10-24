/**
 ******************************************************************************
 * @file    fsmc_gpio_driver.c
 * @brief   FSMC-based GPIO expansion hardware driver implementation
 * @date    2025
 ******************************************************************************
 */

#include "fsmc_gpio_driver.h"

#include <string.h>

#include "stm32f4xx_fsmc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

/* ============================================================================
 * Private Function Prototypes
 * ============================================================================ */

static void FSMC_GPIO_InitPins(void);
static void FSMC_GPIO_InitFSMC(void);
static void FSMC_GPIO_ApplyDebounceSingle(uint8_t current, uint8_t* filtered, int16_t* delay_counter);
static void FSMC_GPIO_ReadWord(uint16_t addr, uint8_t* buffer);
static void FSMC_GPIO_WriteWord(uint16_t addr, const uint8_t* buffer);

/* ============================================================================
 * Public Function Implementations - Initialization
 * ============================================================================ */

/**
 * @brief Initialize FSMC GPIO driver instance
 */
bool FSMC_GPIO_Init(fsmc_gpio_instance_t* gpio)
{
    if (gpio == NULL) {
        return false;
    }

    // Clear instance structure
    memset(gpio, 0, sizeof(fsmc_gpio_instance_t));

    // Initialize hardware
    FSMC_GPIO_InitPins();
    FSMC_GPIO_InitFSMC();

    gpio->initialized = true;
    gpio->read_count = 0;
    gpio->write_count = 0;

    // Perform initial read and double-filter to synchronize states
    FSMC_GPIO_ReadInputs(gpio);
    FSMC_GPIO_ApplyDebounce(gpio);
    FSMC_GPIO_ApplyDebounce(gpio);  // Force di_raw == di_debounced at startup

    return true;
}

/**
 * @brief Reset FSMC GPIO driver state
 */
void FSMC_GPIO_Reset(fsmc_gpio_instance_t* gpio)
{
    if (gpio == NULL) {
        return;
    }

    // Clear all I/O buffers and delay counters
    memset(gpio->di_raw, 0, sizeof(gpio->di_raw));
    memset(gpio->di_debounced, 0, sizeof(gpio->di_debounced));
    memset(gpio->di_delay, 0, sizeof(gpio->di_delay));
    memset(gpio->do_state, 0, sizeof(gpio->do_state));

    // Initialize LEDs to default state
    gpio->led_run = 1;    // RUN LED blinks during operation
    gpio->led_fault = 0;  // FAULT LED off by default
    gpio->led_alarm = 0;  // ALARM LED off by default
}

/* ============================================================================
 * Public Function Implementations - Raw Hardware Access
 * ============================================================================ */

/**
 * @brief Read 16 digital inputs from hardware
 */
static void FSMC_GPIO_ReadWord(uint16_t addr, uint8_t* buffer)
{
    if (buffer == NULL) {
        return;
    }

    // Read 16-bit word from FSMC address
    volatile uint16_t* fsmc_addr = (volatile uint16_t*)(FSMC_GPIO_BASE_ADDR + addr * 2);
    uint16_t data = *fsmc_addr;

    // Unpack bits into byte array (LSB first, matching legacy order)
    for (int i = 0; i < 16; i++) {
        buffer[i] = (data & 0x0001) ? 1 : 0;
        data >>= 1;
    }
}

/**
 * @brief Write 16 digital outputs to hardware
 */
static void FSMC_GPIO_WriteWord(uint16_t addr, const uint8_t* buffer)
{
    if (buffer == NULL) {
        return;
    }

    uint16_t data = 0;

    // Pack byte array into 16-bit word (matching legacy order: buffer[0] -> bit0)
    for (int i = 0; i < 16; i++) {
        data >>= 1;
        if (buffer[i]) {
            data |= 0x8000;
        }
    }

    // Write to FSMC address
    volatile uint16_t* fsmc_addr = (volatile uint16_t*)(FSMC_GPIO_BASE_ADDR + addr * 2);
    *fsmc_addr = data;

    // Small delay for hardware settling (6 CPU cycles @ 168MHz = ~36ns)
    for (volatile int i = 0; i < 6; i++) {
        __NOP();
    }
}

/* ============================================================================
 * Public Function Implementations - Digital Input Operations
 * ============================================================================ */

/**
 * @brief Read all digital inputs from hardware (raw, no debouncing)
 */
void FSMC_GPIO_ReadInputs(fsmc_gpio_instance_t* gpio)
{
    if (gpio == NULL) {
        return;
    }

    // Read digital inputs from hardware (48 inputs across 3 words)
    FSMC_GPIO_ReadWord(FSMC_GPIO_DI_ADDR_0, &gpio->di_raw[0]);   // DI1..DI16
    FSMC_GPIO_ReadWord(FSMC_GPIO_DI_ADDR_1, &gpio->di_raw[16]);  // DI17..DI32
    FSMC_GPIO_ReadWord(FSMC_GPIO_DI_ADDR_2, &gpio->di_raw[32]);  // DI33..DI48

    gpio->read_count++;
}

/**
 * @brief Apply debounce filter to all digital inputs
 */
void FSMC_GPIO_ApplyDebounce(fsmc_gpio_instance_t* gpio)
{
    if (gpio == NULL) {
        return;
    }

    // Apply debounce filter to all inputs
    for (int i = 0; i < FSMC_GPIO_NUM_DI; i++) {
        FSMC_GPIO_ApplyDebounceSingle(gpio->di_raw[i], &gpio->di_debounced[i], &gpio->di_delay[i]);
    }
}

/**
 * @brief Get raw input state
 */
uint8_t FSMC_GPIO_GetRawInput(const fsmc_gpio_instance_t* gpio, uint8_t input)
{
    if (gpio == NULL || input < 1 || input > FSMC_GPIO_NUM_DI) {
        return 0;
    }
    return gpio->di_raw[input - 1];  // Convert 1-based to 0-based index
}

/**
 * @brief Get debounced input state
 */
uint8_t FSMC_GPIO_GetDebouncedInput(const fsmc_gpio_instance_t* gpio, uint8_t input)
{
    if (gpio == NULL || input < 1 || input > FSMC_GPIO_NUM_DI) {
        return 0;
    }
    return gpio->di_debounced[input - 1];  // Convert 1-based to 0-based index
}

/* ============================================================================
 * Public Function Implementations - Digital Output Operations
 * ============================================================================ */

/**
 * @brief Set output state
 */
void FSMC_GPIO_SetOutput(fsmc_gpio_instance_t* gpio, uint8_t output, uint8_t value)
{
    if (gpio == NULL || output < 1 || output > FSMC_GPIO_NUM_DO) {
        return;
    }

    gpio->do_state[output - 1] = value ? 1 : 0;  // Convert 1-based to 0-based index
}

/**
 * @brief Get output state
 */
uint8_t FSMC_GPIO_GetOutput(const fsmc_gpio_instance_t* gpio, uint8_t output)
{
    if (gpio == NULL || output < 1 || output > FSMC_GPIO_NUM_DO) {
        return 0;
    }
    return gpio->do_state[output - 1];  // Convert 1-based to 0-based index
}

/**
 * @brief Write all digital outputs to hardware
 */
void FSMC_GPIO_WriteOutputs(fsmc_gpio_instance_t* gpio)
{
    if (gpio == NULL) {
        return;
    }

    // Write digital outputs (48 outputs across 3 words)
    FSMC_GPIO_WriteWord(FSMC_GPIO_DO_ADDR_0, &gpio->do_state[0]);   // DO1..DO16
    FSMC_GPIO_WriteWord(FSMC_GPIO_DO_ADDR_1, &gpio->do_state[16]);  // DO17..DO32
    FSMC_GPIO_WriteWord(FSMC_GPIO_DO_ADDR_2, &gpio->do_state[32]);  // DO33..DO48

    // Strobe signal (address 6, required for CPLD/FPGA to latch outputs)
    // Legacy driver does this between DO and LED writes
    volatile uint16_t* fsmc_addr = (volatile uint16_t*)(FSMC_GPIO_BASE_ADDR + 12);
    *fsmc_addr = 0;
    for (volatile int i = 0; i < 6; i++) {
        __NOP();
    }

    // NOTE: LED indicators are written separately via FSMC_GPIO_WriteLEDs()
    // Do not overwrite them here!

    gpio->write_count++;
}

/* ============================================================================
 * Public Function Implementations - Combined Operations
 * ============================================================================ */

/**
 * @brief Complete I/O update cycle (read inputs, apply debounce, write outputs)
 */
void FSMC_GPIO_Update(fsmc_gpio_instance_t* gpio)
{
    if (gpio == NULL) {
        return;
    }

    FSMC_GPIO_ReadInputs(gpio);
    FSMC_GPIO_ApplyDebounce(gpio);
    FSMC_GPIO_WriteOutputs(gpio);
}

/* ============================================================================
 * Private Function Implementations - Hardware Initialization
 * ============================================================================ */

/**
 * @brief Initialize GPIO pins for FSMC
 */
static void FSMC_GPIO_InitPins(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // Enable GPIO clocks
    RCC_AHB1PeriphClockCmd(
        RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOF | RCC_AHB1Periph_GPIOG, ENABLE);

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
}

/**
 * @brief Initialize FSMC interface for GPIO expansion hardware
 */
static void FSMC_GPIO_InitFSMC(void)
{
    FSMC_NORSRAMInitTypeDef FSMC_NORSRAMInitStructure;
    FSMC_NORSRAMTimingInitTypeDef timing;

    // Enable FSMC clock
    RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC, ENABLE);

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

/* ============================================================================
 * Private Function Implementations - Debounce Logic
 * ============================================================================ */

/**
 * @brief Apply debounce filter to a single input
 */
static void FSMC_GPIO_ApplyDebounceSingle(uint8_t current, uint8_t* filtered, int16_t* delay_counter)
{
    if (*filtered != current) {
        // Input has changed - increment delay counter
        (*delay_counter)++;

        if (*delay_counter >= FSMC_GPIO_DEBOUNCE_CYCLES) {
            // Input stable for required cycles - accept change
            *filtered = current;
            *delay_counter = 0;
        }
    }
    else {
        // Input matches filtered value - reset delay counter
        *delay_counter = 0;
    }
}

/* ============================================================================
 * Public Function Implementations - LED Control
 * ============================================================================ */

/**
 * @brief Set RUN LED state
 */
void FSMC_GPIO_SetLED_Run(fsmc_gpio_instance_t* gpio, uint8_t state)
{
    if (gpio == NULL) {
        return;
    }
    gpio->led_run = (state != 0) ? 1 : 0;
}

/**
 * @brief Set FAULT LED state
 */
void FSMC_GPIO_SetLED_Fault(fsmc_gpio_instance_t* gpio, uint8_t state)
{
    if (gpio == NULL) {
        return;
    }
    gpio->led_fault = (state != 0) ? 1 : 0;
}

/**
 * @brief Set ALARM LED state
 */
void FSMC_GPIO_SetLED_Alarm(fsmc_gpio_instance_t* gpio, uint8_t state)
{
    if (gpio == NULL) {
        return;
    }
    gpio->led_alarm = (state != 0) ? 1 : 0;
}

/**
 * @brief Write all LED states to hardware
 */
void FSMC_GPIO_WriteLEDs(fsmc_gpio_instance_t* gpio)
{
    if (gpio == NULL || !gpio->initialized) {
        return;
    }

    // Create LED buffer (16 bits, only first 3 used)
    uint8_t led_buffer[16] = {0};

    led_buffer[0] = gpio->led_run;    // RUN LED (bit 0)
    led_buffer[1] = gpio->led_fault;  // FAULT LED (bit 1)
    led_buffer[2] = gpio->led_alarm;  // ALARM LED (bit 2)

    // Write to LED address (address 7)
    FSMC_GPIO_WriteWord(FSMC_GPIO_LED_ADDR, led_buffer);
}
