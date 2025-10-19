/**
 ******************************************************************************
 * @file    fsmc_gpio_driver.h
 * @brief   FSMC-based GPIO expansion hardware driver for STM32F407
 * @date    2025
 ******************************************************************************
 * @attention
 *
 * FSMC GPIO Expansion Configuration:
 * - 64 digital inputs: DI00-DI63 (8x8 matrix)
 * - 64 digital outputs: DO00-DO63 (8x8 matrix)
 * - Interface: FSMC Bank 4 (0x6C000000)
 * - Data width: 16-bit words
 * - Debounce filter: 2-cycle delay for stable input detection
 *
 * Hardware Mapping:
 * - Data lines: PD0,1,8-10,14,15 + PE7-15 (D0-D15)
 * - Address lines: PF0-5,12-15 + PG0-5 (A0-A17)
 * - Control lines: PD4(NOE), PD5(NWE), PG12(NE4)
 *
 ******************************************************************************
 */

#ifndef __FSMC_GPIO_DRIVER_H
#define __FSMC_GPIO_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ============================================================================
 * Memory Map and Register Definitions
 * ============================================================================ */

/** @brief FSMC Bank 4 base address for GPIO expansion hardware */
#define FSMC_GPIO_BASE_ADDR 0x6C000000U

/** @brief Digital input register addresses (word offsets) */
#define FSMC_GPIO_DI_ADDR_0 201U /**< DI00-DI15 */
#define FSMC_GPIO_DI_ADDR_1 202U /**< DI16-DI31 */
#define FSMC_GPIO_DI_ADDR_2 203U /**< DI32-DI47 */

/** @brief Digital output register addresses (word offsets) */
#define FSMC_GPIO_DO_ADDR_0 3U /**< DO00-DO15 */
#define FSMC_GPIO_DO_ADDR_1 4U /**< DO16-DO31 */
#define FSMC_GPIO_DO_ADDR_2 5U /**< DO32-DO47 */
#define FSMC_GPIO_LED_ADDR  7U /**< LED indicators */

/* ============================================================================
 * Configuration Constants
 * ============================================================================ */

/** @brief Number of digital I/O rows (8 bits per row) */
#define FSMC_GPIO_NUM_ROWS 8

/** @brief Number of digital I/O columns (8 bits per column) */
#define FSMC_GPIO_NUM_COLS 8

/** @brief Debounce filter delay (number of consecutive stable reads required) */
#define FSMC_GPIO_DEBOUNCE_CYCLES 2

/* ============================================================================
 * Data Structures
 * ============================================================================ */

/**
 * @brief FSMC GPIO driver instance structure
 */
typedef struct {
    uint8_t di_raw[FSMC_GPIO_NUM_ROWS][FSMC_GPIO_NUM_COLS];       /**< Raw digital input state */
    uint8_t di_debounced[FSMC_GPIO_NUM_ROWS][FSMC_GPIO_NUM_COLS]; /**< Debounced digital input state */
    uint8_t do_state[FSMC_GPIO_NUM_ROWS][FSMC_GPIO_NUM_COLS];     /**< Digital output state buffer */
    int16_t di_delay[FSMC_GPIO_NUM_ROWS][FSMC_GPIO_NUM_COLS];     /**< Debounce delay counters */
    uint8_t led_run;                                              /**< Running LED state (1=Blinking, 0=Off) */
    uint8_t led_fault;                                            /**< Fault LED state (1=On, 0=Off) */
    uint8_t led_alarm;                                            /**< Alarm LED state (1=On, 0=Off) */
    bool initialized;                                             /**< Initialization flag */
    uint32_t read_count;                                          /**< Total input reads performed */
    uint32_t write_count;                                         /**< Total output writes performed */
} fsmc_gpio_instance_t;

/* ============================================================================
 * Function Prototypes - Initialization
 * ============================================================================ */

/**
 * @brief Initialize FSMC GPIO driver instance
 * @param gpio Pointer to FSMC GPIO instance structure
 * @retval true if initialization successful
 * @retval false if initialization failed
 * @note Configures FSMC interface and initializes all I/O buffers to zero
 */
bool FSMC_GPIO_Init(fsmc_gpio_instance_t* gpio);

/**
 * @brief Reset FSMC GPIO driver state
 * @param gpio Pointer to FSMC GPIO instance
 * @note Clears all I/O buffers and debounce counters, resets LED states
 */
void FSMC_GPIO_Reset(fsmc_gpio_instance_t* gpio);

/* ============================================================================
 * Function Prototypes - Raw Hardware Access
 * ============================================================================ */

/**
 * @brief Read 16 digital inputs from hardware
 * @param addr Word address offset (relative to FSMC_GPIO_BASE_ADDR)
 * @param buffer Pointer to 16-byte buffer to store input bits
 * @note Each byte in buffer represents one input bit (1=High, 0=Low)
 */
void FSMC_GPIO_ReadWord(uint16_t addr, uint8_t* buffer);

/**
 * @brief Write 16 digital outputs to hardware
 * @param addr Word address offset (relative to FSMC_GPIO_BASE_ADDR)
 * @param buffer Pointer to 16-byte buffer containing output bits
 * @note Each byte in buffer represents one output bit (1=High, 0=Low)
 */
void FSMC_GPIO_WriteWord(uint16_t addr, const uint8_t* buffer);

/* ============================================================================
 * Function Prototypes - Digital Input Operations
 * ============================================================================ */

/**
 * @brief Read all digital inputs from hardware (raw, no debouncing)
 * @param gpio Pointer to FSMC GPIO instance
 * @note Updates di_raw array with current hardware state
 */
void FSMC_GPIO_ReadInputs(fsmc_gpio_instance_t* gpio);

/**
 * @brief Apply debounce filter to all digital inputs
 * @param gpio Pointer to FSMC GPIO instance
 * @note Updates di_debounced array with filtered input state
 *       Input must be stable for FSMC_GPIO_DEBOUNCE_CYCLES before change is accepted
 */
void FSMC_GPIO_ApplyDebounce(fsmc_gpio_instance_t* gpio);

/**
 * @brief Get raw input bit state
 * @param gpio Pointer to FSMC GPIO instance
 * @param row Row index (0-7)
 * @param col Column index (0-7)
 * @retval Input bit value (1=High, 0=Low)
 */
static inline uint8_t FSMC_GPIO_GetRawInput(const fsmc_gpio_instance_t* gpio, uint8_t row, uint8_t col)
{
    if (gpio == NULL || row >= FSMC_GPIO_NUM_ROWS || col >= FSMC_GPIO_NUM_COLS) {
        return 0;
    }
    return gpio->di_raw[row][col];
}

/**
 * @brief Get debounced input bit state
 * @param gpio Pointer to FSMC GPIO instance
 * @param row Row index (0-7)
 * @param col Column index (0-7)
 * @retval Input bit value (1=High, 0=Low)
 */
static inline uint8_t FSMC_GPIO_GetDebouncedInput(const fsmc_gpio_instance_t* gpio, uint8_t row, uint8_t col)
{
    if (gpio == NULL || row >= FSMC_GPIO_NUM_ROWS || col >= FSMC_GPIO_NUM_COLS) {
        return 0;
    }
    return gpio->di_debounced[row][col];
}

/**
 * @brief Get pointer to raw input array
 * @param gpio Pointer to FSMC GPIO instance
 * @retval Pointer to di_raw array (read-only)
 */
static inline const uint8_t* FSMC_GPIO_GetRawInputArray(const fsmc_gpio_instance_t* gpio)
{
    if (gpio == NULL) {
        return NULL;
    }
    return (const uint8_t*)gpio->di_raw;
}

/**
 * @brief Get pointer to debounced input array
 * @param gpio Pointer to FSMC GPIO instance
 * @retval Pointer to di_debounced array (read-only)
 */
static inline const uint8_t* FSMC_GPIO_GetDebouncedInputArray(const fsmc_gpio_instance_t* gpio)
{
    if (gpio == NULL) {
        return NULL;
    }
    return (const uint8_t*)gpio->di_debounced;
}

/* ============================================================================
 * Function Prototypes - Digital Output Operations
 * ============================================================================ */

/**
 * @brief Set output bit state
 * @param gpio Pointer to FSMC GPIO instance
 * @param row Row index (0-7)
 * @param col Column index (0-7)
 * @param value Output bit value (1=High, 0=Low)
 */
void FSMC_GPIO_SetOutput(fsmc_gpio_instance_t* gpio, uint8_t row, uint8_t col, uint8_t value);

/**
 * @brief Get output bit state
 * @param gpio Pointer to FSMC GPIO instance
 * @param row Row index (0-7)
 * @param col Column index (0-7)
 * @retval Output bit value (1=High, 0=Low)
 */
static inline uint8_t FSMC_GPIO_GetOutput(const fsmc_gpio_instance_t* gpio, uint8_t row, uint8_t col)
{
    if (gpio == NULL || row >= FSMC_GPIO_NUM_ROWS || col >= FSMC_GPIO_NUM_COLS) {
        return 0;
    }
    return gpio->do_state[row][col];
}

/**
 * @brief Get pointer to output array
 * @param gpio Pointer to FSMC GPIO instance
 * @retval Pointer to do_state array
 */
static inline uint8_t* FSMC_GPIO_GetOutputArray(fsmc_gpio_instance_t* gpio)
{
    if (gpio == NULL) {
        return NULL;
    }
    return (uint8_t*)gpio->do_state;
}

/**
 * @brief Write all digital outputs to hardware
 * @param gpio Pointer to FSMC GPIO instance
 * @note Writes do_state array and LED indicators to hardware
 */
void FSMC_GPIO_WriteOutputs(fsmc_gpio_instance_t* gpio);

/* ============================================================================
 * Function Prototypes - LED Control
 * ============================================================================ */

/**
 * @brief Set LED indicator states (all at once)
 * @param gpio Pointer to FSMC GPIO instance
 * @param led_run Running LED state (1=Blinking, 0=Off)
 * @param led_fault Fault LED state (1=On, 0=Off)
 * @param led_alarm Alarm LED state (1=On, 0=Off)
 */
void FSMC_GPIO_SetLEDs(fsmc_gpio_instance_t* gpio, uint8_t led_run, uint8_t led_fault, uint8_t led_alarm);

/**
 * @brief Set RUN LED state
 * @param gpio Pointer to FSMC GPIO instance
 * @param state LED state (1=On, 0=Off)
 * @note Used as system heartbeat indicator
 */
void FSMC_GPIO_SetLED_Run(fsmc_gpio_instance_t* gpio, uint8_t state);

/**
 * @brief Set FAULT LED state
 * @param gpio Pointer to FSMC GPIO instance
 * @param state LED state (1=On, 0=Off)
 * @note Used to indicate hardware initialization or critical errors
 */
void FSMC_GPIO_SetLED_Fault(fsmc_gpio_instance_t* gpio, uint8_t state);

/**
 * @brief Set ALARM LED state
 * @param gpio Pointer to FSMC GPIO instance
 * @param state LED state (1=On, 0=Off)
 * @note Used to indicate application-level warnings or alarms
 */
void FSMC_GPIO_SetLED_Alarm(fsmc_gpio_instance_t* gpio, uint8_t state);

/**
 * @brief Toggle RUN LED state
 * @param gpio Pointer to FSMC GPIO instance
 * @note Convenience function for blinking the RUN LED
 */
void FSMC_GPIO_ToggleLED_Run(fsmc_gpio_instance_t* gpio);

/**
 * @brief Toggle FAULT LED state
 * @param gpio Pointer to FSMC GPIO instance
 */
void FSMC_GPIO_ToggleLED_Fault(fsmc_gpio_instance_t* gpio);

/**
 * @brief Toggle ALARM LED state
 * @param gpio Pointer to FSMC GPIO instance
 */
void FSMC_GPIO_ToggleLED_Alarm(fsmc_gpio_instance_t* gpio);

/**
 * @brief Get LED indicator states
 * @param gpio Pointer to FSMC GPIO instance
 * @param led_run Pointer to store running LED state (can be NULL)
 * @param led_fault Pointer to store fault LED state (can be NULL)
 * @param led_alarm Pointer to store alarm LED state (can be NULL)
 */
void FSMC_GPIO_GetLEDs(const fsmc_gpio_instance_t* gpio, uint8_t* led_run, uint8_t* led_fault, uint8_t* led_alarm);

/* ============================================================================
 * Function Prototypes - Combined Operations
 * ============================================================================ */

/**
 * @brief Complete I/O update cycle (read inputs, apply debounce, write outputs)
 * @param gpio Pointer to FSMC GPIO instance
 * @note Convenience function for periodic I/O update
 */
void FSMC_GPIO_Update(fsmc_gpio_instance_t* gpio);

#ifdef __cplusplus
}
#endif

#endif /* __FSMC_GPIO_DRIVER_H */
