/**
 ******************************************************************************
 * @file    fsmc_gpio_driver.h
 * @brief   FSMC-based GPIO expansion hardware driver for STM32F407
 * @date    2025
 ******************************************************************************
 * @attention
 *
 * FSMC GPIO Expansion Configuration:
 * - 48 digital inputs: DI1-DI48 (3 banks of 16 bits)
 * - 48 digital outputs: DO1-DO48 (3 banks of 16 bits)
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
#define FSMC_GPIO_DI_ADDR_0 201U /**< DI1-DI16 */
#define FSMC_GPIO_DI_ADDR_1 202U /**< DI17-DI32 */
#define FSMC_GPIO_DI_ADDR_2 203U /**< DI33-DI48 */

/** @brief Digital output register addresses (word offsets) */
#define FSMC_GPIO_DO_ADDR_0 3U /**< DO1-DO16 */
#define FSMC_GPIO_DO_ADDR_1 4U /**< DO17-DO32 */
#define FSMC_GPIO_DO_ADDR_2 5U /**< DO33-DO48 */
#define FSMC_GPIO_LED_ADDR  7U /**< LED indicators (DO49-DO51) */

/* ============================================================================
 * Configuration Constants
 * ============================================================================ */

/** @brief Total number of digital inputs */
#define FSMC_GPIO_NUM_DI 48

/** @brief Total number of digital outputs */
#define FSMC_GPIO_NUM_DO 48

/** @brief Debounce filter delay (number of consecutive stable reads required) */
#define FSMC_GPIO_DEBOUNCE_CYCLES 4

/* ============================================================================
 * Data Structures
 * ============================================================================ */

/**
 * @brief FSMC GPIO driver instance structure
 */
typedef struct {
    uint8_t di_raw[FSMC_GPIO_NUM_DI];       /**< Raw digital input state (DI1-DI48) */
    uint8_t di_debounced[FSMC_GPIO_NUM_DI]; /**< Debounced digital input state (DI1-DI48) */
    uint8_t do_state[FSMC_GPIO_NUM_DO];     /**< Digital output state buffer (DO1-DO48) */
    int16_t di_delay[FSMC_GPIO_NUM_DI];     /**< Debounce delay counters */
    uint8_t led_run;                        /**< RUN LED state (1=ON/Blink, 0=OFF) */
    uint8_t led_fault;                      /**< FAULT LED state (1=ON, 0=OFF) */
    uint8_t led_alarm;                      /**< ALARM LED state (1=ON, 0=OFF) */
    bool initialized;                       /**< Initialization flag */
    uint32_t read_count;                    /**< Total input reads performed */
    uint32_t write_count;                   /**< Total output writes performed */
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
 * @note Clears all I/O buffers and debounce counters
 */
void FSMC_GPIO_Reset(fsmc_gpio_instance_t* gpio);

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
 * @brief Get raw input state
 * @param gpio Pointer to FSMC GPIO instance
 * @param input Input number (1-48)
 * @retval Input bit value (1=High, 0=Low)
 */
uint8_t FSMC_GPIO_GetRawInput(const fsmc_gpio_instance_t* gpio, uint8_t input);

/**
 * @brief Get debounced input state
 * @param gpio Pointer to FSMC GPIO instance
 * @param input Input number (1-48)
 * @retval Input bit value (1=High, 0=Low)
 */
uint8_t FSMC_GPIO_GetDebouncedInput(const fsmc_gpio_instance_t* gpio, uint8_t input);

/* ============================================================================
 * Function Prototypes - Digital Output Operations
 * ============================================================================ */

/**
 * @brief Set output state
 * @param gpio Pointer to FSMC GPIO instance
 * @param output Output number (1-48)
 * @param value Output bit value (1=High, 0=Low)
 */
void FSMC_GPIO_SetOutput(fsmc_gpio_instance_t* gpio, uint8_t output, uint8_t value);

/**
 * @brief Get output state
 * @param gpio Pointer to FSMC GPIO instance
 * @param output Output number (1-48)
 * @retval Output bit value (1=High, 0=Low)
 */
uint8_t FSMC_GPIO_GetOutput(const fsmc_gpio_instance_t* gpio, uint8_t output);

/**
 * @brief Write all digital outputs to hardware
 * @param gpio Pointer to FSMC GPIO instance
 * @note Writes do_state array to hardware
 */
void FSMC_GPIO_WriteOutputs(fsmc_gpio_instance_t* gpio);

/* ============================================================================
 * Function Prototypes - Combined Operations
 * ============================================================================ */

/**
 * @brief Complete I/O update cycle (read inputs, apply debounce, write outputs)
 * @param gpio Pointer to FSMC GPIO instance
 * @note Convenience function for periodic I/O update
 */
void FSMC_GPIO_Update(fsmc_gpio_instance_t* gpio);

/* ============================================================================
 * Function Prototypes - LED Control (Separate from Digital Outputs)
 * ============================================================================ */

/**
 * @brief Set RUN LED state (address 7, bit 0)
 * @param gpio Pointer to FSMC GPIO instance
 * @param state LED state (1=ON/Blink, 0=OFF)
 * @note RUN LED typically blinks during normal operation
 */
void FSMC_GPIO_SetLED_Run(fsmc_gpio_instance_t* gpio, uint8_t state);

/**
 * @brief Set FAULT LED state (address 7, bit 1)
 * @param gpio Pointer to FSMC GPIO instance
 * @param state LED state (1=ON, 0=OFF)
 * @note FAULT LED indicates error conditions
 */
void FSMC_GPIO_SetLED_Fault(fsmc_gpio_instance_t* gpio, uint8_t state);

/**
 * @brief Set ALARM LED state (address 7, bit 2)
 * @param gpio Pointer to FSMC GPIO instance
 * @param state LED state (1=ON, 0=OFF)
 * @note ALARM LED indicates warning conditions
 */
void FSMC_GPIO_SetLED_Alarm(fsmc_gpio_instance_t* gpio, uint8_t state);

/**
 * @brief Write all LED states to hardware
 * @param gpio Pointer to FSMC GPIO instance
 * @note Updates LEDs at address 7 based on internal LED state variables
 */
void FSMC_GPIO_WriteLEDs(fsmc_gpio_instance_t* gpio);

#ifdef __cplusplus
}
#endif

#endif /* __FSMC_GPIO_DRIVER_H */
