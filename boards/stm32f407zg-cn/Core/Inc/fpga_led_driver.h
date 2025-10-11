/**
 ******************************************************************************
 * @file    fpga_led_driver.h
 * @brief   FPGA-controlled status LED driver for STM32F407ZG-CN board
 * @date    2025
 ******************************************************************************
 * @attention
 *
 * LED Configuration:
 * - 3 status LEDs controlled via FPGA (FSMC Bank4 memory-mapped interface)
 * - LED 0: Task 1 heartbeat (5 kHz @ 1Hz blink = Running indicator)
 * - LED 1: Task 2 heartbeat (1 kHz @ 1Hz blink = Fault indicator)
 * - LED 2: Task 3 heartbeat (100 Hz @ 1Hz blink = Alarm indicator)
 *
 * Hardware Interface:
 * - FSMC Bank4 base address: 0x6C000000
 * - LED register offset: 7 (word address → byte offset 14)
 * - Register address: 0x6C00000E
 * - Data format: 16-bit word, bits [2:0] = LED states
 *
 ******************************************************************************
 */

#ifndef FPGA_LED_DRIVER_H
#define FPGA_LED_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* ============================================================================
 * Hardware Configuration
 * ============================================================================ */

/** @brief FSMC Bank4 base address (FPGA digital I/O interface) */
#define FPGA_LED_BASE_ADDR 0x6C000000U

/** @brief LED register word offset (multiply by 2 for byte offset) */
#define FPGA_LED_WORD_OFFSET 7U

/** @brief Complete LED register address */
#define FPGA_LED_REG_ADDR (FPGA_LED_BASE_ADDR + (FPGA_LED_WORD_OFFSET * 2))

/** @brief Number of controllable LEDs */
#define FPGA_LED_COUNT 3U

/* ============================================================================
 * LED Enumeration
 * ============================================================================ */

/**
 * @brief LED identifiers
 */
typedef enum {
    FPGA_LED_TASK1 = 0, /**< LED 0: Task 1 heartbeat (5 kHz) - Running indicator */
    FPGA_LED_TASK2 = 1, /**< LED 1: Task 2 heartbeat (1 kHz) - Fault indicator */
    FPGA_LED_TASK3 = 2, /**< LED 2: Task 3 heartbeat (100 Hz) - Alarm indicator */
} fpga_led_id_t;

/* ============================================================================
 * Data Structures
 * ============================================================================ */

/**
 * @brief LED state structure for all 3 LEDs
 */
typedef struct {
    bool task1_led; /**< LED 0 state: true = ON, false = OFF */
    bool task2_led; /**< LED 1 state: true = ON, false = OFF */
    bool task3_led; /**< LED 2 state: true = ON, false = OFF */
} fpga_led_state_t;

/* ============================================================================
 * Public Function Prototypes
 * ============================================================================ */

/**
 * @brief Initialize FPGA LED driver (turn all LEDs OFF)
 * @note FSMC Bank4 must be initialized before calling this function
 */
void FPGA_LED_Init(void);

/**
 * @brief Set individual LED state
 * @param led LED identifier (FPGA_LED_TASK1, FPGA_LED_TASK2, or FPGA_LED_TASK3)
 * @param state LED state: true = ON, false = OFF
 */
void FPGA_LED_Set(fpga_led_id_t led, bool state);

/**
 * @brief Toggle individual LED
 * @param led LED identifier (FPGA_LED_TASK1, FPGA_LED_TASK2, or FPGA_LED_TASK3)
 */
void FPGA_LED_Toggle(fpga_led_id_t led);

/**
 * @brief Set all LEDs at once
 * @param state Pointer to LED state structure
 * @note This is the most efficient method for updating multiple LEDs
 */
void FPGA_LED_SetAll(const fpga_led_state_t* state);

/**
 * @brief Get current LED state
 * @param led LED identifier
 * @return true if LED is ON, false if OFF
 */
bool FPGA_LED_GetState(fpga_led_id_t led);

/**
 * @brief Turn all LEDs OFF
 */
void FPGA_LED_AllOff(void);

/**
 * @brief Turn all LEDs ON
 */
void FPGA_LED_AllOn(void);

/**
 * @brief Get all LED states
 * @param state Pointer to structure to receive LED states
 */
void FPGA_LED_GetAll(fpga_led_state_t* state);

#ifdef __cplusplus
}
#endif

#endif /* FPGA_LED_DRIVER_H */
