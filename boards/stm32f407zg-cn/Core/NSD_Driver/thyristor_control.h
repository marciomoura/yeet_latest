/**
 ******************************************************************************
 * @file    thyristor_control.h
 * @brief   Thyristor firing pulse control for 3-phase converter
 * @date    2025
 ******************************************************************************
 * @attention
 *
 * Thyristor Configuration:
 * - 6 GPIO outputs for 3-phase positive and negative arms
 * - Direct GPIO register access for high-speed control
 * - Designed for 200µs interrupt timing
 *
 * Pin Mapping:
 * PC6 - Phase A Positive Arm (+A)
 * PB1 - Phase C Negative Arm (-C)
 * PC7 - Phase B Positive Arm (+B)
 * PC9 - Phase A Negative Arm (-A)
 * PC8 - Phase C Positive Arm (+C)
 * PB0 - Phase B Negative Arm (-B)
 *
 ******************************************************************************
 */

#ifndef __THYRISTOR_CONTROL_H
#define __THYRISTOR_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h" /* For GPIO_Pin_x definitions */

/* ============================================================================
 * Function Prototypes
 * ============================================================================ */

void Thyristor_DO_Init(void);

/**
 * @brief Set thyristor firing pulses by individual named arms
 *
 * This function provides direct control of each thyristor arm by name,
 * making the code more readable and maintainable. Uses direct GPIO
 * register access for maximum speed.
 *
 * @param p1 Phase A positive arm firing pulse (PC6)
 * @param p2 Phase C negative arm firing pulse (PB1)
 * @param p3 Phase B positive arm firing pulse (PC7)
 * @param p4 Phase A negative arm firing pulse (PC9)
 * @param p5 Phase C positive arm firing pulse (PC8)
 * @param p6 Phase B negative arm firing pulse (PB0)
 *
 * @note This function is optimized for real-time control (200µs interrupt)
 * @note Pin mapping follows the hardware schematic
 */
void set_thyristor_pulses(bool p1, bool p2, bool p3, bool p4, bool p5, bool p6);

/**
 * @brief Set all thyristor pulses to OFF (safe state)
 *
 * @note Use this function for emergency shutdown or initialization
 */
void thyristor_all_off(void);

/**
 * @brief Set thyristor pulses for a specific phase and polarity
 *
 * @param phase Phase selector: 0=A, 1=B, 2=C
 * @param positive true for positive arm, false for negative arm
 * @param state true to fire, false to disable
 */
void thyristor_set_phase(uint8_t phase, bool positive, bool state);

#ifdef __cplusplus
}
#endif

#endif /* __THYRISTOR_CONTROL_H */
