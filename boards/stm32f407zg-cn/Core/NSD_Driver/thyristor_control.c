/**
 ******************************************************************************
 * @file    thyristor_control.c
 * @brief   Thyristor firing pulse control implementation
 * @date    2025
 ******************************************************************************
 */

#include "thyristor_control.h"

/* ============================================================================
 * Public Function Implementations
 * ============================================================================ */

/**
 * @brief Set thyristor firing pulses by individual named arms
 *
 * Pin Mapping Reference:
 * - thyristor_a_plus  -> PC6 (Phase A Positive Arm)
 * - thyristor_c_minus -> PB1 (Phase C Negative Arm)
 * - thyristor_b_plus  -> PC7 (Phase B Positive Arm)
 * - thyristor_a_minus -> PC9 (Phase A Negative Arm)
 * - thyristor_c_plus  -> PC8 (Phase C Positive Arm)
 * - thyristor_b_minus -> PB0 (Phase B Negative Arm)
 */
void set_thyristor_pulses(bool thyristor_a_plus,
    bool thyristor_c_minus,
    bool thyristor_b_plus,
    bool thyristor_a_minus,
    bool thyristor_c_plus,
    bool thyristor_b_minus)
{
    // Phase A Positive Arm - PC6
    if (thyristor_a_plus) {
        GPIOC->BSRRL = GPIO_Pin_6;  // Set PC6 high
    }
    else {
        GPIOC->BSRRH = GPIO_Pin_6;  // Set PC6 low
    }

    // Phase C Negative Arm - PB1
    if (thyristor_c_minus) {
        GPIOB->BSRRL = GPIO_Pin_1;  // Set PB1 high
    }
    else {
        GPIOB->BSRRH = GPIO_Pin_1;  // Set PB1 low
    }

    // Phase B Positive Arm - PC7
    if (thyristor_b_plus) {
        GPIOC->BSRRL = GPIO_Pin_7;  // Set PC7 high
    }
    else {
        GPIOC->BSRRH = GPIO_Pin_7;  // Set PC7 low
    }

    // Phase A Negative Arm - PC9
    if (thyristor_a_minus) {
        GPIOC->BSRRL = GPIO_Pin_9;  // Set PC9 high
    }
    else {
        GPIOC->BSRRH = GPIO_Pin_9;  // Set PC9 low
    }

    // Phase C Positive Arm - PC8
    if (thyristor_c_plus) {
        GPIOC->BSRRL = GPIO_Pin_8;  // Set PC8 high
    }
    else {
        GPIOC->BSRRH = GPIO_Pin_8;  // Set PC8 low
    }

    // Phase B Negative Arm - PB0
    if (thyristor_b_minus) {
        GPIOB->BSRRL = GPIO_Pin_0;  // Set PB0 high
    }
    else {
        GPIOB->BSRRH = GPIO_Pin_0;  // Set PB0 low
    }
}

/**
 * @brief Set all thyristor pulses to OFF (safe state)
 */
void thyristor_all_off(void)
{
    // Reset all thyristor output pins to low
    GPIOC->BSRRH = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
    GPIOB->BSRRH = GPIO_Pin_0 | GPIO_Pin_1;
}

/**
 * @brief Set thyristor pulses for a specific phase and polarity
 */
void thyristor_set_phase(uint8_t phase, bool positive, bool state)
{
    // Phase mapping:
    // Phase A: PC6 (+), PC9 (-)
    // Phase B: PC7 (+), PB0 (-)
    // Phase C: PC8 (+), PB1 (-)

    switch (phase) {
        case 0:  // Phase A
            if (positive) {
                if (state) {
                    GPIOC->BSRRL = GPIO_Pin_6;
                }
                else {
                    GPIOC->BSRRH = GPIO_Pin_6;
                }
            }
            else {
                if (state) {
                    GPIOC->BSRRL = GPIO_Pin_9;
                }
                else {
                    GPIOC->BSRRH = GPIO_Pin_9;
                }
            }
            break;

        case 1:  // Phase B
            if (positive) {
                if (state) {
                    GPIOC->BSRRL = GPIO_Pin_7;
                }
                else {
                    GPIOC->BSRRH = GPIO_Pin_7;
                }
            }
            else {
                if (state) {
                    GPIOB->BSRRL = GPIO_Pin_0;
                }
                else {
                    GPIOB->BSRRH = GPIO_Pin_0;
                }
            }
            break;

        case 2:  // Phase C
            if (positive) {
                if (state) {
                    GPIOC->BSRRL = GPIO_Pin_8;
                }
                else {
                    GPIOC->BSRRH = GPIO_Pin_8;
                }
            }
            else {
                if (state) {
                    GPIOB->BSRRL = GPIO_Pin_1;
                }
                else {
                    GPIOB->BSRRH = GPIO_Pin_1;
                }
            }
            break;

        default:
            // Invalid phase - do nothing
            break;
    }
}
