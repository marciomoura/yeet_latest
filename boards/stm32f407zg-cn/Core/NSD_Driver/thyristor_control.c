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

void Thyristor_DO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // Enable GPIO clocks
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    // Configure GPIOC pins: PC6, PC7, PC8, PC9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;    // Push-Pull
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;  // No pull-up/down
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // Configure GPIOB pins: PB0, PB1
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // Initialize all outputs to LOW (no pulses)
    set_thyristor_pulses(false, false, false, false, false, false);
}

/**
 * @brief Set thyristor firing pulses by individual named arms
 *
 * Pin Mapping Reference:
 * - p1 -> PC6 (Phase A Positive Arm)
 * - p2 -> PB1 (Phase C Negative Arm)
 * - p3 -> PC7 (Phase B Positive Arm)
 * - p4 -> PC9 (Phase A Negative Arm)
 * - p5 -> PC8 (Phase C Positive Arm)
 * - p6 -> PB0 (Phase B Negative Arm)
 */
void set_thyristor_pulses(bool p1, bool p2, bool p3, bool p4, bool p5, bool p6)
{
    // Phase A Positive Arm - PC6
    if (p1) {
        GPIOC->BSRRL = GPIO_Pin_6;  // Set PC6 high
    }
    else {
        GPIOC->BSRRH = GPIO_Pin_6;  // Set PC6 low
    }

    // Phase C Negative Arm - PB1
    if (p2) {
        GPIOB->BSRRL = GPIO_Pin_1;  // Set PB1 high
    }
    else {
        GPIOB->BSRRH = GPIO_Pin_1;  // Set PB1 low
    }

    // Phase B Positive Arm - PC7
    if (p3) {
        GPIOC->BSRRL = GPIO_Pin_7;  // Set PC7 high
    }
    else {
        GPIOC->BSRRH = GPIO_Pin_7;  // Set PC7 low
    }

    // Phase A Negative Arm - PC9
    if (p4) {
        GPIOC->BSRRL = GPIO_Pin_9;  // Set PC9 high
    }
    else {
        GPIOC->BSRRH = GPIO_Pin_9;  // Set PC9 low
    }

    // Phase C Positive Arm - PC8
    if (p5) {
        GPIOC->BSRRL = GPIO_Pin_8;  // Set PC8 high
    }
    else {
        GPIOC->BSRRH = GPIO_Pin_8;  // Set PC8 low
    }

    // Phase B Negative Arm - PB0
    if (p6) {
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
