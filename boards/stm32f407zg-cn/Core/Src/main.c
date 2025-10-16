/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body (SPL-based with NSD_Driver)
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>

#include "TIM.h"
#include "para.h"  // NSD_Driver initialization functions
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

// LED pointer for testing (FPGA LED at Bank4 address)
volatile uint16_t* led = (volatile uint16_t*)0x6C00000E;

volatile static uint32_t _counter_200us = 0;
volatile static uint32_t _counter_1ms = 0;
volatile static uint32_t _counter_10ms = 0;

/**
 * @brief  200us periodic task - ADC sampling
 * @note   Called from TIM3 ISR at 5 kHz rate (every 200 microseconds)
 *         This is your highest priority task - keep it fast!
 */
void task_200us(void)
{
    static uint16_t counter = 0;
    _counter_200us++;

    /* Blink LED0 at 5 Hz (toggle every 100ms = 500 cycles @ 200µs) */
    counter++;
    if (counter >= 500) {  // 500 × 200µs = 100ms
        *led ^= 0x0001;    // Toggle only LED0 (bit 0)
        counter = 0;
    }

    /* ADC read/start is already called in AD() by Init_Hardware() */
    /* Add your 200us task code here */
}

/**
 * @brief  1ms periodic task - Fast control loop
 * @note   Called from TIM4 ISR at 1 kHz rate (every 1 millisecond)
 */
void task_1ms(void)
{
    static uint16_t counter = 0;
    _counter_1ms++;

    /* Blink LED1 at 2 Hz (toggle every 250ms = 250 cycles @ 1ms) */
    counter++;
    if (counter >= 250) {  // 250 × 1ms = 250ms
        *led ^= 0x0002;    // Toggle only LED1 (bit 1)
        counter = 0;
    }

    /* Add your 1ms control code here */
}

/**
 * @brief  10ms periodic task - Slow monitoring
 * @note   Called from TIM2 ISR at 100 Hz rate (every 10 milliseconds)
 */
void task_10ms(void)
{
    static uint16_t counter = 0;
    _counter_10ms++;

    /* Blink LED2 at 1 Hz (toggle every 500ms = 50 cycles @ 10ms) */
    counter++;
    if (counter >= 50) {  // 50 × 10ms = 500ms
        *led ^= 0x0004;   // Toggle only LED2 (bit 2)
        counter = 0;
    }

    /* Add your 10ms monitoring code here */
}

static float current_us_3 = 0;
static float current_us_4 = 0;
static float current_us_5 = 0;
static float max_us_3 = 0;
static float max_us_4 = 0;
static float max_us_5 = 0;

int main(void)
{
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* Initialize all hardware using NSD_Driver */
    Init_Hardware();

    /* Initialize all data structures */
    // Init_Data();

    /* USER CODE BEGIN 2 */

    /* USER CODE END 2 */
    TIM_Print_Clock_Config();

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        // LEDs are now blinking in the timer tasks
        // LED0: 5 Hz blink (from task_200us)
        // LED1: 2 Hz blink (from task_1ms)
        // LED2: 1 Hz blink (from task_10ms)

        // Task 0 (200us task)
        TIM_Get_ISR_Timing(3, &current_us_3, &max_us_3);

        // Task 1 (1ms task)
        TIM_Get_ISR_Timing(4, &current_us_4, &max_us_4);

        // Task 2 (10ms task)
        TIM_Get_ISR_Timing(2, &current_us_5, &max_us_5);

        printf(
            "200us Task: Current=%.2f us, Max=%.2f us | 1ms Task: Current=%.2f us, Max=%.2f us | 10ms Task: "
            "Current=%.2f us, Max=%.2f us\r\n",
            current_us_3, max_us_3, current_us_4, max_us_4, current_us_5, max_us_5);

        // Sleep for 2 seconds using 1ms counter
        uint32_t start = _counter_1ms;
        while ((_counter_1ms - start) < 2000) {
            __NOP();
        }
    }
    /* USER CODE END 3 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
