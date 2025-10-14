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

/**
 * @brief  200us periodic task - ADC sampling
 * @note   Called from TIM3 ISR at 5 kHz rate (every 200 microseconds)
 *         This is your highest priority task - keep it fast!
 */
void task_200us(void)
{
    /* ADC read/start is already called in AD() by Init_Hardware() */
    /* Add your 200us task code here */
}

/**
 * @brief  1ms periodic task - Fast control loop
 * @note   Called from TIM4 ISR at 1 kHz rate (every 1 millisecond)
 */
void task_1ms(void) { /* Add your 1ms control code here */ }

/**
 * @brief  10ms periodic task - Slow monitoring
 * @note   Called from TIM2 ISR at 100 Hz rate (every 10 milliseconds)
 */
void task_10ms(void) { /* Add your 10ms monitoring code here */ }

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* Initialize all hardware using NSD_Driver */
    Init_Hardware();

    /* Initialize all data structures */
    Init_Data();

    /* USER CODE BEGIN 2 */

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        // Simple LED test - toggle LED
        *led = 0x0007;                              // Turn on LED0-2
        for (volatile int i = 0; i < 100000; i++);  // Simple delay
        *led = 0x0000;                              // Turn off all LEDs
        for (volatile int i = 0; i < 100000; i++);  // Simple delay
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
