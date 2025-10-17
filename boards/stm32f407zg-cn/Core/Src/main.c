#include "stm32f4xx.h"

//
#include "TIM.h"
#include "para.h"

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
    _counter_200us++;

    /* Blink LED0 at 5 Hz (toggle every 100ms = 500 cycles @ 200µs) */
    static uint16_t counter = 0;
    counter++;
    if (counter >= 50000) {  // 500 × 200µs = 100ms
        *led ^= 0x0001;      // Toggle only LED0 (bit 0)
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
    _counter_1ms++;

    /* Blink LED1 at 2 Hz (toggle every 250ms = 250 cycles @ 1ms) */
    static uint16_t counter = 0;
    counter++;
    if (counter >= 500) {  // 250 × 1ms = 250ms
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
    _counter_10ms++;

    /* Blink LED2 at 1 Hz (toggle every 500ms = 50 cycles @ 10ms) */
    static uint16_t counter = 0;
    counter++;
    if (counter >= 50) {  // 50 × 10ms = 500ms
        *led ^= 0x7;      // Toggle only LED2 (bit 2)
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
static uint8_t preemption_ok = 0;

int main(void)
{
    /* Initialize all hardware using NSD_Driver */
    Init_Hardware(); /* Initialize all data structures */
    // Init_Data();

    while (1) {
        // // Simple sequential LED pattern (doesn't depend on timers)
        // *led = (1 << led_pattern);  // Turn on one LED at a time

        // Use CPU cycles for delay instead of timer counter
        for (volatile uint32_t i = 0; i < 10000000; i++) {
            __NOP();
        }

        // Task timing measurements
        TIM_Get_ISR_Timing(3, &current_us_3, &max_us_3);
        TIM_Get_ISR_Timing(4, &current_us_4, &max_us_4);
        TIM_Get_ISR_Timing(2, &current_us_5, &max_us_5);
        preemption_ok = TIM_Verify_Preemption();
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
