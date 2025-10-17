#include "driver_init.h"
#include "stm32f4xx.h"
#include "tasks.h"

/* Task prototypes (called by timer ISRs in tasks.c) ----------------------- */
void task_200us(void);
void task_1ms(void);
void task_10ms(void);

/* Private variables -------------------------------------------------------- */

/**
 * @brief FSMC GPIO driver instance (for LEDs and digital I/O via FSMC Bank 4)
 */
fsmc_gpio_instance_t fsmc_gpio_instance = {0};

/**
 * @brief  200us periodic task - ADC sampling
 * @note   Called from TIM3 ISR at 5 kHz rate (every 200 microseconds)
 *         This is your highest priority task - keep it fast!
 */
void task_200us(void) { __NOP(); }

/**
 * @brief  1ms periodic task - Fast control loop
 * @note   Called from TIM4 ISR at 1 kHz rate (every 1 millisecond)
 */
void task_1ms(void) { __NOP(); }

/**
 * @brief  10ms periodic task - Slow monitoring
 * @note   Called from TIM2 ISR at 100 Hz rate (every 10 milliseconds)
 *         Controls RUN LED as a "heartbeat" indicator if tasks are running correctly
 */
void task_10ms(void)
{
    static uint16_t counter = 0;
    counter++;
    if (counter >= 50) {  // 50 × 10ms = 500ms
        counter = 0;
        // Check if task preemption ratios are correct
        if (TIM_Verify_Preemption()) {
            // System healthy - RUN LED for heartbeat
            FSMC_GPIO_SetLED_Run(&fsmc_gpio_instance, 1);
        }
        else {
            // Preemption failed - turn off RUN LED as error indicator
            FSMC_GPIO_SetLED_Run(&fsmc_gpio_instance, 0);
            // Trigger alarm LED
            FSMC_GPIO_SetLED_Alarm(&fsmc_gpio_instance, 1);
        }

        // Write LED state to hardware
        FSMC_GPIO_WriteOutputs(&fsmc_gpio_instance);
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
    int init_status;

    /* Initialize all hardware using NSD_Driver */
    init_status = Init_Hardware();

    /* Initialize all data structures */
    // Init_Data();

    // If initialization failed, FAULT LED is already set by Init_Hardware()
    // The system will continue but with limited functionality
    (void)init_status;  // Acknowledge the status

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
