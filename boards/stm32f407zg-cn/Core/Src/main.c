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

/* Timing monitoring variables */
static task_timing_t timing_200us;
static task_timing_t timing_1ms;
static task_timing_t timing_10ms;

int main(void)
{
    int init_status;
    uint32_t last_check_time = 0;
    bool overrun_detected = false;

    /* Initialize all hardware using NSD_Driver */
    init_status = Init_Hardware();

    // If initialization failed, FAULT LED is already set by Init_Hardware()
    // The system will continue but with limited functionality
    (void)init_status;  // Acknowledge the status

    while (1) {
        // Check task timing every 100ms
        // Note: This overflow-safe subtraction works due to unsigned arithmetic wrapping
        uint32_t current_time = get_1ms_counter();
        uint32_t elapsed_time = current_time - last_check_time;

        if (elapsed_time >= 100) {
            last_check_time = current_time;

            // Get timing information for all tasks
            TIM_Get_Task200us_Timing(&timing_200us);
            TIM_Get_Task1ms_Timing(&timing_1ms);
            TIM_Get_Task10ms_Timing(&timing_10ms);

            // Check for task overruns
            if (TIM_Check_Task_Overruns()) {
                if (!overrun_detected) {
                    // First time detecting overrun - set FAULT LED
                    overrun_detected = true;
                    FSMC_GPIO_SetLED_Fault(&fsmc_gpio_instance, 1);
                    FSMC_GPIO_WriteOutputs(&fsmc_gpio_instance);
                }
            }

            // Optional: Clear overrun flag if system recovers
            // (uncomment if you want auto-recovery)
            // else {
            //     if (overrun_detected) {
            //         overrun_detected = false;
            //         FSMC_GPIO_SetLED_Fault(&fsmc_gpio_instance, 0);
            //         FSMC_GPIO_WriteOutputs(&fsmc_gpio_instance);
            //     }
            // }
        }

        // Small delay to avoid tight polling
        delay_ms(10);
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
