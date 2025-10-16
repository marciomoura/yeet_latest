/**
 ******************************************************************************
 * @file    TASK_IMPLEMENTATION_EXAMPLE.c
 * @brief   Example implementation of periodic tasks for main.c
 * @note    Copy these function implementations into your main.c file
 ******************************************************************************
 */

#include "AI.h"  // For AD() function if needed
#include "TIM.h"

/**
 * @brief  200us periodic task - ADC sampling
 * @note   Called from TIM3 ISR at 5 kHz rate (every 200 microseconds)
 *         This is your highest priority task - keep it fast!
 */
void Task_200us_ADC_Sampling(void)
{
    /* Call ADC read/start function (from AI.c) */
    AD();

    /* Add your 200us task code here */
    /* Keep execution time under ~20us for 10% CPU usage */

    /* Example:
     * - Read ADC values
     * - Fast signal processing
     * - Critical timing operations
     */
}

/**
 * @brief  1ms periodic task - Fast control loop
 * @note   Called from TIM4 ISR at 1 kHz rate (every 1 millisecond)
 *         Medium priority task - suitable for control loops
 */
void Task_1ms_FastControl(void)
{
    /* Add your 1ms control code here */
    /* Keep execution time under ~50us for 5% CPU usage */

    /* Example:
     * - Current control loop
     * - Voltage control loop
     * - PID controllers
     * - Fast protection checks
     * - State machine updates
     */
}

/**
 * @brief  10ms periodic task - Slow monitoring
 * @note   Called from TIM2 ISR at 100 Hz rate (every 10 milliseconds)
 *         Lowest priority task - suitable for housekeeping
 */
void Task_10ms_SlowMonitoring(void)
{
    /* Add your 10ms monitoring code here */
    /* Keep execution time under ~100us for 1% CPU usage */

    /* Example:
     * - Temperature monitoring
     * - Status LED updates
     * - Communication handling (UART, CAN, etc.)
     * - Diagnostics and logging
     * - User interface updates
     */
}

/**
 ******************************************************************************
 * USAGE IN MAIN.C:
 *
 * #include "TIM.h"
 *
 * int main(void)
 * {
 *     // System initialization
 *     SystemInit();
 *
 *     // Initialize peripherals
 *     // ... your initialization code ...
 *
 *     // Initialize timers (this starts the periodic tasks)
 *     TIMx_Init();
 *
 *     // Main loop
 *     while(1)
 *     {
 *         // Optional: Monitor task execution
 *         uint32_t count_200us = TIM_Get_200us_Counter();
 *         uint32_t count_1ms = TIM_Get_1ms_Counter();
 *         uint32_t count_10ms = TIM_Get_10ms_Counter();
 *
 *         // Your main loop code here
 *         // Tasks run automatically in interrupt context
 *     }
 * }
 ******************************************************************************
 */
