/**
 ******************************************************************************
 * @file    TIM.h
 * @author  Techperts Team
 * @version V2.0.0
 * @date    14-Oct-2025
 * @brief   Timer configuration header for periodic tasks
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIM_H
#define __TIM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Task timing information structure
 */
typedef struct {
    uint32_t period_us;         /**< Task period in microseconds */
    float current_execution_us; /**< Current execution time in microseconds */
    float max_execution_us;     /**< Maximum execution time in microseconds */
    uint32_t execution_count;   /**< Total number of task executions */
    bool overrun;               /**< True if task execution exceeds period */
} task_timing_t;

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  Initialize all timers for periodic tasks
 */
void TIMx_Init(void);

/**
 * @brief  Timer-based millisecond delay function
 * @param  ms: Delay time in milliseconds
 * @note   Requires TIMx_Init() to be called first to start the 1ms timer
 */
void delay_ms(uint32_t ms);

/**
 * @brief  Get 200us task execution counter
 * @retval Task counter value
 */
uint32_t get_200us_counter(void);

/**
 * @brief  Get 1ms task execution counter
 * @retval Task counter value
 */
uint32_t get_1ms_counter(void);

/**
 * @brief  Get 10ms task execution counter
 * @retval Task counter value
 */
uint32_t get_10ms_counter(void);

/**
 * @brief  Get system clock configuration details
 * @note   Call in debugger to verify SYSCLK is 168MHz (not 16MHz)
 *         Set breakpoint inside function to inspect clock frequencies
 */
void TIM_Print_Clock_Config(void);

/**
 * @brief  Get ISR execution timing statistics
 * @param  timer_num: 3 for TIM3, 4 for TIM4, 2 for TIM2
 * @param  current_us: Pointer to store current execution time in microseconds
 * @param  max_us: Pointer to store maximum execution time in microseconds
 * @note   Use to verify ISR execution stays within period budget:
 *         - TIM3 (200us period): ISR should be < 200us
 *         - TIM4 (1ms period): ISR should be < 1000us
 *         - TIM2 (10ms period): ISR should be < 10000us
 */
void TIM_Get_ISR_Timing(uint8_t timer_num, float* current_us, float* max_us);

/**
 * @brief  Reset ISR timing statistics (max values)
 */
void TIM_Reset_ISR_Timing(void);

/**
 * @brief  Verify timer preemption is working correctly
 * @retval 1 if preemption verified, 0 if failed
 * @note   Checks that timer execution ratios match expected frequencies:
 *         - TIM3:TIM4 should be ~5:1 (5kHz : 1kHz)
 *         - TIM3:TIM2 should be ~50:1 (5kHz : 100Hz)
 *         Call after system has been running for a few seconds
 */
uint8_t TIM_Verify_Preemption(void);

/**
 * @brief  Get timing information for 200us task (TIM3)
 * @param  timing Pointer to structure to store timing information
 */
void TIM_Get_Task200us_Timing(task_timing_t* timing);

/**
 * @brief  Get timing information for 1ms task (TIM4)
 * @param  timing Pointer to structure to store timing information
 */
void TIM_Get_Task1ms_Timing(task_timing_t* timing);

/**
 * @brief  Get timing information for 10ms task (TIM2)
 * @param  timing Pointer to structure to store timing information
 */
void TIM_Get_Task10ms_Timing(task_timing_t* timing);

/**
 * @brief  Check if any task has overrun (execution time exceeds period)
 * @retval true if any task overrun detected, false otherwise
 */
bool TIM_Check_Task_Overruns(void);

/* Exported task functions - to be implemented by user -----------------------*/

/**
 * @brief  200us periodic task - ADC sampling
 * @note   Called from TIM3 ISR at 5 kHz rate
 *         Implement this function in your main.c or application code
 */
extern void task_200us(void);

/**
 * @brief  1ms periodic task - Fast control loop
 * @note   Called from TIM4 ISR at 1 kHz rate
 *         Implement this function in your main.c or application code
 */
extern void task_1ms(void);

/**
 * @brief  10ms periodic task - Slow monitoring
 * @note   Called from TIM2 ISR at 100 Hz rate
 *         Implement this function in your main.c or application code
 */
extern void task_10ms(void);

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H */

/* End of file ---------------------------------------------------------------*/
