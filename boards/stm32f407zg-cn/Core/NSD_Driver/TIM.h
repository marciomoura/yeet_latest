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
#include <stdint.h>

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  Initialize all timers for periodic tasks
 */
void TIMx_Init(void);

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
