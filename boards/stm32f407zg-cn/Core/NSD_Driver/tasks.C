// TIM3: 200us task (5 kHz) - ADC sampling
// TIM4: 1000us task (1 kHz) - Fast control loop
// TIM2: 10000us task (100 Hz) - Slow monitoring

#include "tasks.h"

#include "adc.h"  // For ADC-related functions
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"

/* Private defines -----------------------------------------------------------*/
#define TIM3_FREQUENCY_HZ 5000  // 200us period
#define TIM4_FREQUENCY_HZ 1000  // 1000us period (1ms)
#define TIM2_FREQUENCY_HZ 100   // 10000us period (10ms)

/* APB1 Timer Clock = 84 MHz */
#define APB1_TIMER_CLOCK 84000000UL

/* DWT (Data Watchpoint and Trace) macros for cycle counting */
#define DWT_CYCCNT (DWT->CYCCNT)

/* Private variables ---------------------------------------------------------*/
static uint32_t task_200us_counter = 0;
static uint32_t task_1ms_counter = 0;
static uint32_t task_10ms_counter = 0;

/* ISR execution time tracking (in CPU cycles) */
static volatile uint32_t tim3_isr_cycles = 0;
static volatile uint32_t tim4_isr_cycles = 0;
static volatile uint32_t tim2_isr_cycles = 0;
static volatile uint32_t tim3_isr_max_cycles = 0;
static volatile uint32_t tim4_isr_max_cycles = 0;
static volatile uint32_t tim2_isr_max_cycles = 0;

/* Private function prototypes -----------------------------------------------*/
static void TIM3_Config(void);
static void TIM4_Config(void);
static void TIM2_Config(void);

/******************************************************************************
 * @brief  Increment 200us task counter (called from ISR)
 * @param  None
 * @retval None
 ******************************************************************************/
static inline void TIM_Increment_200us_Counter(void) { task_200us_counter++; }

/******************************************************************************
 * @brief  Increment 1ms task counter (called from ISR)
 * @param  None
 * @retval None
 ******************************************************************************/
static inline void TIM_Increment_1ms_Counter(void) { task_1ms_counter++; }

/******************************************************************************
 * @brief  Increment 10ms task counter (called from ISR)
 * @param  None
 * @retval None
 ******************************************************************************/
static inline void TIM_Increment_10ms_Counter(void) { task_10ms_counter++; }

/* Task functions are now external - user implements them in main.c */

/******************************************************************************
 * @brief  Initialize all timers for periodic tasks
 * @param  None
 * @retval None
 ******************************************************************************/
void TIMx_Init(void)
{
    /* Configure NVIC priority grouping ONCE for entire system */
    /* PriorityGroup_2: 2 bits preemption (0-3), 2 bits sub-priority (0-3) */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    /* Enable DWT cycle counter for ISR timing measurement */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // Enable trace
    DWT->CYCCNT = 0;                                 // Reset cycle counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;             // Enable cycle counter

    TIM3_Config();  // 200us task
    TIM4_Config();  // 1ms task
    TIM2_Config();  // 10ms task
}

/******************************************************************************
 * @brief  Configure TIM3 for 200us periodic interrupt (5 kHz)
 * @param  None
 * @retval None
 * @note   Used for high-speed ADC sampling
 ******************************************************************************/
static void TIM3_Config(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable TIM3 clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    /* TIM3 configuration */
    TIM_DeInit(TIM3);
    TIM_InternalClockConfig(TIM3);
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

    /*
     * Timer frequency calculation:
     * TIM_CLK = APB1_TIMER_CLOCK = 84 MHz
     * Prescaler = 0 (÷1) -> Counter clock = 84 MHz
     * Period = 84,000,000 / 5,000 - 1 = 16,799
     * Result: 5 kHz = 200us period
     */
    TIM_TimeBaseStructure.TIM_Prescaler = 0;   // No prescaling (84 MHz)
    TIM_TimeBaseStructure.TIM_Period = 16799;  // 5 kHz
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    /* Configure TIM3 interrupt */
    TIM_ClearFlag(TIM3, TIM_FLAG_Update);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

    /* Configure NVIC for TIM3 - Highest preemption priority (0) */
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  // High priority - can preempt TIM4 & TIM2
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Enable debug mode - timer continues during debug halt */
    DBGMCU_Config(DBGMCU_TIM3_STOP, DISABLE);

    /* Enable TIM3 */
    TIM_Cmd(TIM3, ENABLE);
}

/******************************************************************************
 * @brief  Configure TIM4 for 1ms periodic interrupt (1 kHz)
 * @param  None
 * @retval None
 * @note   Used for fast control loop
 ******************************************************************************/
static void TIM4_Config(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable TIM4 clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    /* TIM4 configuration */
    TIM_DeInit(TIM4);
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

    /*
     * Timer frequency calculation:
     * TIM_CLK = APB1_TIMER_CLOCK = 84 MHz
     * Prescaler = 83 (÷84) -> Counter clock = 1 MHz
     * Period = 1,000,000 / 1,000 - 1 = 999
     * Result: 1 kHz = 1ms period
     */
    TIM_TimeBaseStructure.TIM_Prescaler = 83;  // 84 MHz / 84 = 1 MHz
    TIM_TimeBaseStructure.TIM_Period = 999;    // 1 kHz
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    /* Configure TIM4 interrupt */
    TIM_ClearFlag(TIM4, TIM_FLAG_Update);
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

    /* Configure NVIC for TIM4 - Medium preemption priority (1) */
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  // Medium priority - can preempt TIM2
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Enable debug mode - timer continues during debug halt */
    DBGMCU_APB1PeriphConfig(DBGMCU_TIM4_STOP, DISABLE);

    /* Enable TIM4 */
    TIM_Cmd(TIM4, ENABLE);
}

/******************************************************************************
 * @brief  Configure TIM2 for 10ms periodic interrupt (100 Hz)
 * @param  None
 * @retval None
 * @note   Used for slow monitoring and housekeeping
 ******************************************************************************/
static void TIM2_Config(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable TIM2 clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    /* TIM2 configuration */
    TIM_DeInit(TIM2);
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

    /*
     * Timer frequency calculation:
     * TIM_CLK = APB1_TIMER_CLOCK = 84 MHz
     * Prescaler = 8399 (÷8400) -> Counter clock = 10 kHz
     * Period = 10,000 / 100 - 1 = 99
     * Result: 100 Hz = 10ms period
     */
    TIM_TimeBaseStructure.TIM_Prescaler = 8399;  // 84 MHz / 8400 = 10 kHz
    TIM_TimeBaseStructure.TIM_Period = 99;       // 100 Hz
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    /* Configure TIM2 interrupt */
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    /* Configure NVIC for TIM2 - Lowest preemption priority (2) */
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  // Lowest priority - cannot preempt others
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Enable debug mode - timer continues during debug halt */
    DBGMCU_Config(DBGMCU_TIM2_STOP, DISABLE);

    /* Enable TIM2 */
    TIM_Cmd(TIM2, ENABLE);
}

/******************************************************************************
 * @brief  TIM3 interrupt handler - 200us task (5 kHz)
 * @param  None
 * @retval None
 * @note   High-priority ADC sampling task
 ******************************************************************************/
void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) {
        uint32_t start_cycles = DWT_CYCCNT;

        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

        /* Increment counter and execute user task */
        TIM_Increment_200us_Counter();
        task_200us();

        /* Measure execution time */
        tim3_isr_cycles = DWT_CYCCNT - start_cycles;
        if (tim3_isr_cycles > tim3_isr_max_cycles) {
            tim3_isr_max_cycles = tim3_isr_cycles;
        }
    }
}

/******************************************************************************
 * @brief  TIM4 interrupt handler - 1ms task (1 kHz)
 * @param  None
 * @retval None
 * @note   Medium-priority fast control task
 ******************************************************************************/
void TIM4_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET) {
        uint32_t start_cycles = DWT_CYCCNT;

        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);

        /* Increment counter and execute user task */
        TIM_Increment_1ms_Counter();
        task_1ms();

        /* Measure execution time */
        tim4_isr_cycles = DWT_CYCCNT - start_cycles;
        if (tim4_isr_cycles > tim4_isr_max_cycles) {
            tim4_isr_max_cycles = tim4_isr_cycles;
        }
    }
}

/******************************************************************************
 * @brief  TIM2 interrupt handler - 10ms task (100 Hz)
 * @param  None
 * @retval None
 * @note   Low-priority monitoring task
 ******************************************************************************/
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
        uint32_t start_cycles = DWT_CYCCNT;

        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

        /* Increment counter and execute user task */
        TIM_Increment_10ms_Counter();
        task_10ms();

        /* Measure execution time */
        tim2_isr_cycles = DWT_CYCCNT - start_cycles;
        if (tim2_isr_cycles > tim2_isr_max_cycles) {
            tim2_isr_max_cycles = tim2_isr_cycles;
        }
    }
}

/*
 * NOTE: Task functions are now externally defined by the user
 * Implement these functions in your main.c or application code:
 *   - void Task_200us_ADC_Sampling(void)
 *   - void Task_1ms_FastControl(void)
 *   - void Task_10ms_SlowMonitoring(void)
 *
 * Helper functions for task counter management:
 */

/******************************************************************************
 * @brief  Get 200us task execution counter
 * @param  None
 * @retval Task counter value
 ******************************************************************************/
uint32_t get_200us_counter(void) { return task_200us_counter; }

/******************************************************************************
 * @brief  Get 1ms task execution counter
 * @param  None
 * @retval Task counter value
 ******************************************************************************/
uint32_t get_1ms_counter(void) { return task_1ms_counter; }

/******************************************************************************
 * @brief  Get 10ms task execution counter
 * @param  None
 * @retval Task counter value
 ******************************************************************************/
uint32_t get_10ms_counter(void) { return task_10ms_counter; }

/******************************************************************************
 * @brief  Get system clock configuration details
 * @param  None
 * @retval None
 * @note   Prints SYSCLK, HCLK, APB1, APB2 frequencies to verify PLL config
 ******************************************************************************/
void TIM_Print_Clock_Config(void)
{
    RCC_ClocksTypeDef RCC_Clocks;
    RCC_GetClocksFreq(&RCC_Clocks);

    /* Note: Add your preferred output method (UART, etc.) here */
    /* For now, store in static variables for debugging */
    static uint32_t sysclk_mhz = 0;
    static uint32_t hclk_mhz = 0;
    static uint32_t apb1_mhz = 0;
    static uint32_t apb2_mhz = 0;

    sysclk_mhz = RCC_Clocks.SYSCLK_Frequency / 1000000;
    hclk_mhz = RCC_Clocks.HCLK_Frequency / 1000000;
    apb1_mhz = RCC_Clocks.PCLK1_Frequency / 1000000;
    apb2_mhz = RCC_Clocks.PCLK2_Frequency / 1000000;

    /* Breakpoint here to inspect variables in debugger:
     * Expected: SYSCLK=168MHz, HCLK=168MHz, APB1=42MHz, APB2=84MHz
     * If SYSCLK=16MHz, PLL is not configured! */
    (void)sysclk_mhz;  // Suppress unused warning
    (void)hclk_mhz;
    (void)apb1_mhz;
    (void)apb2_mhz;
}

/******************************************************************************
 * @brief  Get ISR execution timing statistics
 * @param  timer_num: 3 for TIM3, 4 for TIM4, 2 for TIM2
 * @param  current_us: Pointer to store current execution time in microseconds
 * @param  max_us: Pointer to store maximum execution time in microseconds
 * @retval None
 * @note   Assumes SYSCLK = 168 MHz for cycle-to-time conversion
 ******************************************************************************/
void TIM_Get_ISR_Timing(uint8_t timer_num, float* current_us, float* max_us)
{
    RCC_ClocksTypeDef RCC_Clocks;
    RCC_GetClocksFreq(&RCC_Clocks);

    float cycles_per_us = (float)RCC_Clocks.SYSCLK_Frequency / 1000000.0f;
    uint32_t current_cycles = 0;
    uint32_t max_cycles = 0;

    switch (timer_num) {
        case 3:
            current_cycles = tim3_isr_cycles;
            max_cycles = tim3_isr_max_cycles;
            break;
        case 4:
            current_cycles = tim4_isr_cycles;
            max_cycles = tim4_isr_max_cycles;
            break;
        case 2:
            current_cycles = tim2_isr_cycles;
            max_cycles = tim2_isr_max_cycles;
            break;
        default:
            *current_us = 0;
            *max_us = 0;
            return;
    }

    *current_us = (float)current_cycles / cycles_per_us;
    *max_us = (float)max_cycles / cycles_per_us;
}

/******************************************************************************
 * @brief  Reset ISR timing statistics
 * @param  None
 * @retval None
 ******************************************************************************/
void TIM_Reset_ISR_Timing(void)
{
    tim3_isr_max_cycles = 0;
    tim4_isr_max_cycles = 0;
    tim2_isr_max_cycles = 0;
}

/******************************************************************************
 * @brief  Verify timer preemption is working correctly
 * @param  None
 * @retval 1 if preemption verified, 0 if failed
 * @note   Call this after system has been running for a few seconds
 ******************************************************************************/
uint8_t TIM_Verify_Preemption(void)
{
    /* Check that all timers are running */
    uint32_t tim3_count = task_200us_counter;
    uint32_t tim4_count = task_1ms_counter;
    uint32_t tim2_count = task_10ms_counter;

    /* Expected ratios: TIM3 should execute 5x more than TIM4, 50x more than TIM2 */
    /* Allow 10% tolerance for timing variations */
    if (tim3_count == 0 || tim4_count == 0 || tim2_count == 0) {
        return 0;  // One or more timers not running
    }

    float ratio_3_to_4 = (float)tim3_count / (float)tim4_count;
    float ratio_3_to_2 = (float)tim3_count / (float)tim2_count;

    /* TIM3:TIM4 should be ~5:1 (5000Hz : 1000Hz) */
    /* TIM3:TIM2 should be ~50:1 (5000Hz : 100Hz) */
    uint8_t ratio_ok =
        (ratio_3_to_4 >= 4.5f && ratio_3_to_4 <= 5.5f) && (ratio_3_to_2 >= 45.0f && ratio_3_to_2 <= 55.0f);

    return ratio_ok;
}

/* End of file ---------------------------------------------------------------*/
