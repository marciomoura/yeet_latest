/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ad7606_driver.h"
#include "fpga_io_driver.h"
#include "optical_fiber_driver.h"
#include "sfc_embedded/task_io.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// LED toggle counts for 1 Hz blink rate (500ms ON, 500ms OFF)
#define TASK1_TOGGLE_COUNT 2500  // Task 1 @ 5 kHz: 2500 × 200µs = 500ms
#define TASK2_TOGGLE_COUNT 500   // Task 2 @ 1 kHz: 500 × 1ms = 500ms
#define TASK3_TOGGLE_COUNT 50    // Task 3 @ 100 Hz: 50 × 10ms = 500ms

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

SRAM_HandleTypeDef hsram2;
SRAM_HandleTypeDef hsram4;

/* USER CODE BEGIN PV */
// Task execution counters for LED visual feedback
static int task1_counter = 0;
static int task2_counter = 0;
static int task3_counter = 0;

// ADC DMA buffer - must be static/global for DMA access
static volatile uint16_t adc_dma_buffer[4] = {0};

// AD7606 external ADC data (8 channels @ 16-bit)
static ad7606_data_t ad7606_data = {0};

// FPGA digital I/O data (48 inputs + 48 outputs)
static fpga_io_t fpga_io = {0};

#ifdef ENABLE_OPTICAL_FIBER
// Optical fiber communication (7 bidirectional channels)
static optical_fiber_t optical_fiber = {0};
#endif

// Task 1 I/O structures (static global to avoid stack allocation in 5 kHz loop)
static task1_inputs_t task1_inputs = {0};
static task1_outputs_t task1_outputs = {0};

// Task 2 I/O structures (for optical fiber communication)
static task2_inputs_t task2_inputs = {0};
static task2_outputs_t task2_outputs = {0};

// Diagnostic counters for debugging
volatile uint32_t g_adc_callback_count = 0;
volatile uint32_t g_tim3_callback_count = 0;
volatile uint32_t g_tim4_callback_count = 0;
volatile uint32_t g_adc_error_count = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_FSMC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Enable optical fiber communication feature
// #define ENABLE_OPTICAL_FIBER

// Forward declarations for task functions (defined in yeet application layer)
extern void yeet_embedded_init(void);
extern void yeet_task_1(const task1_inputs_t* inputs, task1_outputs_t* outputs);
extern void yeet_task_2(void);
extern void yeet_task_3(void);

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    /* USER CODE BEGIN 1 */
    // Initialize YEET application layer
    yeet_embedded_init();
    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_FSMC_Init();
    /* USER CODE BEGIN 2 */

    // Initialize AD7606 external ADC (must be after FSMC init)
    AD7606_Init();

    // Initialize FPGA digital I/O (must be after FSMC init)
    FPGA_IO_Init();
    FPGA_IO_Reset(&fpga_io);

#ifdef ENABLE_OPTICAL_FIBER
    // Initialize Optical Fiber Communication (must be after FSMC init)
    // Node ID: 0 (line-side controller), Baud rate: 230400 bps
    OpticalFiber_Init(&optical_fiber, 230400, 0);
#endif

    // Start first AD7606 conversion
    AD7606_StartConversion();

    // Start ADC with DMA (4 channels: IN10, IN11, IN12, IN13)
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_buffer, 4) != HAL_OK) {
        Error_Handler();
    }

    // Start TIM2 (ADC trigger timer - 5 kHz / 200 µs)
    if (HAL_TIM_Base_Start(&htim2) != HAL_OK) {
        Error_Handler();
    }

    // Start TIM3 with interrupt (Task 2 - 1 kHz / 1 ms)
    if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK) {
        Error_Handler();
    }

    // Start TIM4 with interrupt (Task 3 - 100 Hz / 10 ms)
    if (HAL_TIM_Base_Start_IT(&htim4) != HAL_OK) {
        Error_Handler();
    }

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
     */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{
    /* USER CODE BEGIN ADC1_Init 0 */

    /* USER CODE END ADC1_Init 0 */

    ADC_ChannelConfTypeDef sConfig = {0};

    /* USER CODE BEGIN ADC1_Init 1 */

    /* USER CODE END ADC1_Init 1 */

    /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
     */
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = ENABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 4;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        Error_Handler();
    }

    /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
     */
    sConfig.Channel = ADC_CHANNEL_10;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
     */
    sConfig.Channel = ADC_CHANNEL_11;
    sConfig.Rank = 2;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
     */
    sConfig.Channel = ADC_CHANNEL_12;
    sConfig.Rank = 3;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
     */
    sConfig.Channel = ADC_CHANNEL_13;
    sConfig.Rank = 4;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN ADC1_Init 2 */

    /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{
    /* USER CODE BEGIN TIM2_Init 0 */

    /* USER CODE END TIM2_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM2_Init 1 */

    /* USER CODE END TIM2_Init 1 */
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 16799;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM2_Init 2 */

    /* USER CODE END TIM2_Init 2 */
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{
    /* USER CODE BEGIN TIM3_Init 0 */

    /* USER CODE END TIM3_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM3_Init 1 */

    /* USER CODE END TIM3_Init 1 */
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 83;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 999;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM3_Init 2 */

    /* USER CODE END TIM3_Init 2 */
}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{
    /* USER CODE BEGIN TIM4_Init 0 */

    /* USER CODE END TIM4_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM4_Init 1 */

    /* USER CODE END TIM4_Init 1 */
    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 83;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 9999;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM4_Init 2 */

    /* USER CODE END TIM4_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{
    /* DMA controller clock enable */
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA2_Stream0_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* USER CODE BEGIN MX_GPIO_Init_1 */

    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_9 | GPIO_PIN_10, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);

    /*Configure GPIO pins : PF6 PF7 PF9 PF10 */
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    /*Configure GPIO pins : PB0 PB1 */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pin : PD13 */
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /*Configure GPIO pins : PC6 PC7 PC8 PC9 */
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* USER CODE BEGIN MX_GPIO_Init_2 */

    /* USER CODE END MX_GPIO_Init_2 */
}

/* FSMC initialization function */
static void MX_FSMC_Init(void)
{
    /* USER CODE BEGIN FSMC_Init 0 */

    /* USER CODE END FSMC_Init 0 */

    FSMC_NORSRAM_TimingTypeDef Timing = {0};

    /* USER CODE BEGIN FSMC_Init 1 */

    /* USER CODE END FSMC_Init 1 */

    /** Perform the SRAM2 memory initialization sequence
     */
    hsram2.Instance = FSMC_NORSRAM_DEVICE;
    hsram2.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
    /* hsram2.Init */
    hsram2.Init.NSBank = FSMC_NORSRAM_BANK2;
    hsram2.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
    hsram2.Init.MemoryType = FSMC_MEMORY_TYPE_PSRAM;
    hsram2.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
    hsram2.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
    hsram2.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
    hsram2.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
    hsram2.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
    hsram2.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
    hsram2.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
    hsram2.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
    hsram2.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
    hsram2.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
    hsram2.Init.PageSize = FSMC_PAGE_SIZE_NONE;
    /* Timing */
    Timing.AddressSetupTime = 2;
    Timing.AddressHoldTime = 15;
    Timing.DataSetupTime = 11;
    Timing.BusTurnAroundDuration = 1;
    Timing.CLKDivision = 16;
    Timing.DataLatency = 17;
    Timing.AccessMode = FSMC_ACCESS_MODE_A;
    /* ExtTiming */

    if (HAL_SRAM_Init(&hsram2, &Timing, NULL) != HAL_OK) {
        Error_Handler();
    }

    /** Perform the SRAM4 memory initialization sequence
     */
    hsram4.Instance = FSMC_NORSRAM_DEVICE;
    hsram4.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
    /* hsram4.Init */
    hsram4.Init.NSBank = FSMC_NORSRAM_BANK4;
    hsram4.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
    hsram4.Init.MemoryType = FSMC_MEMORY_TYPE_PSRAM;
    hsram4.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
    hsram4.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
    hsram4.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
    hsram4.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
    hsram4.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
    hsram4.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
    hsram4.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
    hsram4.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
    hsram4.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
    hsram4.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
    hsram4.Init.PageSize = FSMC_PAGE_SIZE_NONE;
    /* Timing */
    Timing.AddressSetupTime = 6;
    Timing.AddressHoldTime = 15;
    Timing.DataSetupTime = 21;
    Timing.BusTurnAroundDuration = 1;
    Timing.CLKDivision = 16;
    Timing.DataLatency = 17;
    Timing.AccessMode = FSMC_ACCESS_MODE_A;
    /* ExtTiming */

    if (HAL_SRAM_Init(&hsram4, &Timing, NULL) != HAL_OK) {
        Error_Handler();
    }

    /* USER CODE BEGIN FSMC_Init 2 */

    /* USER CODE END FSMC_Init 2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief Set thyristor firing pulses atomically using BSRR for simultaneous pin updates
 * @param outputs Pointer to task1 outputs structure containing pulse states
 * @note Executes in ~10 CPU cycles for deterministic real-time performance
 *
 * Pin mapping:
 *   pulse_1 -> PC6, pulse_2 -> PC7, pulse_3 -> PC8, pulse_4 -> PC9
 *   pulse_5 -> PB0, pulse_6 -> PB1
 */
inline void set_thyristor_firing_pulses(const task1_outputs_t* outputs)
{
    // GPIOC: PC6-PC9 (4 pins)
    uint32_t gpioc_set = 0, gpioc_reset = 0;
    if (outputs->pulse_1)
        gpioc_set |= GPIO_PIN_6;
    else
        gpioc_reset |= GPIO_PIN_6;
    if (outputs->pulse_2)
        gpioc_set |= GPIO_PIN_7;
    else
        gpioc_reset |= GPIO_PIN_7;
    if (outputs->pulse_3)
        gpioc_set |= GPIO_PIN_8;
    else
        gpioc_reset |= GPIO_PIN_8;
    if (outputs->pulse_4)
        gpioc_set |= GPIO_PIN_9;
    else
        gpioc_reset |= GPIO_PIN_9;
    GPIOC->BSRR = gpioc_set | (gpioc_reset << 16);

    // GPIOB: PB0-PB1 (2 pins)
    uint32_t gpiob_set = 0, gpiob_reset = 0;
    if (outputs->pulse_5)
        gpiob_set |= GPIO_PIN_0;
    else
        gpiob_reset |= GPIO_PIN_0;
    if (outputs->pulse_6)
        gpiob_set |= GPIO_PIN_1;
    else
        gpiob_reset |= GPIO_PIN_1;
    GPIOB->BSRR = gpiob_set | (gpiob_reset << 16);
}

/**
 * @brief ADC conversion complete callback - Task 1 (5 kHz / 200 µs)
 * @param hadc Pointer to ADC handle
 * @note Triggered by TIM2 TRGO after 4-channel DMA transfer completes
 *
 * This is the highest priority task in the system, responsible for:
 * - Reading 4 internal ADC channels (IN10-IN13) from DMA buffer
 * - Reading 8 external AD7606 channels via FSMC
 * - Executing inner current control loop
 * - Updating thyristor firing pulses via set_thyristor_firing_pulses()
 * - Triggering next AD7606 conversion (pipeline mode)
 *
 * @note FPGA digital I/O is handled in Task 2 (1 kHz) for lower update rate
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1) {
        g_adc_callback_count++;

        // Read AD7606 data from previous conversion and start next one
        AD7606_ReadAndStart(&ad7606_data);

        // Populate ADC channels 1-9 from AD7606 (external 16-bit ADC)
        // Channels 1-8 map to AD7606 channels 0-7, channel 9 reserved/unused
        task1_inputs.adc_channels[1] = ad7606_data.raw[0];  // Va (Phase A Voltage)
        task1_inputs.adc_channels[2] = ad7606_data.raw[1];  // Vb (Phase B Voltage)
        task1_inputs.adc_channels[3] = ad7606_data.raw[2];  // Vc (Phase C Voltage)
        task1_inputs.adc_channels[4] = ad7606_data.raw[3];  // Vfd (Field Voltage)
        task1_inputs.adc_channels[5] = ad7606_data.raw[4];  // Ia (Phase A Current)
        task1_inputs.adc_channels[6] = ad7606_data.raw[5];  // Ib (Phase B Current)
        task1_inputs.adc_channels[7] = ad7606_data.raw[6];  // Ic (Phase C Current)
        task1_inputs.adc_channels[8] = ad7606_data.raw[7];  // Ifd (Field Current)
        task1_inputs.adc_channels[9] = 0;                   // Reserved (unused)

        // Populate ADC channels 10-13 from STM32 internal ADC (12-bit)
        task1_inputs.adc_channels[10] = adc_dma_buffer[0];  // ADC1_IN10 (PC0)
        task1_inputs.adc_channels[11] = adc_dma_buffer[1];  // ADC1_IN11 (PC1)
        task1_inputs.adc_channels[12] = adc_dma_buffer[2];  // ADC1_IN12 (PC2)
        task1_inputs.adc_channels[13] = adc_dma_buffer[3];  // ADC1_IN13 (PC3)

        // Clear outputs from previous cycle
        task1_outputs.pulse_1 = false;
        task1_outputs.pulse_2 = false;
        task1_outputs.pulse_3 = false;
        task1_outputs.pulse_4 = false;
        task1_outputs.pulse_5 = false;
        task1_outputs.pulse_6 = false;
        task1_outputs.fpga_outputs.update_outputs = false;

        // Call high-frequency control task with structured I/O (no data copy)
        yeet_task_1(&task1_inputs, &task1_outputs);

        // Apply thyristor firing pulses atomically using BSRR
        set_thyristor_firing_pulses(&task1_outputs);

        // Visual feedback: Toggle PC6 at 1 Hz (2500 * 200µs = 500ms)
        if (++task1_counter >= TASK1_TOGGLE_COUNT) {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
            task1_counter = 0;
        }
    }
}

/**
 * @brief Timer period elapsed callback - Task 2 (1 kHz) and Task 3 (100 Hz)
 * @param htim Pointer to timer handle
 *
 * Task 2 (1 kHz): Medium-frequency control
 * - Speed/torque regulation
 * - Flux control
 * - Protection monitoring
 * - FPGA digital I/O (48 inputs + 48 outputs)
 *
 * Task 3 (100 Hz): Low-frequency control
 * - State machine
 * - Communication
 * - Diagnostics and logging
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim->Instance == TIM3) {
        // Task 2: 1 kHz (1 ms period)
        g_tim3_callback_count++;

#ifdef ENABLE_OPTICAL_FIBER
        // Poll optical fiber channels for received data
        OpticalFiber_Poll(&optical_fiber);

        // Process received optical fiber frames
        for (int ch = 1; ch <= 7; ch++) {
            int ch_idx = ch - 1;  // Convert to 0-indexed array

            // Check if frame is ready on this channel
            if (OpticalFiber_IsFrameReady(&optical_fiber, ch)) {
                uint8_t frame[128];
                if (OpticalFiber_GetFrame(&optical_fiber, ch, frame) > 0) {
                    // Extract payload and source node
                    uint8_t source_node = 0;
                    uint16_t payload_len = OpticalFiber_ExtractPayload(frame, task2_inputs.channel[ch_idx].payload,
                        sizeof(task2_inputs.channel[ch_idx].payload), &source_node);

                    if (payload_len > 0) {
                        // Valid frame received
                        task2_inputs.channel[ch_idx].payload_length = (uint8_t)payload_len;
                        task2_inputs.channel[ch_idx].source_node = source_node;
                        task2_inputs.channel[ch_idx].valid = true;
                    }
                }
            }

            // Update statistics
            const optical_channel_status_t* status = OpticalFiber_GetChannelStatus(&optical_fiber, ch);
            if (status != NULL) {
                task2_inputs.frames_received[ch_idx] = status->frames_received;
                task2_inputs.crc_errors[ch_idx] = status->crc_errors;
            }
        }
#endif  // ENABLE_OPTICAL_FIBER

        // Read FPGA digital inputs (raw read + debounce update)
        FPGA_IO_ReadInputs(&fpga_io);
        FPGA_IO_UpdateDebounce(&fpga_io);

        // Mark FPGA inputs as valid
        task1_inputs.fpga_inputs.data_valid = true;

        // Copy FPGA debounced inputs to task inputs (48 channels)
        for (int i = 0; i < 48; i++) {
            task1_inputs.fpga_inputs.inputs[i] = fpga_io.inputs.debounced[i];
        }

        // Execute Task 2 control logic
        yeet_task_2();

#ifdef ENABLE_OPTICAL_FIBER
        // Process optical fiber transmit requests
        if (task2_outputs.transmit_broadcast) {
            // Transmit broadcast frame
            if (task2_outputs.broadcast.valid) {
                OpticalFiber_TransmitFrame(
                    &optical_fiber, 0, task2_outputs.broadcast.payload, task2_outputs.broadcast.payload_length);
            }
            task2_outputs.transmit_broadcast = false;
        }

        // Transmit frames on specific channels
        for (int ch = 1; ch <= 7; ch++) {
            int ch_idx = ch - 1;
            if (task2_outputs.transmit_request[ch_idx]) {
                if (task2_outputs.channel[ch_idx].valid) {
                    OpticalFiber_TransmitFrame(&optical_fiber, ch, task2_outputs.channel[ch_idx].payload,
                        task2_outputs.channel[ch_idx].payload_length);
                }
                task2_outputs.transmit_request[ch_idx] = false;
            }
        }
#endif  // ENABLE_OPTICAL_FIBER

        // Write FPGA digital outputs if update requested
        if (task1_outputs.fpga_outputs.update_outputs) {
            // Copy task outputs to FPGA output structure (48 channels)
            for (int i = 0; i < 48; i++) {
                fpga_io.outputs.state[i] = task1_outputs.fpga_outputs.outputs[i];
            }
            FPGA_IO_WriteOutputs(&fpga_io);
            // Clear flag after writing
            task1_outputs.fpga_outputs.update_outputs = false;
        }

        // Visual feedback: Toggle PC7 at 1 Hz
        if (++task2_counter >= TASK2_TOGGLE_COUNT) {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
            task2_counter = 0;
        }
    }
    else if (htim->Instance == TIM4) {
        // Task 3: 100 Hz (10 ms period)
        g_tim4_callback_count++;
        yeet_task_3();

        // Visual feedback: Toggle PC8 at 1 Hz
        if (++task3_counter >= TASK3_TOGGLE_COUNT) {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
            task3_counter = 0;
        }
    }
}

/**
 * @brief ADC error callback - Safety shutdown
 * @param hadc Pointer to ADC handle
 * @note All thyristor firing pulses are forced LOW and timers are stopped
 */
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1) {
        g_adc_error_count++;

        // Safety: All thyristor firing pulses OFF
        task1_outputs.pulse_1 = false;
        task1_outputs.pulse_2 = false;
        task1_outputs.pulse_3 = false;
        task1_outputs.pulse_4 = false;
        task1_outputs.pulse_5 = false;
        task1_outputs.pulse_6 = false;
        set_thyristor_firing_pulses(&task1_outputs);

        // Stop ADC and timers
        HAL_ADC_Stop_DMA(&hadc1);
        HAL_TIM_Base_Stop(&htim2);
        HAL_TIM_Base_Stop_IT(&htim3);
        HAL_TIM_Base_Stop_IT(&htim4);
    }
}

/* USER CODE END 4 */

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
