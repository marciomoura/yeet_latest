#include "adc.h"  // AD7606 ADC driver
#include "dac.h"  // AD5724 DAC driver
#include "driver_init.h"
#include "network_config_helper.h"  // Network configuration helpers
#include "stm32f4xx.h"
#include "tasks.h"
#include "thyristor_control.h"
#include "w5200_driver.h"  // New W5200 driver

/* Network Configuration Flags --------------------------------------------- */
/**
 * @brief Compile-time network configuration
 *
 * Define which W5200 network controllers to enable:
 * - ENABLE_W5200_SPI2: SPI2 network (192.168.2.x)
 * - ENABLE_W5200_SPI3: SPI3 network (192.168.10.x)
 *
 * To customize at build time, pass flags to compiler:
 *   -DENABLE_W5200_SPI2=1 -DENABLE_W5200_SPI3=0
 *
 * Build Examples:
 *   Both networks (default):  cmake --build . --config Debug
 *   SPI2 only:                cmake -DENABLE_W5200_SPI2=1 -DENABLE_W5200_SPI3=0 && cmake --build .
 *   SPI3 only:                cmake -DENABLE_W5200_SPI2=0 -DENABLE_W5200_SPI3=1 && cmake --build .
 */
#ifndef ENABLE_W5200_SPI2
#define ENABLE_W5200_SPI2 1  // Default: SPI2 enabled
#endif

#ifndef ENABLE_W5200_SPI3
#define ENABLE_W5200_SPI3 0  // Default: SPI3 enabled
#endif

// Compile-time validation
#if !ENABLE_W5200_SPI2 && !ENABLE_W5200_SPI3
#warning "No W5200 networks enabled! At least one of ENABLE_W5200_SPI2 or ENABLE_W5200_SPI3 should be set to 1"
#endif /* Task prototypes (called by timer ISRs in tasks.c) ----------------------- */
void task_200us(void);
void task_1ms(void);
void task_10ms(void);

/* Private variables -------------------------------------------------------- */

/**
 * @brief FSMC GPIO driver instance (for LEDs and digital I/O via FSMC Bank 4)
 */
fsmc_gpio_instance_t fsmc_gpio_instance = {0};

/**
 * @brief AD7606 8-channel ADC instance
 */
ad7606_instance_t ad7606_instance = {0};

/**
 * @brief Latest ADC reading (updated every 200µs)
 */
ad7606_reading_t adc_reading = {0};

/**
 * @brief AD5724 4-channel DAC instance
 */
ad5724_instance_t ad5724_instance = {0};

/**
 * @brief W5200 Ethernet controller instances
 *
 * Network Configuration (based on DIP switch node ID):
 *
 * SPI2 Network:
 *   - IP Address: 192.168.2.(100 + uchNodeNum)
 *   - Gateway:    192.168.2.1
 *   - Subnet:     255.255.255.0
 *   - Port:       7000 (TCP Server)
 *   - DMA:        Disabled (conflicts with USART3)
 *
 * SPI3 Network:
 *   - IP Address: 192.168.10.(31 + uchNodeNum)
 *   - Gateway:    192.168.10.1
 *   - Subnet:     255.255.255.0
 *   - Port:       7000 (TCP Server)
 *   - DMA:        8KB TX/RX buffers
 *
 * Examples:
 *   - DIP switch = 0:  SPI2 = 192.168.2.100,  SPI3 = 192.168.10.31
 *   - DIP switch = 5:  SPI2 = 192.168.2.105,  SPI3 = 192.168.10.36
 *   - DIP switch = 15: SPI2 = 192.168.2.115,  SPI3 = 192.168.10.46
 */
#if ENABLE_W5200_SPI2
w5200_instance_t w5200_spi2 = {0};
#endif

#if ENABLE_W5200_SPI3
w5200_instance_t w5200_spi3 = {0};
#endif

/**
 * @brief Network buffers for W5200 controllers
 * SPI2: 512 bytes (no DMA)
 * SPI3: 8KB (with DMA)
 */
#if ENABLE_W5200_SPI2
static uint8_t spi2_tx_buffer[512];
static uint8_t spi2_rx_buffer[512];
#endif

#if ENABLE_W5200_SPI3
static uint8_t spi3_tx_buffer[8192];
static uint8_t spi3_rx_buffer[8192];
#endif

/**
 * @brief TCP packet buffers for application layer
 */
static uint8_t tcp_packet_buffer[1460];  // Max TCP MSS

/**
 * @brief  200us periodic task - ADC sampling and control
 * @note   Called from TIM3 ISR at 5 kHz rate (every 200 microseconds)
 *         This is your highest priority task - keep it fast!
 *
 * ADC Pipeline Operation:
 *   1. Read previous conversion results (from last cycle)
 *   2. Process data and update control outputs
 *   3. Start next conversion
 */
void task_200us(void)
{
    static bool first_run = true;

    // Pipeline operation: Read previous conversion, then start next
    if (!first_run) {
        // Read the conversion that was started in the previous cycle
        AD7606_Read(&ad7606_instance, &adc_reading);

        // TODO: Use adc_reading.raw[0..7] for control algorithm
        // Example: Access individual channels
        // uint16_t current_a = adc_reading.raw[0];
        // uint16_t current_b = adc_reading.raw[1];
        // uint16_t voltage   = adc_reading.raw[2];

        // TODO: Implement thyristor firing control based on ADC data
        // For now, all thyristors off
        set_thyristor_pulses(false, false, false, false, false, false);
    }
    else {
        first_run = false;
    }

    // Start conversion for next cycle (takes ~5µs)
    AD7606_StartConversion(&ad7606_instance);
}

/**
 * @brief  1ms periodic task - Fast control loop and DAC output
 * @note   Called from TIM4 ISR at 1 kHz rate (every 1 millisecond)
 *         Updates DAC outputs based on control algorithm
 */
void task_1ms(void)
{
    // TODO: Implement your control algorithm here
    // Example: Process ADC data and generate control outputs

    // For demonstration: Simple test pattern (can be removed)
    // static float test_phase = 0.0f;
    // test_phase += 0.01f;
    // if (test_phase > 6.28f) test_phase = 0.0f;
    // dac_outputs[0] = 2.5f * sinf(test_phase);

    /**
     * @brief DAC output values (updated every 1ms)
     * Channel mapping (example - customize for your application):
     * - Channel A: Control signal 1
     * - Channel B: Control signal 2
     * - Channel C: Control signal 3
     * - Channel D: Control signal 4
     */
    float dac_outputs[AD5724_NUM_CHANNELS] = {0.0f, 0.0f, 0.0f, 0.0f};

    // Write DAC outputs (all channels updated every 1ms)
    for (int ch = 0; ch < AD5724_NUM_CHANNELS; ch++) {
        AD5724_SetVoltage(&ad5724_instance, (ad5724_channel_t)ch, dac_outputs[ch]);
    }
}

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

#if ENABLE_W5200_SPI2
/**
 * @brief Initialize SPI2 W5200 network controller (192.168.2.x)
 * @retval true if successful, false if initialization failed
 */
static bool TCP_Server_SPI2_Init(void)
{
    uint8_t node_id = uchNodeNum;

    // Initialize W5200 hardware driver
    if (!W5200_Init(&w5200_spi2, SPI2, spi2_tx_buffer, spi2_rx_buffer, 0)) {
        return false;  // SPI2 init failed (no DMA)
    }

    // Configure network: 192.168.2.(100 + node_id), port 7000
    if (!NetCfg_ConfigureSPI2(&w5200_spi2, node_id)) {
        return false;
    }

    // Configure socket 0 as TCP server on port 7000
    if (!NetCfg_ConfigureTCPServer(&w5200_spi2, 0, 7000)) {
        return false;
    }

    // Open socket
    if (!W5200_SocketOpen(&w5200_spi2, 0)) {
        return false;
    }

    // Start listening
    if (!W5200_SocketListen(&w5200_spi2, 0)) {
        return false;
    }

    return true;
}
#endif  // ENABLE_W5200_SPI2

#if ENABLE_W5200_SPI3
/**
 * @brief Initialize SPI3 W5200 network controller (192.168.10.x)
 * @retval true if successful, false if initialization failed
 */
static bool TCP_Server_SPI3_Init(void)
{
    uint8_t node_id = uchNodeNum;

    // Initialize W5200 hardware driver (with 8K DMA)
    if (!W5200_Init(&w5200_spi3, SPI3, spi3_tx_buffer, spi3_rx_buffer, 0)) {  // W5200_DMA_SIZE_8K
        return false;                                                         // SPI3 init failed
    }

    // Configure network: 192.168.10.(31 + node_id), port 7000
    if (!NetCfg_ConfigureSPI3(&w5200_spi3, node_id)) {
        return false;
    }

    // Configure socket 0 as TCP server on port 7000
    if (!NetCfg_ConfigureTCPServer(&w5200_spi3, 0, 7000)) {
        return false;
    }

    // Open socket
    if (!W5200_SocketOpen(&w5200_spi3, 0)) {
        return false;
    }

    // Start listening
    if (!W5200_SocketListen(&w5200_spi3, 0)) {
        return false;
    }

    return true;
}
#endif  // ENABLE_W5200_SPI3

/**
 * @brief Initialize both W5200 network controllers
 * @retval true if successful, false if initialization failed
 */
static bool TCP_Server_Init(void)
{
    bool success = true;

#if ENABLE_W5200_SPI2
    if (!TCP_Server_SPI2_Init()) {
        success = false;
    }
#endif

#if ENABLE_W5200_SPI3
    if (!TCP_Server_SPI3_Init()) {
        success = false;
    }
#endif

    return success;
}

#if ENABLE_W5200_SPI2
/**
 * @brief Handle SPI2 TCP server socket (192.168.2.x network)
 * @note Handles connection, data transfer, and reconnection logic
 */
static void TCP_Server_SPI2_Task(void)
{
    uint16_t rx_size;
    uint16_t bytes_received;
    uint8_t status;
    static bool deadbeef_sent = false;

    // Get socket status
    status = W5200_GetSocketStatus(&w5200_spi2, 0);

    // Handle different socket states
    if (status == W5200_SOCK_ESTABLISHED) {
        // Send 0xDEADBEEF once when connection is first established
        if (!deadbeef_sent) {
            uint8_t deadbeef[4] = {0xDE, 0xAD, 0xBE, 0xEF};
            W5200_Send(&w5200_spi2, 0, deadbeef, 4);
            deadbeef_sent = true;
        }

        // Connection established - check for received data
        rx_size = W5200_GetRxSize(&w5200_spi2, 0);
        if (rx_size > 0) {
            // Limit to buffer size
            if (rx_size > sizeof(tcp_packet_buffer)) {
                rx_size = sizeof(tcp_packet_buffer);
            }

            // Receive data
            bytes_received = W5200_Receive(&w5200_spi2, 0, tcp_packet_buffer, rx_size);
            if (bytes_received > 0) {
                // Process received data here
                // For now, echo it back (example)
                W5200_Send(&w5200_spi2, 0, tcp_packet_buffer, bytes_received);
            }
        }
    }
    else if (status == W5200_SOCK_CLOSE_WAIT) {
        // Client requested disconnect
        W5200_SocketDisconnect(&w5200_spi2, 0);
        W5200_SocketClose(&w5200_spi2, 0);
        // Reopen and listen again
        W5200_SocketOpen(&w5200_spi2, 0);
        W5200_SocketListen(&w5200_spi2, 0);
        deadbeef_sent = false;  // Reset flag for next connection
    }
    else if (status == W5200_SOCK_CLOSED) {
        // Socket closed unexpectedly, reopen
        W5200_SocketOpen(&w5200_spi2, 0);
        W5200_SocketListen(&w5200_spi2, 0);
        deadbeef_sent = false;  // Reset flag for next connection
    }
}
#endif  // ENABLE_W5200_SPI2

#if ENABLE_W5200_SPI3
/**
 * @brief Handle SPI3 TCP server socket (192.168.10.x network)
 * @note Handles connection, data transfer, and reconnection logic
 */
static void TCP_Server_SPI3_Task(void)
{
    uint16_t rx_size;
    uint16_t bytes_received;
    uint8_t status;

    // Get socket status
    status = W5200_GetSocketStatus(&w5200_spi3, 0);

    // Handle different socket states
    if (status == W5200_SOCK_ESTABLISHED) {
        // Connection established - check for received data
        rx_size = W5200_GetRxSize(&w5200_spi3, 0);
        if (rx_size > 0) {
            // Limit to buffer size
            if (rx_size > sizeof(tcp_packet_buffer)) {
                rx_size = sizeof(tcp_packet_buffer);
            }

            // Receive data
            bytes_received = W5200_Receive(&w5200_spi3, 0, tcp_packet_buffer, rx_size);
            if (bytes_received > 0) {
                // Process received data
                W5200_Send(&w5200_spi3, 0, tcp_packet_buffer, bytes_received);
            }
        }
    }
    else if (status == W5200_SOCK_CLOSE_WAIT) {
        // Client requested disconnect
        W5200_SocketDisconnect(&w5200_spi3, 0);
        W5200_SocketClose(&w5200_spi3, 0);
        // Reopen and listen again
        W5200_SocketOpen(&w5200_spi3, 0);
        W5200_SocketListen(&w5200_spi3, 0);
    }
    else if (status == W5200_SOCK_CLOSED) {
        // Socket closed unexpectedly, reopen
        W5200_SocketOpen(&w5200_spi3, 0);
        W5200_SocketListen(&w5200_spi3, 0);
    }
}
#endif  // ENABLE_W5200_SPI3

/**
 * @brief Network communication task - handles both TCP servers
 * @note Call this periodically from main loop (similar to legacy Net_Check)
 */
static void TCP_Server_Task(void)
{
#if ENABLE_W5200_SPI2
    TCP_Server_SPI2_Task();
#endif

#if ENABLE_W5200_SPI3
    TCP_Server_SPI3_Task();
#endif
}

int main(void)
{
    int init_status;
    uint32_t last_check_time = 0;
    bool overrun_detected = false;
    bool tcp_initialized = false;

    /* Initialize all hardware using NSD_Driver */
    init_status = Init_Hardware();

    // If initialization failed, FAULT LED is already set by Init_Hardware()
    // The system will continue but with limited functionality
    (void)init_status;  // Acknowledge the status

    // Initialize network (W5200 on SPI2 and SPI3)
    // IP addresses configured based on DIP switch node ID (uchNodeNum)
    tcp_initialized = TCP_Server_Init();

    if (!tcp_initialized) {
        // Network initialization failed - set FAULT LED
        FSMC_GPIO_SetLED_Fault(&fsmc_gpio_instance, 1);
        FSMC_GPIO_WriteOutputs(&fsmc_gpio_instance);
    }

    while (1) {
        // Network communication task - handles TCP server operations
        if (tcp_initialized) {
            TCP_Server_Task();
        }

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
        // delay_ms(10);
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
