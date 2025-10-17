#include "driver_init.h"
#include "stm32f4xx.h"
#include "tasks.h"
#include "w5200_driver.h"

/* Task prototypes (called by timer ISRs in tasks.c) ----------------------- */
void task_200us(void);
void task_1ms(void);
void task_10ms(void);

/* Network function prototypes ---------------------------------------------- */
void TCP_Server_Task(void);

/* Private variables -------------------------------------------------------- */

/**
 * @brief FSMC GPIO driver instance (for LEDs and digital I/O via FSMC Bank 4)
 */
fsmc_gpio_instance_t fsmc_gpio_instance = {0};

/**
 * @brief W5200 Ethernet controller instance (SPI2)
 */
static w5200_instance_t w5200_spi2 = {0};

/**
 * @brief TX/RX DMA buffers for W5200 (must be global/static for DMA)
 */
static uint8_t w5200_tx_buffer[W5200_DMA_SIZE_1K + 4] __attribute__((aligned(4)));
static uint8_t w5200_rx_buffer[W5200_DMA_SIZE_1K + 4] __attribute__((aligned(4)));

/**
 * @brief TCP packet buffer for transmission
 */
static uint8_t tcp_tx_packet[256] __attribute__((aligned(4)));

/**
 * @brief TCP packet buffer for reception
 */
static uint8_t tcp_rx_packet[256] __attribute__((aligned(4)));

/**
 * @brief TCP transmission counter
 */
static uint32_t tcp_packet_count = 0;

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

/**
 * @brief  Initialize W5200 Ethernet controller and TCP socket
 * @retval true if initialization successful, false otherwise
 */
static bool TCP_Server_Init(void)
{
    w5200_network_config_t net_config;
    w5200_socket_config_t socket_config;

    // Initialize W5200 hardware on SPI2
    if (!W5200_Init(&w5200_spi2, SPI2, w5200_tx_buffer, w5200_rx_buffer, W5200_DMA_SIZE_1K)) {
        return false;
    }

    // Configure network parameters
    // Note: Adjust these to match your network configuration
    net_config.ip_address.bytes[0] = 192;
    net_config.ip_address.bytes[1] = 168;
    net_config.ip_address.bytes[2] = 1;
    net_config.ip_address.bytes[3] = 100;  // STM32 IP: 192.168.1.100

    net_config.subnet_mask.bytes[0] = 255;
    net_config.subnet_mask.bytes[1] = 255;
    net_config.subnet_mask.bytes[2] = 255;
    net_config.subnet_mask.bytes[3] = 0;

    net_config.gateway.bytes[0] = 192;
    net_config.gateway.bytes[1] = 168;
    net_config.gateway.bytes[2] = 1;
    net_config.gateway.bytes[3] = 1;

    // MAC address (make sure it's unique on your network)
    net_config.mac_address.bytes[0] = 0x00;
    net_config.mac_address.bytes[1] = 0x08;
    net_config.mac_address.bytes[2] = 0xDC;
    net_config.mac_address.bytes[3] = 0xAB;
    net_config.mac_address.bytes[4] = 0xCD;
    net_config.mac_address.bytes[5] = 0xEF;

    net_config.retry_time = 400;  // 400ms retry time
    net_config.retry_count = 8;   // 8 retries

    if (!W5200_ConfigureNetwork(&w5200_spi2, &net_config)) {
        return false;
    }

    // Configure TCP socket (Socket 0) as SERVER
    socket_config.enabled = true;
    socket_config.protocol = W5200_PROTO_TCP;
    socket_config.mode = W5200_MODE_SERVER;  // TCP server mode (like archive)
    socket_config.source_port = 7000;        // Local port 7000 (matches archive)

    // Destination not needed for TCP server (client will connect to us)
    socket_config.dest_ip.bytes[0] = 0;
    socket_config.dest_ip.bytes[1] = 0;
    socket_config.dest_ip.bytes[2] = 0;
    socket_config.dest_ip.bytes[3] = 0;
    socket_config.dest_port = 0;

    if (!W5200_ConfigureSocket(&w5200_spi2, 0, &socket_config)) {
        return false;
    }

    // Open TCP socket (will open and start listening)
    if (!W5200_SocketOpen(&w5200_spi2, 0)) {
        return false;
    }

    return true;
}

/**
 * @brief  TCP server task - handles connections and sends periodic packets
 * @note   Call this function from main loop
 */
void TCP_Server_Task(void)
{
    static uint32_t last_send_time = 0;
    static bool client_connected = false;
    uint32_t current_time = get_1ms_counter();

    // Check socket status
    uint8_t socket_status = W5200_GetSocketStatus(&w5200_spi2, 0);

    // Handle different TCP states
    switch (socket_status) {
        case W5200_SOCK_LISTEN:
            // Waiting for client connection
            client_connected = false;
            break;

        case W5200_SOCK_ESTABLISHED:
            // Client connected!
            if (!client_connected) {
                client_connected = true;
                tcp_packet_count = 0;
            }

            // Check for incoming data
            uint16_t rx_size = W5200_GetRxSize(&w5200_spi2, 0);
            if (rx_size > 0) {
                // Receive data from client
                uint16_t received = W5200_Receive(&w5200_spi2, 0, tcp_rx_packet, sizeof(tcp_rx_packet));
                if (received > 0) {
                    // Echo back or process received data here
                    // For now, just acknowledge receipt by toggling ALARM LED
                    FSMC_GPIO_ToggleLED_Alarm(&fsmc_gpio_instance);
                    FSMC_GPIO_WriteOutputs(&fsmc_gpio_instance);
                }
            }

            // Send periodic packets every 100ms when connected
            if ((current_time - last_send_time) >= 100) {
                last_send_time = current_time;

                // Prepare packet with magic value 0xDEADBEEF
                tcp_tx_packet[0] = 0xDE;
                tcp_tx_packet[1] = 0xAD;
                tcp_tx_packet[2] = 0xBE;
                tcp_tx_packet[3] = 0xEF;

                // Add packet counter (4 bytes, big-endian)
                tcp_tx_packet[4] = (tcp_packet_count >> 24) & 0xFF;
                tcp_tx_packet[5] = (tcp_packet_count >> 16) & 0xFF;
                tcp_tx_packet[6] = (tcp_packet_count >> 8) & 0xFF;
                tcp_tx_packet[7] = tcp_packet_count & 0xFF;

                // Add timestamp (4 bytes, big-endian)
                tcp_tx_packet[8] = (current_time >> 24) & 0xFF;
                tcp_tx_packet[9] = (current_time >> 16) & 0xFF;
                tcp_tx_packet[10] = (current_time >> 8) & 0xFF;
                tcp_tx_packet[11] = current_time & 0xFF;

                // Send 12 bytes via TCP
                if (W5200_Send(&w5200_spi2, 0, tcp_tx_packet, 12)) {
                    tcp_packet_count++;
                }
            }
            break;

        case W5200_SOCK_CLOSE_WAIT:
            // Client disconnecting - close our side
            W5200_SocketDisconnect(&w5200_spi2, 0);
            client_connected = false;
            break;

        case W5200_SOCK_CLOSED:
            // Socket closed - reopen and start listening
            W5200_SocketClose(&w5200_spi2, 0);
            W5200_SocketOpen(&w5200_spi2, 0);
            client_connected = false;
            break;

        default:
            // Unknown state or transitioning
            break;
    }
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

    // Initialize TCP server
    tcp_initialized = TCP_Server_Init();
    if (!tcp_initialized) {
        // TCP initialization failed - set ALARM LED
        FSMC_GPIO_SetLED_Alarm(&fsmc_gpio_instance, 1);
        FSMC_GPIO_WriteOutputs(&fsmc_gpio_instance);
    }

    while (1) {
        // TCP server task - handle connections and send periodic packets
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
