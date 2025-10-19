#include "adc.h"                    // AD7606 ADC driver
#include "dac.h"                    // AD5724 DAC driver
#include "driver_init.h"            // Driver initialization functions
#include "fiber_optic.h"            // Fiber-optic communication driver
#include "fiber_protocol.h"         // Fiber-optic protocol layer
#include "network_config_helper.h"  // Network configuration helpers
#include "stm32f4xx.h"              // STM32F4xx HAL
#include "tasks.h"                  // 200us, 1ms, 10ms task prototypes
#include "thyristor_control.h"      // Thyristor control driver
#include "w5200_driver.h"           // New W5200 driver

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
 * @brief Subsystem initialization status flags
 * Used to track which hardware components initialized successfully
 */
typedef struct {
    bool timers_ok;       ///< Timer system initialized
    bool fpga_ok;         ///< FPGA configured successfully
    bool fsmc_gpio_ok;    ///< FSMC GPIO driver initialized
    bool adc_ok;          ///< AD7606 ADC initialized
    bool dac_ok;          ///< AD5724 DAC initialized
    bool fiber_optic_ok;  ///< Fiber-optic communication initialized
    bool network_ok;      ///< W5200 network initialized
} system_status_t;

/**
 * @brief Runtime status for fiber-optic communication
 * Used to track communication health and control LED indicators
 */
typedef struct {
    bool communication_timeout;  ///< True if no valid packets received within timeout (10ms)
    bool communication_stable;   ///< True if stable communication for 5s (clears FAULT LED)
    uint32_t last_rx_time_ms;    ///< Timestamp of last valid packet received
} fiber_runtime_status_t;

static system_status_t system_status = {0};
static fiber_runtime_status_t fiber_status = {0};

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
 * @brief Fiber-optic communication instance
 */
fiber_optic_instance_t fiber_optic_instance = {0};

/**
 * @brief Fiber-optic protocol instance
 */
fiber_proto_instance_t fiber_proto_instance = {0};

/**
 * @brief Fiber-optic communication payload buffers
 * These buffers store the latest TX/RX payload data for integration with control logic
 */
static float fiber_tx_payload[FIBER_PROTO_MAX_PAYLOAD_WORDS] = {0};  ///< TX payload (from control logic)
static float fiber_rx_payload[FIBER_PROTO_MAX_PAYLOAD_WORDS] = {0};  ///< RX payload (to control logic)
static uint8_t fiber_rx_length = 0;                                  ///< Number of floats received
static bool fiber_rx_data_ready = false;                             ///< New data available flag

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
    // Only execute ADC operations if ADC initialized successfully
    if (!system_status.adc_ok) {
        return;  // Skip task if ADC not available
    }

    static bool first_run = true;

    // Pipeline operation: Read previous conversion, then start next
    if (first_run) {
        first_run = false;  // Skip read on first run (no prior conversion)
        // Start conversion for next cycle (takes ~5µs)
        AD7606_StartConversion(&ad7606_instance);
        return;
    }

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

    // Start conversion for next cycle (takes ~5µs)
    AD7606_StartConversion(&ad7606_instance);
}

/**
 * @brief Fiber-optic communication timeout configuration
 */
#define FIBER_RX_TIMEOUT_MS      10    ///< Communication timeout (10ms) - triggers ALARM LED
#define FIBER_STABLE_DURATION_MS 5000  ///< Stability duration (5s) - clears FAULT LED after stable comms

/**
 * @brief  Get TX payload pointer for control logic to populate
 * @note   Call this before task_1ms to prepare outgoing data
 * @retval Pointer to TX payload buffer (FIBER_PROTO_MAX_PAYLOAD_WORDS floats)
 */
float* FiberComm_GetTxPayloadBuffer(void) { return fiber_tx_payload; }

/**
 * @brief  Get RX payload data received from remote board
 * @param  payload_out: Buffer to copy received data (min FIBER_PROTO_MAX_PAYLOAD_WORDS floats)
 * @param  length_out: Pointer to store number of floats received
 * @retval true if new data available, false if no new data since last call
 */
bool FiberComm_GetRxPayload(float* payload_out, uint8_t* length_out)
{
    if (!fiber_rx_data_ready) {
        return false;
    }

    // Copy data to output buffer
    for (uint8_t i = 0; i < fiber_rx_length; i++) {
        payload_out[i] = fiber_rx_payload[i];
    }
    *length_out = fiber_rx_length;

    // Clear ready flag
    fiber_rx_data_ready = false;

    return true;
}

/**
 * @brief  1ms periodic task - Fast control loop and fiber-optic communication
 * @note   Called from TIM4 ISR at 1 kHz rate (every 1 millisecond)
 *
 * Execution Order:
 * 1. Fiber-Optic RX: Receive data from remote board (needed for control algorithm)
 * 2. Control Algorithm: Process data and compute outputs
 * 3. Fiber-Optic TX: Transmit control outputs to remote board
 *
 * Integration Points:
 * - fiber_rx_payload[]: Contains data from remote board (updated at start of task)
 * - fiber_tx_payload[]: Populate with control outputs (before TX at end of task)
 *
 * Runtime Status:
 * - Updates fiber_status structure for LED management in 10ms task
 * - fiber_status.communication_timeout: Set when >10ms without valid packets
 * - fiber_status.communication_stable: Set after 5s of continuous valid communication
 */
void task_1ms(void)
{
    // ===== Fiber-Optic RX: Receive data FIRST (before control algorithm) =====
    if (system_status.fiber_optic_ok && fiber_proto_instance.initialized) {
        // State tracking
        static uint32_t first_valid_rx_time_ms = 0;
        static bool first_cycle = true;

        // Get current time once per cycle
        uint32_t current_time_ms = get_1ms_counter();

        // Initialize on first cycle to avoid false timeout
        if (first_cycle) {
            fiber_status.last_rx_time_ms = current_time_ms;
            first_cycle = false;
        }

        // ===== RX: Receive payload from remote board =====
        uint32_t rx_command;
        uint32_t rx_payload_words[FIBER_PROTO_MAX_PAYLOAD_WORDS];
        uint8_t rx_length;

        fiber_proto_error_t rx_status =
            FiberProto_ReceivePacket(&fiber_proto_instance, &rx_command, rx_payload_words, &rx_length);

        if (rx_status == FIBER_PROTO_OK) {
            // Valid frame received - parse payload into fiber_rx_payload[]
            FiberProto_ParseFloats(rx_payload_words, rx_length, fiber_rx_payload, rx_length);
            fiber_rx_length = rx_length;
            fiber_rx_data_ready = true;

            // TODO: Process command word if needed
            // uint8_t rx_trip_status = CMD_GET_TRIP_STATUS(rx_command);
            // uint8_t rx_user_cmd = CMD_GET_USER_COMMAND(rx_command);
            // if (rx_trip_status == TRIP_STATUS_FAULT) { /* handle remote fault */ }

            // Update RX timestamp
            fiber_status.last_rx_time_ms = current_time_ms;

            // Track stability for FAULT LED (5s of stable communication before clearing)
            if (!fiber_status.communication_stable) {
                if (first_valid_rx_time_ms == 0) {
                    first_valid_rx_time_ms = current_time_ms;
                }
                else {
                    uint32_t stable_duration = current_time_ms - first_valid_rx_time_ms;
                    if (stable_duration >= FIBER_STABLE_DURATION_MS) {
                        fiber_status.communication_stable = true;
                    }
                }
            }

            // Clear timeout flag on successful RX
            fiber_status.communication_timeout = false;
        }
        else if (rx_status != FIBER_PROTO_ERR_TIMEOUT) {
            // Error (not timeout) - reset stability tracking
            fiber_status.communication_stable = false;
            first_valid_rx_time_ms = 0;
        }

        // ===== Timeout Detection (10ms) =====
        uint32_t time_since_last_rx = current_time_ms - fiber_status.last_rx_time_ms;

        if (time_since_last_rx > FIBER_RX_TIMEOUT_MS) {
            // Communication timeout detected
            fiber_status.communication_timeout = true;

            // Reset stability tracking
            fiber_status.communication_stable = false;
            first_valid_rx_time_ms = 0;
        }
    }

    // ===== Control Algorithm Integration Point =====
    // At this point, fiber_rx_payload[] contains latest data from remote board
    // TODO: Implement your control algorithm here
    // Example: Process ADC data and fiber_rx_payload[] to generate control outputs

    // ===== DAC Output (only if DAC initialized) =====
    if (system_status.dac_ok) {
        float dac_outputs[AD5724_NUM_CHANNELS] = {0.0f, 0.0f, 0.0f, 0.0f};

        // TODO: Populate dac_outputs[] from control algorithm
        // Example: dac_outputs[0] = pid_controller_output;
        // Example: dac_outputs[1] = fiber_rx_payload[0] * gain;  // Use remote data

        // Write DAC outputs (all channels updated every 1ms)
        for (int ch = 0; ch < AD5724_NUM_CHANNELS; ch++) {
            AD5724_SetVoltage(&ad5724_instance, (ad5724_channel_t)ch, dac_outputs[ch]);
        }
    }

    // ===== Fiber-Optic TX: Transmit AFTER control algorithm =====
    if (system_status.fiber_optic_ok && fiber_proto_instance.initialized) {
        static uint8_t heartbeat_counter = 0;

        // Update heartbeat (0-3, rolls over every 4ms)
        heartbeat_counter = (heartbeat_counter + 1) & 0x03;

        // Build command word
        uint8_t board_side = BOARD_SIDE_GRID;      // TODO: Replace with actual board side detection
        uint8_t trip_status = TRIP_STATUS_NORMAL;  // TODO: Read from fault detection system
        uint8_t user_command = USER_CMD_NONE;      // TODO: Set from upstream command logic
        uint8_t current_state = STATE_IDLE;        // TODO: Set from state machine

        uint32_t tx_command = CMD_BUILD(
            heartbeat_counter, board_side, trip_status, user_command, current_state, PAYLOAD_TYPE_CONTROL_DATA);

        // TODO: Populate fiber_tx_payload[] from control logic
        // Example: fiber_tx_payload[0] = voltage_setpoint;
        //          fiber_tx_payload[1] = current_setpoint;
        //          fiber_tx_payload[2] = frequency_setpoint;
        //          fiber_tx_payload[3] = phase_setpoint;

        // Transmit frame (~347µs @ 691,200 baud for 24-byte frame)
        FiberProto_SendFloats(&fiber_proto_instance, tx_command, fiber_tx_payload, FIBER_PROTO_MAX_PAYLOAD_WORDS);
    }
}

/**
 * @brief  Update system LEDs based on runtime status
 * @note   Centralized LED control - all LED updates happen here
 *         This prevents confusion about which code controls which LED
 */
static void Update_System_LEDs()
{
    // ===== RUN LED: Heartbeat and preemption status =====
    if (TIM_Verify_Preemption()) {
        FSMC_GPIO_SetLED_Run(&fsmc_gpio_instance, 1);
    }
    else {
        // Preemption failed - turn off RUN LED as error indicator
        FSMC_GPIO_SetLED_Run(&fsmc_gpio_instance, 0);
    }

    // ===== FAULT LED: Set on timeout, cleared after 5s of stable communication =====
    if (fiber_status.communication_timeout) {
        FSMC_GPIO_SetLED_Fault(&fsmc_gpio_instance, 1);
    }
    else if (fiber_status.communication_stable) {
        FSMC_GPIO_SetLED_Fault(&fsmc_gpio_instance, 0);
    }
    // Note: FAULT LED stays ON between timeout and stability (hysteresis behavior)

    // Write all LED states to hardware in one operation
    FSMC_GPIO_WriteOutputs(&fsmc_gpio_instance);
}

/**
 * @brief  10ms periodic task - Slow monitoring and LED management
 * @note   Called from TIM2 ISR at 100 Hz rate (every 10 milliseconds)
 *         Controls RUN LED heartbeat and manages system LEDs
 *         Monitors task execution time and blinks ALARM if load > 50%
 */
void task_10ms(void)
{
    // Update all system LEDs based on runtime status (called every 10ms)
    Update_System_LEDs();
}

#if ENABLE_W5200_SPI2
/**
 * @brief Initialize SPI2 W5200 network controller (192.168.2.x)
 * @retval true if successful, false if initialization failed
 */
static bool TCP_Server_SPI2_Init(void)
{
    uint8_t node_id = dip_switch_channel_number;

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

    /* Initialize all hardware using NSD_Driver */
    // This returns 0 on success, -1 if any component failed
    // Individual component status is tracked in system_status structure
    init_status = Init_Hardware();

    // Update system status flags based on hardware initialization
    // Timers are assumed OK if we reached here (required for Init_Hardware to work)
    system_status.timers_ok = true;

    // Check individual subsystem initialization
    // FPGA check: If init failed, FPGA-dependent features won't work
    system_status.fpga_ok = (init_status == 0);  // For now, assume FPGA is critical

    // FSMC GPIO check: Try to verify by checking if instance is initialized
    system_status.fsmc_gpio_ok = (init_status == 0);

    // ADC check: Instance will have initialized flag
    system_status.adc_ok = ad7606_instance.initialized;

    // DAC check: Instance will have initialized flag
    system_status.dac_ok = ad5724_instance.initialized;

    // Fiber-optic check: Instance will have initialized flag
    system_status.fiber_optic_ok = fiber_optic_instance.initialized;

    // Initialize fiber-optic protocol layer
    if (system_status.fiber_optic_ok) {
        FiberProto_Init(&fiber_proto_instance, &fiber_optic_instance, FIBER_CH1);
    }

    // Initialize network (W5200 on SPI2 and SPI3)
    // IP addresses configured based on DIP switch node ID
    system_status.network_ok = TCP_Server_Init();

    // Set FAULT LED if critical subsystems failed
    if (!system_status.fsmc_gpio_ok || !system_status.fpga_ok) {
        // Can't set FAULT LED if FSMC GPIO failed
        if (system_status.fsmc_gpio_ok) {
            FSMC_GPIO_SetLED_Fault(&fsmc_gpio_instance, 1);
            FSMC_GPIO_WriteOutputs(&fsmc_gpio_instance);
        }
    }
    else if (!system_status.network_ok) {
        // Network failed but GPIO works - indicate with FAULT LED
        FSMC_GPIO_SetLED_Fault(&fsmc_gpio_instance, 1);
        FSMC_GPIO_WriteOutputs(&fsmc_gpio_instance);
    }

    while (1) {
        // Network communication task - handles TCP server operations
        // Only run if network initialized successfully
        if (system_status.network_ok) {
            TCP_Server_Task();
        }
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
