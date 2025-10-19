#include "adc.h"  // AD7606 ADC driver
#include "dac.h"  // AD5724 DAC driver
#include "driver_init.h"
#include "fiber_optic.h"            // Fiber-optic communication driver
#include "fiber_protocol.h"         // Fiber-optic protocol layer
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

static system_status_t system_status = {0};

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

// Convert uint32 payload to floats
static float remote_values[FIBER_PROTO_MAX_PAYLOAD_WORDS] = {0};

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

#define LOGGER_COMMUNICATION_LENGTH 128
static uint32_t log_communication_counter = 0;
static bool log_communication_state[LOGGER_COMMUNICATION_LENGTH] = {0};  // To avoid log flooding

/**
 * @brief  1ms periodic task - Fast control loop and DAC output
 * @note   Called from TIM4 ISR at 1 kHz rate (every 1 millisecond)
 *         Updates DAC outputs based on control algorithm
 */
void task_1ms(void)
{
    // TODO: Implement your control algorithm here
    // Example: Process ADC data and generate control outputs

    // ===== DAC Output (only if DAC initialized) =====
    if (system_status.dac_ok) {
        float dac_outputs[AD5724_NUM_CHANNELS] = {0.0f, 0.0f, 0.0f, 0.0f};

        // Write DAC outputs (all channels updated every 1ms)
        for (int ch = 0; ch < AD5724_NUM_CHANNELS; ch++) {
            AD5724_SetVoltage(&ad5724_instance, (ad5724_channel_t)ch, dac_outputs[ch]);
        }
    }

    bool received_data = false;

    // ===== Fiber-Optic Communication (Buffered RX for Reliable 1ms Operation) =====
    // The protocol layer now buffers incoming data across multiple 1ms cycles.
    // This ensures reliable reception even if frames arrive slowly or span cycles.
    // Benefits:
    // - No packet loss due to timing variations
    // - Automatic frame accumulation and parsing
    // - Robust resynchronization if frames are corrupted
    if (system_status.fiber_optic_ok && fiber_proto_instance.initialized) {
        // Static variables for heartbeat and state tracking
        static uint8_t heartbeat_counter = 0;
        static uint8_t current_state = STATE_IDLE;
        static uint8_t user_command = USER_CMD_NONE;

        // Packet reception timeout tracking (for FAULT LED)
        static uint32_t last_rx_time_ms = 0;
        static uint32_t first_valid_rx_time_ms = 0;  // Time when valid RX started
        static bool fault_led_on = false;
        static bool communication_stable = false;  // Tracks if we've had stable comms
        static bool first_cycle = true;            // Track first execution
        const uint32_t RX_TIMEOUT_MS = 1;          // 5ms timeout (5x the expected 1ms rate)
        const uint32_t STABLE_DURATION_MS = 5000;  // 5000ms (5 seconds) of stable data before clearing FAULT

        // Get current time once per cycle
        uint32_t current_time_ms = get_1ms_counter();

        // Initialize last_rx_time_ms on first cycle to avoid false timeout
        if (first_cycle) {
            last_rx_time_ms = current_time_ms;
            first_cycle = false;
        }

        // Update heartbeat (0-3, rolls over)
        heartbeat_counter = (heartbeat_counter + 1) & 0x03;

        // ===== TX: Build and transmit frame =====
        // Build command word
        uint8_t board_side = BOARD_SIDE_GRID;      // TODO: Replace with actual board side detection
        uint8_t trip_status = TRIP_STATUS_NORMAL;  // TODO: Read from fault detection system

        uint32_t tx_command = CMD_BUILD(
            heartbeat_counter, board_side, trip_status, user_command, current_state, PAYLOAD_TYPE_CONTROL_DATA);

        // Prepare payload (4 control values)
        float tx_payload[4];
        tx_payload[0] = 10.0f;  // Voltage command
        tx_payload[1] = 20.0f;  // Current command
        tx_payload[2] = 30.0f;  // Frequency command
        tx_payload[3] = 40.0f;  // Phase command

        // Transmit the frame (~277us @ 691,200 baud for 24-byte frame: START+CMD+LEN+4×float+CRC)
        fiber_proto_error_t tx_status = FiberProto_SendFloats(&fiber_proto_instance, tx_command, tx_payload, 4);

        // ===== RX: Receive and process frame (buffered reception) =====
        // The protocol layer accumulates data across multiple 1ms calls.
        // Returns FIBER_PROTO_OK only when a complete, valid frame is received.
        // Partial frames are buffered automatically - no data loss!
        uint32_t rx_command_temp;
        uint32_t rx_payload_temp[FIBER_PROTO_MAX_PAYLOAD_WORDS];
        uint8_t rx_length_temp;

        fiber_proto_error_t rx_status =
            FiberProto_ReceivePacket(&fiber_proto_instance, &rx_command_temp, rx_payload_temp, &rx_length_temp);

        if (rx_status == FIBER_PROTO_OK) {
            // Extract command word fields
            uint8_t rx_heartbeat = CMD_GET_HEARTBEAT(rx_command_temp);
            uint8_t rx_board_side = CMD_GET_BOARD_SIDE(rx_command_temp);
            uint8_t rx_trip_status = CMD_GET_TRIP_STATUS(rx_command_temp);
            uint8_t rx_user_cmd = CMD_GET_USER_COMMAND(rx_command_temp);
            uint8_t rx_state = CMD_GET_STATE_MACHINE(rx_command_temp);
            uint16_t rx_payload_type = CMD_GET_PAYLOAD_TYPE(rx_command_temp);

            // Check if remote board is tripped
            if (rx_trip_status == TRIP_STATUS_TRIPPED || rx_trip_status == TRIP_STATUS_FAULT) {
                // Remote board has fault - take action
                // Example: Stop operation, enter safe mode
            }

            // Process user command from remote board
            if (rx_user_cmd == USER_CMD_START) {
                current_state = STATE_STARTING;
            }
            else if (rx_user_cmd == USER_CMD_STOP) {
                current_state = STATE_STOPPING;
            }
            else if (rx_user_cmd == USER_CMD_EMERGENCY_STOP) {
                current_state = STATE_EMERGENCY;
            }

            // Parse payload
            FiberProto_ParseFloats(rx_payload_temp, rx_length_temp, remote_values, rx_length_temp);

            // ===== PAYLOAD VALIDATION TEST =====
            // Verify that received payload matches expected values (10.0f, 20.0f, 30.0f, 40.0f)
            // If mismatch detected, turn on ALARM LED
            static const float expected_payload[4] = {10.0f, 20.0f, 30.0f, 40.0f};
            bool payload_valid = true;

            // Check payload length
            if (rx_length_temp != 4) {
                payload_valid = false;
            }
            else {
                // Compare each value (with small tolerance for floating point comparison)
                const float tolerance = 0.01f;
                for (uint8_t i = 0; i < 4; i++) {
                    float diff = remote_values[i] - expected_payload[i];
                    if (diff < 0.0f) diff = -diff;  // abs()
                    if (diff > tolerance) {
                        payload_valid = false;
                        break;
                    }
                }
            }

            // Update FAULT LED based on validation result
            if (!payload_valid) {
                FSMC_GPIO_SetLED_Fault(&fsmc_gpio_instance, 1);  // Turn on FAULT for bad payload
                FSMC_GPIO_WriteOutputs(&fsmc_gpio_instance);
            }

            received_data = true;

            // Update last RX time (using current_time_ms declared at top of fiber-optic block)
            last_rx_time_ms = current_time_ms;

            // Track stability for FAULT LED hysteresis (requires both valid frames AND valid payload)
            if (!communication_stable) {
                if (payload_valid) {
                    // Payload is valid - accumulate stability
                    if (first_valid_rx_time_ms == 0) {
                        // First valid packet with valid payload - start stability timer
                        first_valid_rx_time_ms = current_time_ms;
                    }
                    else {
                        // Check if we've maintained stable communication for 100ms
                        uint32_t stable_duration = current_time_ms - first_valid_rx_time_ms;
                        if (stable_duration >= STABLE_DURATION_MS) {
                            communication_stable = true;
                        }
                    }
                }
                else {
                    // Payload invalid - reset stability tracking
                    first_valid_rx_time_ms = 0;
                }
            }
            else {
                // Already stable, but check if payload becomes invalid
                if (!payload_valid) {
                    // Payload corrupted - reset stability
                    communication_stable = false;
                    first_valid_rx_time_ms = 0;
                }
            }

            // Clear FAULT LED only if communication has been stable for 100ms (with valid payload)
            if (fault_led_on && communication_stable && system_status.fsmc_gpio_ok) {
                FSMC_GPIO_SetLED_Fault(&fsmc_gpio_instance, 0);
                FSMC_GPIO_WriteOutputs(&fsmc_gpio_instance);
                fault_led_on = false;
            }
        }
        else if (rx_status != FIBER_PROTO_ERR_TIMEOUT) {
            // Any error besides timeout resets stability tracking
            communication_stable = false;
            first_valid_rx_time_ms = 0;
        }

        // Check for RX timeout - if no packet received within timeout period, set FAULT LED
        // (using current_time_ms declared at top of fiber-optic block)
        uint32_t time_since_last_rx = current_time_ms - last_rx_time_ms;

        if (time_since_last_rx > RX_TIMEOUT_MS) {
            // Timeout exceeded - no packets received
            if (!fault_led_on && system_status.fsmc_gpio_ok) {
                // Turn on FAULT LED immediately
                FSMC_GPIO_SetLED_Fault(&fsmc_gpio_instance, 1);
                FSMC_GPIO_WriteOutputs(&fsmc_gpio_instance);
                fault_led_on = true;
                // Diagnostics_Log(DIAG_EVENT_USER_1, 0xF041, time_since_last_rx);  // Log "FOAL" (fiber optic fail)
            }

            // Reset stability tracking - need another 100ms of good data to clear fault
            communication_stable = false;
            first_valid_rx_time_ms = 0;
        }
    }

    // Append to logger
    log_communication_state[log_communication_counter++ % LOGGER_COMMUNICATION_LENGTH] = received_data;
}

/**
 * @brief  10ms periodic task - Slow monitoring
 * @note   Called from TIM2 ISR at 100 Hz rate (every 10 milliseconds)
 *         Controls RUN LED as a "heartbeat" indicator if tasks are running correctly
 */
void task_10ms(void)
{
    // Only update LEDs if FSMC GPIO initialized
    if (!system_status.fsmc_gpio_ok) {
        return;  // Skip task if GPIO not available
    }

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
    uint32_t last_check_time = 0;
    bool overrun_detected = false;

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

        // Check task timing every 100ms
        // Note: This overflow-safe subtraction works due to unsigned arithmetic wrapping
        if (system_status.timers_ok) {
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
                    if (!overrun_detected && system_status.fsmc_gpio_ok) {
                        // First time detecting overrun - set FAULT LED
                        overrun_detected = true;
                        FSMC_GPIO_SetLED_Fault(&fsmc_gpio_instance, 1);
                        FSMC_GPIO_WriteOutputs(&fsmc_gpio_instance);
                    }
                }

                // Optional: Clear overrun flag if system recovers
                // (uncomment if you want auto-recovery)
                // else {
                //     if (overrun_detected && system_status.fsmc_gpio_ok) {
                //         overrun_detected = false;
                //         FSMC_GPIO_SetLED_Fault(&fsmc_gpio_instance, 0);
                //         FSMC_GPIO_WriteOutputs(&fsmc_gpio_instance);
                //     }
                // }
            }
        }

        // Small delay to avoid tight polling (only if timers work)
        // if (system_status.timers_ok) {
        //     delay_ms(10);
        // }
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
