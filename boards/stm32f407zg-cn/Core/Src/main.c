#include "adc.h"                    // AD7606 ADC driver
#include "dac.h"                    // AD5724 DAC driver
#include "digital_ios.h"            // Digital I/O driver (inputs/outputs)
#include "driver_init.h"            // Driver initialization functions
#include "fiber_optic.h"            // Fiber-optic communication driver
#include "fiber_protocol.h"         // Fiber-optic protocol layer
#include "fpga.h"                   // FPGA configuration
#include "fsmc_gpio_driver.h"       // FSMC GPIO driver
#include "network_config_helper.h"  // Network configuration helpers
#include "sfc_embedded/task_io.h"   // Generic I/O interface for application layer
#include "stm32f4xx.h"              // STM32F4xx HAL
#include "tasks.h"                  // 200us, 1ms, 10ms task prototypes
#include "thyristor_control.h"      // Thyristor control driver
#include "w5200_driver.h"           // New W5200 driver

/* Application Layer Function Declarations --------------------------------- */
// Defined in yeet_application layer (grid_side_tasks.cpp or machine_side_tasks.cpp)
extern void yeet_embedded_init(void);
extern void yeet_task_200us(const task_inputs_t* inputs, task_outputs_t* outputs);
extern void yeet_task_1000us(const task_inputs_t* inputs, task_outputs_t* outputs);
extern void yeet_task_10000us(const task_inputs_t* inputs, task_outputs_t* outputs);
extern void yeet_network(const network_inputs_t* inputs, network_outputs_t* outputs);

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
 * @brief Digital I/O instance for reading inputs and writing outputs
 */
static dio_instance_t dio_instance = {0};

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
static uint8_t tcp_packet_buffer[1460];  // Max TCP MSS for SPI3 network
#endif

/**
 * @brief Node ID read from DIP switches (0-15)
 * Used to configure network IP addresses
 */
static uint8_t dip_switch_channel_number = 0;

/* Task I/O Structure Instances --------------------------------------------- */

/**
 * @brief Shared real-time task I/O structures
 * Single instance passed to all tasks (200us, 1ms, 10ms)
 * Application layer manages state - board support layer only populates inputs
 * and reads outputs without clearing/overriding between tasks
 */
static task_inputs_t task_inputs = {0};
static task_outputs_t task_outputs = {0};

/**
 * @brief Network I/O structures (non-real-time)
 * Separate structures for network communication (called from main loop)
 */
static network_inputs_t network_inputs = {0};
static network_outputs_t network_outputs = {0};

/* Application-Specific Functions ------------------------------------------- */

/**
 * @brief  Initialize DIP switch GPIO pins (PA8, PB2, PF11)
 * @note   DIP switches are configured as plain inputs with no pull-up/down
 *         PA8 = Bit 0, PB2 = Bit 1, PF11 = Bit 2
 */
static void DIP_Switch_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // Enable GPIO clocks for GPIOA, GPIOB, and GPIOF
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);

    // Configure PA8 as input (Bit 0)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;  // No pull-up/down
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Configure PB2 as input (Bit 1)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // Configure PF11 as input (Bit 2)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_Init(GPIOF, &GPIO_InitStructure);
}

/**
 * @brief  Read node ID from DIP switches
 * @retval Node ID value (0-7 for 3-bit configuration)
 * @note   DIP switches use direct logic (no pull resistors):
 *         - Reading GPIO state directly as binary value
 *         - Bit mapping: PA8 = Bit 0, PB2 = Bit 1, PF11 = Bit 2
 */
static uint8_t Read_DIP_Switch(void)
{
    uint8_t node_id = 0;

    // Read 3 bits from PA8, PB2, PF11 (direct logic: HIGH = 1, LOW = 0)
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8) == 1) {
        node_id |= 0x01;  // Bit 0 (PA8)
    }
    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_2) == 1) {
        node_id |= 0x02;  // Bit 1 (PB2)
    }
    if (GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_11) == 1) {
        node_id |= 0x04;  // Bit 2 (PF11)
    }

    return node_id;
}

/* Task Implementations ----------------------------------------------------- */

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

    // ===== Populate Generic Task Inputs =====
    // Map AD7606 ADC channels to generic analog inputs
    for (int i = 0; i < TASK_IO_NUM_ANALOG_INPUTS; i++) {
        task_inputs.analog_inputs.channels[i] = adc_reading.raw[i];
    }
    task_inputs.analog_inputs.valid = true;

    // Digital inputs (not available in 200us task - too fast for FSMC)
    task_inputs.digital_inputs.valid = false;

    // Fiber-optic data (updated in 1ms task, not read here)
    task_inputs.fiber_optic_rx.data_ready = false;
    task_inputs.fiber_optic_rx.link_ok = system_status.fiber_optic_ok;

    // ===== Call Application Layer Control Task =====
    yeet_task_200us(&task_inputs, &task_outputs);

    // ===== Apply Task Outputs to Hardware =====
    // Apply thyristor gate pulses
    set_thyristor_pulses(task_outputs.thyristor_pulses.a_plus, task_outputs.thyristor_pulses.c_minus,
        task_outputs.thyristor_pulses.b_plus, task_outputs.thyristor_pulses.a_minus,
        task_outputs.thyristor_pulses.c_plus, task_outputs.thyristor_pulses.b_minus);

    // ===== DAC Output (moved to 200us task for atomic operation with ADC) =====
    if (system_status.dac_ok && task_outputs.analog_outputs.valid) {
        // Extract DAC values from control outputs and apply to hardware
        for (int ch = 0; ch < AD5724_NUM_CHANNELS; ch++) {
            AD5724_SetVoltage(&ad5724_instance, (ad5724_channel_t)ch, task_outputs.analog_outputs.channels[ch]);
        }
    }

    // Digital outputs not updated here (too fast for FSMC)

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
    // ===== Populate Generic Task Inputs =====

    // Analog inputs (from ADC, but typically not used in 1ms task)
    task_inputs.analog_inputs.valid = false;  // ADC is read in 200us task

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

            // Populate generic task inputs with fiber-optic RX data
            for (int i = 0; i < rx_length && i < TASK_IO_FIBER_MAX_PAYLOAD; i++) {
                task_inputs.fiber_optic_rx.payload[i] = fiber_rx_payload[i];
            }
            task_inputs.fiber_optic_rx.payload_length = rx_length;
            task_inputs.fiber_optic_rx.data_ready = true;
            task_inputs.fiber_optic_rx.link_ok = true;

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
            task_inputs.fiber_optic_rx.data_ready = false;
            task_inputs.fiber_optic_rx.link_ok = false;
        }

        // ===== Timeout Detection (10ms) =====
        uint32_t time_since_last_rx = current_time_ms - fiber_status.last_rx_time_ms;

        if (time_since_last_rx > FIBER_RX_TIMEOUT_MS) {
            // Communication timeout detected
            fiber_status.communication_timeout = true;

            // Reset stability tracking
            fiber_status.communication_stable = false;
            first_valid_rx_time_ms = 0;
            task_inputs.fiber_optic_rx.data_ready = false;
            task_inputs.fiber_optic_rx.link_ok = false;
        }
    }
    else {
        // Fiber-optic not available
        task_inputs.fiber_optic_rx.data_ready = false;
        task_inputs.fiber_optic_rx.link_ok = false;
    }

    // ===== Digital I/O Input: Read hardware inputs (fuses, breakers, commands) =====
    if (system_status.fsmc_gpio_ok && dio_instance.initialized) {
        // Read all digital inputs with debouncing (~50µs)
        DIO_ReadInputs(&dio_instance);
        const dio_input_flags_t* digital_inputs = DIO_GetInputFlags(&dio_instance);

        // Map hardware digital inputs to generic task inputs (all 16 channels)
        // TODO: Document what each input signal represents in your application
        task_inputs.digital_inputs.signals[0] = digital_inputs->digital_input_1;
        task_inputs.digital_inputs.signals[1] = digital_inputs->digital_input_2;
        task_inputs.digital_inputs.signals[2] = digital_inputs->digital_input_3;
        task_inputs.digital_inputs.signals[3] = digital_inputs->digital_input_4;
        task_inputs.digital_inputs.signals[4] = digital_inputs->digital_input_5;
        task_inputs.digital_inputs.signals[5] = digital_inputs->digital_input_6;
        task_inputs.digital_inputs.signals[6] = digital_inputs->digital_input_7;
        task_inputs.digital_inputs.signals[7] = digital_inputs->digital_input_8;
        task_inputs.digital_inputs.signals[8] = digital_inputs->digital_input_9;
        task_inputs.digital_inputs.signals[9] = digital_inputs->digital_input_10;
        task_inputs.digital_inputs.signals[10] = digital_inputs->digital_input_11;
        task_inputs.digital_inputs.signals[11] = digital_inputs->digital_input_12;
        task_inputs.digital_inputs.signals[12] = digital_inputs->digital_input_13;
        task_inputs.digital_inputs.signals[13] = digital_inputs->digital_input_14;
        task_inputs.digital_inputs.signals[14] = digital_inputs->digital_input_15;
        task_inputs.digital_inputs.signals[15] = digital_inputs->digital_input_16;
        task_inputs.digital_inputs.valid = true;
    }
    else {
        task_inputs.digital_inputs.valid = false;
    }

    // ===== Call Application Layer Control Task =====
    yeet_task_1000us(&task_inputs, &task_outputs);

    // ===== Apply Task Outputs to Hardware =====

    // Digital I/O Output: Write status flags to hardware
    if (system_status.fsmc_gpio_ok && dio_instance.initialized) {
        dio_output_flags_t digital_outputs = {0};

        // Map generic digital outputs to hardware outputs
        // TODO: Complete signal mapping based on hardware documentation
        digital_outputs.digital_output_16 = !fiber_status.communication_timeout;  // Online status
        // Map remaining signals from task_outputs_1ms.digital_outputs.signals[]
        // digital_outputs.digital_output_1 = task_outputs_1ms.digital_outputs.signals[0];
        // ... map remaining signals ...

        DIO_SetOutputFlags(&dio_instance, &digital_outputs);
        DIO_WriteOutputs(&dio_instance);
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

        // Copy TX payload from application outputs
        for (int i = 0; i < TASK_IO_FIBER_MAX_PAYLOAD; i++) {
            fiber_tx_payload[i] = task_outputs.fiber_optic_tx.payload[i];
        }

        // Transmit frame (~347µs @ 691,200 baud for 24-byte frame)
        FiberProto_SendFloats(&fiber_proto_instance, tx_command, fiber_tx_payload,
            task_outputs.fiber_optic_tx.payload_length > 0 ? task_outputs.fiber_optic_tx.payload_length
                                                           : FIBER_PROTO_MAX_PAYLOAD_WORDS);
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
    // ===== Populate Generic Task Inputs =====
    // Analog and digital inputs not typically read at this slow rate
    task_inputs.analog_inputs.valid = false;
    task_inputs.digital_inputs.valid = false;
    task_inputs.fiber_optic_rx.data_ready = false;
    task_inputs.fiber_optic_rx.link_ok = system_status.fiber_optic_ok;

    // ===== Call Application Layer Task =====
    yeet_task_10000us(&task_inputs, &task_outputs);

    // ===== Apply Task Outputs (if any) =====
    // Typically 10ms task doesn't update outputs, but structure allows it

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
 *       Integrates with yeet_network() for application layer processing
 */
static void TCP_Server_SPI2_Task(void)
{
    uint16_t rx_size;
    uint16_t bytes_received;
    uint8_t status;
    bool sent;

    // Get socket status
    status = W5200_GetSocketStatus(&w5200_spi2, 0);

    // Handle different socket states
    if (status == W5200_SOCK_ESTABLISHED) {
        // Update network input status
        network_inputs.network_rx.tcp_connected = true;
        network_inputs.network_rx.socket_id = 0;

        // Default values when no RX data is present
        network_inputs.network_rx.rx_length = 0;
        network_inputs.network_rx.data_ready = false;

        // Connection established - check for received data
        rx_size = W5200_GetRxSize(&w5200_spi2, 0);
        if (rx_size > 0) {
            // Limit to buffer size (use network_inputs buffer directly)
            if (rx_size > TASK_IO_NETWORK_MAX_PACKET_SIZE) {
                rx_size = TASK_IO_NETWORK_MAX_PACKET_SIZE;
            }

            // Receive data directly into network_inputs buffer
            bytes_received = W5200_Receive(&w5200_spi2, 0, network_inputs.network_rx.rx_buffer, rx_size);
            if (bytes_received > 0) {
                // Populate network inputs for application layer
                network_inputs.network_rx.rx_length = bytes_received;
                network_inputs.network_rx.data_ready = true;
            }
        }

        // Call application layer network handler every loop (even without new RX data)
        yeet_network(&network_inputs, &network_outputs);

        // Clear data_ready flag after processing so one-shot commands do not repeat
        network_inputs.network_rx.data_ready = false;

        // Attempt to transmit if application requested it
        if (network_outputs.network_tx.transmit_request) {
            sent = W5200_Send(&w5200_spi2, network_outputs.network_tx.socket_id, network_outputs.network_tx.tx_buffer,
                network_outputs.network_tx.tx_length);
            if (sent) {
                network_outputs.network_tx.transmit_request = false;
            }
            // If send fails (e.g., TX buffer full), keep transmit_request true to retry next iteration
        }
    }
    else if (status == W5200_SOCK_CLOSE_WAIT) {
        // Client requested disconnect
        network_inputs.network_rx.tcp_connected = false;
        W5200_SocketDisconnect(&w5200_spi2, 0);
        W5200_SocketClose(&w5200_spi2, 0);
        // Reopen and listen again
        W5200_SocketOpen(&w5200_spi2, 0);
        W5200_SocketListen(&w5200_spi2, 0);
    }
    else if (status == W5200_SOCK_CLOSED) {
        // Socket closed unexpectedly, reopen
        network_inputs.network_rx.tcp_connected = false;
        W5200_SocketOpen(&w5200_spi2, 0);
        W5200_SocketListen(&w5200_spi2, 0);
    }
    else {
        // Other states (INIT, LISTEN, etc.) - not connected
        network_inputs.network_rx.tcp_connected = false;
        network_outputs.network_tx.transmit_request = false;
        network_outputs.network_tx.tx_length = 0;
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
    FPGA_Status_t fpga_status;
    bool hardware_ok = true;

    /* ========================================================================
     * HARDWARE INITIALIZATION SEQUENCE
     * ======================================================================== */

    // 1. Read board configuration from DIP switches (PA8, PB2, PF11)
    //    DIP switches determine node ID for network addressing (0-7 for 3-bit)
    DIP_Switch_Init();
    dip_switch_channel_number = Read_DIP_Switch();
    // Initialize thyristor driver outputs (GPIO)
    Thyristor_DO_Init();

    // 2. Initialize timers FIRST (required for FPGA delay functions and tasks)
    TIMx_Init();
    system_status.timers_ok = true;

    // 3. Initialize FPGA (required for fiber-optic communication)
    //    Configure FPGA with 5-second timeout protection
    FPGA_Init_Pins();
    fpga_status = FPGA_Configure(5000);

    if (fpga_status == FPGA_CONFIG_SUCCESS) {
        FPGA_Reset();  // Perform internal reset after successful configuration
        system_status.fpga_ok = true;
    }
    else {
        // FPGA configuration failed - fiber-optic won't work
        system_status.fpga_ok = false;
        hardware_ok = false;
    }

    // 4. Initialize FSMC GPIO (LEDs and digital I/O at 0x6C000000)
    //    Must be after FPGA for some board configurations
    if (FSMC_GPIO_Init(&fsmc_gpio_instance)) {
        system_status.fsmc_gpio_ok = true;
    }
    else {
        system_status.fsmc_gpio_ok = false;
        hardware_ok = false;
    }

    // 5. Initialize external ADC (AD7606 8-channel via FSMC Bank 2)
    if (AD7606_Init(&ad7606_instance)) {
        system_status.adc_ok = true;
    }
    else {
        system_status.adc_ok = false;
        hardware_ok = false;
    }

    // 6. Initialize external DAC (AD5724 4-channel via SPI1)
    if (AD5724_Init(&ad5724_instance)) {
        system_status.dac_ok = true;
    }
    else {
        system_status.dac_ok = false;
        hardware_ok = false;
    }

    // 7. Initialize fiber-optic communication (691,200 baud)
    //    Requires successful FPGA configuration
    if (FiberOptic_Init(&fiber_optic_instance, 691200, dip_switch_channel_number)) {
        system_status.fiber_optic_ok = true;
    }
    else {
        system_status.fiber_optic_ok = false;
        hardware_ok = false;
    }

    // 8. Initialize fiber-optic protocol layer (frame handling)
    if (system_status.fiber_optic_ok) {
        FiberProto_Init(&fiber_proto_instance, &fiber_optic_instance, FIBER_CH1);
    }

    // 9. Initialize digital I/O driver (64 inputs + 64 outputs via FSMC GPIO)
    if (system_status.fsmc_gpio_ok) {
        if (!DIO_Init(&dio_instance, &fsmc_gpio_instance)) {
            dio_instance.initialized = false;
        }
    }

    // 11. Initialize network (W5200 on SPI2 and SPI3)
    //     IP addresses configured based on DIP switch node ID
    system_status.network_ok = TCP_Server_Init();

    // 11. Initialize application layer (yeet control algorithms)
    yeet_embedded_init();

    /* ========================================================================
     * ERROR HANDLING - Set FAULT LED if critical subsystems failed
     * ======================================================================== */

    if (!hardware_ok) {
        // Hardware initialization failed
        if (system_status.fsmc_gpio_ok) {
            FSMC_GPIO_SetLED_Fault(&fsmc_gpio_instance, 1);
            FSMC_GPIO_WriteOutputs(&fsmc_gpio_instance);
        }
    }
    else if (!system_status.network_ok) {
        // Network failed but hardware OK - indicate with FAULT LED
        FSMC_GPIO_SetLED_Fault(&fsmc_gpio_instance, 1);
        FSMC_GPIO_WriteOutputs(&fsmc_gpio_instance);
    }

    while (1) {
        // ===== Network Communication Task (Non-Real-Time) =====
        // Handles TCP server operations only if network initialized successfully
        // Note: yeet_network() is now called directly inside TCP_Server_SPI2_Task()
        //       when data is received, so it's not called here separately
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
