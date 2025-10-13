/**
 ******************************************************************************
 * @file    fpga_io_driver.c
 * @brief   FPGA Digital I/O driver implementation
 * @date    2025
 ******************************************************************************
 */

#include "fpga_io_driver.h"

/* ============================================================================
 * Private Variables
 * ============================================================================ */

/** @brief Debounce counters for each input (internal use) */
static int s_debounce_count[FPGA_NUM_DI] = {0};

/* ============================================================================
 * Private Function Prototypes
 * ============================================================================ */

static void FPGA_IO_Read16Inputs(uint32_t reg_addr, bool* output_array);
static void FPGA_IO_Write16Outputs(uint32_t reg_addr, const bool* input_array);
static void FPGA_IO_DebounceInput(uint8_t channel, bool current_state, bool* debounced_state, int* delay_counter);

/* ============================================================================
 * Public Function Implementations - Initialization
 * ============================================================================ */

/**
 * @brief Initialize FPGA I/O driver
 */
void FPGA_IO_Init(void)
{
    // Note: FSMC already initialized by MX_FSMC_Init() in main.c

    // Initialize debounce counters
    for (int i = 0; i < FPGA_NUM_DI; i++) {
        s_debounce_count[i] = 0;
    }
}

/**
 * @brief Reset FPGA I/O state to defaults
 */
void FPGA_IO_Reset(fpga_io_t* io)
{
    if (io == NULL) {
        return;
    }

    // Clear all input states
    for (int i = 0; i < FPGA_NUM_DI; i++) {
        io->inputs.raw[i] = false;
        io->inputs.debounced[i] = false;
        s_debounce_count[i] = 0;
    }
    io->inputs.update_count = 0;

    // Clear all output states
    for (int i = 0; i < FPGA_NUM_DO; i++) {
        io->outputs.state[i] = false;
    }
    io->outputs.update_count = 0;

    // Set default LED states
    io->leds.run = true;  // Run LED typically blinks
    io->leds.fault = false;
    io->leds.alarm = false;

    // Write cleared states to FPGA
    FPGA_IO_WriteOutputs(io);
    FPGA_IO_WriteLEDs(io);

    // Clear control register
    *(__IO uint16_t*)CTRL_REG_ADDR = 0;
}

/* ============================================================================
 * Public Function Implementations - Digital Inputs
 * ============================================================================ */

/**
 * @brief Read all digital inputs from FPGA
 */
void FPGA_IO_ReadInputs(fpga_io_t* io)
{
    if (io == NULL) {
        return;
    }

    // Read 3 groups of 16 inputs each
    FPGA_IO_Read16Inputs(DI_REG0_ADDR, &io->inputs.raw[0]);   // DI0-DI15
    FPGA_IO_Read16Inputs(DI_REG1_ADDR, &io->inputs.raw[16]);  // DI16-DI31
    FPGA_IO_Read16Inputs(DI_REG2_ADDR, &io->inputs.raw[32]);  // DI32-DI47

    io->inputs.update_count++;
}

/**
 * @brief Update debounce filter for all inputs
 */
void FPGA_IO_UpdateDebounce(fpga_io_t* io)
{
    if (io == NULL) {
        return;
    }

    // Apply debounce filter to each input
    for (int i = 0; i < FPGA_NUM_DI; i++) {
        FPGA_IO_DebounceInput(i, io->inputs.raw[i], &io->inputs.debounced[i], &s_debounce_count[i]);
    }
}

/**
 * @brief Get single digital input state (debounced)
 */
bool FPGA_IO_GetInput(const fpga_io_t* io, uint8_t channel)
{
    if (io == NULL || !FPGA_IO_IsValidChannel(channel)) {
        return false;
    }

    return io->inputs.debounced[channel];
}

/**
 * @brief Get single digital input state (raw, no debounce)
 */
bool FPGA_IO_GetInputRaw(const fpga_io_t* io, uint8_t channel)
{
    if (io == NULL || !FPGA_IO_IsValidChannel(channel)) {
        return false;
    }

    return io->inputs.raw[channel];
}

/* ============================================================================
 * Public Function Implementations - Digital Outputs
 * ============================================================================ */

/**
 * @brief Write all digital outputs to FPGA
 */
void FPGA_IO_WriteOutputs(fpga_io_t* io)
{
    if (io == NULL) {
        return;
    }

    // Write 3 groups of 16 outputs each
    FPGA_IO_Write16Outputs(DO_REG0_ADDR, &io->outputs.state[0]);   // DO0-DO15
    FPGA_IO_Write16Outputs(DO_REG1_ADDR, &io->outputs.state[16]);  // DO16-DO31
    FPGA_IO_Write16Outputs(DO_REG2_ADDR, &io->outputs.state[32]);  // DO32-DO47

    io->outputs.update_count++;
}

/**
 * @brief Set single digital output
 */
void FPGA_IO_SetOutput(fpga_io_t* io, uint8_t channel, bool state)
{
    if (io == NULL || !FPGA_IO_IsValidChannel(channel)) {
        return;
    }

    io->outputs.state[channel] = state;
}

/**
 * @brief Get single digital output state
 */
bool FPGA_IO_GetOutput(const fpga_io_t* io, uint8_t channel)
{
    if (io == NULL || !FPGA_IO_IsValidChannel(channel)) {
        return false;
    }

    return io->outputs.state[channel];
}

/**
 * @brief Toggle single digital output
 */
void FPGA_IO_ToggleOutput(fpga_io_t* io, uint8_t channel)
{
    if (io == NULL || !FPGA_IO_IsValidChannel(channel)) {
        return;
    }

    io->outputs.state[channel] = !io->outputs.state[channel];
}

/**
 * @brief Set multiple outputs at once
 */
void FPGA_IO_SetOutputs(fpga_io_t* io, uint8_t start_channel, uint8_t count, const bool* states)
{
    if (io == NULL || states == NULL || !FPGA_IO_IsValidChannel(start_channel)) {
        return;
    }

    // Limit count to available channels
    if (start_channel + count > FPGA_NUM_DO) {
        count = FPGA_NUM_DO - start_channel;
    }

    // Copy states
    for (uint8_t i = 0; i < count; i++) {
        io->outputs.state[start_channel + i] = states[i];
    }
}

/* ============================================================================
 * Public Function Implementations - LED Control
 * ============================================================================ */

/**
 * @brief Write LED states to FPGA
 */
void FPGA_IO_WriteLEDs(fpga_io_t* io)
{
    if (io == NULL) {
        return;
    }

    // Pack LED states into bool array for Write16Outputs
    bool led_array[16] = {0};
    led_array[0] = io->leds.run;
    led_array[1] = io->leds.fault;
    led_array[2] = io->leds.alarm;

    // Write to LED register
    FPGA_IO_Write16Outputs(LED_REG_ADDR, led_array);
}

/**
 * @brief Set run LED state
 */
void FPGA_IO_SetRunLED(fpga_io_t* io, bool state)
{
    if (io == NULL) {
        return;
    }

    io->leds.run = state;
}

/**
 * @brief Set fault LED state
 */
void FPGA_IO_SetFaultLED(fpga_io_t* io, bool state)
{
    if (io == NULL) {
        return;
    }

    io->leds.fault = state;
}

/**
 * @brief Set alarm LED state
 */
void FPGA_IO_SetAlarmLED(fpga_io_t* io, bool state)
{
    if (io == NULL) {
        return;
    }

    io->leds.alarm = state;
}

/* ============================================================================
 * Public Function Implementations - Combined Operations
 * ============================================================================ */

/**
 * @brief Update all I/O (read inputs, update outputs, write LEDs)
 */
void FPGA_IO_Update(fpga_io_t* io)
{
    if (io == NULL) {
        return;
    }

    // Read all inputs
    FPGA_IO_ReadInputs(io);

    // Apply debounce filter
    FPGA_IO_UpdateDebounce(io);

    // Write all outputs
    FPGA_IO_WriteOutputs(io);

    // Write LEDs
    FPGA_IO_WriteLEDs(io);

    // Clear control register (as per documentation)
    *(__IO uint16_t*)CTRL_REG_ADDR = 0;

    // Small delay for FPGA to process
    for (volatile int i = 0; i < 6; i++) {
        __NOP();
    }
}

/* ============================================================================
 * Private Function Implementations
 * ============================================================================ */

/**
 * @brief Read 16 digital inputs from FPGA register
 * @param reg_addr FSMC register address
 * @param output_array Pointer to 16-element bool array to store results
 */
static void FPGA_IO_Read16Inputs(uint32_t reg_addr, bool* output_array)
{
    // Read 16-bit register from FPGA via FSMC
    uint16_t data = *(__IO uint16_t*)reg_addr;

    // Unpack 16 bits into individual bool values
    for (int i = 0; i < 16; i++) {
        output_array[i] = (data & 0x0001) ? true : false;
        data >>= 1;
    }
}

/**
 * @brief Write 16 digital outputs to FPGA register
 * @param reg_addr FSMC register address
 * @param input_array Pointer to 16-element bool array containing states
 */
static void FPGA_IO_Write16Outputs(uint32_t reg_addr, const bool* input_array)
{
    uint16_t data = 0;

    // Pack 16 individual bool values into a 16-bit word
    // Process in reverse order to build word MSB-first
    for (int i = 0; i < 16; i++) {
        data >>= 1;
        if (input_array[i]) {
            data |= 0x8000;
        }
    }

    // Write to FPGA via FSMC
    *(__IO uint16_t*)reg_addr = data;

    // Small delay for FPGA to process (6 NOPs ≈ 35ns @ 168MHz)
    for (volatile int i = 0; i < 6; i++) {
        __NOP();
    }
}

/**
 * @brief Debounce filter for single input
 * @param channel Channel number (for debugging)
 * @param current_state Current input state (raw)
 * @param debounced_state Pointer to debounced state to update
 * @param delay_counter Pointer to delay counter for this input
 */
static void FPGA_IO_DebounceInput(uint8_t channel, bool current_state, bool* debounced_state, int* delay_counter)
{
    (void)channel;  // Unused parameter

    if (*debounced_state != current_state) {
        // State differs - increment delay counter
        (*delay_counter)++;

        if (*delay_counter >= FPGA_DEBOUNCE_CYCLES) {
            // Delay threshold reached - accept new state
            *debounced_state = current_state;
            *delay_counter = 0;
        }
    }
    else {
        // State matches - reset delay counter
        *delay_counter = 0;
    }
}
