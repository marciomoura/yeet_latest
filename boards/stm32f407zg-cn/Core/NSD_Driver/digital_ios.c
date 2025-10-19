/**
 ******************************************************************************
 * @file    digital_ios.c
 * @brief   Generic Digital I/O Driver Implementation
 * @date    2025
 ******************************************************************************
 * @attention
 *
 * This driver provides generic digital I/O signal handling.
 * Hardware GPIO mapping:
 *
 * Inputs (16 signals):
 *   digital_input_1..7   -> Row 3, Columns 0-6
 *   digital_input_8      -> Row 4, Column 1
 *   digital_input_9..15  -> Row 4, Columns 2-7
 *   digital_input_16     -> Row 5, Column 0
 *
 * Outputs (17 signals):
 *   digital_output_1..8  -> Byte 1 (Row 0, bits 0-7)
 *   digital_output_9..16 -> Byte 2 (Row 1, bits 0-7)
 *   digital_output_17    -> Running indicator (auto-toggled)
 *
 ******************************************************************************
 */

#include "digital_ios.h"

#include <string.h>

/* ============================================================================
 * Helper Macros for GPIO Driver Access
 * ============================================================================ */

/** @brief Get debounced input from GPIO driver */
#define GET_DI_DEBOUNCED(dio, row, col) FSMC_GPIO_GetDebouncedInput((dio)->gpio_driver, (row), (col))

/** @brief Set output to GPIO driver */
#define SET_DO(dio, row, col, val) FSMC_GPIO_SetOutput((dio)->gpio_driver, (row), (col), (val))

/* ============================================================================
 * External Dependencies
 * ============================================================================ */

// Note: This module no longer depends on global variables.
// All I/O state is managed through the dio_instance_t structure.

/* ============================================================================
 * Private Function Prototypes
 * ============================================================================ */

static void DIO_ProcessInputByte1(dio_instance_t* dio);
static void DIO_ProcessInputByte2(dio_instance_t* dio);
static void DIO_MapOutputByte1(dio_instance_t* dio);
static void DIO_MapOutputByte2(dio_instance_t* dio);
static void DIO_MapOutputByte3(dio_instance_t* dio);
static void DIO_MapOutputByte4(dio_instance_t* dio);
static void DIO_MapOutputByte5(dio_instance_t* dio);
static void DIO_MapOutputByte6(dio_instance_t* dio);
static void DIO_MapOutputByte8(dio_instance_t* dio);
static void DIO_UpdateRunningIndicator(dio_instance_t* dio);

/* ============================================================================
 * Public Function Implementations - Initialization
 * ============================================================================ */

/**
 * @brief Initialize digital I/O driver instance
 */
bool DIO_Init(dio_instance_t* dio, fsmc_gpio_instance_t* gpio_driver)
{
    if (dio == NULL || gpio_driver == NULL) {
        return false;
    }

    // Clear instance structure
    memset(dio, 0, sizeof(dio_instance_t));

    // Store reference to hardware driver
    dio->gpio_driver = gpio_driver;

    dio->initialized = true;
    dio->update_count = 0;
    dio->running_count = 0;

    return true;
}

/**
 * @brief Reset digital I/O driver state
 */
void DIO_Reset(dio_instance_t* dio)
{
    if (dio == NULL) {
        return;
    }

    // Clear all flags
    memset(&dio->inputs, 0, sizeof(dio_input_flags_t));
    memset(&dio->outputs, 0, sizeof(dio_output_flags_t));
    memset(dio->input_data, 0, sizeof(dio->input_data));
    memset(dio->output_data, 0, sizeof(dio->output_data));

    dio->running_count = 0;
}

/* ============================================================================
 * Public Function Implementations - Input Processing
 * ============================================================================ */

/**
 * @brief Read and process all digital inputs
 */
void DIO_ReadInputs(dio_instance_t* dio)
{
    if (dio == NULL || dio->gpio_driver == NULL) {
        return;
    }

    // Read raw inputs from hardware using driver
    FSMC_GPIO_ReadInputs(dio->gpio_driver);

    // Apply debounce filter using driver
    FSMC_GPIO_ApplyDebounce(dio->gpio_driver);

    // Process input bytes and update flags
    DIO_ProcessInputByte1(dio);
    DIO_ProcessInputByte2(dio);
}

/**
 * @brief Get input flags structure
 */
const dio_input_flags_t* DIO_GetInputFlags(const dio_instance_t* dio)
{
    if (dio == NULL) {
        return NULL;
    }
    return &dio->inputs;
}

/* ============================================================================
 * Public Function Implementations - Output Processing
 * ============================================================================ */

/**
 * @brief Update and write all digital outputs
 */
void DIO_WriteOutputs(dio_instance_t* dio)
{
    if (dio == NULL || dio->gpio_driver == NULL) {
        return;
    }

    // Update status indicators
    DIO_UpdateRunningIndicator(dio);

    // Map output flags to data bytes
    DIO_MapOutputByte1(dio);
    DIO_MapOutputByte2(dio);
    DIO_MapOutputByte3(dio);
    DIO_MapOutputByte4(dio);
    DIO_MapOutputByte5(dio);
    DIO_MapOutputByte6(dio);
    DIO_MapOutputByte8(dio);

    // Write outputs to hardware using driver
    FSMC_GPIO_WriteOutputs(dio->gpio_driver);
}

/**
 * @brief Set output flags structure
 */
void DIO_SetOutputFlags(dio_instance_t* dio, const dio_output_flags_t* outputs)
{
    if (dio == NULL || outputs == NULL) {
        return;
    }

    memcpy(&dio->outputs, outputs, sizeof(dio_output_flags_t));
}

/* ============================================================================
 * Public Function Implementations - Combined Operations
 * ============================================================================ */

/**
 * @brief Complete digital I/O update cycle
 */
void DIO_Update(dio_instance_t* dio)
{
    if (dio == NULL) {
        return;
    }

    DIO_ReadInputs(dio);
    DIO_WriteOutputs(dio);

    dio->update_count++;
}

/* ============================================================================
 * Public Function Implementations - Status and Diagnostics
 * ============================================================================ */

/**
 * @brief Get raw input data bytes
 */
void DIO_GetInputData(const dio_instance_t* dio, uint8_t* buffer)
{
    if (dio == NULL || buffer == NULL) {
        return;
    }

    memcpy(buffer, dio->input_data, DIO_NUM_INPUT_BYTES);
}

/**
 * @brief Get raw output data bytes
 */
void DIO_GetOutputData(const dio_instance_t* dio, uint8_t* buffer)
{
    if (dio == NULL || buffer == NULL) {
        return;
    }

    memcpy(buffer, dio->output_data, DIO_NUM_OUTPUT_BYTES);
}

/* ============================================================================
 * Private Function Implementations - Input Processing
 * ============================================================================ */

/**
 * @brief Process digital input byte 1 (inputs 1-8)
 */
static void DIO_ProcessInputByte1(dio_instance_t* dio)
{
    if (dio == NULL || dio->gpio_driver == NULL) {
        return;
    }

    // Map hardware inputs to generic input signals
    // Inputs 1-7: Row 3, Columns 0-6
    dio->inputs.digital_input_1 = GET_DI_DEBOUNCED(dio, 3, 0);
    dio->inputs.digital_input_2 = GET_DI_DEBOUNCED(dio, 3, 1);
    dio->inputs.digital_input_3 = GET_DI_DEBOUNCED(dio, 3, 2);
    dio->inputs.digital_input_4 = GET_DI_DEBOUNCED(dio, 3, 3);
    dio->inputs.digital_input_5 = GET_DI_DEBOUNCED(dio, 3, 4);
    dio->inputs.digital_input_6 = GET_DI_DEBOUNCED(dio, 3, 5);
    dio->inputs.digital_input_7 = GET_DI_DEBOUNCED(dio, 3, 6);

    // Input 8: Row 4, Column 1 (with inverted logic)
    dio->inputs.digital_input_8 = (GET_DI_DEBOUNCED(dio, 4, 1) == 0);

    // Pack into communication byte 1
    uint8_t di_temp = 0;
    di_temp |= (dio->inputs.digital_input_1 << 0);
    di_temp |= (dio->inputs.digital_input_2 << 1);
    di_temp |= (dio->inputs.digital_input_3 << 2);
    di_temp |= (dio->inputs.digital_input_4 << 3);
    di_temp |= (dio->inputs.digital_input_5 << 4);
    di_temp |= (dio->inputs.digital_input_6 << 5);
    di_temp |= (dio->inputs.digital_input_7 << 6);
    di_temp |= (dio->inputs.digital_input_8 << 7);
    dio->input_data[0] = di_temp;
}

/**
 * @brief Process digital input byte 2 (inputs 9-16)
 */
static void DIO_ProcessInputByte2(dio_instance_t* dio)
{
    if (dio == NULL || dio->gpio_driver == NULL) {
        return;
    }

    // Map hardware inputs to generic input signals
    // Inputs 9-15: Row 4, Columns 2-7 (with inverted logic for some)
    dio->inputs.digital_input_9 = GET_DI_DEBOUNCED(dio, 4, 2);
    dio->inputs.digital_input_10 = GET_DI_DEBOUNCED(dio, 4, 3);
    dio->inputs.digital_input_11 = GET_DI_DEBOUNCED(dio, 4, 4);
    dio->inputs.digital_input_12 = (1 - GET_DI_DEBOUNCED(dio, 4, 5));  // Inverted
    dio->inputs.digital_input_13 = (1 - GET_DI_DEBOUNCED(dio, 4, 6));  // Inverted
    dio->inputs.digital_input_14 = (1 - GET_DI_DEBOUNCED(dio, 4, 7));  // Inverted

    // Input 16: Row 5, Column 0
    dio->inputs.digital_input_15 = GET_DI_DEBOUNCED(dio, 5, 0);
    dio->inputs.digital_input_16 = GET_DI_DEBOUNCED(dio, 5, 1);

    // Pack into communication byte 2
    uint8_t di_temp = 0;
    di_temp |= (dio->inputs.digital_input_9 << 0);
    di_temp |= (dio->inputs.digital_input_10 << 1);
    di_temp |= (dio->inputs.digital_input_11 << 2);
    di_temp |= (dio->inputs.digital_input_12 << 3);
    di_temp |= (dio->inputs.digital_input_13 << 4);
    di_temp |= (dio->inputs.digital_input_14 << 5);
    di_temp |= (dio->inputs.digital_input_15 << 6);
    di_temp |= (dio->inputs.digital_input_16 << 7);
    dio->input_data[1] = di_temp;
}

/* ============================================================================
 * Private Function Implementations - Output Mapping
 * ============================================================================ */

/**
 * @brief Map output flags to byte 1 (outputs 1-8)
 */
static void DIO_MapOutputByte1(dio_instance_t* dio)
{
    if (dio == NULL || dio->gpio_driver == NULL) {
        return;
    }

    // Pack output byte 1
    uint8_t do_temp = 0;
    do_temp |= (dio->outputs.digital_output_1 << 0);
    do_temp |= (dio->outputs.digital_output_2 << 1);
    do_temp |= (dio->outputs.digital_output_3 << 2);
    do_temp |= (dio->outputs.digital_output_4 << 3);
    do_temp |= (dio->outputs.digital_output_5 << 4);
    do_temp |= (dio->outputs.digital_output_6 << 5);
    do_temp |= (dio->outputs.digital_output_7 << 6);
    do_temp |= (dio->outputs.digital_output_8 << 7);
    dio->output_data[0] = do_temp;

    // Map to hardware GPIO driver outputs (Row 0)
    SET_DO(dio, 0, 0, dio->outputs.digital_output_1);
    SET_DO(dio, 0, 1, dio->outputs.digital_output_2);
    SET_DO(dio, 0, 2, dio->outputs.digital_output_3);
    SET_DO(dio, 0, 3, dio->outputs.digital_output_4);
    SET_DO(dio, 0, 4, dio->outputs.digital_output_5);
    SET_DO(dio, 0, 5, dio->outputs.digital_output_6);
    SET_DO(dio, 0, 6, dio->outputs.digital_output_7);
    SET_DO(dio, 0, 7, dio->outputs.digital_output_8);

    // Copy to extended bank (Row 4)
    for (int i = 0; i < 8; i++) {
        uint8_t val = FSMC_GPIO_GetOutput(dio->gpio_driver, 0, i);
        SET_DO(dio, 4, i, val);
    }
}

/**
 * @brief Map output flags to byte 2 (outputs 9-16)
 */
static void DIO_MapOutputByte2(dio_instance_t* dio)
{
    if (dio == NULL || dio->gpio_driver == NULL) {
        return;
    }

    // Pack output byte 2
    uint8_t do_temp = 0;
    do_temp |= (dio->outputs.digital_output_9 << 0);
    do_temp |= (dio->outputs.digital_output_10 << 1);
    do_temp |= (dio->outputs.digital_output_11 << 2);
    do_temp |= (dio->outputs.digital_output_12 << 3);
    do_temp |= (dio->outputs.digital_output_13 << 4);
    do_temp |= (dio->outputs.digital_output_14 << 5);
    do_temp |= (dio->outputs.digital_output_15 << 6);
    do_temp |= (dio->outputs.digital_output_16 << 7);
    dio->output_data[1] = do_temp;

    // Map to hardware GPIO driver (Row 1)
    SET_DO(dio, 1, 0, dio->outputs.digital_output_9);
    SET_DO(dio, 1, 1, dio->outputs.digital_output_10);
    SET_DO(dio, 1, 2, dio->outputs.digital_output_11);
    SET_DO(dio, 1, 3, dio->outputs.digital_output_12);
    SET_DO(dio, 1, 4, dio->outputs.digital_output_13);
    SET_DO(dio, 1, 5, dio->outputs.digital_output_14);
    SET_DO(dio, 1, 6, dio->outputs.digital_output_15);
    SET_DO(dio, 1, 7, dio->outputs.digital_output_16);
}

/**
 * @brief Map status flags to output byte 3
 */
static void DIO_MapOutputByte3(dio_instance_t* dio)
{
    if (dio == NULL || dio->gpio_driver == NULL) {
        return;
    }

    uint8_t do_temp = 0;  // Reserved for future expansion

    dio->output_data[2] = do_temp;

    // Map to hardware GPIO driver (all zeros for now)
    for (int i = 0; i < 8; i++) {
        SET_DO(dio, 2, i, 0);
    }
}

/**
 * @brief Map status flags to output byte 4
 */
static void DIO_MapOutputByte4(dio_instance_t* dio)
{
    if (dio == NULL) {
        return;
    }

    // Reserved for future expansion
    dio->output_data[3] = 0;
}

/**
 * @brief Map status flags to output byte 5
 */
static void DIO_MapOutputByte5(dio_instance_t* dio)
{
    if (dio == NULL) {
        return;
    }

    uint8_t do_temp = 0;  // Optical communication status would go here

    dio->output_data[4] = do_temp;
}

/**
 * @brief Map status flags to output byte 6
 */
static void DIO_MapOutputByte6(dio_instance_t* dio)
{
    if (dio == NULL) {
        return;
    }

    uint8_t do_temp = 0;  // Optical communication status would go here

    dio->output_data[5] = do_temp;
}

/**
 * @brief Map status flags to output byte 8
 */
static void DIO_MapOutputByte8(dio_instance_t* dio)
{
    if (dio == NULL) {
        return;
    }

    // Reserved for future expansion
    dio->output_data[7] = 0;
}

/* ============================================================================
 * Private Function Implementations - Status Updates
 * ============================================================================ */

/**
 * @brief Update running indicator LED (digital_output_17)
 */
static void DIO_UpdateRunningIndicator(dio_instance_t* dio)
{
    if (dio == NULL) {
        return;
    }

    if (++dio->running_count >= DIO_RUNNING_TOGGLE_CYCLES) {
        dio->running_count = 0;
        dio->outputs.digital_output_17 = !dio->outputs.digital_output_17;
    }
}
