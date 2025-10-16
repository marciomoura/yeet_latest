/**
 ******************************************************************************
 * @file    digital_ios.c
 * @brief   Digital I/O status monitoring driver implementation
 * @date    2025
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
 * External Dependencies (from legacy modules for backward compatibility)
 * ============================================================================ */

// External breaker check function (defined elsewhere)
extern void Breaker_Check(void);

// External status variables (from pub.h and other modules for legacy compatibility)
extern unsigned char DI_Data[8];
extern unsigned char DO_Data[8];
extern unsigned char Arm_Fuse_Flag[6];
extern unsigned char Fuse_Flag;
extern unsigned char Over_Temperature_Flag;
extern unsigned char Fan_Flag;
extern unsigned char Ref_Up_Flag;
extern unsigned char Ref_Down_Flag;
extern unsigned char Stop_Command;
extern unsigned char Pulse_Enabled_Flag;
extern unsigned char Breaker_Flag;
extern unsigned char Pulse_Power_Flag;

extern unsigned char SCR_Error_Flag;
extern unsigned char Fan_On_Flag;
extern unsigned char Sec_A_Flag, Sec_B_Flag, Sec_C_Flag;
extern unsigned char Syn_A_Err_Flag, Syn_B_Err_Flag, Syn_C_Err_Flag;
extern unsigned char ROM_Flag;
extern unsigned char Scr_Exit_Flag;
extern unsigned char Low_F_Flag;
extern unsigned char Over_Ex_Flag;
extern unsigned char PT_Break_Flag;
extern unsigned char Standby_Flag;
extern unsigned char Online_Flag;
extern unsigned char Running_Flag;
extern unsigned short int Running_Count;

extern short int DO_Lock_Delay;
extern unsigned char DO_Lock_Flag;

extern unsigned short int Optical_Comm_Status[8];
extern unsigned char Online_Source[8];
extern unsigned char SynSource;

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
static void DIO_UpdateOutputLock(dio_instance_t* dio);
static void DIO_SyncToExternalFlags(dio_instance_t* dio);
static void DIO_SyncFromExternalFlags(dio_instance_t* dio);

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
    dio->do_lock_delay = 0;
    dio->do_lock_flag = false;

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
    dio->do_lock_delay = 0;
    dio->do_lock_flag = false;
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

    // Sync to external flag variables for legacy code compatibility
    DIO_SyncToExternalFlags(dio);
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
    DIO_UpdateOutputLock(dio);

    // Sync from external flag variables for legacy code compatibility
    DIO_SyncFromExternalFlags(dio);

    // Map output flags to data bytes
    DIO_MapOutputByte1(dio);
    DIO_MapOutputByte2(dio);
    DIO_MapOutputByte3(dio);
    DIO_MapOutputByte4(dio);
    DIO_MapOutputByte5(dio);
    DIO_MapOutputByte6(dio);
    dio->output_data[6] = SynSource;  // Direct assignment
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

/**
 * @brief Set output lock
 */
void DIO_SetOutputLock(dio_instance_t* dio, int16_t delay_cycles)
{
    if (dio == NULL) {
        return;
    }

    dio->do_lock_delay = delay_cycles;
    dio->do_lock_flag = (delay_cycles > 0);
}

/* ============================================================================
 * Private Function Implementations - Input Processing
 * ============================================================================ */

/**
 * @brief Process digital input byte 1
 */
static void DIO_ProcessInputByte1(dio_instance_t* dio)
{
    if (dio == NULL || dio->gpio_driver == NULL) {
        return;
    }

    // Combine all 6 arm fuse flags
    dio->inputs.fuse_flag =
        Arm_Fuse_Flag[0] | Arm_Fuse_Flag[1] | Arm_Fuse_Flag[2] | Arm_Fuse_Flag[3] | Arm_Fuse_Flag[4] | Arm_Fuse_Flag[5];

    // Copy individual arm flags
    for (int i = 0; i < 6; i++) {
        dio->inputs.arm_fuse_flag[i] = Arm_Fuse_Flag[i];
    }

    // Temperature status (inverted logic: 1=OK, 0=Fault)
    dio->inputs.over_temperature_flag = (GET_DI_DEBOUNCED(dio, 4, 1) == 0);

    // Direct mapped inputs
    dio->inputs.fan_flag = GET_DI_DEBOUNCED(dio, 4, 2);
    dio->inputs.ref_up_flag = (1 - GET_DI_DEBOUNCED(dio, 4, 5));
    dio->inputs.ref_down_flag = (1 - GET_DI_DEBOUNCED(dio, 4, 6));
    dio->inputs.stop_command = (1 - GET_DI_DEBOUNCED(dio, 4, 7));

    // Pack into communication byte
    uint8_t di_temp = dio->inputs.fuse_flag;
    for (int i = 1; i < 8; i++) {
        if (GET_DI_DEBOUNCED(dio, 4, i)) {
            di_temp |= (1 << i);
        }
    }
    dio->input_data[0] = di_temp;
    DI_Data[0] = di_temp;
}

/**
 * @brief Process digital input byte 2
 */
static void DIO_ProcessInputByte2(dio_instance_t* dio)
{
    if (dio == NULL || dio->gpio_driver == NULL) {
        return;
    }

    // Breaker status determined by power calculation
    Breaker_Check();
    dio->inputs.breaker_flag = Breaker_Flag;

    // Pulse power supply fault
    dio->inputs.pulse_power_flag = GET_DI_DEBOUNCED(dio, 5, 2);

    // Pulse enabled flag
    dio->inputs.pulse_enabled_flag = Pulse_Enabled_Flag;

    // Pack into communication byte
    uint8_t di_temp = dio->inputs.pulse_enabled_flag;
    for (int i = 1; i < 8; i++) {
        if (GET_DI_DEBOUNCED(dio, 5, i)) {
            di_temp |= (1 << i);
        }
    }
    dio->input_data[1] = di_temp;
    DI_Data[1] = di_temp;
}

/* ============================================================================
 * Private Function Implementations - Output Mapping
 * ============================================================================ */

/**
 * @brief Map status flags to output byte 1
 */
static void DIO_MapOutputByte1(dio_instance_t* dio)
{
    if (dio == NULL || dio->gpio_driver == NULL) {
        return;
    }

    uint8_t do_temp = dio->outputs.scr_error_flag;

    if (dio->outputs.fuse_flag) do_temp |= 0x02;
    if (dio->outputs.fan_on_flag) do_temp |= 0x04;
    if (dio->outputs.sec_a_flag || dio->outputs.syn_a_err_flag) do_temp |= 0x08;
    if (dio->outputs.sec_b_flag || dio->outputs.syn_b_err_flag) do_temp |= 0x10;
    if (dio->outputs.sec_c_flag || dio->outputs.syn_c_err_flag) do_temp |= 0x20;

    dio->output_data[0] = do_temp;
    DO_Data[0] = do_temp;

    // Map to hardware GPIO driver outputs
    SET_DO(dio, 0, 0, dio->outputs.scr_error_flag);
    SET_DO(dio, 0, 1, dio->outputs.fuse_flag);
    SET_DO(dio, 0, 2, dio->outputs.fan_on_flag);
    SET_DO(dio, 0, 3, dio->outputs.sec_a_flag || dio->outputs.syn_a_err_flag);
    SET_DO(dio, 0, 4, dio->outputs.sec_b_flag || dio->outputs.syn_b_err_flag);
    SET_DO(dio, 0, 5, dio->outputs.sec_c_flag || dio->outputs.syn_c_err_flag);
    SET_DO(dio, 0, 6, 0);
    SET_DO(dio, 0, 7, 0);

    // Copy to extended bank
    for (int i = 0; i < 8; i++) {
        uint8_t val = FSMC_GPIO_GetOutput(dio->gpio_driver, 0, i);
        SET_DO(dio, 4, i, val);
    }
}

/**
 * @brief Map status flags to output byte 2
 */
static void DIO_MapOutputByte2(dio_instance_t* dio)
{
    if (dio == NULL || dio->gpio_driver == NULL) {
        return;
    }

    uint8_t do_temp = dio->outputs.rom_flag;

    if (Arm_Fuse_Flag[5]) do_temp |= 0x02;
    if (Arm_Fuse_Flag[4]) do_temp |= 0x04;
    if (Arm_Fuse_Flag[3]) do_temp |= 0x08;
    if (Arm_Fuse_Flag[2]) do_temp |= 0x10;
    if (Arm_Fuse_Flag[1]) do_temp |= 0x20;
    if (Arm_Fuse_Flag[0]) do_temp |= 0x40;
    if (dio->outputs.scr_exit_flag) do_temp |= 0x80;

    dio->output_data[1] = do_temp;
    DO_Data[1] = do_temp;

    // Map to hardware GPIO driver
    SET_DO(dio, 1, 0, dio->outputs.rom_flag);
    SET_DO(dio, 1, 1, Arm_Fuse_Flag[5]);
    SET_DO(dio, 1, 2, Arm_Fuse_Flag[4]);
    SET_DO(dio, 1, 3, Arm_Fuse_Flag[3]);
    SET_DO(dio, 1, 4, Arm_Fuse_Flag[2]);
    SET_DO(dio, 1, 5, Arm_Fuse_Flag[1]);
    SET_DO(dio, 1, 6, Arm_Fuse_Flag[0]);
    SET_DO(dio, 1, 7, dio->outputs.scr_exit_flag);
}

/**
 * @brief Map status flags to output byte 3
 */
static void DIO_MapOutputByte3(dio_instance_t* dio)
{
    if (dio == NULL || dio->gpio_driver == NULL) {
        return;
    }

    uint8_t do_temp = Optical_Comm_Status[0];
    do_temp |= (Optical_Comm_Status[1] << 2);

    if (dio->outputs.low_f_flag) do_temp |= 0x10;
    if (dio->outputs.over_ex_flag) do_temp |= 0x20;
    if (dio->outputs.pt_break_flag) do_temp |= 0x40;
    if (dio->outputs.standby_flag) do_temp |= 0x80;

    dio->output_data[2] = do_temp;
    DO_Data[2] = do_temp;

    // Map to hardware GPIO driver
    SET_DO(dio, 2, 0, Optical_Comm_Status[0] & 1);
    SET_DO(dio, 2, 1, Optical_Comm_Status[0] >> 1);
    SET_DO(dio, 2, 2, Optical_Comm_Status[1] & 1);
    SET_DO(dio, 2, 3, Optical_Comm_Status[1] >> 1);
    SET_DO(dio, 2, 4, dio->outputs.low_f_flag);
    SET_DO(dio, 2, 5, dio->outputs.over_ex_flag);
    SET_DO(dio, 2, 6, dio->outputs.pt_break_flag);
    SET_DO(dio, 2, 7, dio->outputs.standby_flag);
}

/**
 * @brief Map status flags to output byte 4
 */
static void DIO_MapOutputByte4(dio_instance_t* dio)
{
    if (dio == NULL) {
        return;
    }

    uint8_t do_temp = 0;
    int j = 1;

    if (Online_Flag) {
        for (int i = 0; i < 8; i++) {
            if (Online_Source[i]) {
                do_temp = j;
                break;
            }
            j <<= 1;
        }
    }
    else {
        do_temp = 0;
    }

    dio->output_data[3] = do_temp;
    DO_Data[3] = do_temp;
}

/**
 * @brief Map status flags to output byte 5
 */
static void DIO_MapOutputByte5(dio_instance_t* dio)
{
    if (dio == NULL) {
        return;
    }

    uint8_t do_temp = Optical_Comm_Status[2];
    do_temp |= (Optical_Comm_Status[3] << 2);
    do_temp |= (Optical_Comm_Status[4] << 4);
    do_temp |= (Optical_Comm_Status[5] << 6);

    dio->output_data[4] = do_temp;
    DO_Data[4] = do_temp;
}

/**
 * @brief Map status flags to output byte 6
 */
static void DIO_MapOutputByte6(dio_instance_t* dio)
{
    if (dio == NULL) {
        return;
    }

    uint8_t do_temp = Optical_Comm_Status[6];
    do_temp |= (Optical_Comm_Status[7] << 2);

    dio->output_data[5] = do_temp;
    DO_Data[5] = do_temp;
}

/**
 * @brief Map status flags to output byte 8
 */
static void DIO_MapOutputByte8(dio_instance_t* dio)
{
    if (dio == NULL) {
        return;
    }

    uint8_t do_temp = 0;
    int j = 1;

    for (int i = 0; i < 6; i++) {
        j <<= 1;
        if (Arm_Fuse_Flag[i]) {
            do_temp |= j;
        }
    }

    dio->output_data[7] = do_temp;
    DO_Data[7] = do_temp;
}

/* ============================================================================
 * Private Function Implementations - Status Updates
 * ============================================================================ */

/**
 * @brief Update running indicator LED
 */
static void DIO_UpdateRunningIndicator(dio_instance_t* dio)
{
    if (dio == NULL) {
        return;
    }

    if (++dio->running_count >= DIO_RUNNING_TOGGLE_CYCLES) {
        dio->running_count = 0;
        dio->outputs.running_flag = !dio->outputs.running_flag;
    }
}

/**
 * @brief Update digital output lock status
 */
static void DIO_UpdateOutputLock(dio_instance_t* dio)
{
    if (dio == NULL) {
        return;
    }

    if (dio->do_lock_delay > 0) {
        dio->do_lock_delay--;
        if (dio->do_lock_delay <= 0) {
            dio->do_lock_delay = 0;
            dio->do_lock_flag = false;
        }
    }
}

/* ============================================================================
 * Private Function Implementations - Legacy Compatibility
 * ============================================================================ */

/**
 * @brief Sync driver state TO external flag variables
 */
static void DIO_SyncToExternalFlags(dio_instance_t* dio)
{
    if (dio == NULL) {
        return;
    }

    // Input flags
    Fuse_Flag = dio->inputs.fuse_flag;
    Over_Temperature_Flag = dio->inputs.over_temperature_flag;
    Fan_Flag = dio->inputs.fan_flag;
    Ref_Up_Flag = dio->inputs.ref_up_flag;
    Ref_Down_Flag = dio->inputs.ref_down_flag;
    Stop_Command = dio->inputs.stop_command;
    Breaker_Flag = dio->inputs.breaker_flag;
    Pulse_Power_Flag = dio->inputs.pulse_power_flag;
}

/**
 * @brief Sync driver state FROM external flag variables
 */
static void DIO_SyncFromExternalFlags(dio_instance_t* dio)
{
    if (dio == NULL) {
        return;
    }

    // Output flags
    dio->outputs.scr_error_flag = SCR_Error_Flag;
    dio->outputs.fuse_flag = Fuse_Flag;
    dio->outputs.fan_on_flag = Fan_On_Flag;
    dio->outputs.sec_a_flag = Sec_A_Flag;
    dio->outputs.sec_b_flag = Sec_B_Flag;
    dio->outputs.sec_c_flag = Sec_C_Flag;
    dio->outputs.syn_a_err_flag = Syn_A_Err_Flag;
    dio->outputs.syn_b_err_flag = Syn_B_Err_Flag;
    dio->outputs.syn_c_err_flag = Syn_C_Err_Flag;
    dio->outputs.rom_flag = ROM_Flag;
    dio->outputs.scr_exit_flag = Scr_Exit_Flag;
    dio->outputs.low_f_flag = Low_F_Flag;
    dio->outputs.over_ex_flag = Over_Ex_Flag;
    dio->outputs.pt_break_flag = PT_Break_Flag;
    dio->outputs.standby_flag = Standby_Flag;
    dio->outputs.online_flag = Online_Flag;

    // Status tracking
    Running_Flag = dio->outputs.running_flag;
    Running_Count = dio->running_count;
    DO_Lock_Flag = dio->do_lock_flag;
    DO_Lock_Delay = dio->do_lock_delay;
}

/* ============================================================================
 * Legacy Function Implementations (preserved for backward compatibility)
 * ============================================================================ */

/**
 * @brief Legacy DIDO update function
 * @note Preserved for backward compatibility with existing code
 */
void DIDO(void)
{
    static fsmc_gpio_instance_t legacy_gpio = {.initialized = false};
    static dio_instance_t legacy_dio = {.initialized = false};

    // Initialize on first call
    if (!legacy_dio.initialized) {
        FSMC_GPIO_Init(&legacy_gpio);
        DIO_Init(&legacy_dio, &legacy_gpio);
    }

    // Perform update cycle
    DIO_Update(&legacy_dio);
}
