/**
 ******************************************************************************
 * @file    fpga_led_driver.c
 * @brief   FPGA-controlled status LED driver implementation
 * @date    2025
 ******************************************************************************
 */

#include "fpga_led_driver.h"

#include "main.h"


/* ============================================================================
 * Private Variables
 * ============================================================================ */

/** @brief Current LED states (cached for read-modify-write operations) */
static fpga_led_state_t s_led_state = {.task1_led = false, .task2_led = false, .task3_led = false};

/* ============================================================================
 * Private Function Prototypes
 * ============================================================================ */

static void FPGA_LED_WriteRegister(uint16_t led_bits);

/* ============================================================================
 * Public Function Implementations
 * ============================================================================ */

/**
 * @brief Initialize FPGA LED driver (turn all LEDs OFF)
 */
void FPGA_LED_Init(void)
{
    // Initialize state to all OFF
    s_led_state.task1_led = false;
    s_led_state.task2_led = false;
    s_led_state.task3_led = false;

    // Write to hardware
    FPGA_LED_WriteRegister(0x0000);
}

/**
 * @brief Set individual LED state
 */
void FPGA_LED_Set(fpga_led_id_t led, bool state)
{
    // Update cached state
    switch (led) {
        case FPGA_LED_TASK1:
            s_led_state.task1_led = state;
            break;
        case FPGA_LED_TASK2:
            s_led_state.task2_led = state;
            break;
        case FPGA_LED_TASK3:
            s_led_state.task3_led = state;
            break;
        default:
            return;  // Invalid LED ID
    }

    // Build 16-bit register value from cached states
    uint16_t led_bits = 0;
    if (s_led_state.task1_led) led_bits |= (1 << 0);
    if (s_led_state.task2_led) led_bits |= (1 << 1);
    if (s_led_state.task3_led) led_bits |= (1 << 2);

    // Write to FPGA register
    FPGA_LED_WriteRegister(led_bits);
}

/**
 * @brief Toggle individual LED
 */
void FPGA_LED_Toggle(fpga_led_id_t led)
{
    bool current_state = false;

    // Read current state
    switch (led) {
        case FPGA_LED_TASK1:
            current_state = s_led_state.task1_led;
            s_led_state.task1_led = !current_state;
            break;
        case FPGA_LED_TASK2:
            current_state = s_led_state.task2_led;
            s_led_state.task2_led = !current_state;
            break;
        case FPGA_LED_TASK3:
            current_state = s_led_state.task3_led;
            s_led_state.task3_led = !current_state;
            break;
        default:
            return;  // Invalid LED ID
    }

    // Build 16-bit register value from cached states
    uint16_t led_bits = 0;
    if (s_led_state.task1_led) led_bits |= (1 << 0);
    if (s_led_state.task2_led) led_bits |= (1 << 1);
    if (s_led_state.task3_led) led_bits |= (1 << 2);

    // Write to FPGA register
    FPGA_LED_WriteRegister(led_bits);
}

/**
 * @brief Set all LEDs at once (most efficient method)
 */
void FPGA_LED_SetAll(const fpga_led_state_t* state)
{
    if (state == NULL) {
        return;
    }

    // Update cached state
    s_led_state.task1_led = state->task1_led;
    s_led_state.task2_led = state->task2_led;
    s_led_state.task3_led = state->task3_led;

    // Build 16-bit register value
    uint16_t led_bits = 0;
    if (s_led_state.task1_led) led_bits |= (1 << 0);
    if (s_led_state.task2_led) led_bits |= (1 << 1);
    if (s_led_state.task3_led) led_bits |= (1 << 2);

    // Write to FPGA register
    FPGA_LED_WriteRegister(led_bits);
}

/**
 * @brief Get current LED state
 */
bool FPGA_LED_GetState(fpga_led_id_t led)
{
    switch (led) {
        case FPGA_LED_TASK1:
            return s_led_state.task1_led;
        case FPGA_LED_TASK2:
            return s_led_state.task2_led;
        case FPGA_LED_TASK3:
            return s_led_state.task3_led;
        default:
            return false;
    }
}

/**
 * @brief Turn all LEDs OFF
 */
void FPGA_LED_AllOff(void)
{
    s_led_state.task1_led = false;
    s_led_state.task2_led = false;
    s_led_state.task3_led = false;

    FPGA_LED_WriteRegister(0x0000);
}

/**
 * @brief Turn all LEDs ON
 */
void FPGA_LED_AllOn(void)
{
    s_led_state.task1_led = true;
    s_led_state.task2_led = true;
    s_led_state.task3_led = true;

    FPGA_LED_WriteRegister(0x0007);  // Bits [2:0] = 0b111
}

/**
 * @brief Get all LED states
 */
void FPGA_LED_GetAll(fpga_led_state_t* state)
{
    if (state != NULL) {
        state->task1_led = s_led_state.task1_led;
        state->task2_led = s_led_state.task2_led;
        state->task3_led = s_led_state.task3_led;
    }
}

/* ============================================================================
 * Private Function Implementations
 * ============================================================================ */

/**
 * @brief Write 16-bit value to FPGA LED register
 * @param led_bits LED bit pattern (bits [2:0] control LEDs 0-2)
 * @note This function performs a direct memory-mapped write via FSMC
 */
static void FPGA_LED_WriteRegister(uint16_t led_bits)
{
    // Direct memory-mapped write to FPGA LED register
    // FSMC Bank4 is already configured and enabled
    *(volatile uint16_t*)FPGA_LED_REG_ADDR = led_bits;

    // Small delay to ensure FPGA registers the write (optional)
    // At 168 MHz, a few NOPs give FPGA time to latch data
    __NOP();
    __NOP();
}
