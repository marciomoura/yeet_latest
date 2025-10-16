/***************************************************************************************
 * GPIO.H - Digital I/O Driver Interface
 ***************************************************************************************
 * Company:     Three Gorges Nengshida Electric Co., Ltd
 * Version:     NSD_STM32F4_Driver_2.0 (Cleaned)
 * Date:        2013-08-22 (Updated 2025-10-16)
 * Description: Digital input/output driver for FSMC-based GPIO expansion hardware
 *
 * OVERVIEW:
 * ---------
 * Provides 64 digital inputs and 64 digital outputs via FSMC interface to external
 * GPIO hardware at Bank1_SRAM4 (0x6C000000). Includes debounce filtering for inputs.
 *
 * GLOBAL VARIABLES:
 * -----------------
 * Digital Inputs (1=High, 0=Low):
 *   DI_Flag[8][8]          - Raw input state (real-time, updated every scan)
 *   DI_Delayed_Flag[8][8]  - Debounced input state (2-cycle delay filter)
 *
 *   Bit Mapping: Row 0 = DI00..DI07, Row 1 = DI08..DI15, ..., Row 7 = DI56..DI63
 *
 * Digital Outputs (1=High, 0=Low):
 *   DO_Flag[8][8]          - Output state buffer (written by DO_Out())
 *
 *   Bit Mapping: Row 0 = DO00..DO07, Row 1 = DO08..DO15, ..., Row 7 = DO56..DO63
 *
 * LED Indicators:
 *   DO_LED_Run             - Running indicator (1=Blinking, 0=Off)
 *   DO_LED_Fault           - Fault indicator (1=On, 0=Off)
 *   DO_LED_alarm           - Alarm indicator (1=On, 0=Off)
 *
 * USAGE EXAMPLE:
 * --------------
 * // Initialization (call once at startup)
 * Init_GPIO();
 *
 * // Reading inputs (in periodic task, e.g., 10ms timer)
 * Refresh_DI();           // Update DI_Flag from hardware
 * Refresh_Delayed_DI();   // Apply debounce filter
 * if (DI_Delayed_Flag[4][7]) {
 *     // React to debounced input DI47
 * }
 *
 * // Writing outputs
 * DO_Flag[0][0] = fault_active;    // Set individual outputs
 * DO_LED_Run = 1;                  // Enable running LED
 * DO_Out();                        // Write all outputs to hardware
 ***************************************************************************************/

#ifndef __GPIO_H
#define __GPIO_H

//=======================================================================================
// Digital I/O State Variables
//=======================================================================================

/** Raw digital input state (real-time, no filtering) */
extern unsigned char DI_Flag[8][8];

/** Debounced digital input state (2-cycle delay filter applied) */
extern unsigned char DI_Delayed_Flag[8][8];

/** Digital output state buffer */
extern unsigned char DO_Flag[8][8];

/** LED indicator states */
extern unsigned char DO_LED_Run;   /**< Running indicator (1=Blinking, 0=Off) */
extern unsigned char DO_LED_Fault; /**< Fault indicator (1=On, 0=Off) */
extern unsigned char DO_LED_alarm; /**< Alarm indicator (1=On, 0=Off) */

//=======================================================================================
// Function Prototypes
//=======================================================================================

/**
 * @brief Initialize digital I/O hardware and data structures
 *
 * Configures FSMC interface for GPIO expansion hardware, initializes all DI/DO
 * buffers to zero, and performs initial read of digital inputs.
 * Call this function once during system initialization.
 */
void Init_GPIO(void);

/**
 * @brief Read digital inputs from hardware
 *
 * Updates DI_Flag array with current state of all digital inputs.
 * This function performs direct hardware read without filtering.
 * Call periodically (e.g., every 10ms) to keep input state current.
 */
void Refresh_DI(void);

/**
 * @brief Apply debounce filter to digital inputs
 *
 * Updates DI_Delayed_Flag array with filtered input state. Uses 2-cycle
 * delay to eliminate switch bounce. Input must be stable for 2 consecutive
 * reads before DI_Delayed_Flag is updated.
 * Call after Refresh_DI() in each scan cycle.
 */
void Refresh_Delayed_DI(void);

/**
 * @brief Write digital outputs to hardware
 *
 * Writes DO_Flag array and LED indicators (DO_LED_Run, DO_LED_Fault,
 * DO_LED_alarm) to hardware GPIO. Application should set desired output
 * states in DO_Flag before calling this function.
 */
void DO_Out(void);

/**
 * @brief Refresh both inputs and outputs (combined operation)
 *
 * Convenience function that calls Refresh_DI(), Refresh_Delayed_DI(),
 * and DO_Out() in sequence. Suitable for simple periodic I/O update.
 */
void GPIO(void);

#endif /* __GPIO_H */

//===========================================================================
// End of file
//===========================================================================
