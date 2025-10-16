/***************************************************************************************
 * DIDO.C - Digital I/O Status Monitoring Implementation
 ***************************************************************************************
 * Description: Maps hardware status flags to digital I/O for monitoring and control
 * 
 * This module provides hardware-level status monitoring including:
 * - SCR (thyristor) fault detection
 * - Fuse status monitoring (6 arms)
 * - Circuit breaker state detection
 * - Temperature monitoring
 * - Optical fiber communication status
 * - System running/fault/alarm indication
 * 
 * The module reads digital inputs and maps internal status flags to digital outputs
 * for external monitoring equipment.
 ***************************************************************************************/

#include <math.h>
#include <stm32f4xx.h>
#include "GPIO.H"
#include "pub.h"

//=======================================================================================
// External Function Declarations
//=======================================================================================

/** Set firing angle delay (from synchronization module) */
extern void Set_Alpha_Delay(void);

/** Initialize synchronization pulse generation */
extern void Syn_Pulse_Init(void);

//=======================================================================================
// External Status Variables
//=======================================================================================

/** Optical fiber communication status for each channel */
extern unsigned short int Optical_Comm_Status[8];  // 0=Disconnected, 1=TX timeout, 3=RX timeout

/** Optical communication error flag */
extern unsigned char Optical_Comm_Err_Flag;

/** Online communication lost flag */
extern unsigned char Online_Comm_Err_Flag;

/** Online module source flags */
extern unsigned char Online_Source[8];

/** Selected synchronization source */
extern unsigned char SynSource;

/** Optical transmission flag */
extern unsigned char Optical_Tx_Flag;

//=======================================================================================
// Hardware Status Monitoring Functions
//=======================================================================================

/**
 * @brief Update running indicator LED
 * 
 * Toggles running indicator every 100 cycles (typically 1 second @ 10ms rate).
 * Provides visual heartbeat indication that system is operational.
 */
static void Running_Signal(void)
{
    if (++Running_Count >= 100) {
        Running_Count = 0;
        Running_Flag = ~Running_Flag;
    }
}

/**
 * @brief Check thyristor (SCR) status
 * 
 * Evaluates SCR health based on multiple conditions:
 * - Output lock status
 * - AC voltage presence
 * - Fuse status
 * - Pulse enable state
 * - Low frequency condition
 * - Temperature status
 * - Communication errors
 * - Pulse power supply status
 * 
 * Sets SCR_Error_Flag if any fault condition is detected.
 */
static void SCR_Check(void)
{
    // Check for SCR fault conditions:
    // 1. Output not locked AND
    // 2. AC voltage present (> 0.1V) AND  
    // 3. Any fault condition active
    if ((DO_Lock_Flag == 0 && Vabc > 0.1f) &&
        ((Fuse_Flag || (Pulse_Enabled_Flag && Low_F_Flag) || 
          Over_Temperature_Flag || Optical_Comm_Err_Flag ||
          Online_Comm_Err_Flag) || Pulse_Power_Flag))
    {
        SCR_Error_Flag = 1;
    }
    else {
        SCR_Error_Flag = 0;
    }
}

/**
 * @brief Check circuit breaker status
 * 
 * Determines breaker state based on apparent power calculation.
 * Uses hysteresis to prevent chattering:
 * - Closes (sets flag) when S > 0.11 (110VA)
 * - Opens (clears flag) when S < 0.09 (90VA)
 */
static void Breaker_Check(void)
{
    double S;

    // Calculate apparent power: S = sqrt(P^2 + Q^2)
    S = sqrt(P * P + Q * Q);

    // Hysteresis comparison
    if (S > 0.11) {
        Breaker_Flag = 1;  // Breaker closed
    }
    if (S < 0.09) {
        Breaker_Flag = 0;  // Breaker open
    }
}

/**
 * @brief Check digital output lock status
 * 
 * Manages DO_Lock_Delay countdown timer. When delay expires,
 * clears DO_Lock_Flag to enable normal output operation.
 */
static void DO_Lock_Check(void)
{
    if (--DO_Lock_Delay <= 0) {
        DO_Lock_Delay = 0;
        DO_Lock_Flag = 0;
    }
}

/**
 * @brief Trigger optical transmission during low frequency
 * 
 * During low frequency condition, periodically triggers optical
 * fiber transmission and updates firing angle if pulse is not enabled.
 * 
 * NOTE: Currently commented out in original code - placeholder for future use.
 */
static void Low_F_Trigger(void)
{
    // Low frequency trigger logic currently disabled
    // Uncomment and modify as needed for your application:
    
    // if (Low_F_Flag) {
    //     if (++Low_F_Trig_Delay >= 40) {
    //         Low_F_Trig_Delay = 0;
    //         if (Pulse_Low_F_Flag) {
    //             Optical_Tx_Flag = 1;
    //             if (Pulse_Enabled_Flag == 0) {
    //                 Set_Alpha_Delay();  // Update firing angle
    //             }
    //         }
    //     }
    // }
    // else {
    //     Low_F_Trig_Delay = 0;
    // }
}

/**
 * @brief Check for low frequency condition
 * 
 * Monitors grid frequency via synchronization interrupts.
 * If no sync pulse received for 200 cycles (~2 seconds), declares
 * low frequency condition and reinitializes sync pulse system.
 */
void Low_F_Check(void)
{
    if (++Low_F_Count >= 200) {
        Low_F_Count = 200;  // Saturate counter
        if (Low_F_Flag == 0) {
            // First detection of low frequency - reinit sync
            Syn_Pulse_Init();
        }
    }
    else {
        Low_F_Flag = 0;  // Frequency OK
    }
    
    Low_F_Trigger();
}

/**
 * @brief Refresh all status flags
 * 
 * Calls all status checking functions to update system health flags.
 * Called before mapping flags to digital outputs.
 */
static void Refresh_DO_Flag(void)
{
    Running_Signal();
    DO_Lock_Check();
    SCR_Check();
    Low_F_Check();
}

//=======================================================================================
// Digital Input Processing Functions
//=======================================================================================

/**
 * @brief Process digital input byte 1
 * 
 * Maps debounced input byte 1 (DI_Delayed_Flag[4][x]) to internal flags:
 * - Bit 0: Combined fuse status (OR of all 6 arm fuses)
 * - Bit 1: Over temperature (inverted - 1=OK, 0=Fault)
 * - Bit 2: Fan status
 * - Bit 5: Reference up command (inverted)
 * - Bit 6: Reference down command (inverted)
 * - Bit 7: Stop command (inverted)
 * 
 * Packs processed flags into DI_Data[0] for communication.
 */
static void Get_DI1(void)
{
    unsigned char DI_temp;
    int i;

    // Combine all 6 arm fuse flags
    Fuse_Flag = Arm_Fuse_Flag[0] | Arm_Fuse_Flag[1] | Arm_Fuse_Flag[2] |
                Arm_Fuse_Flag[3] | Arm_Fuse_Flag[4] | Arm_Fuse_Flag[5];

    // Temperature status (inverted logic: 1=OK, 0=Fault)
    if (DI_Delayed_Flag[4][1]) {
        Over_Temperature_Flag = 0;  // Temperature OK
    }
    else {
        Over_Temperature_Flag = 1;  // Over temperature fault
    }

    // Direct mapped inputs
    Fan_Flag      = DI_Delayed_Flag[4][2];
    Ref_Up_Flag   = 1 - DI_Delayed_Flag[4][5];  // Inverted
    Ref_Down_Flag = 1 - DI_Delayed_Flag[4][6];  // Inverted
    Stop_Command  = 1 - DI_Delayed_Flag[4][7];  // Inverted

    // Pack into communication byte
    DI_temp = Fuse_Flag;
    for (i = 1; i < 8; i++) {
        if (DI_Delayed_Flag[4][i]) {
            DI_temp |= (1 << i);
        }
    }
    DI_Data[0] = DI_temp;
}

/**
 * @brief Process digital input byte 2
 * 
 * Maps debounced input byte 2 (DI_Delayed_Flag[5][x]) to internal flags:
 * - Bit 0: Pulse enable state
 * - Bit 1: Breaker status (calculated, not direct input)
 * - Bit 2: Pulse power supply fault
 * 
 * Packs processed flags into DI_Data[1] for communication.
 */
static void Get_DI2(void)
{
    unsigned char DI_temp;
    int i;

    // Breaker status determined by power calculation, not direct input
    Breaker_Check();

    // Pulse power supply fault
    Pulse_Power_Flag = DI_Delayed_Flag[5][2];

    // Pack into communication byte
    DI_temp = Pulse_Enabled_Flag;
    for (i = 1; i < 8; i++) {
        if (DI_Delayed_Flag[5][i]) {
            DI_temp |= (1 << i);
        }
    }
    DI_Data[1] = DI_temp;
}

/**
 * @brief Read and process all digital inputs
 * 
 * Complete input processing sequence:
 * 1. Read raw inputs from hardware (Refresh_DI)
 * 2. Apply debounce filter (Refresh_Delayed_DI)
 * 3. Process input bytes and update flags (Get_DI1, Get_DI2)
 */
static void DI(void)
{
    Refresh_DI();
    Refresh_Delayed_DI();
    Get_DI1();
    Get_DI2();
}

//=======================================================================================
// Digital Output Mapping Functions
//=======================================================================================

/**
 * @brief Map status flags to output byte 1
 * 
 * Output byte 1 (DO_Data[0] / DO_Flag[0][x]):
 * - Bit 0: SCR error flag
 * - Bit 1: Fuse fault
 * - Bit 2: Cooling fan on
 * - Bit 3: Phase A sync secondary fault OR sync error
 * - Bit 4: Phase B sync secondary fault OR sync error
 * - Bit 5: Phase C sync secondary fault OR sync error
 * - Bits 6-7: Reserved
 * 
 * Also copies to extended output bank (DO_Flag[4][x]).
 */
static void DO1(void)
{
    unsigned char DO_temp;

    DO_temp = SCR_Error_Flag;

    if (Fuse_Flag) {
        DO_temp |= 0x02;  // Bit 1
    }

    Fan_On_Flag = Pulse_Enabled_Flag;
    if (Fan_On_Flag) {
        DO_temp |= 0x04;  // Bit 2
    }

    if (Sec_A_Flag || Syn_A_Err_Flag) {
        DO_temp |= 0x08;  // Bit 3
    }

    if (Sec_B_Flag || Syn_B_Err_Flag) {
        DO_temp |= 0x10;  // Bit 4
    }

    if (Sec_C_Flag || Syn_C_Err_Flag) {
        DO_temp |= 0x20;  // Bit 5
    }

    DO_Data[0] = DO_temp;

    // Map to DO_Flag for hardware output
    DO_Flag[0][0] = SCR_Error_Flag;
    DO_Flag[0][1] = Fuse_Flag;
    DO_Flag[0][2] = Fan_On_Flag;
    DO_Flag[0][3] = Sec_A_Flag || Syn_A_Err_Flag;
    DO_Flag[0][4] = Sec_B_Flag || Syn_B_Err_Flag;
    DO_Flag[0][5] = Sec_C_Flag || Syn_C_Err_Flag;
    DO_Flag[0][6] = 0;
    DO_Flag[0][7] = 0;

    // Copy to extended bank
    for (int i = 0; i < 8; i++) {
        DO_Flag[4][i] = DO_Flag[0][i];
    }
}

/**
 * @brief Map status flags to output byte 2
 * 
 * Output byte 2 (DO_Data[1] / DO_Flag[1][x]):
 * - Bit 0: ROM (Flash memory) status
 * - Bits 1-6: Individual arm fuse flags (arms 5,4,3,2,1,0)
 * - Bit 7: SCR exit flag
 */
static void DO2(void)
{
    unsigned char DO_temp;

    DO_temp = ROM_Flag;

    if (Arm_Fuse_Flag[5]) DO_temp |= 0x02;  // Bit 1
    if (Arm_Fuse_Flag[4]) DO_temp |= 0x04;  // Bit 2
    if (Arm_Fuse_Flag[3]) DO_temp |= 0x08;  // Bit 3
    if (Arm_Fuse_Flag[2]) DO_temp |= 0x10;  // Bit 4
    if (Arm_Fuse_Flag[1]) DO_temp |= 0x20;  // Bit 5
    if (Arm_Fuse_Flag[0]) DO_temp |= 0x40;  // Bit 6
    if (Scr_Exit_Flag)    DO_temp |= 0x80;  // Bit 7

    DO_Data[1] = DO_temp;

    // Map to DO_Flag
    DO_Flag[1][0] = ROM_Flag;
    DO_Flag[1][1] = Arm_Fuse_Flag[5];
    DO_Flag[1][2] = Arm_Fuse_Flag[4];
    DO_Flag[1][3] = Arm_Fuse_Flag[3];
    DO_Flag[1][4] = Arm_Fuse_Flag[2];
    DO_Flag[1][5] = Arm_Fuse_Flag[1];
    DO_Flag[1][6] = Arm_Fuse_Flag[0];
    DO_Flag[1][7] = Scr_Exit_Flag;
}

/**
 * @brief Map status flags to output byte 3
 * 
 * Output byte 3 (DO_Data[2] / DO_Flag[2][x]):
 * - Bits 0-1: Optical channel 1 status (TX/RX timeout)
 * - Bits 2-3: Optical channel 2 status
 * - Bit 4: Low frequency flag
 * - Bit 5: Over excitation flag
 * - Bit 6: PT (potential transformer) disconnection
 * - Bit 7: Standby mode flag
 */
static void DO3(void)
{
    unsigned char DO_temp;

    DO_temp = Optical_Comm_Status[0];
    DO_temp |= (Optical_Comm_Status[1] << 2);

    if (Low_F_Flag)      DO_temp |= 0x10;  // Bit 4
    if (Over_Ex_Flag)    DO_temp |= 0x20;  // Bit 5
    if (PT_Break_Flag)   DO_temp |= 0x40;  // Bit 6
    if (Standby_Flag)    DO_temp |= 0x80;  // Bit 7

    DO_Data[2] = DO_temp;

    // Map to DO_Flag
    DO_Flag[2][0] = Optical_Comm_Status[0] & 1;
    DO_Flag[2][1] = Optical_Comm_Status[0] >> 1;
    DO_Flag[2][2] = Optical_Comm_Status[1] & 1;
    DO_Flag[2][3] = Optical_Comm_Status[1] >> 1;
    DO_Flag[2][4] = Low_F_Flag;
}

/**
 * @brief Map status flags to output byte 4
 * 
 * Output byte 4 (DO_Data[3]):
 * Encodes online communication source selection (1-of-n encoding).
 * Bit position indicates which channel is active.
 */
static void DO4(void)
{
    unsigned char DO_temp = 0;
    int i, j;

    j = 1;
    if (Online_Flag) {
        // Use configured node number
        j = 1 << (uchNodeNum + 1);
    }
    else {
        // Find first active online source
        for (i = 0; i < 6; i++) {
            if (Online_Source[i]) {
                break;
            }
            j <<= 1;
        }
        if (i == 6) {
            j = 0;  // No source active
        }
    }
    DO_temp = j;

    DO_Data[3] = DO_temp;
}

/**
 * @brief Map status flags to output byte 5
 * 
 * Output byte 5 (DO_Data[4]):
 * - Bits 0-1: Optical SCR channel 1 status
 * - Bits 2-3: Optical SCR channel 2 status
 * - Bits 4-5: Optical SCR channel 3 status
 * - Bits 6-7: Optical SCR channel 4 status
 */
static void DO5(void)
{
    unsigned char DO_temp = 0;

    DO_temp  = Optical_Comm_Status[2];
    DO_temp |= (Optical_Comm_Status[3] << 2);
    DO_temp |= (Optical_Comm_Status[4] << 4);
    DO_temp |= (Optical_Comm_Status[5] << 6);

    DO_Data[4] = DO_temp;
}

/**
 * @brief Map status flags to output byte 6
 * 
 * Output byte 6 (DO_Data[5]):
 * - Bits 0-1: Optical SCR channel 5 status
 * - Bit 2: AVR mode flag
 * - Bit 3: FCR mode flag
 * - Bit 4: AER mode flag
 * - Bits 5-7: Reserved
 */
static void DO6(void)
{
    unsigned char DO_temp = 0;

    DO_temp = Optical_Comm_Status[6];
    if (AVR_Flag) DO_temp |= 0x04;  // Bit 2
    if (FCR_Flag) DO_temp |= 0x08;  // Bit 3
    if (AER_Flag) DO_temp |= 0x10;  // Bit 4

    DO_Data[5] = DO_temp;
}

/**
 * @brief Map status flags to output byte 8
 * 
 * Output byte 8 (DO_Data[7]):
 * Individual arm fuse status (bits 0-5 for arms 0-5).
 */
static void DO8(void)
{
    unsigned char DO_temp = 0;
    int i, j;

    DO_temp = Arm_Fuse_Flag[0];

    j = 1;
    for (i = 1; i < 6; i++) {
        j <<= 1;
        if (Arm_Fuse_Flag[i]) {
            DO_temp |= j;
        }
    }
    DO_Data[7] = DO_temp;
}

/**
 * @brief Map all status flags to digital outputs and write to hardware
 * 
 * Complete output processing sequence:
 * 1. Refresh all status flags
 * 2. Map flags to output bytes (DO1-DO8)
 * 3. Write outputs to hardware
 */
static void DO(void)
{
    Refresh_DO_Flag();
    DO1();
    DO2();
    DO3();
    DO4();
    DO5();
    DO6();
    DO8();
    DO_Data[6] = SynSource;
    DO_Out();
}

//=======================================================================================
// Main DIDO Update Function
//=======================================================================================

/**
 * @brief Complete digital I/O update cycle
 * 
 * Performs full input/output processing:
 * - Reads digital inputs
 * - Updates internal status flags
 * - Maps flags to outputs
 * - Writes to hardware
 * 
 * Call this function periodically (e.g., every 10ms) from main loop or timer ISR.
 */
void DIDO(void)
{
    DI();
    DO();
}

//===========================================================================
// End of file
//===========================================================================
