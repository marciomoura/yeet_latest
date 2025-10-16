/********************************************************************
 *pub.h - Minimal Hardware Status Variables                          *
 *                                                                    *
 *This file contains only hardware-level status variables needed     *
 *by DIDO.c for system status monitoring (LEDs, temperature, fuses,  *
 *communication status, etc.)                                        *
 *                                                                    *
 *Application developers should define their own control variables   *
 *in their application code, not here.                               *
 ********************************************************************/

#ifndef __PUB_H
#define __PUB_H

// Hardware status monitoring variables (used by DIDO.c)
extern unsigned char DI_Data[8];  // Digital input data bytes
extern unsigned char DO_Data[8];  // Digital output data bytes

extern unsigned int Running_Count;  // Running LED blink counter (10ms tick)
extern unsigned int Running_Flag;   // Running LED state

extern unsigned char DO_Lock_Flag;        // Output lock flag
extern unsigned short int DO_Lock_Delay;  // Output lock delay counter

extern unsigned char SCR_Error_Flag;         // Thyristor fault flag
extern unsigned char Fuse_Flag;              // Fuse fault flag
extern unsigned char Arm_Fuse_Flag[6];       // Individual arm fuse flags
extern unsigned char Over_Temperature_Flag;  // Over temperature flag
extern unsigned char Fan_Flag;               // Fan control flag
extern unsigned char Fan_On_Flag;            // Fan output state

extern unsigned char Pulse_Enabled_Flag;  // Pulse enable state
extern unsigned char Pulse_Power_Flag;    // Pulse power supply fault
extern unsigned char Low_F_Flag;          // Low frequency flag
extern int Low_F_Count;                   // Low frequency counter
extern unsigned char Breaker_Flag;        // Circuit breaker state

extern unsigned char Ref_Up_Flag;    // Reference up command
extern unsigned char Ref_Down_Flag;  // Reference down command
extern unsigned char Stop_Command;   // Stop command input

extern unsigned char Sec_A_Flag;      // Phase A sync secondary fault
extern unsigned char Sec_B_Flag;      // Phase B sync secondary fault
extern unsigned char Sec_C_Flag;      // Phase C sync secondary fault
extern unsigned char Syn_A_Err_Flag;  // Phase A sync error
extern unsigned char Syn_B_Err_Flag;  // Phase B sync error
extern unsigned char Syn_C_Err_Flag;  // Phase C sync error

extern unsigned char ROM_Flag;       // Flash memory status
extern unsigned char Scr_Exit_Flag;  // Thyristor exit flag

extern unsigned char AVR_Flag;       // AVR mode flag
extern unsigned char FCR_Flag;       // FCR mode flag
extern unsigned char AER_Flag;       // AER mode flag
extern unsigned char PT_Break_Flag;  // PT断线 (PT disconnection)
extern unsigned char Standby_Flag;   // Standby mode flag
extern unsigned char Over_Ex_Flag;   // Over excitation flag

extern unsigned char Online_Flag;  // Online communication flag
extern unsigned char uchNodeNum;   // Node number

extern float Vabc;  // Three-phase voltage (for SCR check)
extern float P, Q;  // Active/Reactive power (for breaker check)

#endif
//===========================================================================
// End of file.
//===========================================================================
