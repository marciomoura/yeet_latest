/**
 * @file pub.h
 * @brief Stub header for legacy network code dependencies
 *
 * This file provides minimal declarations/stubs for variables
 * referenced by the legacy network code. Variables are either
 * replaced with dummy values or linked to existing driver state.
 */
#ifndef PUB_H
#define PUUB_H

#include <stdint.h>

/* External variables used by Net_Check() - provided as stubs/dummies */

/* Power/reactive values (not used in this refactored version) */
extern float P, Q;      // Active/reactive power from network
extern float Pav, Qav;  // Averaged values

/* Network communication arrays */
extern unsigned char ucQTx[12];             // TX buffer for Q data
extern unsigned char ucMornitor[256][256];  // Monitor data array (stubbed large)

/* Network state tracking */
extern int Net_Monitor_C;        // Monitor counter
extern int Net_Monitor_Index;    // Monitor index
extern int ms_C;                 // Millisecond counter
extern int Monitor_Upload_Flag;  // Upload flag

/* Node ID is provided by driver_init.c */
extern uint8_t uchNodeNum;

/* Byte to float conversion stub (if data processing is needed later) */
void Byte_To_Float(float* data, unsigned char* b);

#endif /* PUB_H */
