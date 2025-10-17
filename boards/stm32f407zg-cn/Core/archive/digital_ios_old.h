/***************************************************************************************
 * DIDO.H - Digital I/O Status Monitoring Interface
 ***************************************************************************************
 * Description: Hardware status monitoring via digital I/O
 *
 * - System running/fault/alarm LEDs
 ***************************************************************************************/

#ifndef __DIDO_H
#define __DIDO_H

/**
 * @brief Main digital I/O update function
 *
 * Performs complete DI/DO cycle:
 * 1. Reads digital inputs from hardware
 *
 * Call this function periodically (e.g., every 10ms) to maintain I/O state.
 */
void DIDO(void);

#endif /* __DIDO_H */

//===========================================================================
// End of file
//===========================================================================
