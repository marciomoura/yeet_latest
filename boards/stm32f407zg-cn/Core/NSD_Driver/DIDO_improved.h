/***************************************************************************************
 * DIDO.H - Digital I/O Status Monitoring Interface
 ***************************************************************************************
 * Description: Hardware status monitoring via digital I/O
 * 
 * This module maps internal hardware status flags to digital outputs for monitoring
 * and reads digital inputs for system control. It provides status indication for:
 * - Thyristor (SCR) faults
 * - Fuse status
 * - Temperature monitoring  
 * - Circuit breaker state
 * - Optical fiber communication status
 * - System running/fault/alarm LEDs
 ***************************************************************************************/

#ifndef __DIDO_H
#define __DIDO_H

/**
 * @brief Main digital I/O update function
 * 
 * Performs complete DI/DO cycle:
 * 1. Reads digital inputs from hardware
 * 2. Updates internal flags based on input states
 * 3. Evaluates system status (SCR, breaker, temperature, etc.)
 * 4. Maps status flags to digital outputs
 * 5. Writes outputs to hardware
 * 
 * Call this function periodically (e.g., every 10ms) to maintain I/O state.
 */
void DIDO(void);

/**
 * @brief Check for low frequency condition
 * 
 * Monitors grid frequency and sets Low_F_Flag if frequency drops below threshold.
 * Updates Low_F_Count counter for frequency loss timing.
 * 
 * Called internally by DIDO() but can be used standalone if needed.
 */
void Low_F_Check(void);

#endif /* __DIDO_H */

//===========================================================================
// End of file
//===========================================================================
