/**
 ******************************************************************************
 * @file    fpga.h
 * @author  Techperts Team
 * @version V1.0.0
 * @date    17-Oct-2025
 * @brief   FPGA Configuration Driver Header
 ******************************************************************************
 * @attention
 *
 * This module handles FPGA initialization and configuration for the
 * STM32F407ZG-CN board. The FPGA must be configured before it can
 * handle optical fiber communication and FSMC peripherals.
 *
 * Hardware Connections:
 * - PG15: PROGRAM_B (FPGA configuration trigger, active low)
 * - PD2:  DONE (FPGA configuration complete indicator, active high)
 * - PG11: Internal FPGA reset
 * - PE3:  INIT_B (optional, not currently used)
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FPGA_H
#define __FPGA_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/**
 * @brief FPGA Configuration Status
 */
typedef enum {
    FPGA_CONFIG_SUCCESS = 0,  ///< FPGA configured successfully
    FPGA_CONFIG_TIMEOUT = 1   ///< FPGA configuration timeout/failure
} FPGA_Status_t;

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  Initialize FPGA internal reset pin (PG11)
 * @note   Must be called before FPGA_Reset()
 */
void FPGA_RST_Init(void);

/**
 * @brief  Initialize FPGA INIT_B input pin (PE3)
 * @note   Optional - currently not used in configuration logic
 */
void FPGA_INITB_Init(void);

/**
 * @brief  Initialize FPGA DONE input pin (PD2)
 * @note   Must be called before FPGA_Configure()
 *         DONE signal goes HIGH when FPGA configuration is complete
 */
void FPGA_DONE_Init(void);

/**
 * @brief  Initialize FPGA PROGRAM_B output pin (PG15)
 * @note   Must be called before FPGA_Configure()
 *         Pulsing this pin LOW triggers FPGA reconfiguration
 */
void FPGA_PROGRAMB_Init(void);

/**
 * @brief  Initialize all FPGA control pins
 * @note   Convenience function that calls all pin init functions
 */
void FPGA_Init_Pins(void);

/**
 * @brief  Configure FPGA and wait for DONE signal with timeout protection
 * @param  timeout_ms: Maximum time to wait for FPGA configuration (milliseconds)
 * @retval FPGA_CONFIG_SUCCESS: FPGA configured successfully
 *         FPGA_CONFIG_TIMEOUT: FPGA failed to configure within timeout
 * @note   Requires TIMx_Init() to be called first for timer-based delays
 *         This function will retry FPGA reset every 100ms until timeout
 */
FPGA_Status_t FPGA_Configure(uint32_t timeout_ms);

/**
 * @brief  Perform internal FPGA reset sequence
 * @note   Must be called after successful FPGA_Configure()
 *         Uses PG11 pin to reset internal FPGA logic
 */
void FPGA_Reset(void);

/**
 * @brief  Check if FPGA configuration is complete
 * @retval 1: FPGA is configured (DONE pin is HIGH)
 *         0: FPGA is not configured (DONE pin is LOW)
 */
uint8_t FPGA_Is_Configured(void);

#ifdef __cplusplus
}
#endif

#endif /* __FPGA_H */

/******************************************************************************
 * END OF FILE
 ******************************************************************************/
