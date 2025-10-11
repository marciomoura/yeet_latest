/**
 ******************************************************************************
 * @file    w5200_reg.h
 * @brief   WIZnet W5200 Register Definitions and Constants
 * @note    Ported from legacy SPL implementation to HAL for STM32F407ZG
 ******************************************************************************
 * @attention
 *
 * Hardware Configuration:
 * - W5200 #1 (SPI2): 192.168.2.100+NodeID, Port 7000, TCP Server, No DMA
 * - W5200 #2 (SPI3): 192.168.10.31+NodeID, Port 7000, TCP Server, 8K DMA
 *
 * SPI2 Pins: PB12(NSS), PB13(SCK), PB14(MISO), PB15(MOSI), PA11(CS), PA12(RST)
 * SPI3 Pins: PA15(NSS), PC10(SCK), PC11(MISO), PC12(MOSI), PB3(CS), PB4(RST)
 *
 ******************************************************************************
 */

#ifndef W5200_REG_H
#define W5200_REG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ==================== W5200 Memory Map Constants ==================== */

#define W5200_COMMON_BASE     0x0000  ///< Common register base address
#define W5200_CH_BASE         0x4000  ///< Socket register base address
#define W5200_CH_SIZE         0x0100  ///< Socket register size (256 bytes)
#define W5200_TX_MEM          0x8000  ///< TX memory base address
#define W5200_RX_MEM          0xC000  ///< RX memory base address
#define W5200_SOCKET_BUF_SIZE 8192    ///< Socket buffer size (8KB per socket)

/* ==================== DMA Buffer Size Definitions ==================== */

#define W5200_DMA_SIZE_128 128
#define W5200_DMA_SIZE_256 256
#define W5200_DMA_SIZE_512 512
#define W5200_DMA_SIZE_1K  1024
#define W5200_DMA_SIZE_2K  2048
#define W5200_DMA_SIZE_4K  4096
#define W5200_DMA_SIZE_8K  8192

/* ==================== Common Register Addresses ==================== */

#define W5200_MR    (W5200_COMMON_BASE + 0x0000)  ///< Mode Register
#define W5200_GAR0  (W5200_COMMON_BASE + 0x0001)  ///< Gateway Address Register
#define W5200_SUBR0 (W5200_COMMON_BASE + 0x0005)  ///< Subnet Mask Register
#define W5200_SHAR0 (W5200_COMMON_BASE + 0x0009)  ///< Source Hardware Address (MAC)
#define W5200_SIPR0 (W5200_COMMON_BASE + 0x000F)  ///< Source IP Address
#define W5200_RTR0  (W5200_COMMON_BASE + 0x0017)  ///< Retry Time Register
#define W5200_RCR   (W5200_COMMON_BASE + 0x0019)  ///< Retry Count Register
#define W5200_IMR   (W5200_COMMON_BASE + 0x0036)  ///< Interrupt Mask Register
#define W5200_IR2   (W5200_COMMON_BASE + 0x0034)  ///< Interrupt Register

/* ==================== Socket Register Macros ==================== */

/// Socket Mode Register
#define W5200_Sn_MR(ch) (W5200_CH_BASE + (ch) * W5200_CH_SIZE + 0x0000)
/// Socket Command Register
#define W5200_Sn_CR(ch) (W5200_CH_BASE + (ch) * W5200_CH_SIZE + 0x0001)
/// Socket Interrupt Register
#define W5200_Sn_IR(ch) (W5200_CH_BASE + (ch) * W5200_CH_SIZE + 0x0002)
/// Socket Status Register
#define W5200_Sn_SR(ch) (W5200_CH_BASE + (ch) * W5200_CH_SIZE + 0x0003)
/// Socket Source Port Register
#define W5200_Sn_PORT0(ch) (W5200_CH_BASE + (ch) * W5200_CH_SIZE + 0x0004)
/// Socket Destination IP Address Register
#define W5200_Sn_DIPR0(ch) (W5200_CH_BASE + (ch) * W5200_CH_SIZE + 0x000C)
/// Socket Destination Port Register
#define W5200_Sn_DPORT0(ch) (W5200_CH_BASE + (ch) * W5200_CH_SIZE + 0x0010)
/// Socket Maximum Segment Size Register
#define W5200_Sn_MSSR0(ch) (W5200_CH_BASE + (ch) * W5200_CH_SIZE + 0x0012)
/// Socket RX Memory Size Register
#define W5200_Sn_RXMEM_SIZE(ch) (W5200_CH_BASE + (ch) * W5200_CH_SIZE + 0x001E)
/// Socket TX Memory Size Register
#define W5200_Sn_TXMEM_SIZE(ch) (W5200_CH_BASE + (ch) * W5200_CH_SIZE + 0x001F)
/// Socket TX Free Size Register
#define W5200_Sn_TX_FSR0(ch) (W5200_CH_BASE + (ch) * W5200_CH_SIZE + 0x0020)
/// Socket TX Write Pointer Register
#define W5200_Sn_TX_WR0(ch) (W5200_CH_BASE + (ch) * W5200_CH_SIZE + 0x0024)
/// Socket RX Received Size Register
#define W5200_Sn_RX_RSR0(ch) (W5200_CH_BASE + (ch) * W5200_CH_SIZE + 0x0026)
/// Socket RX Read Pointer Register
#define W5200_Sn_RX_RD0(ch) (W5200_CH_BASE + (ch) * W5200_CH_SIZE + 0x0028)

/* ==================== Socket Mode Register Values ==================== */

#define W5200_Sn_MR_CLOSED 0x00  ///< Closed
#define W5200_Sn_MR_TCP    0x01  ///< TCP mode
#define W5200_Sn_MR_UDP    0x02  ///< UDP mode
#define W5200_Sn_MR_IPRAW  0x03  ///< IP RAW mode
#define W5200_Sn_MR_MACRAW 0x04  ///< MAC RAW mode
#define W5200_Sn_MR_PPPOE  0x05  ///< PPPoE mode
#define W5200_Sn_MR_ND     0x20  ///< No Delayed ACK
#define W5200_Sn_MR_MULTI  0x80  ///< Multicast version

/* ==================== Socket Command Register Values ==================== */

#define W5200_Sn_CR_OPEN      0x01  ///< Initialize socket
#define W5200_Sn_CR_LISTEN    0x02  ///< TCP server mode
#define W5200_Sn_CR_CONNECT   0x04  ///< TCP client mode - connect
#define W5200_Sn_CR_DISCON    0x08  ///< Disconnect
#define W5200_Sn_CR_CLOSE     0x10  ///< Close socket
#define W5200_Sn_CR_SEND      0x20  ///< Send data
#define W5200_Sn_CR_SEND_MAC  0x21  ///< Send MAC raw packet
#define W5200_Sn_CR_SEND_KEEP 0x22  ///< Send keep-alive packet
#define W5200_Sn_CR_RECV      0x40  ///< Receive data

/* ==================== Socket Status Register Values ==================== */

#define W5200_SOCK_CLOSED      0x00  ///< Socket closed
#define W5200_SOCK_INIT        0x13  ///< Socket initialized (TCP)
#define W5200_SOCK_LISTEN      0x14  ///< Socket listening (TCP server)
#define W5200_SOCK_ESTABLISHED 0x17  ///< Connection established (TCP)
#define W5200_SOCK_CLOSE_WAIT  0x1C  ///< Peer requested close (TCP)
#define W5200_SOCK_UDP         0x22  ///< Socket opened (UDP)
#define W5200_SOCK_IPRAW       0x32  ///< Socket opened (IP RAW)
#define W5200_SOCK_MACRAW      0x42  ///< Socket opened (MAC RAW)
#define W5200_SOCK_PPPOE       0x5F  ///< Socket opened (PPPoE)
#define W5200_SOCK_SYNSENT     0x15  ///< SYN sent (TCP)
#define W5200_SOCK_SYNRECV     0x16  ///< SYN received (TCP)
#define W5200_SOCK_FIN_WAIT    0x18  ///< FIN wait (TCP)
#define W5200_SOCK_CLOSING     0x1A  ///< Closing (TCP)
#define W5200_SOCK_TIME_WAIT   0x1B  ///< Time wait (TCP)
#define W5200_SOCK_LAST_ACK    0x1D  ///< Last ACK (TCP)

/* ==================== Socket Interrupt Register Bits ==================== */

#define W5200_Sn_IR_CON     0x01  ///< Connection established
#define W5200_Sn_IR_DISCON  0x02  ///< Disconnected
#define W5200_Sn_IR_RECV    0x04  ///< Data received
#define W5200_Sn_IR_TIMEOUT 0x08  ///< Timeout occurred
#define W5200_Sn_IR_SENDOK  0x10  ///< Send completed

/* ==================== Protocol Definitions ==================== */

#define W5200_PROTO_CLOSED 0x00
#define W5200_PROTO_TCP    0x01
#define W5200_PROTO_UDP    0x02

/* ==================== Operation Status ==================== */

#define W5200_SUCCESS 1
#define W5200_FAILURE 0
#define W5200_TRUE    0xFF
#define W5200_FALSE   0x00

/* ==================== Socket State Flags ==================== */

#define W5200_S_INIT 0x01  ///< Socket initialized
#define W5200_S_CONN 0x02  ///< Socket connected
#define W5200_S_OK   0x04  ///< Socket ready for operation

/* ==================== Network State Flags ==================== */

#define W5200_STATE_IDLE      0x00  ///< Network idle
#define W5200_STATE_CHECKING  0x01  ///< Network checking state
#define W5200_STATE_SENDING   0x02  ///< Network sending data
#define W5200_STATE_RECEIVING 0x03  ///< Network receiving data

/* ==================== Socket Data Flags ==================== */

#define W5200_DATA_RECEIVE 0x01  ///< Data received flag
#define W5200_DATA_TX_OK   0x02  ///< Transmission OK flag

/* ==================== Network Mode Constants ==================== */

#define W5200_MODE_SERVER 1  ///< TCP Server mode
#define W5200_MODE_CLIENT 0  ///< TCP Client mode

/* ==================== SPI Communication Timeout ==================== */

#define W5200_SPI_TIMEOUT 1000  ///< SPI timeout in ms (HAL_MAX_DELAY for blocking)

#ifdef __cplusplus
}
#endif

#endif /* W5200_REG_H */
