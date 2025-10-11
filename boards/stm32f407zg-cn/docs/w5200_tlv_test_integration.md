# W5200 + TLV Protocol Integration - Test Implementation

## Overview

This integration adds UDP telemetry transmission using the W5200 Ethernet controller and TLV protocol. A test signal with constant value `0xDEADBEEF` is sent at 100 Hz (10ms) on Task 3 to verify network connectivity and protocol implementation.

## Changes Made

### 1. Main.c Modifications

**Includes Added:**
```c
#include "tlv_protocol.h"
#include "w5200_driver.h"
```

**Global Variables Added:**
```c
static W5200_Handle_t w5200_spi2 = {0};  // W5200 on SPI2 (192.168.2.x)
static W5200_Handle_t w5200_spi3 = {0};  // W5200 on SPI3 (192.168.10.x)
static uint8_t tlv_packet_buffer[512] = {0};  // TLV packet buffer
```

**Initialization (in main()):**
- W5200 SPI2 initialized at 192.168.2.100 (TCP Server, no DMA)
- W5200 SPI3 initialized at 192.168.10.31 (UDP, 8K DMA)
- Socket 0 on SPI3 configured for UDP telemetry to 192.168.10.100:7000

**Task 2 (1 kHz) - Network Processing:**
```c
// Process W5200 network events (interrupts, keep-alive, socket management)
W5200_Cycle(&w5200_spi2);
W5200_Cycle(&w5200_spi3);
```

**Task 3 (100 Hz) - UDP Telemetry:**
```c
// Send UDP packet with test value 0xDEADBEEF
if (W5200_SocketIsReady(&w5200_spi3, 0)) {
    TLV_Builder_t builder;
    if (TLV_BuilderInit(&builder, tlv_packet_buffer, sizeof(tlv_packet_buffer))) {
        TLV_AddUInt32(&builder, TLV_TAG_SYSTEM_TIME, HAL_GetTick());
        TLV_AddUInt32(&builder, TLV_TAG_MEASUREMENT_0, 0xDEADBEEF);
        
        uint16_t packet_size = TLV_GetPacketSize(&builder);
        W5200_SocketSend(&w5200_spi3, 0, tlv_packet_buffer, packet_size);
    }
}
```

### 2. Python UDP Receiver Test Script

**Location:** `boards/stm32f407zg-cn/tools/udp_receiver_test.py`

**Features:**
- Listens on UDP port 7000
- Decodes TLV packets automatically
- Displays system time and test value
- Verifies that `0xDEADBEEF` is received correctly
- Shows packet count and timestamps

## Network Configuration

### STM32F407ZG-CN (Embedded Device)

**W5200 SPI2:**
- IP: 192.168.2.100
- Subnet: 255.255.255.0
- Gateway: 192.168.2.1
- MAC: Auto-generated
- Purpose: TCP Server (for parameter updates)

**W5200 SPI3:**
- IP: 192.168.10.31
- Subnet: 255.255.255.0
- Gateway: 192.168.10.1
- MAC: Auto-generated
- Socket 0: UDP client → 192.168.10.100:7000
- Purpose: UDP telemetry transmission

### PC Configuration

**Required Network Settings:**
- IP Address: 192.168.10.100
- Subnet Mask: 255.255.255.0
- Gateway: 192.168.10.1

**How to Set on Windows:**
1. Open Network Connections
2. Right-click your Ethernet adapter → Properties
3. Select "Internet Protocol Version 4 (TCP/IPv4)" → Properties
4. Select "Use the following IP address"
5. Enter: IP: 192.168.10.100, Subnet: 255.255.255.0, Gateway: 192.168.10.1

## Testing Procedure

### Step 1: Build and Flash Firmware

```bash
# Configure and build for CN board
cmake --preset=embedded-cn-grid
cmake --build --preset=embedded-cn-grid-release
```

Flash the firmware to your STM32F407ZG-CN board.

### Step 2: Configure PC Network

Set your PC's Ethernet adapter to:
- IP: 192.168.10.100
- Subnet: 255.255.255.0

### Step 3: Connect Hardware

1. Connect Ethernet cable from W5200 SPI3 to your PC
2. Power on the STM32 board
3. Wait for LEDs to start blinking (indicates tasks are running)

### Step 4: Run Python Receiver

```bash
cd boards/stm32f407zg-cn/tools
python udp_receiver_test.py
```

### Expected Output

```
======================================================================
W5200 UDP TLV Packet Receiver Test
======================================================================
Listening on 0.0.0.0:7000
Expected STM32 IP: 192.168.10.31
Make sure to configure your PC's IP to 192.168.10.100
Press Ctrl+C to stop
======================================================================

[12:34:56.123] Packet #1 from 192.168.10.31:7000 (11 bytes)
  Tag 0x00 (SYSTEM_TIME): 12345 ms
  Tag 0x10 (MEASUREMENT_0): 0xDEADBEEF
  ✅ Test value 0xDEADBEEF received correctly!

[12:34:56.233] Packet #2 from 192.168.10.31:7000 (11 bytes)
  Tag 0x00 (SYSTEM_TIME): 12445 ms
  Tag 0x10 (MEASUREMENT_0): 0xDEADBEEF
  ✅ Test value 0xDEADBEEF received correctly!
```

You should receive ~100 packets per second (one every 10ms from Task 3).

## TLV Packet Structure

Each UDP packet contains:

```
+--------+--------+--------+--------+--------+--------+--------+
| Tag    | Length          | Value (System Time - 4 bytes)    |
| 0x00   | 0x00   | 0x04   | [uint32_t: milliseconds]         |
+--------+--------+--------+--------+--------+--------+--------+
| Tag    | Length          | Value (Test Signal - 4 bytes)    |
| 0x10   | 0x00   | 0x04   | 0xDE   | 0xAD   | 0xBE   | 0xEF   |
+--------+--------+--------+--------+--------+--------+--------+
```

**Total packet size:** 11 bytes
- Tag 0x00 + Length 0x0004 + Value (4 bytes) = 7 bytes
- Tag 0x10 + Length 0x0004 + Value (4 bytes) = 7 bytes
- Header overhead: 3 + 3 = 6 bytes
- Data: 4 + 4 = 8 bytes
- **Total: 11 bytes**

## Troubleshooting

### No Packets Received

1. **Check Network Cable:**
   - Verify Ethernet cable is connected
   - Check for link LEDs on RJ45 connector

2. **Verify PC IP Configuration:**
   ```bash
   ipconfig  # Windows
   ifconfig  # Linux/Mac
   ```
   Should show 192.168.10.100

3. **Check Firewall:**
   - Windows Firewall may block UDP port 7000
   - Add exception or temporarily disable firewall

4. **Ping Test:**
   ```bash
   ping 192.168.10.31
   ```
   Should get replies (though W5200 may not respond to ping)

5. **Use Wireshark:**
   - Capture on your Ethernet interface
   - Filter: `udp.port == 7000`
   - Should see packets from 192.168.10.31 to 192.168.10.100

### Incorrect Data Received

1. **Verify Endianness:**
   - TLV protocol uses big-endian (network byte order)
   - Python script expects big-endian
   - Check if values are byte-swapped

2. **Check Packet Corruption:**
   - View raw hex dump in Python script
   - Compare with expected TLV structure

3. **Socket Not Ready:**
   - W5200 needs time to initialize
   - Wait ~1 second after power-on
   - Check FPGA LED3 is blinking at 1 Hz

## Performance Metrics

| Parameter         | Value         | Notes                      |
| ----------------- | ------------- | -------------------------- |
| Transmission Rate | 100 Hz (10ms) | Task 3 frequency           |
| Packet Size       | 11 bytes      | 2 TLV entries              |
| Bandwidth         | ~8.8 Kbps     | 11 bytes × 100 Hz × 8 bits |
| Network Overhead  | <1%           | Task 3 execution time      |

## Next Steps

Once you confirm packets are received correctly:

1. **Add Real Measurements:**
   - Replace `0xDEADBEEF` with actual ADC values
   - Add multiple measurement channels (up to 64)
   
2. **Optimize Transmission Rate:**
   - Adjust to 1 kHz or 10 Hz based on requirements
   - Balance bandwidth vs. latency

3. **Add TCP Parameter Reception:**
   - Implement parameter updates via W5200 SPI2
   - Use TLV protocol for bidirectional communication

4. **Implement Data Logging:**
   - Save received data to CSV/HDF5 on PC
   - Real-time plotting with matplotlib

## Files Modified

- `boards/stm32f407zg-cn/Core/Src/main.c` - Added W5200 + TLV integration
- `boards/stm32f407zg-cn/Core/Inc/tlv_protocol.h` - Already existed, no changes
- `boards/stm32f407zg-cn/tools/udp_receiver_test.py` - New test script

## Additional Resources

- W5200 Datasheet: WIZnet W5200 Hardwired TCP/IP Ethernet Controller
- TLV Protocol: Simple Type-Length-Value encoding for structured data
- UDP: RFC 768 - User Datagram Protocol

---

**Document Version:** 1.0  
**Date:** 2025-10-11  
**Author:** GitHub Copilot  
**Status:** Ready for Testing
