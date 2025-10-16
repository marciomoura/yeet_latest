# Optical Fiber Driver Analysis Report
## Comparison: Legacy vs New Implementation
## Date: 2025

---

## EXECUTIVE SUMMARY

**Overall Status: ⚠️ CRITICAL REGISTER ADDRESS CONFLICTS FOUND**

The new optical fiber driver implementation has **register address mismatches** compared to the legacy working code. While the driver architecture and protocol implementation are sound, the FSMC register addresses do NOT match the legacy implementation, which will prevent communication.

---

## 1. CRITICAL ISSUES FOUND

### 1.1 Transmit Register Addresses ❌ **MISMATCH**

#### Legacy (optical_fiber.c):
```c
// Legacy: TX register = (port + 100) * 2
// For port 0: (0 + 100) * 2 = 200 = 0x00C8
// For port 1: (1 + 100) * 2 = 202 = 0x00CA
void Tx_Buffer(unsigned short int port, unsigned char data[], unsigned short int len)
{
    for(i = 0; i < len; i++)
    {
        *(unsigned short int *) (Bank1_SRAM4_ADDR + (port + 100) * 2) = data[i];
        // 0x6C000000 + (port+100)*2
    }
}
```

**Legacy TX Addresses:**
- Port 0 (Broadcast): `0x6C000000 + 0x00C8 = 0x6C0000C8` ✅
- Port 1: `0x6C000000 + 0x00CA = 0x6C0000CA` ✅
- Port 2: `0x6C000000 + 0x00CC = 0x6C0000CC` ✅
- Port 3: `0x6C000000 + 0x00CE = 0x6C0000CE` ✅
- Port 4: `0x6C000000 + 0x00D0 = 0x6C0000D0` ✅
- Port 5: `0x6C000000 + 0x00D2 = 0x6C0000D2` ✅
- Port 6: `0x6C000000 + 0x00D4 = 0x6C0000D4` ✅
- Port 7: `0x6C000000 + 0x00D6 = 0x6C0000D6` ✅

#### Your Code (optical_fiber_driver.h):
```c
#define FPGA_TX_BROADCAST (FPGA_BASE_ADDR + 0x00C8)  // ✅ CORRECT
#define FPGA_TX_PORT1     (FPGA_BASE_ADDR + 0x00CA)  // ✅ CORRECT
#define FPGA_TX_PORT2     (FPGA_BASE_ADDR + 0x00CC)  // ✅ CORRECT
#define FPGA_TX_PORT3     (FPGA_BASE_ADDR + 0x00CE)  // ✅ CORRECT
#define FPGA_TX_PORT4     (FPGA_BASE_ADDR + 0x00D0)  // ✅ CORRECT
#define FPGA_TX_PORT5     (FPGA_BASE_ADDR + 0x00D2)  // ✅ CORRECT
#define FPGA_TX_PORT6     (FPGA_BASE_ADDR + 0x00D4)  // ✅ CORRECT
#define FPGA_TX_PORT7     (FPGA_BASE_ADDR + 0x00D6)  // ✅ CORRECT
```

**Status: ✅ TRANSMIT ADDRESSES MATCH!**

---

### 1.2 Receive Register Addresses ❌ **CRITICAL MISMATCH**

#### Legacy (optical_fiber.c):
```c
// Legacy RX function reads from port address directly (not port+100)
unsigned short int Rx_Buffer(unsigned short int port, unsigned char data[])
{
    unsigned short int len = 0, i = 0;
    while(((Read_State(port + 100) & 0x0002) == 0) && (i++ < 255))  // Status check
    {
        data[len] = (unsigned char) Read_State(port);  // Data read from 'port'
        len++;
    }
    return len;
}

// Read_State implementation:
unsigned short int Read_State(unsigned short int addr)
{
    ret = *(__IO unsigned short int*) (Bank1_SRAM4_ADDR + addr * 2);
    return ret;
}
```

**Legacy RX Addresses:**
- Port 1 DATA: `0x6C000000 + (1 * 2) = 0x6C000002` ✅
- Port 2 DATA: `0x6C000000 + (2 * 2) = 0x6C000004` ✅
- Port 3 DATA: `0x6C000000 + (3 * 2) = 0x6C000006` ✅
- Port 4 DATA: `0x6C000000 + (4 * 2) = 0x6C000008` ✅
- Port 5 DATA: `0x6C000000 + (5 * 2) = 0x6C00000A` ✅
- Port 6 DATA: `0x6C000000 + (6 * 2) = 0x6C00000C` ✅
- Port 7 DATA: `0x6C000000 + (7 * 2) = 0x6C00000E` ✅

**Legacy STATUS check addresses:**
- Port 1 STATUS: `0x6C000000 + (101 * 2) = 0x6C0000CA` ✅
- Port 2 STATUS: `0x6C000000 + (102 * 2) = 0x6C0000CC` ✅
- Port 3 STATUS: `0x6C000000 + (103 * 2) = 0x6C0000CE` ✅
- Port 4 STATUS: `0x6C000000 + (104 * 2) = 0x6C0000D0` ✅
- Port 5 STATUS: `0x6C000000 + (105 * 2) = 0x6C0000D2` ✅
- Port 6 STATUS: `0x6C000000 + (106 * 2) = 0x6C0000D4` ✅
- Port 7 STATUS: `0x6C000000 + (107 * 2) = 0x6C0000D6` ✅

#### Your Code (optical_fiber_driver.h):
```c
// Receive Registers (Read-Only, Auto-Increment on each read)
#define FPGA_RX_PORT1 (FPGA_BASE_ADDR + 0x0002)  // ✅ Port 1
#define FPGA_RX_PORT2 (FPGA_BASE_ADDR + 0x0004)  // ✅ Port 2
#define FPGA_RX_PORT3 (FPGA_BASE_ADDR + 0x0006)  // ✅ Port 3
#define FPGA_RX_PORT4 (FPGA_BASE_ADDR + 0x0008)  // ✅ Port 4
#define FPGA_RX_PORT5 (FPGA_BASE_ADDR + 0x000A)  // ✅ Port 5
#define FPGA_RX_PORT6 (FPGA_BASE_ADDR + 0x000C)  // ✅ Port 6
#define FPGA_RX_PORT7 (FPGA_BASE_ADDR + 0x000E)  // ✅ Port 7

// Status Registers (Read-Only)
#define FPGA_STATUS_PORT1 (FPGA_BASE_ADDR + 0x00CA)  // ✅ Port 1 status
#define FPGA_STATUS_PORT2 (FPGA_BASE_ADDR + 0x00CC)  // ✅ Port 2 status
#define FPGA_STATUS_PORT3 (FPGA_BASE_ADDR + 0x00CE)  // ✅ Port 3 status
#define FPGA_STATUS_PORT4 (FPGA_BASE_ADDR + 0x00D0)  // ✅ Port 4 status
#define FPGA_STATUS_PORT5 (FPGA_BASE_ADDR + 0x00D2)  // ✅ Port 5 status
#define FPGA_STATUS_PORT6 (FPGA_BASE_ADDR + 0x00D4)  // ✅ Port 6 status
#define FPGA_STATUS_PORT7 (FPGA_BASE_ADDR + 0x00D6)  // ✅ Port 7 status
```

**Status: ✅ RECEIVE ADDRESSES MATCH!**

---

### 1.3 Configuration Register Addresses ✅ **VERIFIED**

#### Legacy (optical_fiber.c):
```c
// Baud rate divisor
BaudRate_Init(BaudRate)
{
    uint32_t data = (1382400 / BaudRate);
    *(unsigned short int *) (Bank1_SRAM4_ADDR + 0x04) = data;
    // Address: 0x6C000000 + 0x04 = 0x6C000004 ✅
}

// Node ID
*(unsigned short int *) (Bank1_SRAM4_ADDR + 16) = (unsigned short int)uchNodeNum;
// Address: 0x6C000000 + 0x10 = 0x6C000010 ✅
```

#### Your Code (optical_fiber_driver.h):
```c
#define FPGA_BAUD_RATE_REG (FPGA_BASE_ADDR + 0x0004)  // ✅ CORRECT
#define FPGA_NODE_ID_REG   (FPGA_BASE_ADDR + 0x0010)  // ✅ CORRECT
```

**Status: ✅ CONFIGURATION ADDRESSES MATCH!**

---

## 2. REGISTER ADDRESS SUMMARY

| Register Type | Port | Legacy Address | Your Address | Status |
| ------------- | ---- | -------------- | ------------ | ------ |
| **Config**    |
| Baud Rate     | -    | 0x6C000004     | 0x6C000004   | ✅      |
| Node ID       | -    | 0x6C000010     | 0x6C000010   | ✅      |
| **Transmit**  |
| TX Broadcast  | 0    | 0x6C0000C8     | 0x6C0000C8   | ✅      |
| TX Port 1     | 1    | 0x6C0000CA     | 0x6C0000CA   | ✅      |
| TX Port 2     | 2    | 0x6C0000CC     | 0x6C0000CC   | ✅      |
| TX Port 3     | 3    | 0x6C0000CE     | 0x6C0000CE   | ✅      |
| TX Port 4     | 4    | 0x6C0000D0     | 0x6C0000D0   | ✅      |
| TX Port 5     | 5    | 0x6C0000D2     | 0x6C0000D2   | ✅      |
| TX Port 6     | 6    | 0x6C0000D4     | 0x6C0000D4   | ✅      |
| TX Port 7     | 7    | 0x6C0000D6     | 0x6C0000D6   | ✅      |
| **Receive**   |
| RX Port 1     | 1    | 0x6C000002     | 0x6C000002   | ✅      |
| RX Port 2     | 2    | 0x6C000004     | 0x6C000004   | ✅      |
| RX Port 3     | 3    | 0x6C000006     | 0x6C000006   | ✅      |
| RX Port 4     | 4    | 0x6C000008     | 0x6C000008   | ✅      |
| RX Port 5     | 5    | 0x6C00000A     | 0x6C00000A   | ✅      |
| RX Port 6     | 6    | 0x6C00000C     | 0x6C00000C   | ✅      |
| RX Port 7     | 7    | 0x6C00000E     | 0x6C00000E   | ✅      |
| **Status**    |
| Status Port 1 | 1    | 0x6C0000CA     | 0x6C0000CA   | ✅      |
| Status Port 2 | 2    | 0x6C0000CC     | 0x6C0000CC   | ✅      |
| Status Port 3 | 3    | 0x6C0000CE     | 0x6C0000CE   | ✅      |
| Status Port 4 | 4    | 0x6C0000D0     | 0x6C0000D0   | ✅      |
| Status Port 5 | 5    | 0x6C0000D2     | 0x6C0000D2   | ✅      |
| Status Port 6 | 6    | 0x6C0000D4     | 0x6C0000D4   | ✅      |
| Status Port 7 | 7    | 0x6C0000D6     | 0x6C0000D6   | ✅      |

**✅ ALL REGISTER ADDRESSES MATCH PERFECTLY!**

---

## 3. PROTOCOL CONSTANTS VERIFICATION

### 3.1 Baud Rate Calculation

#### Legacy:
```c
#define OPTICAL_BAUD_RATE_BASE 1382400  // Implicit

data = (1382400 / BaudRate);  // ✅ divisor = 1382400 / baud_rate
```

#### Your Code:
```c
#define OPTICAL_BAUD_RATE_BASE 1382400  // ✅ Explicit

divisor = OPTICAL_BAUD_RATE_BASE / baud_rate;  // ✅ Same calculation
```

**Status: ✅ BAUD RATE CALCULATION MATCHES**

---

### 3.2 Frame Structure

#### Legacy Frame Format:
- **Frame size:** 128 bytes (implicit from context)
- **Header:** 0x68, 0x68 (2 bytes)
- **Payload:** Variable length
- **CRC:** Modbus CRC-16 (last 2 bytes)

#### Your Frame Format (optical_fiber_driver.h):
```c
#define OPTICAL_FRAME_SIZE   128   // ✅ CORRECT
#define OPTICAL_MAX_PAYLOAD  120   // ✅ CORRECT (128 - 2 header - 2 length/source - 2 CRC - 2 padding)
#define OPTICAL_FRAME_HEADER 0x68  // ✅ CORRECT

// Frame structure offsets
#define FRAME_OFFSET_HEADER1 0       // First header byte (0x68)
#define FRAME_OFFSET_HEADER2 1       // Second header byte (0x68)
#define FRAME_OFFSET_LENGTH  2       // Payload length byte
#define FRAME_OFFSET_SOURCE  3       // Source node ID
#define FRAME_OFFSET_PAYLOAD 4       // Start of payload
#define FRAME_OFFSET_CRC     (OPTICAL_FRAME_SIZE - 2)  // CRC-16 location (bytes 126-127)
```

**Status: ✅ FRAME STRUCTURE MATCHES DOCUMENTATION**

---

### 3.3 CRC-16 Implementation

#### Your Code (optical_fiber_driver.c):
```c
uint16_t OpticalFiber_CRC16(const uint8_t* data, uint16_t length)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i];

        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;  // Modbus polynomial ✅
            }
            else {
                crc = crc >> 1;
            }
        }
    }

    return crc;
}
```

**Status: ✅ STANDARD MODBUS CRC-16 (Correct polynomial 0xA001)**

---

## 4. DATA TRANSFER LOGIC COMPARISON

### 4.1 Transmit Logic

#### Legacy (optical_fiber.c):
```c
void Tx_Buffer(unsigned short int port, unsigned char data[], unsigned short int len)
{
    unsigned short int i;
    for(i = 0; i < len; i++)
    {
        *(unsigned short int *) (Bank1_SRAM4_ADDR + (port + 100) * 2) = (unsigned short int)data[i];
        // Sequential writes to same register (auto-increment in FPGA)
    }
}
```

#### Your Code (optical_fiber_driver.c):
```c
bool OpticalFiber_Transmit(optical_fiber_t* fiber, uint8_t channel, const uint8_t* data, uint16_t length)
{
    // ... validation ...
    
    volatile uint16_t* tx_reg = get_tx_register(channel);
    
    // Write data to TX FIFO (auto-incrementing register)
    for (uint16_t i = 0; i < length; i += 2) {
        uint16_t word;
        if (i + 1 < length) {
            // Two bytes available (little-endian packing)
            word = (uint16_t)data[i] | ((uint16_t)data[i + 1] << 8);
        }
        else {
            // Only one byte available, pad with zero
            word = (uint16_t)data[i];
        }
        REG_WRITE16(tx_reg, word);  // Auto-increments
    }
    
    return true;
}
```

**Status: ⚠️ POTENTIAL ISSUE - BYTE PACKING**

**Problem:** Your code packs two bytes into a 16-bit word before writing, while legacy writes one byte at a time as 16-bit values.

**Legacy behavior:**
- `data[0]` → Write 16-bit word `0x00XX` (where XX = data[0])
- `data[1]` → Write 16-bit word `0x00YY` (where YY = data[1])

**Your behavior:**
- `data[0], data[1]` → Write 16-bit word `0xYYXX` (packed together)

**Impact:** ❌ **CRITICAL** - Data will be corrupted! FPGA expects byte-by-byte writes.

---

### 4.2 Receive Logic

#### Legacy (optical_fiber.c):
```c
unsigned short int Rx_Buffer(unsigned short int port, unsigned char data[])
{
    unsigned short int len = 0, i = 0;
    
    // Read while FIFO not empty (bit 1 = 0x0002 = RX_FIFO_EMPTY)
    while(((Read_State(port + 100) & 0x0002) == 0) && (i++ < 255))
    {
        data[len] = (unsigned char) Read_State(port);  // Read one byte
        len++;
    }
    
    return len;
}
```

#### Your Code (optical_fiber_driver.c):
```c
static void process_received_data(optical_fiber_t* fiber, uint8_t channel_index)
{
    // ... get RX register ...
    
    const uint16_t max_reads_per_poll = 128;
    uint16_t reads = 0;
    
    while (reads < max_reads_per_poll) {
        // Check if FIFO is now empty
        OpticalFiber_UpdateChannelStatus(fiber, channel);
        if (fiber->channel[channel_index].rx_fifo_empty) {  // ✅ Check bit 1
            break;
        }
        
        // Read 16-bit word from FIFO
        uint16_t word = REG_READ16(rx_reg);
        
        // Extract two bytes (little-endian)
        uint8_t byte_low = (uint8_t)(word & 0xFF);
        uint8_t byte_high = (uint8_t)((word >> 8) & 0xFF);
        
        // Store first byte
        if (rx_buf->write_index < sizeof(rx_buf->buffer)) {
            rx_buf->buffer[rx_buf->write_index++] = byte_low;
        }
        
        // Store second byte
        if (rx_buf->write_index < sizeof(rx_buf->buffer)) {
            rx_buf->buffer[rx_buf->write_index++] = byte_high;
        }
        
        reads++;
        
        // ... frame validation ...
    }
}
```

**Status: ⚠️ SAME BYTE PACKING ISSUE**

**Problem:** You're reading 16-bit words and extracting two bytes, but legacy reads one byte at a time.

**Legacy behavior:**
- Read 16-bit word `0x00XX` → Extract byte `XX`
- Read 16-bit word `0x00YY` → Extract byte `YY`

**Your behavior:**
- Read 16-bit word `0xYYXX` → Extract bytes `XX`, `YY`

**Impact:** ❌ **CRITICAL** - If FPGA returns one-byte-per-word, you'll read wrong data.

---

## 5. STATUS REGISTER BIT DEFINITIONS

### 5.1 Status Bits

#### Legacy (optical_fiber.c):
```c
// Check RX FIFO empty flag
while(((Read_State(port + 100) & 0x0002) == 0) && (i++ < 255))
//                                  ^^^^
//                                  Bit 1: RX FIFO Empty (1 = empty)
```

#### Your Code (optical_fiber_driver.h):
```c
// Status Register Bit Masks
#define FPGA_TX_BUSY       0x0001  // Bit 0: Transmitter busy
#define FPGA_RX_FIFO_EMPTY 0x0002  // Bit 1: RX FIFO empty ✅
#define FPGA_RX_FIFO_FULL  0x0004  // Bit 2: RX FIFO full (data loss risk)
```

**Status: ✅ STATUS BIT DEFINITIONS MATCH**

---

## 6. INITIALIZATION SEQUENCE COMPARISON

### 6.1 Legacy Initialization (optical_fiber.c):
```c
void Optical_fiber_Init(uint32_t BaudRate)
{
    // 1. Wait for power stabilization
    for(i = 0; i < 200000; i++);
    
    // 2. FSMC init (GPIO, timing, bank config)
    Optical_fiber_FSMC_Init();
    
    // 3. FPGA control pins init
    FPGA_RST_Init();        // PG11
    FPGA_DONE_Init();       // PD2
    FPGA_PROGRAMB_Init();   // PG15
    
    // 4. Reset FPGA (trigger reconfiguration)
    GPIO_ResetBits(GPIOG, GPIO_Pin_15);  // PROGRAMB low
    for(i = 0; i < 50; i++);
    GPIO_SetBits(GPIOG, GPIO_Pin_15);    // PROGRAMB high
    
    // 5. Wait for FPGA configuration (DONE signal high)
    while(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_2) != 1) {
        // Retry if needed
    }
    
    // 6. Internal FPGA soft reset (PG11)
    GPIO_SetBits(GPIOG, GPIO_Pin_11);    // RST high
    for(i = 0; i < 20; i++);
    GPIO_ResetBits(GPIOG, GPIO_Pin_11);  // RST low (pulse)
    for(i = 0; i < 100; i++);
    GPIO_SetBits(GPIOG, GPIO_Pin_11);    // RST high (idle)
    
    // 7. Configure baud rate
    BaudRate_Init(BaudRate);
    
    // 8. Set node ID
    *(unsigned short int *) (Bank1_SRAM4_ADDR + 16) = (unsigned short int)uchNodeNum;
}
```

### 6.2 Your Initialization (optical_fiber_driver.c):
```c
void OpticalFiber_Init(optical_fiber_t* fiber, uint32_t baud_rate, uint8_t node_id)
{
    // Clear all state
    memset(fiber, 0, sizeof(optical_fiber_t));

    // Store configuration
    fiber->node_id = node_id;
    fiber->baud_rate = baud_rate;

    // Calculate and write baud rate divisor
    uint16_t divisor = calculate_baud_divisor(baud_rate);
    REG_WRITE16(FPGA_BAUD_RATE_REG, divisor);  // ✅

    // Write node ID
    REG_WRITE16(FPGA_NODE_ID_REG, node_id);  // ✅

    // Initialize channel status
    for (uint8_t i = 0; i < OPTICAL_NUM_CHANNELS; i++) {
        fiber->channel[i].rx_fifo_empty = true;
    }
}
```

**Status: ⚠️ MISSING FPGA HARDWARE INITIALIZATION**

**Issues:**
1. ❌ No FPGA reconfiguration sequence (PROGRAMB pulse)
2. ❌ No FPGA soft reset sequence (PG11 pulse)
3. ❌ No DONE signal wait/check
4. ❌ Assumes FPGA already configured and running

**Impact:** If FPGA not configured or in wrong state, driver won't work.

---

## 7. CRITICAL FINDINGS SUMMARY

### ❌ **CRITICAL ISSUES:**

1. **Byte Packing in TX/RX** ❌ **SHOW-STOPPER**
   - **Problem:** You pack/unpack two bytes per 16-bit word
   - **Legacy:** Writes/reads one byte per 16-bit word (MSB = 0x00)
   - **Fix Required:** Change to byte-by-byte access with padding

2. **Missing FPGA Hardware Initialization** ⚠️ **MAJOR**
   - **Problem:** No PROGRAMB/RST sequences, no DONE check
   - **Impact:** FPGA may not be ready for communication
   - **Fix Required:** Add FPGA init sequence to match legacy

3. **Documentation Register Address Conflict** ⚠️ **RESOLVED**
   - **Initial concern:** Documentation showed wrong RX addresses
   - **Actual:** Code has correct addresses matching legacy
   - **Action:** Update documentation to match code

### ✅ **CORRECT IMPLEMENTATIONS:**

1. ✅ All FSMC register addresses correct
2. ✅ Baud rate calculation correct
3. ✅ Frame structure correct (128 bytes, 0x68 header, CRC-16)
4. ✅ CRC-16 implementation correct (Modbus polynomial)
5. ✅ Status bit definitions correct
6. ✅ Auto-increment register behavior understood
7. ✅ Channel numbering correct (0=broadcast, 1-7=channels)

---

## 8. REQUIRED FIXES

### Fix #1: Correct TX Byte Packing

**Change FROM:**
```c
// optical_fiber_driver.c - OpticalFiber_Transmit()
for (uint16_t i = 0; i < length; i += 2) {
    uint16_t word;
    if (i + 1 < length) {
        word = (uint16_t)data[i] | ((uint16_t)data[i + 1] << 8);
    }
    else {
        word = (uint16_t)data[i];
    }
    REG_WRITE16(tx_reg, word);
}
```

**Change TO:**
```c
// Write one byte at a time (MSB = 0x00)
for (uint16_t i = 0; i < length; i++) {
    uint16_t word = (uint16_t)data[i];  // Byte in LSB, MSB = 0
    REG_WRITE16(tx_reg, word);  // FPGA auto-increments
}
```

### Fix #2: Correct RX Byte Unpacking

**Change FROM:**
```c
// Read 16-bit word from FIFO
uint16_t word = REG_READ16(rx_reg);

// Extract two bytes
uint8_t byte_low = (uint8_t)(word & 0xFF);
uint8_t byte_high = (uint8_t)((word >> 8) & 0xFF);

// Store both bytes
rx_buf->buffer[rx_buf->write_index++] = byte_low;
rx_buf->buffer[rx_buf->write_index++] = byte_high;

reads++;
```

**Change TO:**
```c
// Read 16-bit word from FIFO
uint16_t word = REG_READ16(rx_reg);

// Extract only LSB (MSB should be 0x00)
uint8_t byte = (uint8_t)(word & 0xFF);

// Store one byte
if (rx_buf->write_index < sizeof(rx_buf->buffer)) {
    rx_buf->buffer[rx_buf->write_index++] = byte;
}

reads++;
```

### Fix #3: Add FPGA Hardware Initialization

**Add new function:**
```c
/**
 * @brief Initialize FPGA hardware (reconfiguration and reset)
 * 
 * This function must be called once during system startup, BEFORE
 * OpticalFiber_Init(), to ensure FPGA is properly configured.
 * 
 * Hardware pins:
 * - PG15: PROGRAMB (output) - Triggers FPGA reconfiguration
 * - PD2:  DONE (input) - Indicates FPGA configuration complete
 * - PG11: RST (output) - Internal FPGA soft reset
 */
void OpticalFiber_InitHardware(void)
{
    uint32_t i;
    
    // Wait for power stabilization
    for (i = 0; i < 200000; i++);
    
    // FSMC already initialized by MX_FSMC_Init()
    
    // GPIO pins already initialized by MX_GPIO_Init():
    // - PG15: FPGA_PROGRAMB_Pin
    // - PD2:  FPGA_DONE_Pin  
    // - PG11: FPGA_RST_Pin
    
    // Reset FPGA (trigger reconfiguration from flash)
    HAL_GPIO_WritePin(FPGA_PROGRAMB_GPIO_Port, FPGA_PROGRAMB_Pin, GPIO_PIN_RESET);  // Low
    for (i = 0; i < 50; i++);
    HAL_GPIO_WritePin(FPGA_PROGRAMB_GPIO_Port, FPGA_PROGRAMB_Pin, GPIO_PIN_SET);    // High
    
    // Wait for FPGA configuration to complete (DONE signal)
    for (i = 0; i < 200000000; i++) {
        if (HAL_GPIO_ReadPin(FPGA_DONE_GPIO_Port, FPGA_DONE_Pin) == GPIO_PIN_SET) {
            break;  // FPGA ready
        }
    }
    
    // If FPGA not ready, retry once
    if (HAL_GPIO_ReadPin(FPGA_DONE_GPIO_Port, FPGA_DONE_Pin) != GPIO_PIN_SET) {
        HAL_GPIO_WritePin(FPGA_PROGRAMB_GPIO_Port, FPGA_PROGRAMB_Pin, GPIO_PIN_RESET);
        for (i = 0; i < 500; i++);
        HAL_GPIO_WritePin(FPGA_PROGRAMB_GPIO_Port, FPGA_PROGRAMB_Pin, GPIO_PIN_SET);
        for (i = 0; i < 200000000; i++);
    }
    
    // Internal FPGA soft reset
    HAL_GPIO_WritePin(FPGA_RST_GPIO_Port, FPGA_RST_Pin, GPIO_PIN_SET);    // High (idle)
    for (i = 0; i < 20; i++);
    HAL_GPIO_WritePin(FPGA_RST_GPIO_Port, FPGA_RST_Pin, GPIO_PIN_RESET);  // Low (reset pulse)
    for (i = 0; i < 100; i++);
    HAL_GPIO_WritePin(FPGA_RST_GPIO_Port, FPGA_RST_Pin, GPIO_PIN_SET);    // High (idle)
}
```

**Update initialization sequence in main.c:**
```c
// In main() BEFORE OpticalFiber_Init():

OpticalFiber_InitHardware();  // ← ADD THIS FIRST

OpticalFiber_Init(&optical_fiber, 115200, node_id);
```

### Fix #4: Update Documentation

In `optical_fiber_communication_guide.md`, correct the RX register table to match actual implementation.

---

## 9. TESTING RECOMMENDATIONS

### Pre-Testing Checklist
- [ ] Apply Fix #1 (TX byte packing)
- [ ] Apply Fix #2 (RX byte unpacking)
- [ ] Apply Fix #3 (FPGA hardware init)
- [ ] Update documentation
- [ ] Add GPIO pin definitions to main.h:
  ```c
  #define FPGA_PROGRAMB_Pin GPIO_PIN_15
  #define FPGA_PROGRAMB_GPIO_Port GPIOG
  #define FPGA_DONE_Pin GPIO_PIN_2
  #define FPGA_DONE_GPIO_Port GPIOD
  #define FPGA_RST_Pin GPIO_PIN_11
  #define FPGA_RST_GPIO_Port GPIOG
  ```

### Testing Sequence

1. **FPGA Initialization Test:**
   - Power on board
   - Check DONE pin goes high (PD2)
   - Verify no stuck in init loop

2. **Loopback Test:**
   - Connect TX port 1 to RX port 1 (optical loopback)
   - Transmit test frame
   - Verify frame received correctly

3. **Baud Rate Test:**
   - Try different rates: 115200, 230400, 460800 bps
   - Verify communication works at all rates

4. **Multi-Channel Test:**
   - Test all 7 channels independently
   - Verify no cross-talk

5. **Broadcast Test:**
   - Send broadcast frame (port 0)
   - Verify received on all channels

---

## 10. CONCLUSION

**Status: ⚠️ NOT READY - CRITICAL FIXES REQUIRED**

The optical fiber driver has a solid architecture and most constants/addresses are correct, but **critical byte packing/unpacking issues and missing FPGA initialization will prevent operation**.

### Summary:
- ✅ Register addresses: 100% correct
- ✅ Protocol constants: 100% correct
- ✅ Frame structure: 100% correct
- ✅ CRC implementation: 100% correct
- ❌ Data transfer: Byte packing WRONG
- ❌ Initialization: FPGA hardware init MISSING

### Priority:
1. **CRITICAL:** Fix byte packing (Fix #1 and #2)
2. **MAJOR:** Add FPGA init (Fix #3)
3. **MINOR:** Update documentation (Fix #4)

After applying these fixes, the driver should match legacy behavior and work correctly with the FPGA optical transceivers.

---

**Document Status:** COMPLETE ✅  
**Hardware Testing Status:** NOT READY (fixes required) ⚠️  
**Confidence Level:** HIGH (after fixes applied) 🔧

