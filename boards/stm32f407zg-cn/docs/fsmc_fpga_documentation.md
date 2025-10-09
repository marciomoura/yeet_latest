# FSMC-FPGA Interface Configuration Guide

## Overview
This document provides **exact** configuration for STM32F407 FSMC interface to communicate with:
1. **FPGA** for Digital I/O (Bank1_NORSRAM4 at `0x6C000000`)
2. **AD7606** 8-channel ADC (Bank1_NORSRAM2 at `0x64000000`)

### FPGA Digital I/O
- **48 Digital Inputs** (DI0-DI47)
- **48 Digital Outputs** (DO0-DO47)
- **16-bit data bus** via FSMC Bank1_NORSRAM4
- **Base Address**: `0x6C000000`

### AD7606 Analog Inputs
- **8 Analog Inputs** (AI1-AI8): 16-bit simultaneous sampling
- **16-bit data bus** via FSMC Bank1_NORSRAM2
- **Base Address**: `0x64000000`
- **Input Range**: ±10V (configurable to ±5V)
- **Channel Mapping**:
  - AI1 (Ch0): Va - Phase A Voltage
  - AI2 (Ch1): Vb - Phase B Voltage
  - AI3 (Ch2): Vc - Phase C Voltage
  - AI4 (Ch3): Vfd - Field Voltage
  - AI5 (Ch4): Ia - Phase A Current
  - AI6 (Ch5): Ib - Phase B Current
  - AI7 (Ch6): Ic - Phase C Current
  - AI8 (Ch7): Ifd - Field Current

---

## STM32CubeMX Configuration

### Step 1: Enable FSMC Peripheral

```
Connectivity → FSMC
  ✅ Enable: Bank1_NORSRAM2 (for AD7606 ADC)
  ✅ Enable: Bank1_NORSRAM4 (for FPGA Digital I/O)
```

### Step 2: FSMC Parameters

#### Bank1_NORSRAM2 (AD7606 ADC)
```
Memory Type: PSRAM
Data Width: 16 bits
Address/Data Multiplexing: Disable
Burst Access Mode: Disable
Write Operation: Enable
Extended Mode: Disable
Asynchronous Wait: Disable
Wait Signal: Disable

Timing Parameters (Read & Write):
  Address Setup Time: 2 HCLK cycles
  Address Hold Time: 0 HCLK cycles
  Data Setup Time: 11 HCLK cycles
  Bus Turn Around Duration: 1 HCLK cycle
  CLK Division: 0
  Data Latency: 0
  Access Mode: Mode A
```

#### Bank1_NORSRAM4 (FPGA Digital I/O)
```
Memory Type: PSRAM
Data Width: 16 bits
Address/Data Multiplexing: Disable
Burst Access Mode: Disable
Write Operation: Enable
Extended Mode: Disable
Asynchronous Wait: Disable
Wait Signal: Disable
  
Timing Parameters (Read & Write):
  Address Setup Time: 6 HCLK cycles
  Address Hold Time: 15 HCLK cycles
  Data Setup Time: 21 HCLK cycles
  Bus Turn Around Duration: 1 HCLK cycle
  CLK Division: 1
  Data Latency: 0
  Access Mode: Mode A
```

### Step 3: GPIO Pin Configuration

**All pins configured as:**
- Mode: `Alternate Function Push Pull`
- Pull: `No Pull-up No Pull-down`
- Speed: `Very High (100 MHz)`
- Alternate Function: `FSMC`

#### Port D (GPIOD)
| Pin  | FSMC Function | Description                 |
| ---- | ------------- | --------------------------- |
| PD0  | FSMC_D2       | Data bit 2                  |
| PD1  | FSMC_D3       | Data bit 3                  |
| PD4  | FSMC_NOE      | Output Enable (Read strobe) |
| PD5  | FSMC_NWE      | Write Enable (Write strobe) |
| PD8  | FSMC_D13      | Data bit 13                 |
| PD9  | FSMC_D14      | Data bit 14                 |
| PD10 | FSMC_D15      | Data bit 15                 |
| PD11 | FSMC_A16      | Address bit 16              |
| PD12 | FSMC_A17      | Address bit 17              |
| PD14 | FSMC_D0       | Data bit 0                  |
| PD15 | FSMC_D1       | Data bit 1                  |

#### Port E (GPIOE)
| Pin  | FSMC Function | Description |
| ---- | ------------- | ----------- |
| PE7  | FSMC_D4       | Data bit 4  |
| PE8  | FSMC_D5       | Data bit 5  |
| PE9  | FSMC_D6       | Data bit 6  |
| PE10 | FSMC_D7       | Data bit 7  |
| PE11 | FSMC_D8       | Data bit 8  |
| PE12 | FSMC_D9       | Data bit 9  |
| PE13 | FSMC_D10      | Data bit 10 |
| PE14 | FSMC_D11      | Data bit 11 |
| PE15 | FSMC_D12      | Data bit 12 |

#### Port F (GPIOF)
| Pin  | FSMC Function | Description   |
| ---- | ------------- | ------------- |
| PF0  | FSMC_A0       | Address bit 0 |
| PF1  | FSMC_A1       | Address bit 1 |
| PF2  | FSMC_A2       | Address bit 2 |
| PF3  | FSMC_A3       | Address bit 3 |
| PF4  | FSMC_A4       | Address bit 4 |
| PF5  | FSMC_A5       | Address bit 5 |
| PF12 | FSMC_A6       | Address bit 6 |
| PF13 | FSMC_A7       | Address bit 7 |
| PF14 | FSMC_A8       | Address bit 8 |
| PF15 | FSMC_A9       | Address bit 9 |

#### Port G (GPIOG)
| Pin  | FSMC Function | Description                    |
| ---- | ------------- | ------------------------------ |
| PG0  | FSMC_A10      | Address bit 10                 |
| PG1  | FSMC_A11      | Address bit 11                 |
| PG2  | FSMC_A12      | Address bit 12                 |
| PG3  | FSMC_A13      | Address bit 13                 |
| PG4  | FSMC_A14      | Address bit 14                 |
| PG5  | FSMC_A15      | Address bit 15                 |
| PG9  | FSMC_NE2      | Chip Select for Bank2 (AD7606) |
| PG12 | FSMC_NE4      | Chip Select for Bank4 (FPGA)   |

**Total FSMC Pins Used: 43 GPIO pins**

### Step 4: AD7606 Control Pins (Non-FSMC)

These pins are configured as **standard GPIO outputs** (not FSMC alternate function):

| Pin  | AD7606 Signal | Mode        | Description                      |
| ---- | ------------- | ----------- | -------------------------------- |
| PD13 | BUSY          | GPIO Input  | Conversion busy status           |
| PF6  | CONVSTA       | GPIO Output | Conversion start trigger A       |
| PF7  | CONVSTB       | GPIO Output | Conversion start trigger B       |
| PF9  | RANGE         | GPIO Output | Input range (High=±10V, Low=±5V) |
| PF10 | RESET         | GPIO Output | ADC reset                        |

**Configuration:**
```
PD13: Mode=Input, Pull=No Pull
PF6, PF7, PF9, PF10: Mode=Output Push-Pull, Pull=Pull-Up, Speed=50 MHz
```

**Total Pins for AD7606: 5 GPIO pins + FSMC shared bus**

---

## Memory Map

### FPGA Digital I/O Registers (Base: 0x6C000000)

```c
#define FPGA_BASE_ADDR       0x6C000000

// Digital Input Registers (READ ONLY)
#define DI_REG0_ADDR         (FPGA_BASE_ADDR + (201 * 2))  // 0x6C000192 → DI0-DI15
#define DI_REG1_ADDR         (FPGA_BASE_ADDR + (202 * 2))  // 0x6C000194 → DI16-DI31
#define DI_REG2_ADDR         (FPGA_BASE_ADDR + (203 * 2))  // 0x6C000196 → DI32-DI47

// Digital Output Registers (WRITE ONLY)
#define DO_REG0_ADDR         (FPGA_BASE_ADDR + (3 * 2))    // 0x6C000006 → DO0-DO15
#define DO_REG1_ADDR         (FPGA_BASE_ADDR + (4 * 2))    // 0x6C000008 → DO16-DO31
#define DO_REG2_ADDR         (FPGA_BASE_ADDR + (5 * 2))    // 0x6C00000A → DO32-DO47

// LED Indicator Register (WRITE ONLY)
#define LED_REG_ADDR         (FPGA_BASE_ADDR + (7 * 2))    // 0x6C00000E → LED outputs

// Control Register (WRITE ONLY)
#define CTRL_REG_ADDR        (FPGA_BASE_ADDR + (6 * 2))    // 0x6C00000C → Control signals
```

### AD7606 ADC Registers (Base: 0x64000000)

```c
#define AD7606_BASE_ADDR     0x64000000
#define AD7606_DATA_OFFSET   0x800

// Analog Input Data Register (READ ONLY)
#define AI_DATA_ADDR         (AD7606_BASE_ADDR + AD7606_DATA_OFFSET)  // 0x64000800

// Reading sequence (8 consecutive 16-bit reads):
// Read 1: AI1 (Ch0) - Va (Phase A Voltage)
// Read 2: AI2 (Ch1) - Vb (Phase B Voltage)
// Read 3: AI3 (Ch2) - Vc (Phase C Voltage)
// Read 4: AI4 (Ch3) - Vfd (Field Voltage)
// Read 5: AI5 (Ch4) - Ia (Phase A Current)
// Read 6: AI6 (Ch5) - Ib (Phase B Current)
// Read 7: AI7 (Ch6) - Ic (Phase C Current)
// Read 8: AI8 (Ch7) - Ifd (Field Current)
```

**Address Calculation Formula:**
```c
Physical_Address = Base_Address + (Register_Index × 2)
```
*Note: × 2 because each register is 16-bit (2 bytes)*

---

## Data Structures

### Digital I/O

```c
// Digital Input arrays
uint8_t DI_Flag[8][8];          // Raw input state (8 groups × 8 bits)
uint8_t DI_Delayed_Flag[8][8];  // Debounced input state (2-cycle filter)
int     DI_Delay[8][8];         // Debounce counter

// Digital Output arrays
uint8_t DO_Flag[8][8];          // Output state (8 groups × 8 bits)

// LED indicators
uint8_t DO_LED_Run;             // Run LED (1=blink, 0=off)
uint8_t DO_LED_Fault;           // Fault LED (1=on, 0=off)
uint8_t DO_LED_alarm;           // Alarm LED (1=on, 0=off)
```

**Bit Mapping:**
```
DI_Flag[0][0-7]  → DI0  to DI7
DI_Flag[1][0-7]  → DI8  to DI15
DI_Flag[2][0-7]  → DI16 to DI23
DI_Flag[3][0-7]  → DI24 to DI31
DI_Flag[4][0-7]  → DI32 to DI39
DI_Flag[5][0-7]  → DI40 to DI47
```

### Analog Inputs (AD7606)

```c
// AD7606 raw data buffer
uint16_t AI_Data[8];  // 8 channels × 16-bit

// Channel mapping
#define AI1  AI_Data[0]  // Va  - Phase A Voltage
#define AI2  AI_Data[1]  // Vb  - Phase B Voltage
#define AI3  AI_Data[2]  // Vc  - Phase C Voltage
#define AI4  AI_Data[3]  // Vfd - Field Voltage
#define AI5  AI_Data[4]  // Ia  - Phase A Current
#define AI6  AI_Data[5]  // Ib  - Phase B Current
#define AI7  AI_Data[6]  // Ic  - Phase C Current
#define AI8  AI_Data[7]  // Ifd - Field Current

// Data format: Signed 16-bit, ±10V range
// 0x0000 to 0x7FFF → 0 to +10V
// 0x8000 to 0xFFFF → -10V to 0 (two's complement)
```

---

## Driver Code

### AD7606 ADC Driver (Read Analog Inputs)

```c
/**
 * @brief  Initialize AD7606 control pins
 * @retval None
 */
void AD7606_Init_Pin(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // Enable clocks
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOF, ENABLE);
    
    // BUSY input (PD13)
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    // Control outputs (PF6, PF7, PF9, PF10)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOF, &GPIO_InitStructure);
}

/**
 * @brief  Configure AD7606 for ±10V range
 * @retval None
 */
void ConfigAD7606(void)
{
    // Reset sequence
    GPIOF->BSRRH = GPIO_Pin_10;  // RST low
    for (volatile int i = 0; i < 100; i++);
    GPIOF->BSRRL = GPIO_Pin_10;  // RST high
    for (volatile int i = 0; i < 200; i++);
    GPIOF->BSRRH = GPIO_Pin_10;  // RST low
    
    // Set range to ±10V
    GPIOF->BSRRL = GPIO_Pin_9;   // RANGE high (±10V)
}

/**
 * @brief  Start AD7606 conversion
 * @retval None
 */
void Start_AD7606(void)
{
    // Pulse CONVSTA and CONVSTB to trigger simultaneous sampling
    GPIOF->BSRRL = GPIO_Pin_6 | GPIO_Pin_7;  // CONVSTA/B high
    GPIOF->BSRRH = GPIO_Pin_6 | GPIO_Pin_7;  // CONVSTA/B low
    for (volatile int i = 0; i < 20; i++);
    GPIOF->BSRRL = GPIO_Pin_6 | GPIO_Pin_7;  // CONVSTA/B high
}

/**
 * @brief  Read AD7606 conversion results
 * @param  pBuffer: Pointer to 8-element uint16_t array
 * @retval None
 */
void Read_AD7606(uint16_t* pBuffer)
{
    // Wait for conversion to complete (optional - can check BUSY pin)
    for (volatile int i = 0; i < 500; i++);
    
    // Read 8 consecutive 16-bit values via FSMC
    volatile uint16_t* pData = (volatile uint16_t*)(AD7606_BASE_ADDR + 0x800);
    
    for (int i = 0; i < 8; i++)
    {
        pBuffer[i] = *pData;  // Each read auto-increments to next channel
    }
}

/**
 * @brief  Read previous conversion and start new one (pipeline mode)
 * @param  pBuffer: Pointer to 8-element uint16_t array
 * @retval None
 * @note   More efficient - minimizes dead time between conversions
 */
void Read_Start_AD7606(uint16_t* pBuffer)
{
    Read_AD7606(pBuffer);  // Read previous results
    Start_AD7606();        // Start next conversion immediately
}

/**
 * @brief  Convert raw ADC value to voltage
 * @param  raw: Raw 16-bit ADC value
 * @retval Voltage in volts (±10V range)
 */
float AD7606_ToVoltage(uint16_t raw)
{
    int16_t signed_val;
    
    // Convert to signed
    if (raw & 0x8000)
        signed_val = raw - 0x10000;  // Negative value
    else
        signed_val = raw;
    
    // Convert to voltage: ±10V = ±32768 counts
    return (float)signed_val * (10.0f / 32768.0f);
}

/**
 * @brief  Initialize AD7606 (call once at startup)
 * @retval None
 */
void Init_AD7606(void)
{
    AD7606_Init_Pin();
    ConfigAD7606();
    // FSMC already initialized by MX_FSMC_Init() in main
}
```

### Digital Input Driver (Read from FPGA)

```c
/**
 * @brief  Read 16 digital inputs from FPGA register
 * @param  addr: Register address offset (201, 202, or 203)
 * @param  DI_Flag: Pointer to 16-byte array to store input states
 * @retval None
 */
void DI16_Input(uint16_t addr, uint8_t* DI_Flag)
{
    uint16_t data;
    uint16_t i;
    
    // Read 16-bit register from FPGA via FSMC
    data = *(__IO uint16_t*)(FPGA_BASE_ADDR + addr * 2);
    
    // Unpack 16 bits into individual byte flags
    for (i = 0; i < 16; i++)
    {
        if (data & 0x0001)
        {
            *DI_Flag++ = 1;
        }
        else
        {
            *DI_Flag++ = 0;
        }
        data = data >> 1;  // Shift to next bit
    }
}

/**
 * @brief  Read all 48 digital inputs
 * @retval None
 */
void Refresh_DI(void)
{
    DI16_Input(201, &DI_Flag[0][0]);  // Read DI0-DI15
    DI16_Input(202, &DI_Flag[2][0]);  // Read DI16-DI31
    DI16_Input(203, &DI_Flag[4][0]);  // Read DI32-DI47
}
```

### Digital Output Driver (Write to FPGA)

```c
/**
 * @brief  Write 16 digital outputs to FPGA register
 * @param  addr: Register address offset (3, 4, or 5)
 * @param  DO_Flag: Pointer to 16-byte array containing output states
 * @retval None
 */
void DO16_Out(uint16_t addr, uint8_t* DO_Flag)
{
    uint16_t data = 0;
    uint16_t i;
    
    // Pack 16 individual flags into a 16-bit word
    for (i = 0; i < 16; i++)
    {
        data = data >> 1;
        if (*DO_Flag++)
        {
            data |= 0x8000;  // Set MSB if flag is 1
        }
    }
    
    // Write to FPGA via FSMC
    *(__IO uint16_t*)(FPGA_BASE_ADDR + (addr * 2)) = data;
    
    // Small delay for FPGA to process
    for (i = 0; i < 6; i++)
    {
        __NOP();
    }
}

/**
 * @brief  Write all 48 digital outputs
 * @retval None
 */
void DO_Out(void)
{
    uint8_t i;
    uint8_t DO_LED[16] = {0};
    
    // Pack LED states
    DO_LED[0] = DO_LED_Run;    // Run indicator
    DO_LED[1] = DO_LED_Fault;  // Fault indicator
    DO_LED[2] = DO_LED_alarm;  // Alarm indicator
    
    // Write digital outputs
    DO16_Out(3, &DO_Flag[0][0]);  // Write DO0-DO15
    DO16_Out(4, &DO_Flag[2][0]);  // Write DO16-DO31
    DO16_Out(5, &DO_Flag[4][0]);  // Write DO32-DO47
    
    // Clear control register
    *(__IO uint16_t*)(FPGA_BASE_ADDR + 12) = 0;
    for (i = 0; i < 6; i++)
    {
        __NOP();
    }
    
    // Write LED outputs
    DO16_Out(7, DO_LED);
}
```

### Debounce Filter

```c
#define DI_DELAY_CYCLES  2  // Number of cycles for debounce

/**
 * @brief  Debounce single digital input
 * @param  DI_t: Current input state
 * @param  DI_Old: Pointer to previous stable state
 * @param  Delay: Pointer to delay counter
 * @retval None
 */
void Delayed_DI(uint8_t DI_t, uint8_t* DI_Old, int* Delay)
{
    if (*DI_Old != DI_t)
    {
        *Delay = *Delay + 1;
        if (*Delay >= DI_DELAY_CYCLES)
        {
            *DI_Old = DI_t;  // Update stable state
            *Delay = 0;
        }
    }
    else
    {
        *Delay = 0;  // Reset counter when stable
    }
}

/**
 * @brief  Apply debounce filter to all inputs
 * @retval None
 */
void Refresh_Delayed_DI(void)
{
    int i, j;
    
    for (i = 0; i < 8; i++)
    {
        for (j = 0; j < 8; j++)
        {
            Delayed_DI(DI_Flag[i][j], &DI_Delayed_Flag[i][j], &DI_Delay[i][j]);
        }
    }
}
```

### Initialization

```c
/**
 * @brief  Initialize DI/DO data structures
 * @retval None
 */
void Init_DIDO_Para(void)
{
    int i, j;
    
    for (i = 0; i < 8; i++)
    {
        for (j = 0; j < 8; j++)
        {
            DI_Flag[i][j] = 0;
            DI_Delayed_Flag[i][j] = 0;
            DI_Delay[i][j] = 0;
            DO_Flag[i][j] = 0;
        }
    }
    
    DO_LED_Run = 1;    // Run LED blinking
    DO_LED_Fault = 0;  // No fault
    DO_LED_alarm = 0;  // No alarm
}

/**
 * @brief  Initialize GPIO and FSMC (called once at startup)
 * @retval None
 * @note   STM32CubeMX will generate FSMC init code automatically
 */
void Init_GPIO(void)
{
    Init_DIDO_Para();
    
    // FSMC already initialized by STM32CubeMX (MX_FSMC_Init)
    
    DO_LED_Run = 1;
    
    // Initial read to stabilize inputs
    Refresh_DI();
    Refresh_Delayed_DI();
    Refresh_Delayed_DI();  // Second call ensures DI_Flag == DI_Delayed_Flag
}
```

### Main Loop Integration

```c
/**
 * @brief  Refresh DI and DO (call periodically, e.g., 1-5 kHz)
 * @retval None
 */
void GPIO(void)
{
    Refresh_DI();          // Read inputs from FPGA
    Refresh_Delayed_DI();  // Apply debounce filter
    DO_Out();              // Write outputs to FPGA
}
```

---

## Usage Examples

### Example 1: Read Analog Inputs (AD7606)

```c
uint16_t AI_Data[8];
float Va, Vb, Vc, Vfd;
float Ia, Ib, Ic, Ifd;

// Initialize AD7606 (once at startup)
Init_AD7606();
Start_AD7606();  // Start first conversion

// In main loop or timer ISR (e.g., 5 kHz)
Read_Start_AD7606(AI_Data);  // Read previous, start next

// Convert to engineering units
Va  = AD7606_ToVoltage(AI_Data[0]);  // Phase A voltage
Vb  = AD7606_ToVoltage(AI_Data[1]);  // Phase B voltage
Vc  = AD7606_ToVoltage(AI_Data[2]);  // Phase C voltage
Vfd = AD7606_ToVoltage(AI_Data[3]);  // Field voltage
Ia  = AD7606_ToVoltage(AI_Data[4]) * current_scale;  // Phase A current
Ib  = AD7606_ToVoltage(AI_Data[5]) * current_scale;  // Phase B current
Ic  = AD7606_ToVoltage(AI_Data[6]) * current_scale;  // Phase C current
Ifd = AD7606_ToVoltage(AI_Data[7]) * field_scale;    // Field current
```

### Example 2: Read Single Digital Input

```c
// Read DI0 (debounced)
if (DI_Delayed_Flag[0][0] == 1)
{
    // DI0 is HIGH
}

// Read DI15 (debounced)
if (DI_Delayed_Flag[1][7] == 1)
{
    // DI15 is HIGH
}

// Read DI32 (debounced)
if (DI_Delayed_Flag[4][0] == 1)
{
    // DI32 is HIGH
}
```

### Example 3: Set Single Digital Output

```c
// Set DO0 to HIGH
DO_Flag[0][0] = 1;
DO_Out();  // Write to FPGA

// Clear DO0
DO_Flag[0][0] = 0;
DO_Out();

// Set DO31 to HIGH
DO_Flag[3][7] = 1;
DO_Out();
```

### Example 4: Toggle LED

```c
// Blink run LED
DO_LED_Run = !DO_LED_Run;
DO_Out();

// Turn on fault LED
DO_LED_Fault = 1;
DO_Out();

// Turn off alarm LED
DO_LED_alarm = 0;
DO_Out();
```

### Example 5: Read Multiple Inputs

```c
// Check if any fuse has failed (assuming DI0-2 are fuse status, active-low)
uint8_t fuse_a_fail = !DI_Delayed_Flag[0][0];
uint8_t fuse_b_fail = !DI_Delayed_Flag[0][1];
uint8_t fuse_c_fail = !DI_Delayed_Flag[0][2];

if (fuse_a_fail || fuse_b_fail || fuse_c_fail)
{
    // Handle fuse failure
    DO_LED_Fault = 1;
    DO_Out();
}
```

---

## Important Notes

### Timing Considerations
- **FSMC Timing**: Total access time ≈ 6 + 21 = 27 HCLK cycles @ 168 MHz = ~160 ns
- **Debounce Filter**: 2-cycle delay eliminates noise/bouncing
- **Update Rate**: Typically 1-5 kHz in main loop or timer interrupt

### Memory Access
- **DI Registers**: Read-only, reading does not affect FPGA state
- **DO Registers**: Write-only, reading returns undefined data
- **Address Alignment**: All accesses are 16-bit aligned

### FPGA Interface Requirements
- FPGA must decode address lines A0-A17 to select appropriate register
- FPGA must respond to NOE (read enable) and NWE (write enable) strobes
- Setup/hold times must be compatible with FSMC timing parameters

### Debugging Tips
1. Use STM32CubeMX to verify all 41 pins are configured as FSMC alternate function
2. Verify FSMC clock is enabled: `RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC, ENABLE)`
3. Use logic analyzer to verify address/data bus signals
4. Check FPGA chip select (PG12/NE4) is active during transactions

---

## STM32CubeMX Quick Checklist

### 1. Connectivity Tab
- ✅ FSMC → Bank1_NORSRAM2 → **Enabled** (for AD7606)
- ✅ FSMC → Bank1_NORSRAM4 → **Enabled** (for FPGA)

### 2. Parameter Settings

**Bank1_NORSRAM2 (AD7606):**
- ✅ Memory Type: **PSRAM**
- ✅ Data Width: **16 bits**
- ✅ Address Setup Time: **2**
- ✅ Data Setup Time: **11**
- ✅ Address Hold Time: **0**

**Bank1_NORSRAM4 (FPGA):**
- ✅ Memory Type: **PSRAM**
- ✅ Data Width: **16 bits**
- ✅ Address Setup Time: **6**
- ✅ Data Setup Time: **21**
- ✅ Address Hold Time: **15**

### 3. GPIO Configuration (Pinout View)

**FSMC Pins (Alternate Function):**
- ✅ **GPIOD**: PD0, PD1, PD4, PD5, PD8-12, PD14-15 → FSMC
- ✅ **GPIOE**: PE0, PE1, PE7-15 → FSMC (note: PE0/PE1 are NBL0/NBL1)
- ✅ **GPIOF**: PF0-5, PF12-15 → FSMC
- ✅ **GPIOG**: PG0-5, PG9, PG12 → FSMC
- ✅ All FSMC pins: Speed = **Very High**, Mode = **Alternate Function**

**AD7606 Control Pins (GPIO Output):**
- ✅ **PD13**: GPIO Input (BUSY)
- ✅ **PF6**: GPIO Output (CONVSTA)
- ✅ **PF7**: GPIO Output (CONVSTB)
- ✅ **PF9**: GPIO Output (RANGE)
- ✅ **PF10**: GPIO Output (RESET)

### 4. Clock Configuration
- ✅ HCLK: **168 MHz** (for FSMC timing)
- ✅ AHB3 (FSMC): **168 MHz**

### 5. Code Generation
- ✅ Generate initialization code
- ✅ Add driver functions from this document to your project
- ✅ Call `Init_AD7606()` and `Init_GPIO()` after `MX_FSMC_Init()` in `main()`
- ✅ Call `Read_Start_AD7606()` at 5 kHz (e.g., TIM3 ISR)
- ✅ Call `GPIO()` periodically in main loop or timer ISR

---

## File Structure

```
Project/
├── Core/
│   ├── Src/
│   │   ├── main.c           (call Init_AD7606, Init_GPIO, and GPIO)
│   │   ├── fsmc.c           (auto-generated by CubeMX)
│   │   ├── ad7606_driver.c  (add AD7606 driver code here)
│   │   └── gpio_driver.c    (add DI/DO driver code here)
│   └── Inc/
│       ├── fsmc.h
│       ├── ad7606_driver.h  (declare AD7606 functions)
│       └── gpio_driver.h    (declare DI/DO functions)
└── Drivers/
    └── STM32F4xx_HAL_Driver/
```
│   │   └── gpio_driver.c    (add DI/DO driver code here)
│   └── Inc/
│       ├── fsmc.h
│       └── gpio_driver.h    (declare DI/DO functions)
└── Drivers/
    └── STM32F4xx_HAL_Driver/
```

---

## Testing Procedure

1. **Verify FSMC Initialization**
   ```c
   // Check if FSMC is enabled
   if (FSMC_Bank1->BTCR[6] & FSMC_BCR1_MBKEN)
   {
       // FSMC Bank4 is enabled
   }
   ```

2. **Test Digital Outputs**
   ```c
   // Write known pattern
   DO_Flag[0][0] = 1;
   DO_Flag[0][1] = 0;
   DO_Flag[0][2] = 1;
   DO_Out();
   // Verify with oscilloscope/logic analyzer on FPGA side
   ```

3. **Test Digital Inputs**
   ```c
   // Apply known signal to FPGA DI register
   Refresh_DI();
   // Check DI_Flag values match expected pattern
   ```

4. **Test Debounce**
   ```c
   // Toggle input rapidly, verify DI_Delayed_Flag stabilizes after 2 cycles
   ```

---

## Conclusion

This configuration provides a robust, memory-mapped interface between STM32F407 and FPGA for high-speed digital I/O. The debounce filter ensures reliable operation in noisy industrial environments.

For questions or issues, verify:
1. All 41 GPIO pins correctly configured in STM32CubeMX
2. FSMC timing parameters match FPGA requirements
3. FPGA register map matches address definitions
4. Clock configuration provides 168 MHz HCLK

