# STM32CubeMX Quick Setup Guide - FSMC Interface

## 🎯 Quick Reference

**Purpose**: Interface STM32F407 with:
1. **FPGA** via FSMC for 48 DI + 48 DO (Bank4 @ `0x6C000000`)
2. **AD7606** 8-channel ADC (Bank2 @ `0x64000000`)

**Pins Required**: 43 FSMC pins + 5 AD7606 control pins = 48 total

---

## ⚡ STM32CubeMX Setup (5 Steps)

### Step 1️⃣: Enable FSMC
```
Pinout & Configuration → Connectivity → FSMC
☑ PC Card/NAND Flash/NOR Flash
☑ Bank1_NORSRAM2: Checked (AD7606 ADC)
☑ Bank1_NORSRAM4: Checked (FPGA Digital I/O)
```

### Step 2️⃣: Configure FSMC Parameters

**Bank1_NORSRAM2 (AD7606 ADC):**
```
Mode and Configuration → FSMC → Bank1_NORSRAM2
┌─────────────────────────────────────────┐
│ Memory Type: PSRAM                       │
│ Data Width: 16 bits                      │
│ Address/Data Multiplexing: Disable      │
│ Burst Access Mode: Disable              │
│ Write Operation: Enable                 │
│ Extended Mode: Disable                  │
│ Asynchronous Wait: Disable              │
│ Wait Signal: Disable                    │
└─────────────────────────────────────────┘

Timing Configuration (Read & Write):
┌─────────────────────────────────────────┐
│ Address Setup Time: 2                    │
│ Address Hold Time: 0                     │
│ Data Setup Time: 11                      │
│ Bus Turn Around Duration: 1              │
│ Access Mode: Mode A                      │
└─────────────────────────────────────────┘
```

**Bank1_NORSRAM4 (FPGA Digital I/O):**
```
Mode and Configuration → FSMC → Bank1_NORSRAM4
┌─────────────────────────────────────────┐
│ Memory Type: PSRAM                       │
│ Data Width: 16 bits                      │
│ Address/Data Multiplexing: Disable      │
│ Burst Access Mode: Disable              │
│ Write Operation: Enable                 │
│ Extended Mode: Disable                  │
│ Asynchronous Wait: Disable              │
│ Wait Signal: Disable                    │
└─────────────────────────────────────────┘

Timing Configuration (Read & Write):
┌─────────────────────────────────────────┐
│ Address Setup Time: 6                    │
│ Address Hold Time: 15                    │
│ Data Setup Time: 21                      │
│ Bus Turn Around Duration: 1              │
│ Access Mode: Mode A                      │
└─────────────────────────────────────────┘
```

### Step 3️⃣: Verify GPIO Pins (Automatic)
After enabling FSMC Banks 2 and 4, these pins should automatically be configured:

**Port D (11 pins - FSMC)**
```
PD0, PD1, PD4, PD5, PD8, PD9, PD10, PD11, PD12, PD14, PD15
```

**Port D (1 pin - AD7606 GPIO)**
```
PD13: GPIO Input (BUSY) - configure manually
```
```

**Port E (11 pins - FSMC)**
```
PE0, PE1, PE7, PE8, PE9, PE10, PE11, PE12, PE13, PE14, PE15
```

**Port F (10 pins - FSMC)**
```
PF0, PF1, PF2, PF3, PF4, PF5, PF12, PF13, PF14, PF15
```

**Port F (4 pins - AD7606 GPIO)**
```
PF6: GPIO Output (CONVSTA) - configure manually
PF7: GPIO Output (CONVSTB) - configure manually
PF9: GPIO Output (RANGE) - configure manually
PF10: GPIO Output (RESET) - configure manually
```

**Port G (8 pins - FSMC)**
```
PG0, PG1, PG2, PG3, PG4, PG5, PG9 (NE2 for AD7606), PG12 (NE4 for FPGA)
```

**FSMC Pins (auto-configured):**
- **Mode**: Alternate Function Push Pull
- **Pull**: No Pull-up No Pull-down
- **Speed**: Very High
- **AF**: FSMC

**AD7606 Control Pins (configure manually):**
- **PD13 (BUSY)**: Mode=Input, Pull=No Pull
- **PF6, PF7, PF9, PF10**: Mode=Output PP, Pull=Pull-Up, Speed=50 MHz

### Step 4️⃣: Clock Configuration
```
Clock Configuration → RCC
┌─────────────────────────────────────────┐
│ HCLK: 168 MHz                            │
│ APB1: 42 MHz                             │
│ APB2: 84 MHz                             │
│ AHB3 (FSMC): 168 MHz                     │
└─────────────────────────────────────────┘
```

### Step 5️⃣: Generate Code
```
Project Manager → Project
  Project Name: [your_project]
  Toolchain/IDE: [your_IDE]

☑ Generate peripheral initialization as pair of '.c/.h' files
☑ Generate code
```

---

## 📝 Pin Assignment Table (Copy to Your Schematic)

| STM32 Pin                | FSMC Signal | Direction | FPGA Pin | Description          |
| ------------------------ | ----------- | --------- | -------- | -------------------- |
| **Data Bus (16-bit)**    |
| PD14                     | D0          | ↔         | [FPGA]   | Data bit 0           |
| PD15                     | D1          | ↔         | [FPGA]   | Data bit 1           |
| PD0                      | D2          | ↔         | [FPGA]   | Data bit 2           |
| PD1                      | D3          | ↔         | [FPGA]   | Data bit 3           |
| PE7                      | D4          | ↔         | [FPGA]   | Data bit 4           |
| PE8                      | D5          | ↔         | [FPGA]   | Data bit 5           |
| PE9                      | D6          | ↔         | [FPGA]   | Data bit 6           |
| PE10                     | D7          | ↔         | [FPGA]   | Data bit 7           |
| PE11                     | D8          | ↔         | [FPGA]   | Data bit 8           |
| PE12                     | D9          | ↔         | [FPGA]   | Data bit 9           |
| PE13                     | D10         | ↔         | [FPGA]   | Data bit 10          |
| PE14                     | D11         | ↔         | [FPGA]   | Data bit 11          |
| PE15                     | D12         | ↔         | [FPGA]   | Data bit 12          |
| PD8                      | D13         | ↔         | [FPGA]   | Data bit 13          |
| PD9                      | D14         | ↔         | [FPGA]   | Data bit 14          |
| PD10                     | D15         | ↔         | [FPGA]   | Data bit 15          |
| **Address Bus (18-bit)** |
| PF0                      | A0          | →         | [FPGA]   | Address bit 0        |
| PF1                      | A1          | →         | [FPGA]   | Address bit 1        |
| PF2                      | A2          | →         | [FPGA]   | Address bit 2        |
| PF3                      | A3          | →         | [FPGA]   | Address bit 3        |
| PF4                      | A4          | →         | [FPGA]   | Address bit 4        |
| PF5                      | A5          | →         | [FPGA]   | Address bit 5        |
| PF12                     | A6          | →         | [FPGA]   | Address bit 6        |
| PF13                     | A7          | →         | [FPGA]   | Address bit 7        |
| PF14                     | A8          | →         | [FPGA]   | Address bit 8        |
| PF15                     | A9          | →         | [FPGA]   | Address bit 9        |
| PG0                      | A10         | →         | [FPGA]   | Address bit 10       |
| PG1                      | A11         | →         | [FPGA]   | Address bit 11       |
| PG2                      | A12         | →         | [FPGA]   | Address bit 12       |
| PG3                      | A13         | →         | [FPGA]   | Address bit 13       |
| PG4                      | A14         | →         | [FPGA]   | Address bit 14       |
| PG5                      | A15         | →         | [FPGA]   | Address bit 15       |
| PD11                     | A16         | →         | [FPGA]   | Address bit 16       |
| PD12                     | A17         | →         | [FPGA]   | Address bit 17       |
| **Control Signals**      |
| PD4                      | NOE         | →         | [FPGA]   | Output Enable (Read) |
| PD5                      | NWE         | →         | [FPGA]   | Write Enable         |
| PG12                     | NE4         | →         | [FPGA]   | Chip Select Bank4    |

**Total: 41 pins**

---

## 🔌 Register Maps

### FPGA Digital I/O (Base: 0x6C000000)

**Digital Input Registers (Read Only)**
```c
Address 0x6C000192  (offset 201×2)  →  DI[0:15]   (DI0-DI15)
Address 0x6C000194  (offset 202×2)  →  DI[16:31]  (DI16-DI31)
Address 0x6C000196  (offset 203×2)  →  DI[32:47]  (DI32-DI47)
```

**Digital Output Registers (Write Only)**
```c
Address 0x6C000006  (offset 3×2)    →  DO[0:15]   (DO0-DO15)
Address 0x6C000008  (offset 4×2)    →  DO[16:31]  (DO16-DO31)
Address 0x6C00000A  (offset 5×2)    →  DO[32:47]  (DO32-DO47)
Address 0x6C00000E  (offset 7×2)    →  LED[0:15]  (LED outputs)
```

### AD7606 ADC (Base: 0x64000000)

**Analog Input Data Register (Read Only)**
```c
Address 0x64000800  (offset 0x800)  →  8 sequential reads:
  Read 1: AI1 (Ch0) - Va (Phase A Voltage)
  Read 2: AI2 (Ch1) - Vb (Phase B Voltage)
  Read 3: AI3 (Ch2) - Vc (Phase C Voltage)
  Read 4: AI4 (Ch3) - Vfd (Field Voltage)
  Read 5: AI5 (Ch4) - Ia (Phase A Current)
  Read 6: AI6 (Ch5) - Ib (Phase B Current)
  Read 7: AI7 (Ch6) - Ic (Phase C Current)
  Read 8: AI8 (Ch7) - Ifd (Field Current)

Data Format: Signed 16-bit, ±10V range
  0x0000 to 0x7FFF → 0 to +10V
  0x8000 to 0xFFFF → -10V to 0 (two's complement)
```

---

## 💻 Minimal Driver Code

### Add to your project:

**ad7606_driver.h**
```c
#ifndef AD7606_DRIVER_H
#define AD7606_DRIVER_H

#include <stdint.h>

#define AD7606_BASE_ADDR  0x64000000
#define AD7606_DATA_OFFSET 0x800

void Init_AD7606(void);
void Start_AD7606(void);
void Read_AD7606(uint16_t* pBuffer);
void Read_Start_AD7606(uint16_t* pBuffer);
float AD7606_ToVoltage(uint16_t raw);

#endif
```

**ad7606_driver.c**
```c
#include "ad7606_driver.h"
#include "stm32f4xx_hal.h"

void Init_AD7606(void)
{
    // FSMC already initialized by CubeMX
    // Reset AD7606
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET);
    
    // Set range to ±10V
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET);
}

void Start_AD7606(void)
{
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_RESET);
    for (volatile int i = 0; i < 20; i++);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);
}

void Read_AD7606(uint16_t* pBuffer)
{
    for (volatile int i = 0; i < 500; i++);  // Wait for conversion
    volatile uint16_t* pData = (volatile uint16_t*)(AD7606_BASE_ADDR + AD7606_DATA_OFFSET);
    for (int i = 0; i < 8; i++) pBuffer[i] = *pData;
}

void Read_Start_AD7606(uint16_t* pBuffer)
{
    Read_AD7606(pBuffer);
    Start_AD7606();
}

float AD7606_ToVoltage(uint16_t raw)
{
    int16_t val = (raw & 0x8000) ? (raw - 0x10000) : raw;
    return (float)val * (10.0f / 32768.0f);
}
```

**gpio_driver.h**
```c
#ifndef GPIO_DRIVER_H
#define GPIO_DRIVER_H

#include <stdint.h>

// Base address
#define FPGA_BASE_ADDR  0x6C000000

// Data arrays
extern uint8_t DI_Flag[8][8];
extern uint8_t DI_Delayed_Flag[8][8];
extern uint8_t DO_Flag[8][8];
extern uint8_t DO_LED_Run;
extern uint8_t DO_LED_Fault;
extern uint8_t DO_LED_alarm;

// Functions
void Init_GPIO(void);
void GPIO(void);
void Refresh_DI(void);
void DO_Out(void);

#endif
```

**gpio_driver.c**
```c
#include "gpio_driver.h"

uint8_t DI_Flag[8][8];
uint8_t DI_Delayed_Flag[8][8];
int DI_Delay[8][8];
uint8_t DO_Flag[8][8];
uint8_t DO_LED_Run = 1;
uint8_t DO_LED_Fault = 0;
uint8_t DO_LED_alarm = 0;

void DI16_Input(uint16_t addr, uint8_t* DI_Flag)
{
    uint16_t data = *(__IO uint16_t*)(FPGA_BASE_ADDR + addr * 2);
    for (uint16_t i = 0; i < 16; i++)
    {
        *DI_Flag++ = (data & 0x0001) ? 1 : 0;
        data >>= 1;
    }
}

void Refresh_DI(void)
{
    DI16_Input(201, &DI_Flag[0][0]);
    DI16_Input(202, &DI_Flag[2][0]);
    DI16_Input(203, &DI_Flag[4][0]);
}

void DO16_Out(uint16_t addr, uint8_t* DO_Flag)
{
    uint16_t data = 0;
    for (uint16_t i = 0; i < 16; i++)
    {
        data >>= 1;
        if (*DO_Flag++) data |= 0x8000;
    }
    *(__IO uint16_t*)(FPGA_BASE_ADDR + addr * 2) = data;
}

void DO_Out(void)
{
    DO16_Out(3, &DO_Flag[0][0]);
    DO16_Out(4, &DO_Flag[2][0]);
    DO16_Out(5, &DO_Flag[4][0]);
    
    *(__IO uint16_t*)(FPGA_BASE_ADDR + 12) = 0;
    
    uint8_t led[16] = {DO_LED_Run, DO_LED_Fault, DO_LED_alarm};
    DO16_Out(7, led);
}

void Delayed_DI(uint8_t DI_t, uint8_t* DI_Old, int* Delay)
{
    if (*DI_Old != DI_t)
    {
        (*Delay)++;
        if (*Delay >= 2)
        {
            *DI_Old = DI_t;
            *Delay = 0;
        }
    }
    else *Delay = 0;
}

void Refresh_Delayed_DI(void)
{
    for (int i = 0; i < 8; i++)
        for (int j = 0; j < 8; j++)
            Delayed_DI(DI_Flag[i][j], &DI_Delayed_Flag[i][j], &DI_Delay[i][j]);
}

void Init_GPIO(void)
{
    for (int i = 0; i < 8; i++)
        for (int j = 0; j < 8; j++)
        {
            DI_Flag[i][j] = 0;
            DI_Delayed_Flag[i][j] = 0;
            DI_Delay[i][j] = 0;
            DO_Flag[i][j] = 0;
        }
    
    Refresh_DI();
    Refresh_Delayed_DI();
    Refresh_Delayed_DI();
}

void GPIO(void)
{
    Refresh_DI();
    Refresh_Delayed_DI();
    DO_Out();
}
```

**main.c (usage)**
```c
#include "gpio_driver.h"

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_FSMC_Init();  // ← Generated by CubeMX
    
    Init_GPIO();     // ← Initialize DI/DO driver
    
    while (1)
    {
        GPIO();      // ← Refresh DI/DO (call at 1-5 kHz)
        
        // Example: Read DI0
        if (DI_Delayed_Flag[0][0]) {
            // DI0 is HIGH
        }
        
        // Example: Set DO0
        DO_Flag[0][0] = 1;
        
        HAL_Delay(1);
    }
}
```

---

## ✅ Verification Checklist

After generating code:

### 1. Check `fsmc.c`
```c
// Should contain:
RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC, ENABLE);
FSMC_Bank1->BTCR[6] |= FSMC_BCR1_MBKEN;  // Bank4 enabled
```

### 2. Check GPIO Configuration
```c
// In gpio.c, verify all 41 pins configured as AF12 (FSMC)
GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
GPIO_InitStruct.Alternate = GPIO_AF12_FSMC;
```

### 3. Test Memory Access
```c
// Write test
*(__IO uint16_t*)(0x6C000006) = 0xAAAA;

// Read test
uint16_t test = *(__IO uint16_t*)(0x6C000192);
```

### 4. Logic Analyzer Check
- Connect to address/data bus
- Verify NOE, NWE, NE4 strobes
- Confirm timing: setup=6, data=21 HCLK cycles

---

## 🐛 Troubleshooting

| Issue                    | Check                                               |
| ------------------------ | --------------------------------------------------- |
| **Hard fault on access** | FSMC clock enabled? Bank4 enabled?                  |
| **Wrong data read**      | Address calculation correct? (× 2 for 16-bit)       |
| **Outputs not working**  | Write enable configured? FPGA responding?           |
| **Inputs always 0**      | FPGA driving data bus? NOE signal working?          |
| **Timing violations**    | FPGA setup/hold times compatible with 160ns access? |

---

## 📚 File References

See `FSMC_FPGA_Interface_Documentation.md` for:
- Complete register map
- Detailed driver explanations
- Usage examples
- Timing diagrams
- Advanced features

---

## 🎓 Key Concepts

1. **Memory-Mapped I/O**: FPGA registers appear as normal memory at `0x6C000000`
2. **16-bit Access**: All reads/writes are 16-bit aligned
3. **Address Offset**: Physical address = Base + (Index × 2)
4. **Debounce**: 2-cycle filter eliminates input noise
5. **Timing**: ~160ns per transaction @ 168 MHz (FPGA), ~80ns (AD7606)

---

## Summary

✅ **43 FSMC pins** configured as FSMC alternate function  
✅ **5 GPIO pins** for AD7606 control (BUSY, CONVSTA/B, RANGE, RESET)  
✅ **FSMC Bank2** enabled for AD7606 (0x64000000)  
✅ **FSMC Bank4** enabled for FPGA (0x6C000000)  
✅ **168 MHz** clock for fast access (~80-160ns per transaction)  
✅ **48 DI + 48 DO** via FPGA (3 registers each)  
✅ **8 AI** (±10V, 16-bit) via AD7606 simultaneous sampling  
✅ **Debounce filter** for reliable digital I/O operation  
✅ **Channel mapping**: Va/Vb/Vc/Vfd voltages + Ia/Ib/Ic/Ifd currents

**That's it! Your FSMC-FPGA-ADC interface is ready to use.**

