# STM32F407 Clock Configuration Guide for STM32CubeMX

## Overview

This document provides the **exact clock configuration** extracted from the STM32F407VGT6 SCR bridge controller firmware. Use this guide to configure the clock tree in STM32CubeMX to match the original hardware.

---

## Clock Architecture Summary

```
┌────────────────────────────────────────────────────────────────────────┐
│                      STM32F407 CLOCK TREE                              │
│                                                                        │
│  ┌──────────┐                                                         │
│  │ HSE XTAL │                                                         │
│  │ 25 MHz   │───────────┐                                            │
│  └──────────┘           │                                            │
│                         ▼                                            │
│                    ┌─────────┐                                       │
│                    │  /PLL_M │  ÷25                                  │
│                    │   = 25  │                                       │
│                    └────┬────┘                                       │
│                         │                                            │
│                         ▼                                            │
│                     1 MHz                                            │
│                         │                                            │
│                         ▼                                            │
│                    ┌─────────┐                                       │
│                    │ ×PLL_N  │  ×336                                 │
│                    │  = 336  │                                       │
│                    └────┬────┘                                       │
│                         │                                            │
│                         ▼                                            │
│                    336 MHz (VCO)                                     │
│                         │                                            │
│         ┌───────────────┼───────────────┬─────────────┐             │
│         ▼               ▼               ▼             ▼             │
│    ┌─────────┐    ┌─────────┐    ┌─────────┐   ┌─────────┐        │
│    │  /PLL_P │    │  /PLL_Q │    │ PLLI2S  │   │  PLLSAI │        │
│    │   ÷2    │    │   ÷7    │    │(not used│   │(not used│        │
│    └────┬────┘    └────┬────┘    └─────────┘   └─────────┘        │
│         │              │                                            │
│         ▼              ▼                                            │
│    168 MHz         48 MHz                                          │
│    SYSCLK       (USB/SDIO/RNG)                                     │
│         │                                                           │
│         ▼                                                           │
│    ┌─────────┐                                                     │
│    │ AHB     │  ÷1                                                 │
│    │ Prescaler                                                     │
│    └────┬────┘                                                     │
│         │                                                           │
│         ▼                                                           │
│    168 MHz HCLK (AHB Bus)                                          │
│         │                                                           │
│         ├──────────────┬──────────────┬──────────────────┐         │
│         ▼              ▼              ▼                  ▼         │
│    ┌─────────┐   ┌─────────┐   ┌─────────┐       ┌─────────┐     │
│    │ APB2    │   │ APB1    │   │ AHB3    │       │ Cortex  │     │
│    │ ÷2      │   │ ÷4      │   │ ÷1      │       │ System  │     │
│    └────┬────┘   └────┬────┘   └────┬────┘       │ Timer   │     │
│         │             │             │             └─────────┘     │
│         ▼             ▼             ▼                             │
│    84 MHz        42 MHz        168 MHz                            │
│    PCLK2         PCLK1         (FSMC)                             │
│         │             │                                            │
│         │             │                                            │
│    ┌────┴────┐   ┌────┴────┐                                      │
│    │Timer×2  │   │Timer×2  │                                      │
│    │Multiplier│   │Multiplier│                                     │
│    └────┬────┘   └────┬────┘                                      │
│         │             │                                            │
│         ▼             ▼                                            │
│    168 MHz       84 MHz                                           │
│    (TIM1,8-11)   (TIM2-7,12-14)                                   │
│                                                                    │
└────────────────────────────────────────────────────────────────────┘
```

---

## Clock Source Configuration

### External High-Speed Oscillator (HSE)

| Parameter                       | Value                     | Description                           |
| ------------------------------- | ------------------------- | ------------------------------------- |
| **Oscillator Type**             | Crystal/Ceramic Resonator | External crystal oscillator           |
| **HSE Frequency**               | **25 MHz**                | External crystal frequency            |
| **HSE State**                   | **ON**                    | Must be enabled                       |
| **HSE Startup Timeout**         | 0x0500 (1280 cycles)      | Default timeout value                 |
| **CSS (Clock Security System)** | Optional                  | Recommended for critical applications |

**STM32CubeMX Setting:**
- Navigate to **RCC (Reset and Clock Control)**
- Set **High Speed Clock (HSE)** to: `Crystal/Ceramic Resonator`
- HSE frequency: `25 MHz`

---

## PLL Configuration (Main System PLL)

### PLL Parameters

| Parameter                        | Value      | Calculation         | Result             |
| -------------------------------- | ---------- | ------------------- | ------------------ |
| **PLL Source**                   | HSE        | External oscillator | 25 MHz input       |
| **PLL_M** (Input divider)        | **25**     | 25 MHz ÷ 25         | 1 MHz              |
| **PLL_N** (Multiplier)           | **336**    | 1 MHz × 336         | 336 MHz (VCO)      |
| **PLL_P** (System clock divider) | **2** (÷2) | 336 MHz ÷ 2         | **168 MHz SYSCLK** |
| **PLL_Q** (USB/SDIO/RNG divider) | **7** (÷7) | 336 MHz ÷ 7         | 48 MHz             |

### PLL Calculation Formula

```
VCO_Clock = (HSE_Frequency / PLL_M) × PLL_N
          = (25,000,000 / 25) × 336
          = 1,000,000 × 336
          = 336,000,000 Hz

SYSCLK = VCO_Clock / PLL_P
       = 336,000,000 / 2
       = 168,000,000 Hz (168 MHz)

USB/SDIO/RNG Clock = VCO_Clock / PLL_Q
                   = 336,000,000 / 7
                   = 48,000,000 Hz (48 MHz) ← Required for USB
```

**STM32CubeMX Setting:**
- Navigate to **Clock Configuration** tab
- Set **PLL Source Mux** to: `HSE`
- Set **PLL_M** (DIVM) to: `25`
- Set **PLL_N** (MULN) to: `336`
- Set **PLL_P** (DIVP) to: `2`
- Set **PLL_Q** (DIVQ) to: `7`
- Set **System Clock Mux** to: `PLLCLK`

---

## Bus Clock Configuration

### AHB Bus (Advanced High-Performance Bus)

| Parameter            | Value   | Calculation           | Result                |
| -------------------- | ------- | --------------------- | --------------------- |
| **HCLK Prescaler**   | **÷1**  | 168 MHz ÷ 1           | **168 MHz**           |
| **AHB Clock (HCLK)** | 168 MHz | Maximum for STM32F407 | Core, DMA, GPIO, FSMC |

**Peripherals on AHB:**
- Cortex-M4 Core
- DMA1, DMA2
- GPIO (Ports A-G)
- CRC, Flash interface
- Backup SRAM

**STM32CubeMX Setting:**
- Set **AHB Prescaler** to: `/1`

---

### APB2 Bus (High-Speed Peripheral Bus)

| Parameter              | Value       | Calculation                            | Result                           |
| ---------------------- | ----------- | -------------------------------------- | -------------------------------- |
| **APB2 Prescaler**     | **÷2**      | 168 MHz ÷ 2                            | **84 MHz**                       |
| **APB2 Clock (PCLK2)** | 84 MHz      | High-speed peripherals                 | ADC, TIM1, TIM8-11, USART1, SPI1 |
| **APB2 Timer Clock**   | **168 MHz** | Timer multiplier ×2 when prescaler ≠ 1 | TIM1, TIM8, TIM9, TIM10, TIM11   |

**Peripherals on APB2:**
- **TIM1** (Advanced timer - not used in this project)
- **TIM8** (Advanced timer - not used in this project)
- **TIM9** (General-purpose - **USED: External sync**)
- **TIM10** (General-purpose)
- **TIM11** (General-purpose)
- **USART1** (RS-232 debug serial)
- **SPI1** (Not used - SPI2 used for W5200)
- **ADC1** (Internal ADC for temperature)

**Timer Clock Calculation:**
```
APB2 Timer Clock = HCLK / APB2_Prescaler × 2
                 = 168 MHz / 2 × 2
                 = 168 MHz
```

**STM32CubeMX Setting:**
- Set **APB2 Prescaler** to: `/2`

---

### APB1 Bus (Low-Speed Peripheral Bus)

| Parameter              | Value      | Calculation                            | Result           |
| ---------------------- | ---------- | -------------------------------------- | ---------------- |
| **APB1 Prescaler**     | **÷4**     | 168 MHz ÷ 4                            | **42 MHz**       |
| **APB1 Clock (PCLK1)** | 42 MHz     | Max 42 MHz for STM32F407               | Most peripherals |
| **APB1 Timer Clock**   | **84 MHz** | Timer multiplier ×2 when prescaler ≠ 1 | TIM2-7, TIM12-14 |

**Peripherals on APB1:**
- **TIM2** (General-purpose - **USED: Firing angle delay**)
- **TIM3** (General-purpose - **USED: ADC trigger @ 5 kHz**)
- **TIM4** (General-purpose - **USED: Pulse generation**)
- **TIM5** (General-purpose - **USED: Sync capture + main loop timer**)
- **TIM6, TIM7** (Basic timers - not used)
- **USART3** (RS-485 industrial serial)
- **SPI2** (W5200 Ethernet controller)
- **RTC** (Real-Time Clock)
- **IWDG** (Independent Watchdog)

**Timer Clock Calculation:**
```
APB1 Timer Clock = HCLK / APB1_Prescaler × 2
                 = 168 MHz / 4 × 2
                 = 84 MHz
```

**Important Note:**
⚠️ **APB1 must not exceed 42 MHz** - This is a hardware limitation of the STM32F407!

**STM32CubeMX Setting:**
- Set **APB1 Prescaler** to: `/4`

---

### AHB3 Bus (FSMC Bus)

| Parameter          | Value                | Calculation   | Result          |
| ------------------ | -------------------- | ------------- | --------------- |
| **AHB3 Prescaler** | **Same as AHB** (÷1) | 168 MHz ÷ 1   | **168 MHz**     |
| **AHB3 Clock**     | 168 MHz              | Maximum speed | FSMC controller |

**Peripherals on AHB3:**
- **FSMC (Flexible Static Memory Controller)**
  - Bank1_NORSRAM2: AD7606 @ 0x64000000
  - Bank1_NORSRAM4: FPGA @ 0x6C000000

**STM32CubeMX Setting:**
- AHB3 automatically follows AHB prescaler

---

## Flash Memory Configuration

### Flash Access Latency

For **168 MHz HCLK** operation at **3.3V VDD**, the Flash memory requires wait states:

| Parameter             | Value                   | Description                 |
| --------------------- | ----------------------- | --------------------------- |
| **Flash Latency**     | **5 Wait States (5WS)** | Required for 168 MHz @ 3.3V |
| **Instruction Cache** | **Enabled**             | Improves performance        |
| **Data Cache**        | **Enabled**             | Improves performance        |
| **Prefetch Buffer**   | **Disabled**            | As per original code        |

**Flash Latency Table (3.3V VDD):**

| HCLK Frequency | Flash Latency | Wait States           |
| -------------- | ------------- | --------------------- |
| 0-30 MHz       | 0 WS          | FLASH_ACR_LATENCY_0WS |
| 30-60 MHz      | 1 WS          | FLASH_ACR_LATENCY_1WS |
| 60-90 MHz      | 2 WS          | FLASH_ACR_LATENCY_2WS |
| 90-120 MHz     | 3 WS          | FLASH_ACR_LATENCY_3WS |
| 120-150 MHz    | 4 WS          | FLASH_ACR_LATENCY_4WS |
| 150-168 MHz    | 5 WS          | FLASH_ACR_LATENCY_5WS |

**STM32CubeMX Setting:**
- Navigate to **System Core → FLASH**
- Set **Latency** to: `5 Wait States`
- Enable **Instruction Cache**: `Yes`
- Enable **Data Cache**: `Yes`
- Prefetch: `Disabled` (optional)

---

## Power Regulator Configuration

### Voltage Scaling

| Parameter             | Value            | Description          |
| --------------------- | ---------------- | -------------------- |
| **Voltage Scale**     | **Scale 1 Mode** | Maximum performance  |
| **Max Frequency**     | 168 MHz          | Full speed operation |
| **Power Consumption** | High             | Best performance     |

**Voltage Scale Modes:**
- **Scale 1**: Up to 168 MHz (high performance)
- **Scale 2**: Up to 144 MHz (medium performance)
- **Scale 3**: Up to 120 MHz (low power)

**STM32CubeMX Setting:**
- Navigate to **System Core → PWR**
- Set **Regulator Voltage Scale** to: `Scale 1`

---

## Peripheral Clock Enable Summary

The following peripherals must have their clocks enabled:

### AHB1 Peripherals (168 MHz)

| Peripheral | Clock Enable Macro     | Usage                      |
| ---------- | ---------------------- | -------------------------- |
| GPIOA      | `RCC_AHB1Periph_GPIOA` | Timers, USART1, SPI        |
| GPIOB      | `RCC_AHB1Periph_GPIOB` | Timers, USART3, SPI2       |
| GPIOC      | `RCC_AHB1Periph_GPIOC` | Thyristor outputs, ADC     |
| GPIOD      | `RCC_AHB1Periph_GPIOD` | FSMC, FPGA control         |
| GPIOE      | `RCC_AHB1Periph_GPIOE` | FSMC                       |
| GPIOF      | `RCC_AHB1Periph_GPIOF` | FSMC, AD7606 control       |
| GPIOG      | `RCC_AHB1Periph_GPIOG` | FSMC, FPGA control         |
| DMA1       | `RCC_AHB1Periph_DMA1`  | USART3 (not used for SPI2) |
| DMA2       | `RCC_AHB1Periph_DMA2`  | USART1, ADC1               |

### AHB3 Peripherals (168 MHz)

| Peripheral | Clock Enable Macro    | Usage                  |
| ---------- | --------------------- | ---------------------- |
| FSMC       | `RCC_AHB3Periph_FSMC` | AD7606, FPGA interface |

### APB2 Peripherals (84 MHz base, 168 MHz timers)

| Peripheral | Clock Enable Macro      | Usage                        |
| ---------- | ----------------------- | ---------------------------- |
| TIM9       | `RCC_APB2Periph_TIM9`   | External sync capture        |
| USART1     | `RCC_APB2Periph_USART1` | RS-232 debug serial          |
| ADC1       | `RCC_APB2Periph_ADC1`   | Internal temperature sensor  |
| SPI1       | `RCC_APB2Periph_SPI1`   | Not used (SPI2 used instead) |

### APB1 Peripherals (42 MHz base, 84 MHz timers)

| Peripheral | Clock Enable Macro      | Usage                           |
| ---------- | ----------------------- | ------------------------------- |
| TIM2       | `RCC_APB1Periph_TIM2`   | Firing angle delay timer        |
| TIM3       | `RCC_APB1Periph_TIM3`   | ADC trigger @ 5 kHz             |
| TIM4       | `RCC_APB1Periph_TIM4`   | Pulse generation                |
| TIM5       | `RCC_APB1Periph_TIM5`   | Sync capture + free-run counter |
| USART3     | `RCC_APB1Periph_USART3` | RS-485 industrial               |
| SPI2       | `RCC_APB1Periph_SPI2`   | W5200 Ethernet                  |
| RTC        | `RCC_APB1Periph_RTC`    | Real-time clock                 |
| PWR        | `RCC_APB1Periph_PWREN`  | Power control                   |

---

## STM32CubeMX Step-by-Step Configuration

### Step 1: Create New Project

1. Open **STM32CubeMX**
2. Click **New Project**
3. Select MCU: **STM32F407VGTx** (LQFP100 package)
4. Click **Start Project**

### Step 2: Configure RCC (Clock Source)

1. Navigate to **Pinout & Configuration** tab
2. Expand **System Core** → Click **RCC**
3. Set **High Speed Clock (HSE)** to: `Crystal/Ceramic Resonator`
4. Set **Low Speed Clock (LSE)** to: `Disable` (or `Crystal/Ceramic Resonator` if using external 32.768 kHz for RTC)
5. Leave **Master Clock Output 1 (MCO1)** and **MCO2** as `Disable`

### Step 3: Configure Clock Tree

1. Click on **Clock Configuration** tab
2. Configure as follows:

**Input Section:**
- HSE: `25 MHz` (ensure checkbox is checked)

**PLL Configuration:**
- PLL Source Mux: `HSE`
- /M (DIVM): `25`
- ×N (MULN): `336`
- /P (DIVP): `2` (for System Clock)
- /Q (DIVQ): `7` (for USB/SDIO)

**System Clock Section:**
- System Clock Mux: `PLLCLK`
- SYSCLK: Should show `168 MHz`

**Bus Clocks:**
- AHB Prescaler: `/1` → HCLK = `168 MHz`
- APB1 Prescaler: `/4` → PCLK1 = `42 MHz`
- APB2 Prescaler: `/2` → PCLK2 = `84 MHz`

**Cortex System Timer:**
- HCLK to Cortex System Timer: `/8` (default, for SysTick)

**Timers:**
- APB1 Timer Clocks (multiplier ×2): `84 MHz`
- APB2 Timer Clocks (multiplier ×2): `168 MHz`

### Step 4: Verify Clock Configuration

Ensure the following frequencies are displayed:

| Clock Signal      | Frequency | Verification             |
| ----------------- | --------- | ------------------------ |
| SYSCLK            | 168 MHz   | ✓ Maximum for STM32F407  |
| HCLK              | 168 MHz   | ✓ Core, DMA, AHB bus     |
| FCLK (Cortex)     | 168 MHz   | ✓ Cortex-M4 core         |
| APB1 Timer Clocks | 84 MHz    | ✓ TIM2, TIM3, TIM4, TIM5 |
| PCLK1 (APB1)      | 42 MHz    | ✓ Max for APB1           |
| APB2 Timer Clocks | 168 MHz   | ✓ TIM9                   |
| PCLK2 (APB2)      | 84 MHz    | ✓ USART1, ADC1           |
| USB/SDIO/RNG      | 48 MHz    | ✓ Required for USB       |

### Step 5: Configure SYS (System)

1. Navigate to **Pinout & Configuration** tab
2. Expand **System Core** → Click **SYS**
3. Set **Debug** to: `Serial Wire` (for debugging with ST-Link)
4. Set **Timebase Source** to: `SysTick` (default)

### Step 6: Configure Flash and Power

1. Navigate to **System Core** → **FLASH**
   - Latency: `5 Wait States`
   - Instruction Cache: `Enabled`
   - Data Cache: `Enabled`

2. Navigate to **System Core** → **PWR**
   - Regulator Voltage Scale: `Scale 1` (if available in GUI)

### Step 7: Configure NVIC (Interrupt Priorities)

⚠️ **Critical for SCR Controller:**

| Interrupt | Priority | Sub-Priority | Description                            |
| --------- | -------- | ------------ | -------------------------------------- |
| TIM2      | 0        | 0            | Highest - Firing angle timing critical |
| TIM9      | 0        | 0            | External sync capture                  |
| TIM4      | 0        | 1            | Pulse generation                       |
| TIM5      | 2        | 2            | Sync capture + main loop               |
| TIM3      | 3        | 3            | ADC trigger                            |
| USART1    | 1        | 1            | RS-232 serial                          |
| USART3    | 1        | 1            | RS-485 serial                          |
| DMA2      | 1        | 1            | Serial DMA                             |

**STM32CubeMX Setting:**
- Navigate to **System Core** → **NVIC**
- Set priorities as shown above after enabling each peripheral

### Step 8: Generate Code

1. Click **Project** → **Settings**
2. Set **Project Name**: `SCR_Controller`
3. Set **Toolchain/IDE**: Your preference (MDK-ARM, STM32CubeIDE, etc.)
4. Click **Code Generator** tab:
   - ☑ Generate peripheral initialization as pair of '.c/.h' files
   - ☑ Keep user code when regenerating
5. Click **OK**
6. Click **Project** → **Generate Code**

---

## Clock Initialization Code Reference

The generated code will create `system_stm32f4xx.c` with the following key values:

```c
// PLL Parameters (from system_stm32f4xx.c)
#define PLL_M      25   // Input divider
#define PLL_N      336  // VCO multiplier
#define PLL_P      2    // System clock divider
#define PLL_Q      7    // USB/SDIO divider

// System Clock
uint32_t SystemCoreClock = 168000000;  // 168 MHz
```

**Key Initialization Functions:**

```c
void SystemInit(void)
{
    // Enable FPU
    SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));
    
    // Enable HSE
    RCC->CR |= RCC_CR_HSEON;
    
    // Wait for HSE ready
    while((RCC->CR & RCC_CR_HSERDY) == 0);
    
    // Configure voltage regulator
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR |= PWR_CR_VOS;  // Scale 1
    
    // Configure bus prescalers
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;   // AHB = /1
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;  // APB2 = /2
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;  // APB1 = /4
    
    // Configure PLL
    RCC->PLLCFGR = PLL_M | (PLL_N << 6) | (((PLL_P >> 1) -1) << 16) |
                   (RCC_PLLCFGR_PLLSRC_HSE) | (PLL_Q << 24);
    
    // Enable PLL
    RCC->CR |= RCC_CR_PLLON;
    
    // Wait for PLL ready
    while((RCC->CR & RCC_CR_PLLRDY) == 0);
    
    // Configure Flash latency
    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_5WS;
    
    // Select PLL as system clock
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    
    // Wait for PLL to be system clock
    while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}
```

---

## Clock Distribution to Timers

### APB1 Timers (TIM2-TIM7, TIM12-TIM14)

**Configuration:**
```
PCLK1 = 42 MHz
APB1 Prescaler = 4 (not 1)
→ Timer Clock Multiplier = ×2
→ Timer Clock = 42 MHz × 2 = 84 MHz
```

**Used Timers:**
- **TIM2**: 84 MHz base, firing angle delay
- **TIM3**: 84 MHz base, Period = 16799 → 5 kHz ADC trigger
- **TIM4**: 84 MHz base, Prescaler = 6 → 12 MHz pulse timer
- **TIM5**: 84 MHz base, free-running 32-bit counter

### APB2 Timers (TIM1, TIM8-TIM11)

**Configuration:**
```
PCLK2 = 84 MHz
APB2 Prescaler = 2 (not 1)
→ Timer Clock Multiplier = ×2
→ Timer Clock = 84 MHz × 2 = 168 MHz
```

**Used Timers:**
- **TIM9**: 168 MHz base, Prescaler = 20 → 8.4 MHz for external sync

---

## Verification and Testing

### 1. Verify System Clock

Add this code to verify the clock is running at 168 MHz:

```c
#include "stm32f4xx.h"
#include <stdio.h>

void Check_Clock_Config(void)
{
    uint32_t sysclk, hclk, pclk1, pclk2;
    uint32_t pllm, plln, pllp, pllq;
    
    // Read PLL configuration
    pllm = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;
    plln = (RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6;
    pllp = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >> 16) + 1) * 2;
    pllq = (RCC->PLLCFGR & RCC_PLLCFGR_PLLQ) >> 24;
    
    printf("PLL Configuration:\n");
    printf("  PLL_M = %lu (should be 25)\n", pllm);
    printf("  PLL_N = %lu (should be 336)\n", plln);
    printf("  PLL_P = %lu (should be 2)\n", pllp);
    printf("  PLL_Q = %lu (should be 7)\n", pllq);
    
    // Calculate clocks
    SystemCoreClockUpdate();
    sysclk = SystemCoreClock;
    
    printf("\nSystem Clocks:\n");
    printf("  SYSCLK = %lu Hz (should be 168000000)\n", sysclk);
    printf("  HCLK   = %lu Hz (should be 168000000)\n", sysclk);
    printf("  PCLK1  = %lu Hz (should be 42000000)\n", sysclk / 4);
    printf("  PCLK2  = %lu Hz (should be 84000000)\n", sysclk / 2);
}
```

### 2. Verify Timer Clocks

```c
void Check_Timer_Clocks(void)
{
    // TIM5 should count 84,000 times per millisecond
    TIM5->CR1 |= TIM_CR1_CEN;  // Enable counter
    uint32_t start = TIM5->CNT;
    Delay_us(1000);  // 1 ms delay
    uint32_t end = TIM5->CNT;
    uint32_t counts = end - start;
    
    printf("TIM5 counts in 1ms: %lu (should be ~84000)\n", counts);
}
```

### 3. Check HSE Status

```c
void Check_HSE_Status(void)
{
    if (RCC->CR & RCC_CR_HSERDY)
    {
        printf("HSE: Ready (25 MHz crystal active)\n");
    }
    else
    {
        printf("HSE: NOT Ready - Check crystal!\n");
    }
    
    if (RCC->CR & RCC_CR_PLLRDY)
    {
        printf("PLL: Locked\n");
    }
    else
    {
        printf("PLL: NOT Locked!\n");
    }
}
```

---

## Troubleshooting

### Issue 1: System Clock Not Reaching 168 MHz

**Symptoms:**
- Code runs slower than expected
- Timer frequencies incorrect

**Possible Causes:**
1. **HSE frequency mismatch**
   - Check that your crystal is actually 25 MHz
   - Measure with oscilloscope on OSC_OUT pin
   
2. **PLL not locked**
   - Add startup delay after HSE enable
   - Check HSE_STARTUP_TIMEOUT value

3. **Flash latency too low**
   - Must be 5 WS for 168 MHz
   - Check FLASH->ACR register

**Solution:**
```c
// Verify PLL is system clock source
if ((RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_PLL)
{
    printf("System clock source: PLL (correct)\n");
}
else
{
    printf("System clock source: NOT PLL (ERROR)\n");
}
```

### Issue 2: HSE Fails to Start

**Symptoms:**
- System runs on HSI (16 MHz internal oscillator)
- HSE_STARTUP_TIMEOUT expires

**Possible Causes:**
1. No crystal connected or wrong value
2. Loading capacitors incorrect (typically 10-20 pF)
3. Crystal damaged or oscillator disabled

**Solution:**
- Check hardware: crystal between OSC_IN/OSC_OUT
- Verify load capacitors to ground
- Try increasing HSE_STARTUP_TIMEOUT

### Issue 3: APB1 Clock Too High

**Symptoms:**
- Peripheral malfunctions
- USART baud rate errors

**Possible Causes:**
- APB1 prescaler set to /2 instead of /4
- APB1 cannot exceed 42 MHz

**Solution:**
```c
// Check APB1 prescaler
uint32_t ppre1 = (RCC->CFGR & RCC_CFGR_PPRE1) >> 10;
printf("APB1 Prescaler bits: 0x%lX (should be 0x5 for /4)\n", ppre1);
```

---

## Summary Checklist

Use this checklist when configuring STM32CubeMX:

- [ ] **RCC Configuration**
  - [ ] HSE: Crystal/Ceramic Resonator, 25 MHz
  
- [ ] **Clock Tree Configuration**
  - [ ] PLL Source: HSE
  - [ ] PLL_M = 25
  - [ ] PLL_N = 336
  - [ ] PLL_P = 2
  - [ ] PLL_Q = 7
  - [ ] System Clock Mux: PLLCLK
  
- [ ] **Bus Prescalers**
  - [ ] AHB Prescaler: /1 → HCLK = 168 MHz
  - [ ] APB1 Prescaler: /4 → PCLK1 = 42 MHz
  - [ ] APB2 Prescaler: /2 → PCLK2 = 84 MHz
  
- [ ] **Verify Frequencies**
  - [ ] SYSCLK = 168 MHz
  - [ ] HCLK = 168 MHz
  - [ ] APB1 Timer Clocks = 84 MHz
  - [ ] APB2 Timer Clocks = 168 MHz
  - [ ] USB Clock = 48 MHz
  
- [ ] **Flash Configuration**
  - [ ] Latency = 5 Wait States
  - [ ] Instruction Cache = Enabled
  - [ ] Data Cache = Enabled
  
- [ ] **Power Configuration**
  - [ ] Voltage Scale = Scale 1

---

## Reference Documents

| Document | Description                     |
| -------- | ------------------------------- |
| RM0090   | STM32F4xx Reference Manual      |
| DS8626   | STM32F407xx Datasheet           |
| AN4044   | Clock configuration for STM32F4 |
| AN4013   | STM32F4 firmware examples       |

---

## Appendix: Clock Frequencies Summary Table

| Clock Signal      | Source           | Prescaler | Frequency   | Usage                 |
| ----------------- | ---------------- | --------- | ----------- | --------------------- |
| **HSE**           | External Crystal | N/A       | 25 MHz      | PLL input             |
| **VCO**           | PLL × HSE/PLL_M  | ×336      | 336 MHz     | Internal PLL          |
| **SYSCLK**        | VCO / PLL_P      | ÷2        | **168 MHz** | System clock          |
| **HCLK**          | SYSCLK / AHB_PRE | ÷1        | **168 MHz** | AHB bus, core, memory |
| **FCLK**          | HCLK             | ÷1        | **168 MHz** | Cortex-M4 core        |
| **PCLK1**         | HCLK / APB1_PRE  | ÷4        | **42 MHz**  | APB1 peripherals      |
| **PCLK2**         | HCLK / APB2_PRE  | ÷2        | **84 MHz**  | APB2 peripherals      |
| **TIM2-7 CLK**    | PCLK1 × 2        | ×2        | **84 MHz**  | APB1 timers           |
| **TIM1,8-11 CLK** | PCLK2 × 2        | ×2        | **168 MHz** | APB2 timers           |
| **USB/SDIO**      | VCO / PLL_Q      | ÷7        | **48 MHz**  | USB OTG FS            |
| **FSMC CLK**      | HCLK (AHB3)      | ÷1        | **168 MHz** | External memory       |

---

## Document History

| Version | Date       | Author         | Description                                        |
| ------- | ---------- | -------------- | -------------------------------------------------- |
| 1.0     | 2025-10-10 | GitHub Copilot | Complete clock configuration guide for STM32CubeMX |

---

*This documentation was created from reverse-engineering the actual STM32F407 SCR bridge controller firmware source code (system_stm32f4xx.c).*
