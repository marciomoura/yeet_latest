# Interrupt Service Routine (ISR) Analysis
## STM32F407ZG Control Application

**System Clock Configuration:**
- System Clock: 168 MHz (Cortex-M4)
- APB1 Peripheral Clock: 84 MHz (TIM2, TIM3, TIM4, TIM5)
- APB2 Peripheral Clock: 168 MHz (TIM9)

---

## Summary Table

| ISR Handler                  | Timer/Peripheral | Execution Rate          | Frequency   | Priority      | Main Task                         |
| ---------------------------- | ---------------- | ----------------------- | ----------- | ------------- | --------------------------------- |
| **TIM3_IRQHandler**          | TIM3             | **200 μs**              | **5 kHz**   | P:3, S:3      | AD conversion & data acquisition  |
| **TIM4_IRQHandler**          | TIM4             | **~4.76 ms** (variable) | **~210 Hz** | P:0, S:1      | Pulse generation control          |
| **TIM2_IRQHandler**          | TIM2             | Event-driven (α delay)  | Variable    | P:0, S:0      | Fire angle control delay          |
| **TIM1_BRK_TIM9_IRQHandler** | TIM9             | Event-driven (capture)  | Sync signal | P:0, S:0      | FPGA synchronization capture      |
| **TIM5_IRQHandler**          | TIM5             | Event-driven (capture)  | Sync signal | Not specified | Grid synchronization (3 channels) |

**Priority Legend:** P = Preemption Priority (0=highest), S = Sub Priority (0=highest)

---

## Detailed ISR Analysis

### 1. TIM3_IRQHandler - High-Speed ADC Sampling (5 kHz)
**File:** `TIM.C` line 635-640  
**Configuration:** `TIM3_Init()` line 401-426

```
Execution Rate: 200 microseconds (5 kHz)
Timer Setup:
  - Clock Source: APB1 = 84 MHz
  - Prescaler: 0 (÷1) → 84 MHz
  - Period: 16799
  - Interrupt Frequency: 84,000,000 / (1 × 16800) = 5000 Hz = 5 kHz
  - Period: 1/5000 = 200 μs
```

**Tasks Performed:**
- Calls `AD()` function - Analog-to-Digital conversion
- Reads ADC data from multiple channels
- Critical for real-time voltage and current measurements
- Highest frequency task in the system

**Priority:** Preemption=3, Sub=3 (Lower priority - allows critical pulse control to preempt)

---

### 2. TIM4_IRQHandler - Pulse Generation Control (~210 Hz)
**File:** `TIM.C` line 602-607  
**Configuration:** `TIM4_Init()` line 431-451

```
Execution Rate: Variable, approximately 4.76 ms (210 Hz) 
Timer Setup:
  - Clock Source: APB1 = 84 MHz
  - Prescaler: 6 (÷7) → 12 MHz
  - Period: Variable (ARR set dynamically in code)
  - Typical Period: 12,000,000 / Rising_Edge_Time[1]
```

**Tasks Performed:**
- Calls `TIM4_Update()` function
- Generates 6-pulse firing patterns for thyristor/SCR control
- Pattern: [3, 6, 0xC, 0x18, 0x30, 0x21] → 3-phase bridge pulses
- Controls GPIO pins PC6-PC9, PB0-PB1 for firing pulses
- Dynamic period adjustment based on grid synchronization
- Pulse sequence: +A/-C, +B/-C, -A/+B, +C/-A, -B/+C, +A/-B (60° intervals)

**Pulse Generation Logic:**
```
Period_av * 0.0238095 = ~1/42 of grid period
For 50Hz grid: Period_av ≈ 20ms → Rising_Edge_Time[1] ≈ 476μs per step
For 60Hz grid: Period_av ≈ 16.67ms → Rising_Edge_Time[1] ≈ 397μs per step
```

**Priority:** Preemption=0, Sub=1 (HIGHEST - critical for power control!)

---

### 3. TIM2_IRQHandler - Firing Angle (α) Delay Control
**File:** `TIM.C` line 509-513  
**Configuration:** `TIM2_Init()` line 325-341

```
Execution Rate: Event-driven (not periodic)
Timer Setup:
  - Clock Source: APB1 = 84 MHz
  - Prescaler: 0 (÷1) → 84 MHz
  - Period: Dynamic - set by Set_Alpha_Delay()
  - Triggered based on firing angle calculation
```

**Tasks Performed:**
- Calls `First_Rising_Edge()` function
- Implements phase-controlled rectifier delay (α angle)
- Calculates delay: dt = (Alpha + Alpha_C) × Period / 360°
- Triggers the first pulse in each 60° segment
- Resets pulse pointer and initializes TIM4 pulse sequence

**Timing Formula:**
```
For α = 30° on 50Hz grid:
dt = (30°) × (20000μs) / 360° = 1666.7 μs delay
```

**Priority:** Preemption=0, Sub=0 (HIGHEST - most critical timing!)

---

### 4. TIM1_BRK_TIM9_IRQHandler - FPGA Sync Capture
**File:** `TIM.C` line 521-597  
**Configuration:** `TIM9_Init()` line 351-397

```
Execution Rate: Event-driven (external sync signal capture)
Timer Setup:
  - Clock Source: APB2 = 168 MHz
  - Prescaler: 20 (÷21) → 8 MHz (but comment says 4MHz - verify!)
  - Mode: Input Capture on PA2 (TIM9_CH1)
  - Edge: Rising edge triggered
```

**Tasks Performed:**
- Captures FPGA synchronization signal timing
- Calculates grid period from external sync source
- Monitors sync signal validity (checks minimum period)
- Critical for grid-tied operation
- Updates `Period`, `SynSource`, and synchronization flags

**Priority:** Preemption=0, Sub=0 (HIGHEST - grid sync is critical!)

---

### 5. TIM5_IRQHandler - Three-Phase Grid Synchronization
**File:** `TIM.C` line 613-630  
**Configuration:** `TIM5_Init()` line 262-318

```
Execution Rate: Event-driven (3 capture channels)
Timer Setup:
  - Clock Source: APB1 = 84 MHz
  - Prescaler: 0 (÷1) → 84 MHz
  - Mode: Input Capture on 3 channels
  - Captures: CC1, CC2, CC4 (Phase A, B, C sync signals)
```

**Tasks Performed:**
- Three separate capture handlers:
  - `TIM5_CC1()` - Sync A capture (Phase A zero-crossing)
  - `TIM5_CC2()` - Sync B capture (Phase B zero-crossing)  
  - `TIM5_CC4()` - Sync C capture (Phase C zero-crossing)
- Calculates inter-phase timing:
  - `Syn_BA_Time`, `Syn_CB_Time`, `Syn_AC_Time`
- Updates grid frequency and period measurements
- Provides redundant synchronization sources

**Priority:** Not explicitly set in visible code

---

## Control Flow Architecture

### Real-Time Task Hierarchy

```
Level 0 (Highest Priority - Preemption=0):
├─ TIM2: Firing Angle Delay Control (α control)
├─ TIM9: External FPGA Sync Capture
└─ TIM4: Pulse Generation (Sub=1)

Level 3 (Lower Priority - Preemption=3):
└─ TIM3: ADC Sampling @ 5kHz

Event-Driven (No fixed rate):
└─ TIM5: Grid Sync Captures (3 phases)
```

### Timing Synchronization Chain

```
Grid Sync → TIM9/TIM5 Capture
    ↓
Period Calculation
    ↓
Set_Alpha_Delay() → Programs TIM2
    ↓
TIM2 Fires → First_Rising_Edge()
    ↓
Programs TIM4 with Rising_Edge_Time[1]
    ↓
TIM4 Updates → Generate 6 pulses @ 60° intervals
```

---

## Key Control Variables

| Variable                | Purpose                   | Units               |
| ----------------------- | ------------------------- | ------------------- |
| `Period`                | Grid period               | Timer ticks @ 84MHz |
| `Period_av`             | Averaged grid period      | Timer ticks         |
| `Alpha`                 | Firing angle              | Degrees (0-180°)    |
| `Alpha_C`               | Firing angle compensation | Degrees             |
| `Rising_Edge_Time[1-6]` | Pulse timing array        | Timer ticks @ 12MHz |
| `Pulse_Pointer`         | Current pulse index       | 0-11                |
| `Phase_Offset`          | Sync source offset        | 0, 2, or 4          |

---

## Pulse Pattern Details

**6-Pulse Bridge Configuration:**
```
Pulse_Data[12] = {3, 6, 0xC, 0x18, 0x30, 0x21, ...}

Bit mapping:
  Bit 0 (0x01): +A (PC6)
  Bit 1 (0x02): -C (PB1)
  Bit 2 (0x04): +B (PC7)
  Bit 3 (0x08): -A (PC9)
  Bit 4 (0x10): +C (PC8)
  Bit 5 (0x20): -B (PB0)

Sequence (60° intervals):
  0x03 = 0b000011 → +A, -C
  0x06 = 0b000110 → +B, -C
  0x0C = 0b001100 → +B, -A
  0x18 = 0b011000 → +C, -A
  0x30 = 0b110000 → +C, -B
  0x21 = 0b100001 → +A, -B
```

---

## Estimated CPU Load

**Assumptions:**
- ISR execution times (estimated):
  - TIM3: ~20μs (ADC read + processing)
  - TIM4: ~10μs (pulse generation + GPIO)
  - TIM2: ~15μs (first edge calculations)
  - TIM9/TIM5: ~5μs each (capture processing)

**Periodic Load (worst case):**
```
TIM3 @ 5kHz:  20μs × 5000 = 100,000 μs/s = 10.0% CPU
TIM4 @ 210Hz: 10μs × 210  =  2,100 μs/s =  0.2% CPU
TIM2 @ 360Hz: 15μs × 360  =  5,400 μs/s =  0.5% CPU (estimate: 6 pulses × 60Hz)
-----------------------------------------------------------
Total Periodic Load:                      ≈ 10.7% CPU
```

**Notes:**
- TIM9 and TIM5 are event-driven, low overhead (<1%)
- Main loop processing not included in this analysis
- Actual execution times may vary with optimization level
- Priority inversion risk: TIM3 (low priority) runs frequently

---

## Recommendations

1. **Monitor TIM3 Jitter:** At 5kHz with 10% CPU load, ADC timing is critical
2. **Priority Review:** Consider raising TIM3 priority if ADC timing precision is critical
3. **Verify Prescaler:** TIM9 comment says 4MHz but prescaler math gives 8MHz
4. **Add Watchdog:** Monitor ISR execution times for overruns
5. **DMA for ADC:** Consider using DMA instead of polled ADC in TIM3 ISR to reduce CPU load

---

## Application Type

**This is a 3-Phase Thyristor/SCR Bridge Rectifier Controller:**
- Grid-synchronized firing pulse generation
- Phase-controlled rectification (α control)
- 6-pulse bridge topology (3-phase)
- Real-time voltage/current monitoring
- Typical applications: Motor drives, HVDC, industrial rectifiers

---

**Analysis Date:** October 14, 2025  
**MCU:** STM32F407ZG @ 168MHz  
**Firmware:** NSD_Driver legacy code (SPL-based)
