# FPGA LED Driver Integration Guide

## Overview

The FPGA LED driver provides control of 3 status LEDs via the FPGA's FSMC Bank4 interface. These LEDs provide visual feedback for real-time task execution on the STM32F407ZG-CN board.

---

## Hardware Architecture

### LED Connection

```
STM32F407ZG
    ↓ (FSMC Bank4 - 16-bit parallel @ 0x6C000000)
FPGA Digital I/O Controller
    ↓ (GPIO outputs)
LED Driver Circuit
    ↓
┌────────────┐
│ LED 0 (Green) │ ← Task 1 heartbeat (5 kHz)
│ LED 1 (Yellow)│ ← Task 2 heartbeat (1 kHz)  
│ LED 2 (Red)   │ ← Task 3 heartbeat (100 Hz)
└────────────┘
```

### Memory Map

| Address    | Offset  | Register        | Description           |
| ---------- | ------- | --------------- | --------------------- |
| 0x6C000000 | 0       | FSMC Bank4 Base | FPGA digital I/O base |
| 0x6C00000E | 7 words | LED Control     | LED status register   |

**LED Control Register (16-bit):**
```
Bit 15-3: Reserved (unused)
Bit 2: LED 2 state (Task 3 - 100 Hz)
Bit 1: LED 1 state (Task 2 - 1 kHz)
Bit 0: LED 0 state (Task 1 - 5 kHz)
```

---

## LED Assignments

| LED ID    | Enum             | Task   | Frequency | Blink Rate   | Purpose               |
| --------- | ---------------- | ------ | --------- | ------------ | --------------------- |
| **LED 0** | `FPGA_LED_TASK1` | Task 1 | 5 kHz     | 1 Hz (500ms) | Running indicator     |
| **LED 1** | `FPGA_LED_TASK2` | Task 2 | 1 kHz     | 1 Hz (500ms) | Medium-freq indicator |
| **LED 2** | `FPGA_LED_TASK3` | Task 3 | 100 Hz    | 1 Hz (500ms) | Low-freq indicator    |

### LED Behavior

**Normal Operation:**
- All 3 LEDs blink at 1 Hz (synchronized 500ms ON, 500ms OFF)
- Indicates all tasks running correctly
- If any LED stops blinking → corresponding task has stopped/crashed

**Fault Scenarios:**
- LED 0 stops → Task 1 crash (critical - ADC/control loop failure)
- LED 1 stops → Task 2 crash (medium priority - regulation failure)
- LED 2 stops → Task 3 crash (low priority - communication/diagnostics failure)
- All LEDs stop → System crash or main loop hang

---

## API Reference

### Initialization

```c
#include "fpga_led_driver.h"

void setup(void)
{
    // Initialize FSMC Bank4 first
    MX_FSMC_Init();
    
    // Initialize LED driver (turns all LEDs OFF)
    FPGA_LED_Init();
}
```

### Individual LED Control

```c
// Turn LED on
FPGA_LED_Set(FPGA_LED_TASK1, true);

// Turn LED off
FPGA_LED_Set(FPGA_LED_TASK1, false);

// Toggle LED (used in task callbacks)
FPGA_LED_Toggle(FPGA_LED_TASK1);

// Get current LED state
bool is_on = FPGA_LED_GetState(FPGA_LED_TASK1);
```

### Bulk Operations

```c
// Turn all LEDs on
FPGA_LED_AllOn();

// Turn all LEDs off
FPGA_LED_AllOff();

// Set multiple LEDs at once (most efficient)
fpga_led_state_t led_state = {
    .task1_led = true,
    .task2_led = false,
    .task3_led = true
};
FPGA_LED_SetAll(&led_state);

// Get all LED states
fpga_led_state_t current_state;
FPGA_LED_GetAll(&current_state);
```

---

## Integration with Tasks

### Task 1 (5 kHz - High-Frequency Control)

```c
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    static int task1_counter = 0;
    
    // ... Task 1 control logic ...
    
    // Toggle LED at 1 Hz (2500 cycles * 200µs = 500ms)
    if (++task1_counter >= 2500) {
        FPGA_LED_Toggle(FPGA_LED_TASK1);
        task1_counter = 0;
    }
}
```

### Task 2 (1 kHz - Medium-Frequency Control)

```c
void Task2_Callback(void)  // TIM3 IRQ
{
    static int task2_counter = 0;
    
    // ... Task 2 control logic ...
    
    // Toggle LED at 1 Hz (500 cycles * 1ms = 500ms)
    if (++task2_counter >= 500) {
        FPGA_LED_Toggle(FPGA_LED_TASK2);
        task2_counter = 0;
    }
}
```

### Task 3 (100 Hz - Low-Frequency Control)

```c
void Task3_Callback(void)  // TIM4 IRQ
{
    static int task3_counter = 0;
    
    // ... Task 3 control logic ...
    
    // Toggle LED at 1 Hz (50 cycles * 10ms = 500ms)
    if (++task3_counter >= 50) {
        FPGA_LED_Toggle(FPGA_LED_TASK3);
        task3_counter = 0;
    }
}
```

---

## Performance Characteristics

### Memory Access Timing

**FSMC Bank4 Configuration:**
- Address Setup Time: 6 HCLK cycles (35.7 ns @ 168 MHz)
- Data Setup Time: 21 HCLK cycles (125 ns @ 168 MHz)
- **Total write time: ~160 ns**

**Overhead per LED update:**
- Function call: ~10 cycles (60 ns)
- Bit manipulation: ~5 cycles (30 ns)
- FSMC write: ~27 cycles (160 ns)
- **Total: ~42 cycles (250 ns)**

### Task Impact

| Task   | Frequency | LED Update Interval | Overhead |
| ------ | --------- | ------------------- | -------- |
| Task 1 | 5 kHz     | Every 500 ms        | 0.00005% |
| Task 2 | 1 kHz     | Every 500 ms        | 0.00025% |
| Task 3 | 100 Hz    | Every 500 ms        | 0.0025%  |

**Conclusion:** LED updates have **negligible** impact on task timing (<0.003% combined).

---

## Debugging with LEDs

### Visual Diagnostics

**Expected Pattern (Normal):**
```
LED 0: ▓▓▓▓░░░░▓▓▓▓░░░░  (1 Hz blink)
LED 1: ▓▓▓▓░░░░▓▓▓▓░░░░  (1 Hz blink)
LED 2: ▓▓▓▓░░░░▓▓▓▓░░░░  (1 Hz blink)
```

**Task 1 Crash (ADC/Control Loop):**
```
LED 0: ▓▓▓▓ [STUCK ON/OFF]
LED 1: ▓▓▓▓░░░░▓▓▓▓░░░░  (still blinking)
LED 2: ▓▓▓▓░░░░▓▓▓▓░░░░  (still blinking)
```

**System Hang:**
```
LED 0: [STUCK]
LED 1: [STUCK]
LED 2: [STUCK]
```

### Debug Code Examples

```c
// Test all LEDs at startup
void LED_SelfTest(void)
{
    // All on
    FPGA_LED_AllOn();
    HAL_Delay(500);
    
    // All off
    FPGA_LED_AllOff();
    HAL_Delay(500);
    
    // Sequential test
    for (int i = 0; i < 3; i++) {
        FPGA_LED_Set(i, true);
        HAL_Delay(200);
        FPGA_LED_Set(i, false);
    }
}

// Error indication pattern (rapid blink)
void LED_ErrorPattern(void)
{
    for (int i = 0; i < 10; i++) {
        FPGA_LED_AllOn();
        HAL_Delay(100);
        FPGA_LED_AllOff();
        HAL_Delay(100);
    }
}
```

---

## Migration from GPIO LEDs

### Old Code (Direct GPIO)

```c
// Old implementation using GPIO pins
#define LED_TASK1_PIN  GPIO_PIN_6
#define LED_TASK2_PIN  GPIO_PIN_7
#define LED_TASK3_PIN  GPIO_PIN_8

void Task1_Callback(void)
{
    HAL_GPIO_TogglePin(GPIOC, LED_TASK1_PIN);  // ❌ Wrong pins!
}
```

### New Code (FPGA LEDs)

```c
// New implementation using FPGA-controlled LEDs
#include "fpga_led_driver.h"

void Task1_Callback(void)
{
    FPGA_LED_Toggle(FPGA_LED_TASK1);  // ✅ Correct FPGA interface
}
```

**Why the change?**
- Original hardware uses FPGA for all digital I/O (LEDs included)
- GPIO pins PC6/7/8 may not be connected to LEDs on actual board
- FPGA interface matches legacy SCR controller design

---

## Troubleshooting

### LEDs Not Responding

1. **Check FSMC Bank4 initialization:**
   ```c
   // Verify in MX_FSMC_Init()
   hsram4.Init.NSBank = FSMC_NORSRAM_BANK4;
   Timing.AddressSetupTime = 6;
   Timing.DataSetupTime = 21;
   ```

2. **Verify memory-mapped write:**
   ```c
   // Test direct write
   *(volatile uint16_t*)0x6C00000E = 0x0007;  // All LEDs ON
   HAL_Delay(1000);
   *(volatile uint16_t*)0x6C00000E = 0x0000;  // All LEDs OFF
   ```

3. **Check initialization order:**
   - FSMC must be initialized BEFORE LED driver
   - Call `FPGA_LED_Init()` after `MX_FSMC_Init()`

### Only Some LEDs Work

- **Check FPGA firmware:** Verify LED control logic in FPGA
- **Check hardware connections:** Verify LED driver circuit
- **Test bit pattern:** Try writing `0x0001`, `0x0002`, `0x0004` individually

### LEDs Flicker/Unstable

- **Check FSMC timing:** May need slower timing parameters
- **Add delays:** Increase NOP count in `FPGA_LED_WriteRegister()`
- **Check power supply:** Verify FPGA and LED driver have stable power

---

## Files Modified

| File                | Location    | Purpose                   |
| ------------------- | ----------- | ------------------------- |
| `fpga_led_driver.h` | `Core/Inc/` | LED driver header         |
| `fpga_led_driver.c` | `Core/Src/` | LED driver implementation |
| `main.c`            | `Core/Src/` | Task integration          |

---

## Summary

✅ **3 status LEDs** controlled via FPGA (FSMC Bank4)  
✅ **1 Hz blink rate** for all tasks (synchronized)  
✅ **Negligible overhead** (<0.003% CPU time)  
✅ **Visual diagnostics** for task execution monitoring  
✅ **Drop-in replacement** for incorrect GPIO LED code  

The FPGA LED driver provides a clean, efficient interface for status indication that matches the original hardware design of the SCR bridge controller.
