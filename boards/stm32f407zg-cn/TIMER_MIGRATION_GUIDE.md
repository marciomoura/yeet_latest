# Timer Refactoring - Migration Guide
## From Legacy Control Code to Clean Periodic Tasks

**Date:** October 14, 2025  
**Version:** V2.0.0

---

## Overview

The timer system has been completely refactored from the legacy thyristor control application to a clean, modern structure with three periodic tasks at fixed rates.

---

## Changes Summary

### What Was Removed

All legacy control application code has been removed:
- ❌ Pulse generation for thyristor control (`Generate_Pulse`, `Pulse_Data`)
- ❌ Grid synchronization capture (TIM5, TIM9)
- ❌ Firing angle control (`Alpha`, `Set_Alpha_Delay`)
- ❌ Phase tracking (`Syn_A/B/C`, `Phase_Offset`)
- ❌ Optical fiber communication flags
- ❌ All Chinese comments and outdated documentation

### What Was Kept

- ✅ **TIM3:** 200μs ADC sampling task (5 kHz) - **UNCHANGED FUNCTIONALITY**
- ✅ `AD()` function call for AD7606 sampling
- ✅ SPL (Standard Peripheral Library) compatibility

### What Was Added

- ✅ **TIM4:** 1ms fast control task (1 kHz) - **NEW**
- ✅ **TIM2:** 10ms slow monitoring task (100 Hz) - **NEW**
- ✅ Clean, English-documented code
- ✅ Task execution counters
- ✅ Modern coding style

---

## New Timer Configuration

| Timer    | Period | Frequency | Priority     | Purpose                 |
| -------- | ------ | --------- | ------------ | ----------------------- |
| **TIM3** | 200μs  | 5 kHz     | High (P:1)   | ADC sampling via AD7606 |
| **TIM4** | 1ms    | 1 kHz     | Medium (P:2) | Fast control loop       |
| **TIM2** | 10ms   | 100 Hz    | Low (P:3)    | Slow monitoring         |

### Timer Clock Configuration

All timers use **APB1 bus** @ **84 MHz**:

```
TIM3: Prescaler=0    (84 MHz), Period=16799  → 5 kHz
TIM4: Prescaler=83   (1 MHz),  Period=999    → 1 kHz  
TIM2: Prescaler=8399 (10 kHz), Period=99     → 100 Hz
```

---

## File Structure

### New Files Created

```
TIM.C       - Refactored timer implementation (365 lines)
TIM.h       - Clean header file (58 lines)
TIM.C.bak   - Backup of original (658 lines)
```

### Changes to TIM.C

**Old structure (658 lines):**
```c
// Mixed timer init, pulse control, synchronization
Init_Pulse_Pin()
Generate_Pulse()
First_Rising_Edge()
Set_Alpha_Delay()
TIM5_Init()  // Grid sync capture
TIM2_Init()  // Firing angle control
TIM3_Init()  // ADC sampling
TIM4_Init()  // Pulse generation
TIM9_Init()  // FPGA sync
TIM5_CC1/2/4()  // Phase A/B/C handlers
TIM4_Update()  // Pulse sequencing
// + 5 interrupt handlers
```

**New structure (365 lines):**
```c
// Clean separation of concerns
TIM3_Config()  // 200us ADC task
TIM4_Config()  // 1ms control task
TIM2_Config()  // 10ms monitor task
Task_200us_ADC_Sampling()
Task_1ms_FastControl()
Task_10ms_SlowMonitoring()
// + 3 interrupt handlers
// + 3 counter getters
```

### Changes to TIM.h

**Old structure:**
```c
void TIMx_Init(void);
void TIM3_Init(void);
void TIM4_Init(void);
void TIM7_Init(void);  // Not implemented
```

**New structure:**
```c
void TIMx_Init(void);
uint32_t TIM_Get_200us_Counter(void);
uint32_t TIM_Get_1ms_Counter(void);
uint32_t TIM_Get_10ms_Counter(void);
```

---

## API Reference

### Initialization

```c
void TIMx_Init(void);
```

**Description:** Initialize all three timers and start periodic interrupts.

**Usage:**
```c
int main(void)
{
    // System initialization
    SystemInit();
    
    // Initialize ADC
    PSRAM_Init();
    AD7606_Init_Pin();
    ConfigAD7606();
    
    // Initialize timers
    TIMx_Init();  // Starts all 3 tasks
    
    while(1) {
        // Main loop
    }
}
```

### Task Counters

```c
uint32_t TIM_Get_200us_Counter(void);
uint32_t TIM_Get_1ms_Counter(void);
uint32_t TIM_Get_10ms_Counter(void);
```

**Description:** Get execution counters for each task (useful for debugging and timing checks).

**Usage:**
```c
// Check if tasks are running
if (TIM_Get_200us_Counter() > 0) {
    // ADC task is active
}

// Measure time between events
uint32_t start = TIM_Get_1ms_Counter();
// ... do something ...
uint32_t elapsed_ms = TIM_Get_1ms_Counter() - start;
```

---

## Migration Steps

### 1. Remove Old Dependencies

**Files to check:**
- `pub.h` / `pub.c` - Remove if only used for pulse control
- `syn.h` / `syn.c` - Remove synchronization code
- `optical.h` / `optical.c` - Remove if only for legacy comm

**Example:**
```c
// OLD - Remove these includes from main.c
#include "syn.h"
#include "optical.h"

// NEW - Keep only essential
#include "TIM.h"
#include "AI.h"
```

### 2. Update Variable References

**Remove global variables** that are no longer defined:

```c
// ❌ REMOVED - Do not use these
extern unsigned int Period;
extern unsigned int Period_av;
extern float Alpha;
extern unsigned char Pulse_Enabled_Flag;
extern unsigned char SynSource;
```

### 3. Implement Your Control Tasks

#### 200μs Task (ADC Sampling)

**Location:** `Task_200us_ADC_Sampling()` in TIM.C

```c
static void Task_200us_ADC_Sampling(void)
{
    task_200us_counter++;
    
    /* Keep existing ADC read */
    AD();  // Calls AD7606 read/start
    
    /* ADD YOUR CODE HERE */
    // Example: Fast signal processing
    // - Digital filtering
    // - Peak detection
    // - Protection monitoring
}
```

#### 1ms Task (Fast Control)

**Location:** `Task_1ms_FastControl()` in TIM.C

```c
static void Task_1ms_FastControl(void)
{
    task_1ms_counter++;
    
    /* ADD YOUR CONTROL CODE HERE */
    // Example: 
    // - PID control loops
    // - State machine updates
    // - Fast feedback control
    // - Setpoint tracking
}
```

#### 10ms Task (Slow Monitoring)

**Location:** `Task_10ms_SlowMonitoring()` in TIM.C

```c
static void Task_10ms_SlowMonitoring(void)
{
    task_10ms_counter++;
    
    /* ADD YOUR MONITORING CODE HERE */
    // Example:
    // - Temperature readings
    // - Status LED updates
    // - Communication handling
    // - Diagnostics and logging
}
```

### 4. Update CMakeLists.txt (If Needed)

The file list should already be correct, but verify:

```cmake
# Should still include:
${CMAKE_CURRENT_SOURCE_DIR}/Core/NSD_Driver/TIM.C
${CMAKE_CURRENT_SOURCE_DIR}/Core/NSD_Driver/AI.c
${CMAKE_CURRENT_SOURCE_DIR}/Core/NSD_Driver/AD7606.c
# ... other files
```

---

## Testing Checklist

### Step 1: Compilation Test

```bash
cmake --build --preset=embedded-cn-grid-debug
```

**Expected result:** Clean build with no errors.

### Step 2: Basic Functionality Test

Add test code to verify timers are running:

```c
// In main.c, after TIMx_Init()
volatile uint32_t last_200us = 0;
volatile uint32_t last_1ms = 0;
volatile uint32_t last_10ms = 0;

while(1)
{
    uint32_t cnt_200us = TIM_Get_200us_Counter();
    uint32_t cnt_1ms = TIM_Get_1ms_Counter();
    uint32_t cnt_10ms = TIM_Get_10ms_Counter();
    
    if (cnt_200us != last_200us) {
        // 200us task is running
        last_200us = cnt_200us;
    }
    
    if (cnt_1ms != last_1ms) {
        // 1ms task is running
        last_1ms = cnt_1ms;
    }
    
    if (cnt_10ms != last_10ms) {
        // 10ms task is running
        last_10ms = cnt_10ms;
    }
}
```

### Step 3: ADC Test

Verify AD7606 sampling still works:

```c
// In Task_200us_ADC_Sampling(), after AD()
extern unsigned short int ADC_Data[8][300];
extern int AI_Times;

// Check that data is being acquired
if (AI_Times > 100) {
    // ADC buffer is filling up - working!
}
```

### Step 4: Timing Verification

Use oscilloscope or logic analyzer:
- TIM3: Toggle GPIO every interrupt → should see 5 kHz square wave
- TIM4: Toggle GPIO every interrupt → should see 1 kHz square wave
- TIM2: Toggle GPIO every interrupt → should see 100 Hz square wave

---

## Performance Analysis

### CPU Load Estimation

```
Task execution times (estimated):
- 200μs task: ~15μs  (ADC read + processing)
- 1ms task:   ~50μs  (control algorithms - user code)
- 10ms task:  ~100μs (monitoring - user code)

CPU load calculation:
- 200μs task: 15μs × 5000/s = 75,000 μs/s = 7.5%
- 1ms task:   50μs × 1000/s = 50,000 μs/s = 5.0%
- 10ms task:  100μs × 100/s = 10,000 μs/s = 1.0%
------------------------------------------------------
Total periodic load:                      = 13.5%

Remaining CPU: 86.5% available for main loop
```

### Memory Usage

```
Stack usage (per task):
- 200μs: ~256 bytes
- 1ms:   ~512 bytes (estimate)
- 10ms:  ~512 bytes (estimate)

Global variables:
- Old: ~50+ variables, unclear usage
- New: 3 counters = 12 bytes

Code size:
- Old: 658 lines
- New: 365 lines (-45%)
```

---

## Troubleshooting

### Issue: Build Errors

**Problem:** Undefined references to removed functions/variables

**Solution:**
1. Check for references to old variables:
   ```bash
   grep -r "Period_av" .
   grep -r "Pulse_Data" .
   grep -r "SynSource" .
   ```

2. Remove or replace with new code

### Issue: ADC Not Working

**Problem:** No ADC data after refactoring

**Solution:**
1. Verify `AD()` is still called in `Task_200us_ADC_Sampling()`
2. Check TIM3 is actually running:
   ```c
   uint32_t counter = TIM_Get_200us_Counter();
   // Should increment continuously
   ```
3. Verify AD7606 initialization in main()

### Issue: Task Not Executing

**Problem:** Counter not incrementing

**Solution:**
1. Check NVIC priority conflicts
2. Verify timer is enabled: `TIM_Cmd(TIMx, ENABLE);`
3. Use debugger breakpoint in ISR to confirm entry

---

## Code Quality Improvements

### Before (Legacy Code)

```c
/************************************************************************
*Generate_Pulse                                                         *
*�������塣                                                             *
************************************************************************/
void Generate_Pulse(unsigned char Pulse)
{
	//2020.6.30. clear all pulse before generating new pulse
	GPIOC->BSRRH = GPIO_Pin_6;//+A, PC6
	// ... 100+ lines of pulse control ...
}
```

### After (Clean Code)

```c
/******************************************************************************
 * @brief  200us periodic task - ADC sampling
 * @param  None
 * @retval None
 * @note   Reads ADC data via AD7606 (keep existing functionality)
 ******************************************************************************/
static void Task_200us_ADC_Sampling(void)
{
    task_200us_counter++;
    AD();  // Clear, simple, documented
}
```

### Improvements

✅ **Documentation:** English Doxygen-style comments  
✅ **Naming:** Clear, descriptive function names  
✅ **Structure:** Separation of configuration and execution  
✅ **Modularity:** Easy to add/remove tasks  
✅ **Maintainability:** Less than half the original line count  
✅ **Readability:** No Chinese characters, modern formatting  

---

## Next Steps

1. **Add your control algorithms** to the 1ms and 10ms tasks
2. **Test thoroughly** with real hardware
3. **Optimize** task execution times if needed
4. **Document** your application-specific code
5. **Consider** moving to FreeRTOS if you need:
   - More tasks
   - Inter-task communication
   - Dynamic priorities
   - Semaphores/mutexes

---

## Rollback Procedure

If you need to revert to the old version:

```bash
# Restore backup
Copy-Item "TIM.C.bak" "TIM.C" -Force

# Or use git (if committed)
git checkout TIM.C
git checkout TIM.h
```

---

## Support

For questions or issues with the refactored code:
1. Check this migration guide first
2. Review the inline comments in TIM.C
3. Refer to the ISR_ANALYSIS.md and AD7606_SYNC_ANALYSIS.md documents

---

**Migration completed successfully!**  
**Original: 658 lines → Refactored: 365 lines (-45%)**  
**All Chinese comments translated**  
**Modern C coding standards applied**  
**Clean, maintainable structure**

