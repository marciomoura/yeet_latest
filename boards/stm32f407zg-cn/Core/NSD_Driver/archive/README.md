# NSD_Driver Archive

**Date**: October 16, 2025  
**Action**: Application control logic removed from hardware driver layer

---

## Archived Files

These files contain excitation system control logic that was specific to the original SCR/thyristor bridge application:

### Control Modes
- **mode.c/h** - AVR/FCR/AER control mode switching
- **eIfd.c/h** - PID-based field current excitation regulator
- **ref.c/h** - Reference value tracking and step-up/down control

### Synchronization & Firing Control
- **syn.c/h** - Grid synchronization, phase tracking, and SCR pulse generation
- **limit.c/h** - Application-specific limit checking and protection logic

### Communication Protocols
- **command.c/h** - RS232 command protocol parser
- **net.c/h** - Network protocol with application-specific data structures
- **optical.c/h** - Optical fiber protocol with excitation control commands (1800+ lines)
- **optical_fiber.c/h** - FPGA/FSMC optical interface control

### Application State
- **pub.c/h** - 100+ global variables for excitation control (Alpha, Vfd_av, Ifd_av, control coefficients, mode flags, sync variables)

### Original Versions of Cleaned Files
- **AI.c.original** - Original version with RMS calculations, averaging, and control processing
- **AI.h.original** - Original header with control variables
- **para.c.original** - Original with full control initialization (when cleaned)
- **para.h.original** - Original header (when cleaned)

---

## What Was Removed

### From AI.c (570 → ~150 lines):
**Removed Functions**:
- `Init_Avg()` - Averaging state initialization
- `Cal_Avg()` - Running average calculation for control
- `RMS()` - RMS calculation for voltage/current
- `Cal_Arm_I()` - Arm current calculations
- `Get_Monitor()` - Network monitoring data preparation
- Complex `Cal_AD()` - Full ADC processing with RMS/filters

**Removed Variables**:
- `V[4][AD_Data_Num]` - Processed voltage arrays
- `I[4][AD_Data_Num]` - Processed current arrays
- `Va, Vb, Vc, Vabc` - RMS voltages
- `Ia, Ib, Ic, Iabc` - RMS currents
- `Vfd, Ifd` - Field voltage/current
- `Iap, Ian, Ibp, Ibn, Icp, Icn` - Individual arm currents
- All temperature buffers for averaging

**Kept Functions**:
- `Init_AD_Data()` - Simple buffer clear
- `Zero_Calibrate()` - DC offset calibration (useful utility)
- `Get_Medium_n()` / `Get_Medium_Value()` - Median filters (noise rejection)
- `AD()` - **Main ADC read with median filtering - THIS IS THE KEY FUNCTION**
- `Cal_AD()` - Empty placeholder for user application

**Kept Variables**:
- `AI_Times` - Sample counter
- `ADC_Data[8][AD_Data_Num]` - Raw ADC data [channel][sample]
- `AI_Count` - Buffer index
- `nI_Buf[3][2]` - Median filter buffers

---

## How to Use the Cleaned Drivers

### Accessing Raw ADC Data

```c
#include "AI.h"

// Initialize once
Init_AD_Data();

// In periodic task (e.g., 200us ISR)
AD();  // Reads all 8 channels, applies median filter

// Access data in your application
for (int ch = 0; ch < 8; ch++) {
    int raw_value = ADC_Data[ch][AI_Count];
    // Apply your own scaling, filtering, processing
    float voltage = raw_value * YOUR_SCALE_FACTOR;
}
```

### Implementing Custom Processing

Option 1: Directly access `ADC_Data[8][300]` buffer
```c
// Calculate your own averages, RMS, etc.
float calculate_my_rms(int channel) {
    float sum = 0.0;
    for (int i = 0; i < AD_Data_Num; i++) {
        float val = ADC_Data[channel][i];
        sum += val * val;
    }
    return sqrt(sum / AD_Data_Num);
}
```

Option 2: Implement your own `Cal_AD()` function
```c
// In your application code
void Cal_AD(void) {
    // Your custom ADC processing
    // State observers, filters, control algorithms, etc.
}
```

---

## Why Files Were Archived

**Separation of Concerns**:
- Hardware drivers should only interface with hardware
- Application control logic belongs in application layer
- This allows the drivers to be reused for any application

**What the cleaned drivers provide**:
- ✅ Raw ADC data acquisition (8 channels, 16-bit)
- ✅ Median filtering for noise rejection
- ✅ DC offset calibration
- ✅ Circular buffer management
- ✅ Hardware initialization

**What users must implement**:
- ❌ Control algorithms (PID, state feedback, etc.)
- ❌ Signal processing (RMS, averaging, FFT, etc.)
- ❌ Application protocols
- ❌ Mode switching and protection logic

---

## Reference Documentation

For details on the complete refactoring plan, see:
- `REFACTORING_PLAN.md` - Complete 19-day refactoring plan
- `REFACTORING_QUICK_START.md` - Quick reference
- `NSD_DRIVER_CLEANUP_PLAN.md` - Original cleanup plan

---

## Restoring Original Files

If you need to reference or restore the original application control logic:

```powershell
# View original AI.c
Get-Content archive\AI.c.original

# Restore original (if needed)
Copy-Item archive\AI.c.original AI.c
```

**Note**: The original files are preserved for reference but are not maintained going forward. The project is moving to application-agnostic hardware drivers.

---

**Status**: Phase 7 (Cleanup) in progress  
**Next**: Update para.c, create minimal driver documentation
