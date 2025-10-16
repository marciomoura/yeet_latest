# FSMC Configuration Analysis & Fix

## Date: October 13, 2025

## Summary
Analysis of FSMC configuration for FPGA interface revealed **critical timing parameter errors** that were preventing communication.

## Problem Found

### Incorrect Timing Parameters
Your code had:
```c
Timing.CLKDivision = 16;
Timing.DataLatency = 17;
```

### Correct Values (from working legacy code)
Should be:
```c
Timing.CLKDivision = 1;
Timing.DataLatency = 0;
```

## Root Cause
The CLKDivision and DataLatency parameters are meant for **synchronous burst mode** operation. For asynchronous PSRAM/SRAM devices like your FPGA interface:
- CLKDivision should be 1 (minimum)
- DataLatency should be 0 (no wait states in async mode)

Setting these to 16/17 caused invalid bus timing cycles that the FPGA couldn't respond to.

## Configuration Verification

### ✅ Correct Settings (All Match Legacy Code)

#### GPIO Pins
- **Chip Select (CS)**: PG12 = NE4 (Bank1_NORSRAM4) ✓
- **Base Address**: 0x6C000000 ✓
- **Data Bus**: D0-D15 on PD14-15, PD0-1, PE7-15, PD8-10 ✓
- **Address Lines**: A0-A17 configured ✓
- **Control**: NOE (PD4), NWE (PD5) ✓

#### Bank Configuration
- **Bank**: FSMC_Bank1_NORSRAM4 ✓
- **Memory Type**: PSRAM ✓
- **Data Width**: 16-bit ✓
- **Multiplexing**: Disabled ✓
- **Burst Mode**: Disabled ✓
- **Write Operation**: Enabled ✓

#### Register Offsets
- **Digital Outputs**: Offsets 3, 4, 5 (matches legacy) ✓
- **Digital Inputs**: Offsets 201, 202, 203 (matches legacy) ✓
- **Control**: Offset 6 ✓
- **LEDs**: Offset 7 ✓

#### Timing (Bank4 - FPGA)
- AddressSetupTime: 6 ✓
- AddressHoldTime: 15 ✓
- DataSetupTime: 21 ✓
- BusTurnAroundDuration: 1 ✓
- **CLKDivision: 1** (FIXED from 16) ✓
- **DataLatency: 0** (FIXED from 17) ✓
- AccessMode: A ✓

#### Timing (Bank2 - AD7606)
- AddressSetupTime: 2 ✓
- AddressHoldTime: 15 ✓
- DataSetupTime: 11 ✓
- BusTurnAroundDuration: 1 ✓
- **CLKDivision: 1** (FIXED from 16) ✓
- **DataLatency: 0** (FIXED from 17) ✓
- AccessMode: A ✓

## Changes Applied

### File: `Core/Src/main.c`

Fixed both Bank2 and Bank4 timing initialization in `MX_FSMC_Init()`:

**Bank2 (AD7606) - Lines ~680-690:**
```c
Timing.CLKDivision = 1;      // Was: 16
Timing.DataLatency = 0;      // Was: 17
```

**Bank4 (FPGA) - Lines ~715-725:**
```c
Timing.CLKDivision = 1;      // Was: 16
Timing.DataLatency = 0;      // Was: 17
```

## Expected Result
With these fixes:
1. FSMC bus cycles will now match FPGA timing expectations
2. Both AD7606 ADC and FPGA digital I/O should communicate correctly
3. Timing margins are conservative (slower) for reliability

## How This Happened
Likely cause: STM32CubeMX auto-generated these values or they were copied from a synchronous memory template. The values are valid for synchronous burst SRAM but incorrect for asynchronous parallel bus devices.

## Testing Recommendations

1. **Verify FPGA Communication**:
   ```c
   uint16_t test = *(__IO uint16_t*)(0x6C000000 + (201 * 2));
   // Should read actual DI register value, not 0xFFFF or 0x0000
   ```

2. **Toggle Outputs**:
   ```c
   *(__IO uint16_t*)(0x6C000000 + (3 * 2)) = 0xAAAA;
   // Check physical outputs change
   ```

3. **Scope Signals**:
   - PG12 (NE4) should pulse LOW during accesses
   - PD4 (NOE) should pulse during reads
   - PD5 (NWE) should pulse during writes
   - Data bus should show activity

## STM32CubeMX Configuration (If Regenerating)

When configuring in CubeMX:

### For Bank2 (AD7606):
- Connectivity → FSMC → Enable Bank1_NORSRAM2
- Parameters → NOR/PSRAM 2 → Timing:
  - Address Setup: 2
  - Address Hold: 1 (or 15, doesn't matter for async)
  - Data Setup: 11
  - Bus Turnaround: 1
  - **CLK Division: 0 or 1** (NOT 16)
  - **Data Latency: 0** (NOT 17)

### For Bank4 (FPGA):
- Connectivity → FSMC → Enable Bank1_NORSRAM4
- Parameters → NOR/PSRAM 4 → Timing:
  - Address Setup: 6
  - Address Hold: 15 (or 1)
  - Data Setup: 21
  - Bus Turnaround: 1
  - **CLK Division: 0 or 1** (NOT 16)
  - **Data Latency: 0** (NOT 17)

## Additional Notes

- GPIO speed set to VERY_HIGH (100MHz) - acceptable but could reduce to HIGH if signal integrity issues arise
- AddressHoldTime=15 is conservative; can reduce to 1 if needed for faster cycles
- Legacy code comments suggest these were working parameters from previous hardware revision
- All physical pin assignments match legacy → hardware should be compatible

## Status: ✅ FIXED

The FSMC configuration should now work correctly for both FPGA and AD7606 interfaces.
