# NSD_Driver Cleanup Plan
## Remove Application Control Logic, Keep Hardware Drivers Only

**Date:** October 15, 2025  
**Objective:** Extract device drivers from legacy thyristor/SCR bridge control application

---

## Files Analysis

### ✅ KEEP - Hardware Drivers (Pure I/O)
These files implement hardware communication only:

1. **AD7606.c/h** - 8-channel 16-bit ADC via FSMC
2. **AD57x4.c/h** - DAC output driver  
3. **ADC.c/h** - STM32 internal ADC
4. **DIDO.c/h** - Digital I/O (GPIO bulk operations)
5. **GPIO.C/H** - GPIO configuration
6. **TIM.C/h** - Timer/periodic tasks (already cleaned)
7. **USART_T.C/h, USARTx.C/h** - Serial communication
8. **WatchDog.c/h** - Watchdog timer
9. **nandflash_sram.c/h** - FSMC memory interface
10. **M25P32.c/h** - SPI flash memory
11. **w5200.c/h, w5200Reg.h** - Ethernet controller
12. **temperature.c/h** - Temperature sensor reading

### ⚠️ CLEAN - Mixed (Driver + Control Logic)
These need control logic removed:

13. **AI.c/h** - ADC acquisition + RMS/averaging calculations
    - Keep: AD() raw reading, ADC_Data[][] buffer
    - Remove: Cal_AD(), Cal_Avg(), RMS(), Cal_Arm_I(), Get_Monitor()
    
14. **pub.c/h** - Global variables
    - Keep: Hardware status, raw measurements
    - Remove: Control variables (Alpha, Vfd_av, Ifd_av, averaging coefficients)
    
15. **para.c/h** - Initialization
    - Keep: Init_Hardware(), DIP switch reading
    - Remove: Control algorithm initialization

###❌ DELETE - Pure Application Logic
These files implement control algorithms only:

16. **mode.c/h** - Control modes (V/I/Vfd control)
17. **syn.c/h** - Grid synchronization & phase tracking
18. **optical.c/h** - Optical interface control
19. **optical_fiber.c/h** - Fiber optic communication control
20. **eIfd.c/h** - Excitation current control
21. **limit.c/h** - Limit checking & protection
22. **ref.c/h** - Reference value management
23. **command.c/h** - Command processing (network control)
24. **net.c/h** - Network protocol handling

---

## AI.c Cleanup Details

### Current Structure (570 lines)
```
Init_Avg() - averaging state init
Cal_Avg() - running average calculation
Init_AD_Data() - buffer init + control var init
Zero_Calibrate() - DC offset calibration
Get_Medium_n() - median filter
Get_Medium_Value() - median filter (float)
RMS() - RMS calculation
Shift_AD_Data() - data shuffling
Get_Monitor() - network monitoring data
AD() - ★ ADC read (KEEP THIS)
Cal_Arm_I() - arm current calculation
Cal_AD() - full ADC processing with RMS/filters
```

### Cleaned Structure (~120 lines)
```
Init_AD_Data() - simple buffer clear
Get_Medium_n() - median filter (keep for noise rejection)
AD() - ★ Raw ADC read + median filter
Zero_Calibrate() - DC offset calibration (useful utility)
Cal_AD() - empty placeholder
```

### Variables to Keep
```c
int AI_Times;                  // Sample counter
int ADC_Data[8][AD_Data_Num];  // Raw ADC data [ch][sample]
int AI_Count;                   // General counter
unsigned short int nI_Buf[3][2]; // Median filter buffers
```

### Variables to Remove
```c
int V[4][AD_Data_Num];  // Processed voltage arrays
int I[4][AD_Data_Num];  // Processed current arrays
float Va, Vb, Vc, Vabc;  // RMS voltages
float Ia, Ib, Ic, Iabc;  // RMS currents
float Vfd, Ifd;          // Field values
float Iap, Ian, etc.     // Arm currents
All temp buffers for averaging
```

---

## pub.h Cleanup Details

### Variables to KEEP (Hardware Status)
```c
// Hardware errors
unsigned char AD_Err_Flag;
unsigned char SCR_Error_Flag;
unsigned char Fuse_Flag;
unsigned char Over_Temperature_Flag;

// DI/DO states
unsigned char DI_Data[8];
unsigned char DO_Data[8];

// Hardware calibration
float Vfd_AD_Const;
float V_AD_Const;
float I_AD_Const;
```

### Variables to REMOVE (Control Logic)
```c
float Alpha;  // Firing angle
float Vfd_av, Ifd_av;  // Averaged values
float Avg_Ctrl_Coef, Avg_Coef;  // Control coefficients
All sync variables (Syn_*)
All mode flags
All control references
```

---

## para.c Cleanup Details

### Functions to KEEP
```c
Init_Hardware() - hardware peripheral init only
Init_Data() - clear data buffers only
DIP_Switch_Init() - GPIO setup
Read_DIP_Switch() - read hardware state
```

### Remove from Init_Hardware()
- Mode selection logic
- Control algorithm setup
- Sync initialization
- Optical interface init

Keep only:
- GPIO/FSMC/USART setup
- ADC initialization  
- Timer configuration
- Watchdog setup
- SPI/I2C init

---

## CMakeLists.txt Updates

### Remove from source list:
```cmake
Core/NSD_Driver/mode.c
Core/NSD_Driver/syn.c
Core/NSD_Driver/optical.c
Core/NSD_Driver/optical_fiber.c
Core/NSD_Driver/eIfd.c
Core/NSD_Driver/limit.c
Core/NSD_Driver/ref.c
Core/NSD_Driver/command.c
Core/NSD_Driver/net.c
```

---

## Testing Plan

1. **Backup all files** ✅
2. **Clean AI.c** - Remove calculations, keep AD()
3. **Clean pub.h** - Remove control variables
4. **Clean para.c** - Simplify initialization
5. **Delete obsolete files**
6. **Update CMakeLists.txt**
7. **Compile test** - Fix errors
8. **Runtime test** - Verify ADC still works

---

## Expected Outcome

**Before:**
- 24 driver files (9,000+ lines)
- Mixed drivers + control logic
- Complex dependencies
- Chinese comments

**After:**
- 15 driver files (~3,000 lines)
- Pure hardware drivers
- Simple dependencies
- English documentation
- Ready for new application code

**User can implement:**
- Custom control algorithms in task_200us/1ms/10ms
- Access raw ADC data via ADC_Data[8][300]
- Use GPIO/DAC for custom output patterns
- Build any control strategy on clean driver layer

---

## Rollback

All original files backed up with `.bak` extension:
```
AI.c.bak
pub.h.bak
para.c.bak
```

To restore: `Copy-Item *.bak <original_name>`

---

**Status:** Ready to proceed with cleanup

