# AD7606 Driver Integration Guide

## Overview

This document explains how the AD7606 8-channel ADC driver has been integrated into your YEET project for high-speed voltage and current measurements.

---

## 📂 **Files Created/Modified**

### **New Files:**
1. **`boards/stm32f407zg-cn/Core/Inc/ad7606_driver.h`**
   - Driver header with API declarations
   - Memory map definitions
   - Data structure definitions

2. **`boards/stm32f407zg-cn/Core/Src/ad7606_driver.c`**
   - Driver implementation
   - Initialization, conversion, and read functions

### **Modified Files:**
1. **`software/yeet_application/sfc_embedded/include/sfc_embedded/task_io.h`**
   - Extended `task1_inputs_t` structure to include AD7606 data

2. **`boards/stm32f407zg-cn/Core/Src/main.c`**
   - Added AD7606 initialization in setup
   - Integrated AD7606 reading in Task 1 callback

---

## 🔧 **Hardware Configuration**

### **AD7606 Connections (Already Configured by STM32CubeMX)**

| Pin  | Signal  | Function                         | Direction |
| ---- | ------- | -------------------------------- | --------- |
| PF6  | CONVSTA | Conversion Start A               | Output    |
| PF7  | CONVSTB | Conversion Start B               | Output    |
| PF9  | RANGE   | Input Range (High=±10V, Low=±5V) | Output    |
| PF10 | RESET   | ADC Reset                        | Output    |
| PD13 | BUSY    | Conversion Status                | Input     |

### **FSMC Interface (Bank 2)**
- **Base Address:** `0x64000000`
- **Data Register:** `0x64000800`
- **Bus Width:** 16-bit
- **Timing:** Optimized for AD7606 (11 HCLK data setup time)

---

## 📊 **AD7606 Channel Mapping**

| Channel | Index | Signal | Description     | Typical Use          |
| ------- | ----- | ------ | --------------- | -------------------- |
| CH0     | 0     | Va     | Phase A Voltage | Grid/Machine voltage |
| CH1     | 1     | Vb     | Phase B Voltage | Grid/Machine voltage |
| CH2     | 2     | Vc     | Phase C Voltage | Grid/Machine voltage |
| CH3     | 3     | Vfd    | Field Voltage   | Field excitation     |
| CH4     | 4     | Ia     | Phase A Current | Grid/Machine current |
| CH5     | 5     | Ib     | Phase B Current | Grid/Machine current |
| CH6     | 6     | Ic     | Phase C Current | Grid/Machine current |
| CH7     | 7     | Ifd    | Field Current   | Field excitation     |

---

## 🚀 **How It Works**

### **Initialization (Once at Startup)**

```c
// In main.c, after MX_FSMC_Init()
AD7606_Init();
AD7606_StartConversion();  // Start first conversion
```

**What happens:**
1. Performs hardware reset sequence on AD7606
2. Sets voltage range to ±10V (default)
3. Configures control pins to idle state
4. Starts first conversion

### **Data Acquisition (Every 200 µs in Task 1)**

```c
// In HAL_ADC_ConvCpltCallback() - Task 1 @ 5 kHz
AD7606_ReadAndStart(&ad7606_data);  // Read previous, start next
```

**Pipeline Mode Operation:**
1. **Read** results from previous conversion (~160ns via FSMC)
2. **Start** next conversion immediately (minimize dead time)
3. Conversion takes ~3.6 µs (completes before next callback)

### **Data Structure**

The `ad7606_data_t` structure contains:

```c
typedef struct {
    uint16_t raw[8];           // Raw 16-bit ADC values
    float voltage[8];          // Converted voltages (±10V)
    ad7606_range_t range;      // Current range setting
    bool conversion_ready;     // Data valid flag
    uint32_t conversion_count; // Total conversions
} ad7606_data_t;
```

### **Accessing Data in Your Control Code**

In `yeet_task_200us()`, you receive `task1_inputs_t` with AD7606 data:

```cpp
void yeet_task_200us(const task1_inputs_t* inputs, task1_outputs_t* outputs)
{
    // Check if AD7606 data is valid
    if (inputs->ad7606.data_valid) {
        // Get voltages (already converted to float)
        float va  = inputs->ad7606.voltage[0];  // Phase A voltage
        float vb  = inputs->ad7606.voltage[1];  // Phase B voltage
        float vc  = inputs->ad7606.voltage[2];  // Phase C voltage
        float vfd = inputs->ad7606.voltage[3];  // Field voltage
        
        float ia  = inputs->ad7606.voltage[4];  // Phase A current
        float ib  = inputs->ad7606.voltage[5];  // Phase B current
        float ic  = inputs->ad7606.voltage[6];  // Phase C current
        float ifd = inputs->ad7606.voltage[7];  // Field current
        
        // Or access raw 16-bit values
        uint16_t va_raw = inputs->ad7606.raw[0];
        
        // Your control algorithm here...
    }
}
```

---

## 🎯 **API Reference**

### **Initialization Functions**

#### `AD7606_Init(void)`
- Initializes AD7606 ADC
- Call once after `MX_FSMC_Init()` and `MX_GPIO_Init()`
- Sets default range to ±10V

#### `AD7606_SetRange(ad7606_range_t range)`
- Configure voltage range
- `AD7606_RANGE_5V`: ±5V input range
- `AD7606_RANGE_10V`: ±10V input range (default)

#### `AD7606_Reset(void)`
- Performs hardware reset sequence
- Automatically called by `AD7606_Init()`

### **Conversion Functions**

#### `AD7606_StartConversion(void)`
- Triggers simultaneous sampling on all 8 channels
- Conversion takes ~3.6 µs
- Returns immediately (non-blocking)

#### `AD7606_IsConversionReady(void)`
- Checks BUSY pin status
- Returns `true` when conversion complete
- Returns `false` when conversion in progress

#### `AD7606_Read(ad7606_data_t* data)`
- Reads all 8 channels from AD7606
- Waits for conversion to complete (with timeout)
- Converts raw values to voltages
- Updates `data` structure

#### `AD7606_ReadAndStart(ad7606_data_t* data)` ⭐ **Recommended**
- **Pipeline mode:** Read previous + start next
- Minimizes dead time between conversions
- Perfect for 5 kHz sampling rate
- This is what's used in Task 1

### **Utility Functions**

#### `AD7606_RawToVoltage(uint16_t raw, ad7606_range_t range)`
- Converts raw ADC count to voltage
- Handles signed 16-bit two's complement
- Returns float in volts (±5V or ±10V)

---

## ⚙️ **Configuration Options**

### **Changing Voltage Range**

If you need ±5V range for more precision on smaller signals:

```c
// In main.c, USER CODE BEGIN 2 section, after AD7606_Init()
AD7606_SetRange(AD7606_RANGE_5V);
```

**Resolution:**
- ±10V range: 305 µV/LSB (10V / 32768 counts)
- ±5V range: 153 µV/LSB (5V / 32768 counts)

### **Adjusting Sampling Rate**

Currently running at 5 kHz (200 µs period). To change:

1. Modify TIM2 period in STM32CubeMX
2. AD7606 supports up to 200 kSPS (all channels)
3. Ensure conversion time (3.6 µs) < sampling period

---

## 📈 **Performance**

### **Timing Breakdown (at 5 kHz)**

| Operation         | Time     | Notes                   |
| ----------------- | -------- | ----------------------- |
| AD7606 Conversion | ~3.6 µs  | Simultaneous 8 channels |
| FSMC Read (8 ch)  | ~1.3 µs  | 8 × 160ns per channel   |
| Total AD7606 Time | ~5 µs    | 2.5% of 200 µs period   |
| STM32 ADC DMA     | ~10 µs   | 4 channels, overlapped  |
| Task 1 Execution  | Variable | Your control algorithm  |

### **Data Rate**
- **8 channels × 16-bit × 5 kHz = 80 kBytes/sec**
- **40,000 samples/sec** total (all channels)

---

## 🐛 **Debugging**

### **Check if AD7606 is Working**

Add to your code:

```c
// In main.c, after initialization
if (ad7606_data.conversion_count > 0) {
    // AD7606 is reading data
}

// Check data validity in Task 1
if (!inputs->ad7606.data_valid) {
    // Timeout or error occurred
}
```

### **Common Issues**

| Problem                       | Possible Cause         | Solution                                              |
| ----------------------------- | ---------------------- | ----------------------------------------------------- |
| `conversion_count` stays at 0 | FSMC not initialized   | Ensure `MX_FSMC_Init()` called before `AD7606_Init()` |
| `data_valid` always false     | BUSY pin not connected | Check PD13 wiring                                     |
| Noisy readings                | Grounding issues       | Check analog ground connections                       |
| All zeros                     | FSMC address error     | Verify A16, A17 (PD11, PD12) configured               |

### **Diagnostic Variables**

Monitor these in debugger:

```c
ad7606_data.conversion_count  // Increments every read (should be ~5000/sec)
ad7606_data.conversion_ready  // Should be 'true' after successful read
g_adc_callback_count          // Task 1 execution count
```

---

## 🔍 **Example: Simple Voltage Monitoring**

```cpp
void yeet_task_200us(const task1_inputs_t* inputs, task1_outputs_t* outputs)
{
    if (inputs->ad7606.data_valid) {
        // Calculate 3-phase RMS voltage
        float va = inputs->ad7606.voltage[0];
        float vb = inputs->ad7606.voltage[1];
        float vc = inputs->ad7606.voltage[2];
        
        float v_rms = sqrtf((va*va + vb*vb + vc*vc) / 3.0f);
        
        // Overvoltage protection
        if (v_rms > 400.0f) {
            // Disable all gate pulses
            outputs->pulse_1 = false;
            outputs->pulse_2 = false;
            outputs->pulse_3 = false;
            outputs->pulse_4 = false;
            outputs->pulse_5 = false;
            outputs->pulse_6 = false;
        }
    }
}
```

---

## 📝 **Next Steps**

1. **Test the driver:**
   - Build and flash the code
   - Check `ad7606_data.conversion_count` in debugger
   - Verify `ad7606_data.voltage[]` values are reasonable

2. **Calibration (if needed):**
   - Apply known voltage to each channel
   - Compare `inputs->ad7606.voltage[x]` with actual voltage
   - Add scaling factors if needed

3. **Implement your control algorithm:**
   - Use voltage/current measurements in Task 1
   - Generate appropriate gate pulse patterns
   - Update `outputs` structure

---

## 📚 **Reference Documents**

- `FSMC_FPGA_Interface_Documentation.md` - FSMC configuration details
- `AD7606 Datasheet` - ADC specifications and timing
- `task_io.h` - Task I/O structure definitions

---

## ✅ **Summary**

✅ AD7606 driver fully integrated  
✅ 8 channels @ 5 kHz sampling rate  
✅ Raw 16-bit and float voltage data available  
✅ Pipeline mode minimizes latency  
✅ Ready to use in your control algorithm  

**Your Task 1 now receives:**
- 4 channels from internal STM32 ADC (12-bit)
- 8 channels from external AD7606 (16-bit)
- Total: 12 high-speed measurement channels!

🎉 **The fast ADC is ready for your control loop!**
