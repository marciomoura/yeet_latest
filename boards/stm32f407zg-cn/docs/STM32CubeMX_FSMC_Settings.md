# STM32CubeMX FSMC Configuration Quick Reference

## How to Configure FSMC in STM32CubeMX for FPGA Interface

### Step 1: Enable FSMC Peripheral
1. Open your `.ioc` file in STM32CubeMX
2. Go to: **Connectivity** → **FSMC**
3. Enable:
   - ☑ **Bank1_NORSRAM2** (for AD7606 ADC)
   - ☑ **Bank1_NORSRAM4** (for FPGA Digital I/O)

### Step 2: Configure Bank2 (AD7606)

Click on **NOR/PSRAM 2** tab:

#### General Settings:
- Memory Type: `PSRAM`
- Memory Data Width: `16 bits`
- Address/Data Multiplexing: `Disable`
- Burst Access Mode: `Disable`
- Write Operation: `Enable`
- Extended Mode: `Disable`
- Asynchronous Wait: `Disable`
- Wait Signal: `Disable`

#### Timing Settings:
```
Address Setup Time (ADDSET):     2
Address Hold Time (ADDHLD):      1
Data Setup Time (DATAST):        11
Bus Turn Around Duration:        1
CLK Division (CLKDIV):           1    ⚠️ NOT 16!
Data Latency (DATLAT):           0    ⚠️ NOT 17!
Access Mode:                     A
```

### Step 3: Configure Bank4 (FPGA)

Click on **NOR/PSRAM 4** tab:

#### General Settings:
- Memory Type: `PSRAM`
- Memory Data Width: `16 bits`
- Address/Data Multiplexing: `Disable`
- Burst Access Mode: `Disable`
- Write Operation: `Enable`
- Extended Mode: `Disable`
- Asynchronous Wait: `Disable`
- Wait Signal: `Disable`

#### Timing Settings:
```
Address Setup Time (ADDSET):     6
Address Hold Time (ADDHLD):      15
Data Setup Time (DATAST):        21
Bus Turn Around Duration:        1
CLK Division (CLKDIV):           1    ⚠️ NOT 16!
Data Latency (DATLAT):           0    ⚠️ NOT 17!
Access Mode:                     A
```

### Step 4: Verify GPIO Pin Configuration

Go to **Pinout & Configuration** → **System Core** → **GPIO**

Verify these pins are set to **FSMC Alternate Function**:

#### GPIOD (Port D):
- PD0  → FSMC_D2
- PD1  → FSMC_D3
- PD4  → FSMC_NOE (Read Enable)
- PD5  → FSMC_NWE (Write Enable)
- PD8  → FSMC_D13
- PD9  → FSMC_D14
- PD10 → FSMC_D15
- PD11 → FSMC_A16
- PD12 → FSMC_A17
- PD14 → FSMC_D0
- PD15 → FSMC_D1

#### GPIOE (Port E):
- PE7  → FSMC_D4
- PE8  → FSMC_D5
- PE9  → FSMC_D6
- PE10 → FSMC_D7
- PE11 → FSMC_D8
- PE12 → FSMC_D9
- PE13 → FSMC_D10
- PE14 → FSMC_D11
- PE15 → FSMC_D12

#### GPIOF (Port F):
- PF0  → FSMC_A0
- PF1  → FSMC_A1
- PF2  → FSMC_A2
- PF3  → FSMC_A3
- PF4  → FSMC_A4
- PF5  → FSMC_A5
- PF12 → FSMC_A6
- PF13 → FSMC_A7
- PF14 → FSMC_A8
- PF15 → FSMC_A9

#### GPIOG (Port G):
- PG0  → FSMC_A10
- PG1  → FSMC_A11
- PG2  → FSMC_A12
- PG3  → FSMC_A13
- PG4  → FSMC_A14
- PG5  → FSMC_A15
- PG9  → FSMC_NE2 (Chip Select for Bank2 - AD7606)
- PG12 → FSMC_NE4 (Chip Select for Bank4 - FPGA) ⭐

### Step 5: GPIO Configuration Details

For each FSMC pin, verify:
- **Mode**: Alternate Function Push Pull
- **Pull-up/Pull-down**: No pull-up and no pull-down
- **Speed**: Very High (or High if signal integrity issues)
- **Alternate Function**: FSMC (AF12)

### Step 6: Generate Code

1. Click **Project** → **Generate Code**
2. CubeMX will update your project files

## ⚠️ CRITICAL WARNING

**DO NOT let CubeMX set:**
- CLKDivision to 16
- DataLatency to 17

These values are for **synchronous burst mode** SRAM, not asynchronous parallel bus devices like your FPGA!

If CubeMX generates these values, manually edit `main.c` in the `MX_FSMC_Init()` function:

```c
// Change from:
Timing.CLKDivision = 16;
Timing.DataLatency = 17;

// To:
Timing.CLKDivision = 1;
Timing.DataLatency = 0;
```

## Memory Map Reference

### Bank2 (AD7606):
- Base Address: `0x64000000`
- Chip Select: NE2 (PG9)

### Bank4 (FPGA):
- Base Address: `0x6C000000`
- Chip Select: NE4 (PG12)

#### FPGA Register Offsets:
```c
Digital Outputs:
  DO0-DO15:  0x6C000006  (offset 3 × 2)
  DO16-DO31: 0x6C000008  (offset 4 × 2)
  DO32-DO47: 0x6C00000A  (offset 5 × 2)

Control:     0x6C00000C  (offset 6 × 2)
LEDs:        0x6C00000E  (offset 7 × 2)

Digital Inputs:
  DI0-DI15:  0x6C000192  (offset 201 × 2)
  DI16-DI31: 0x6C000194  (offset 202 × 2)
  DI32-DI47: 0x6C000196  (offset 203 × 2)
```

## Timing Calculation (Advanced)

If you need to adjust timing, use these formulas (HCLK = 168 MHz, period = 5.95 ns):

```
Read Cycle Time = (ADDSET + DATAST + 2) × HCLK_period
Write Cycle Time = (ADDSET + DATAST + 2) × HCLK_period

For Bank4 (FPGA):
  Read  = (6 + 21 + 2) × 5.95 ns = 172.5 ns
  Write = (6 + 21 + 2) × 5.95 ns = 172.5 ns

For Bank2 (AD7606):
  Read  = (2 + 11 + 2) × 5.95 ns = 89.25 ns
  Write = (2 + 11 + 2) × 5.95 ns = 89.25 ns
```

These are conservative timings with good margins for reliable operation.

## Troubleshooting

### If FPGA doesn't respond:
1. Verify PG12 (NE4) is configured as FSMC_NE4
2. Check timing values (especially CLKDivision and DataLatency)
3. Scope PG12 - it should go LOW during access
4. Verify FPGA is powered and configured

### If reads return 0xFFFF:
- FPGA may not be driving the data bus
- Check FPGA OE (Output Enable) connection to NOE
- Verify data bus direction control in FPGA logic

### If writes have no effect:
- Check FPGA WE (Write Enable) connection to NWE
- Verify chip select (NE4) is reaching FPGA
- Confirm address decoding in FPGA HDL

## Reference Documents

See also:
- `docs/fsmc_fpga_documentation.md` - Detailed FPGA interface spec
- `docs/FSMC_CONFIGURATION_FIX.md` - Analysis of configuration issues
- `boards/to_compare/GPIO.C` - Working legacy reference code
