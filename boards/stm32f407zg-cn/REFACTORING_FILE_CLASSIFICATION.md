# NSD_Driver File Classification

**Quick reference for what to do with each file during refactoring**

---

## 📊 Classification Legend

| Symbol | Action       | Description                               |
| ------ | ------------ | ----------------------------------------- |
| ✅      | **KEEP**     | Pure hardware driver - minimal changes    |
| 🔄      | **REFACTOR** | Mixed code - separate HW from application |
| 🎯      | **IMPROVE**  | Keep but enhance API/interface            |
| ❌      | **REMOVE**   | Application-specific - delete or archive  |
| 🔧      | **EXTRACT**  | Utility functions - move to middleware    |

---

## 📁 File-by-File Breakdown

### Category 1: Pure Hardware Drivers

| File                 | Status | New Location                                  | Notes              |
| -------------------- | ------ | --------------------------------------------- | ------------------ |
| `AD7606.c/h`         | ✅ KEEP | `Drivers/Peripherals/ad7606_driver.c/h`       | Already refactored |
| `AD57x4.c/h`         | ✅ KEEP | `Drivers/Peripherals/ad57x4_driver.c/h`       | DAC driver         |
| `ADC.c/h`            | ✅ KEEP | `Drivers/Peripherals/adc_internal_driver.c/h` | Internal ADC       |
| `GPIO.c/h`           | ✅ KEEP | `Drivers/Peripherals/gpio_driver.c/h`         | GPIO utilities     |
| `TIM.c/h`            | ✅ KEEP | `Drivers/Peripherals/timer_driver.c/h`        | Timer/PWM          |
| `USART_T.c/h`        | ✅ KEEP | `Drivers/Peripherals/usart_driver.c/h`        | UART (merge files) |
| `USARTx_1.c`         | ✅ KEEP | ↑ merge into usart_driver.c                   |                    |
| `USARTx.c/h`         | ✅ KEEP | ↑ merge into usart_driver.c                   |                    |
| `M25P32.c/h`         | ✅ KEEP | `Drivers/Peripherals/spi_flash_driver.c/h`    | SPI Flash          |
| `nandflash_sram.c/h` | ✅ KEEP | `Drivers/Peripherals/fsmc_sram_driver.c/h`    | External memory    |
| `WatchDog.c/h`       | ✅ KEEP | `Drivers/Peripherals/watchdog_driver.c/h`     | IWDG/WWDG          |
| `temperature.c/h`    | ✅ KEEP | `Drivers/Peripherals/temperature_driver.c/h`  | Temp sensors       |

**Total**: 12 files → **KEEP** (consolidate USART files)

---

### Category 2: Communication Stack - Ethernet

| File         | Status     | New Location                                     | Notes                |
| ------------ | ---------- | ------------------------------------------------ | -------------------- |
| `w5200.c`    | 🎯 IMPROVE  | `Drivers/Communication/w5200/w5200_hal.c`        | Low-level SPI driver |
| `w5200.h`    | 🎯 IMPROVE  | `Drivers/Communication/w5200/w5200_hal.h`        | Public API           |
| `w5200Reg.h` | ✅ KEEP     | `Drivers/Communication/w5200/w5200_reg.h`        | Register definitions |
|              | 🆕 NEW      | `Drivers/Communication/w5200/w5200_socket.c/h`   | Socket abstraction   |
| `net.c`      | 🔄 REFACTOR | `Drivers/Communication/ethernet/eth_transport.c` | Remove app protocol  |
| `net.h`      | 🔄 REFACTOR | `Drivers/Communication/ethernet/eth_transport.h` | New API              |
|              | 🆕 NEW      | `Drivers/Communication/ethernet/eth_config.h`    | Config structures    |
|              | 🆕 NEW      | `Drivers/Communication/ethernet/eth_callbacks.h` | Callback typedefs    |

**Actions**:
- Extract socket management from `net.c`
- Remove P/Q parsing (application-specific)
- Create callback-based RX/TX
- Support dual W5200 chips

---

### Category 3: Communication Stack - Fiber Optic

| File              | Status     | New Location                                          | Notes                 |
| ----------------- | ---------- | ----------------------------------------------------- | --------------------- |
| `optical_fiber.c` | 🔄 REFACTOR | `Drivers/Communication/fiber_optic/fpga_fsmc_hal.c`   | FSMC init only        |
| `optical_fiber.h` | 🔄 REFACTOR | `Drivers/Communication/fiber_optic/fpga_fsmc_hal.h`   | HW interface          |
| `optical.c`       | 🔄 REFACTOR | `Drivers/Communication/fiber_optic/fiber_transport.c` | 1800 lines!           |
| `optical.h`       | 🔄 REFACTOR | `Drivers/Communication/fiber_optic/fiber_transport.h` | Multi-channel API     |
|                   | 🆕 NEW      | `Drivers/Communication/fiber_optic/fiber_frame.c/h`   | Frame parsing         |
|                   | 🆕 NEW      | `Drivers/Communication/fiber_optic/fiber_callbacks.h` | Event callbacks       |
|                   | 🔧 EXTRACT  | `Middleware/crc_utils.c/h`                            | CRC16 from optical.c  |
|                   | 🔧 EXTRACT  | `Middleware/data_converter.c/h`                       | Float/byte conversion |

**Actions**:
- Separate FSMC hardware from protocol
- Extract CRC and data conversion to middleware
- Remove excitation-specific commands
- Keep frame handling, multi-channel, error detection

---

### Category 4: Mixed Hardware/Application

| File                | Status     | New Location                  | Notes                  |
| ------------------- | ---------- | ----------------------------- | ---------------------- |
| `AI.c` (570 lines)  | 🔄 REFACTOR | `Drivers/Board/aio_driver.c`  | Remove Ifd, Vfd, P, Q  |
| `AI.h`              | 🔄 REFACTOR | `Drivers/Board/aio_driver.h`  | Generic AIO API        |
| `DIDO.c`            | 🔄 REFACTOR | `Drivers/Board/dio_driver.c`  | Remove SCR checking    |
| `DIDO.h`            | 🔄 REFACTOR | `Drivers/Board/dio_driver.h`  | Generic DIO API        |
| `syn.c` (471 lines) | 🔄 REFACTOR | `Drivers/Board/sync_driver.c` | Remove firing control  |
| `syn.h`             | 🔄 REFACTOR | `Drivers/Board/sync_driver.h` | Sync capture API       |
| `limit.c/h`         | 🔄 REFACTOR | `Middleware/limit_utils.c/h`  | Generic limit checking |

**Keep from each file**:
- `AI.c`: ADC acquisition, buffering → Remove: Vabc, Iabc, Ifd, Vfd, P, Q calculations
- `DIDO.c`: Pin read/write → Remove: SCR_Check(), Breaker_Check(), fault logic
- `syn.c`: Interrupt timing, period measurement → Remove: SCR pulse generation

---

### Category 5: Application Control Logic

| File          | Status    | Action                              | Reason                      |
| ------------- | --------- | ----------------------------------- | --------------------------- |
| `mode.c/h`    | ❌ REMOVE  | Archive or delete                   | AVR/FCR/AER control modes   |
| `eIfd.c/h`    | ❌ REMOVE  | Archive or delete                   | PID field current regulator |
| `ref.c/h`     | ❌ REMOVE  | Archive or delete                   | Reference tracking/ramping  |
| `command.c/h` | ❌ REMOVE  | Archive or delete                   | RS232 command protocol      |
| `para.c/h`    | 🔄 PARTIAL | Extract HW init → `HAL/board_hal.c` | Remove app params           |
| `pub.c/h`     | ❌ REMOVE  | Archive or delete                   | 100+ app global variables   |

**These are excitation-system specific and should not be in the driver layer.**

---

## 📦 Summary by Action

### ✅ KEEP as-is (12 files)
Pure hardware drivers with minimal changes:
- AD7606, AD57x4, ADC, GPIO, TIM
- USART (merge 3 files), M25P32, nandflash_sram
- WatchDog, temperature, w5200Reg

### 🔄 REFACTOR (10 files)
Separate hardware from application logic:
- w5200, net, optical_fiber, optical
- AI, DIDO, syn, limit
- para (partially)

### ❌ REMOVE (6 files)
Application-specific control:
- mode, eIfd, ref, command
- pub (completely)
- para (app parameters only)

### 🆕 CREATE NEW (15+ files)
- Socket abstraction layers
- Callback interface definitions
- Configuration structures
- Middleware utilities (CRC, conversion)
- Board HAL
- Examples (6+)

---

## 🔄 Refactoring Priority Order

### Phase 2-3: Communication (7 days)
1. `w5200.c/h` → Split into hal + socket
2. `net.c/h` → Extract transport layer
3. `optical_fiber.c/h` → FSMC hardware only
4. `optical.c/h` → Extract utilities, create transport API

### Phase 4: Hardware I/O (3 days)
5. `AI.c/h` → Generic AIO driver
6. `DIDO.c/h` → Generic DIO driver
7. `syn.c/h` → Generic sync driver

### Phase 5: Board HAL (2 days)
8. `para.c` → Extract hardware init sequences
9. Create unified `board_hal.c/h`

### Phase 7: Cleanup (1 day)
10. Remove: `mode`, `eIfd`, `ref`, `command`, `pub`

---

## 📋 File Size Reference

| File        | Lines | Complexity | Refactor Effort |
| ----------- | ----- | ---------- | --------------- |
| `optical.c` | 1824  | High       | 2 days          |
| `AI.c`      | 570   | Medium     | 1 day           |
| `syn.c`     | 471   | High       | 1 day           |
| `DIDO.c`    | 574   | Medium     | 1 day           |
| `net.c`     | 172   | Low        | 0.5 day         |
| `eIfd.c`    | 186   | N/A        | DELETE          |
| `mode.c`    | 158   | N/A        | DELETE          |
| `ref.c`     | 225   | N/A        | DELETE          |

**Total lines to refactor**: ~3,800  
**Total lines to remove**: ~800  
**New lines to create**: ~2,500  

---

## 🎯 Quick Decision Tree

**When reviewing a file, ask:**

1. **Does it directly control hardware?** → ✅ KEEP
2. **Does it parse application data?** → ❌ REMOVE or 🔄 REFACTOR
3. **Is it a utility (CRC, conversion)?** → 🔧 EXTRACT to Middleware
4. **Does it implement control algorithms?** → ❌ REMOVE
5. **Does it mix HW and app logic?** → 🔄 REFACTOR

**Examples:**
- `AD7606.c` reads ADC → ✅ KEEP
- `AI.c` reads ADC + calculates P/Q → 🔄 REFACTOR (keep ADC, remove P/Q)
- `eIfd.c` runs PID controller → ❌ REMOVE (no hardware interaction)
- `optical.c` has CRC function → 🔧 EXTRACT CRC, 🔄 REFACTOR transport

---

## 📊 Migration Checklist

Use this to track progress:

### Phase 2-3: Communication
- [ ] `w5200.c/h` → `w5200_hal.c/h` + `w5200_socket.c/h`
- [ ] `net.c/h` → `eth_transport.c/h`
- [ ] `optical_fiber.c/h` → `fpga_fsmc_hal.c/h`
- [ ] `optical.c/h` → `fiber_transport.c/h` + `fiber_frame.c/h`
- [ ] Extract CRC → `Middleware/crc_utils.c/h`
- [ ] Extract conversion → `Middleware/data_converter.c/h`

### Phase 4: Hardware I/O
- [ ] `AI.c/h` → `aio_driver.c/h`
- [ ] `DIDO.c/h` → `dio_driver.c/h`
- [ ] `syn.c/h` → `sync_driver.c/h`
- [ ] `limit.c/h` → `limit_utils.c/h`

### Phase 5: Board HAL
- [ ] Extract init from `para.c` → `board_hal.c`
- [ ] Create `board_callbacks.h`
- [ ] Create `board_config.h`

### Phase 7: Cleanup
- [ ] Archive/delete `mode.c/h`
- [ ] Archive/delete `eIfd.c/h`
- [ ] Archive/delete `ref.c/h`
- [ ] Archive/delete `command.c/h`
- [ ] Archive/delete `pub.c/h`
- [ ] Remove app code from `para.c/h`

---

**Last Updated**: October 15, 2025  
**Status**: Ready for Phase 1 (Setup)
