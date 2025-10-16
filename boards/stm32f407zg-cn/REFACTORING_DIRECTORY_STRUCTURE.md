# NSD_Driver Refactoring - Directory Structure

**Visual guide to the new organization**

---

## 🗂️ Current Structure (Before)

```
boards/stm32f407zg-cn/Core/NSD_Driver/
├── AD57x4.c/h              # DAC
├── AD7606.c/h              # ADC
├── ADC.c/h                 # Internal ADC
├── AI.c/h                  # Analog input + APP LOGIC ❌
├── command.c/h             # RS232 protocol ❌
├── DIDO.c/h                # Digital I/O + APP LOGIC ❌
├── eIfd.c/h                # PID controller ❌
├── GPIO.c/h                # GPIO
├── limit.c/h               # Limit checking + APP ❌
├── M25P32.c/h              # SPI Flash
├── mode.c/h                # Control modes ❌
├── nandflash_sram.c/h      # External memory
├── net.c/h                 # Ethernet + APP PROTOCOL ❌
├── optical.c/h             # Fiber optic (1800 lines!) ❌
├── optical_fiber.c/h       # FPGA interface
├── para.c/h                # Init + APP PARAMS ❌
├── pub.c/h                 # Global app variables ❌
├── ref.c/h                 # Reference tracking ❌
├── syn.c/h                 # Sync signals + APP LOGIC ❌
├── temperature.c/h         # Temperature
├── TIM.c/h                 # Timers
├── USART_T.c/h             # UART
├── USARTx_1.c              # UART
├── USARTx.c/h              # UART
├── w5200.c/h               # Ethernet chip
├── w5200Reg.h              # Ethernet registers
└── WatchDog.c/h            # Watchdog

❌ = Contains application-specific code to remove
```

**Problems**:
- 40 files in flat structure
- Application logic mixed with drivers
- Hard to find what you need
- No clear separation of concerns

---

## 🎯 Target Structure (After)

```
boards/stm32f407zg-cn/Core/
│
├── 📁 Drivers/                           # Low-level hardware drivers
│   │
│   ├── 📁 Peripherals/                   # MCU peripheral drivers
│   │   ├── ad7606_driver.c              ✅ 8-channel ADC via FSMC
│   │   ├── ad7606_driver.h              
│   │   ├── ad57x4_driver.c              ✅ 4-channel DAC via SPI
│   │   ├── ad57x4_driver.h              
│   │   ├── adc_internal_driver.c        ✅ Internal STM32 ADC
│   │   ├── adc_internal_driver.h        
│   │   ├── gpio_driver.c                ✅ GPIO utilities
│   │   ├── gpio_driver.h                
│   │   ├── timer_driver.c               ✅ Timer/PWM/Capture
│   │   ├── timer_driver.h               
│   │   ├── usart_driver.c               ✅ UART (consolidated)
│   │   ├── usart_driver.h               
│   │   ├── spi_flash_driver.c           ✅ M25P32 SPI Flash
│   │   ├── spi_flash_driver.h           
│   │   ├── fsmc_sram_driver.c           ✅ External SRAM/NAND
│   │   ├── fsmc_sram_driver.h           
│   │   ├── watchdog_driver.c            ✅ IWDG/WWDG
│   │   ├── watchdog_driver.h            
│   │   ├── temperature_driver.c         ✅ Temperature sensors
│   │   └── temperature_driver.h         
│   │
│   ├── 📁 Communication/                 # Communication stacks
│   │   │
│   │   ├── 📁 w5200/                     # W5200 Ethernet chip driver
│   │   │   ├── w5200_hal.c              🎯 Low-level SPI operations
│   │   │   ├── w5200_hal.h              
│   │   │   ├── w5200_socket.c           🆕 Socket abstraction
│   │   │   ├── w5200_socket.h           
│   │   │   └── w5200_reg.h              ✅ Register definitions
│   │   │
│   │   ├── 📁 ethernet/                  # TCP/IP transport layer
│   │   │   ├── eth_transport.c          🔄 Socket management (from net.c)
│   │   │   ├── eth_transport.h          
│   │   │   ├── eth_config.h             🆕 Configuration structures
│   │   │   └── eth_callbacks.h          🆕 Callback type definitions
│   │   │
│   │   └── 📁 fiber_optic/               # Fiber optic via FPGA/FSMC
│   │       ├── fpga_fsmc_hal.c          🔄 FSMC interface (from optical_fiber.c)
│   │       ├── fpga_fsmc_hal.h          
│   │       ├── fiber_transport.c        🔄 Multi-channel protocol (from optical.c)
│   │       ├── fiber_transport.h        
│   │       ├── fiber_frame.c            🆕 Frame parsing/building
│   │       ├── fiber_frame.h            
│   │       └── fiber_callbacks.h        🆕 Event callback definitions
│   │
│   └── 📁 Board/                         # Board-level hardware abstractions
│       ├── dio_driver.c                 🔄 Digital I/O (from DIDO.c)
│       ├── dio_driver.h                 
│       ├── aio_driver.c                 🔄 Analog I/O (from AI.c)
│       ├── aio_driver.h                 
│       ├── sync_driver.c                🔄 Sync signals (from syn.c)
│       └── sync_driver.h                
│
├── 📁 HAL/                               # Hardware Abstraction Layer
│   ├── board_hal.c                      🆕 Board initialization (from para.c)
│   ├── board_hal.h                      
│   ├── board_config.h                   🆕 Configuration structures
│   ├── board_callbacks.h                🆕 Master callback interface
│   └── board_types.h                    🆕 Common type definitions
│
├── 📁 Middleware/                        # Protocol utilities (reusable)
│   ├── crc_utils.c                      🔧 CRC16/CRC32 (from optical.c)
│   ├── crc_utils.h                      
│   ├── data_converter.c                 🔧 Float↔Byte (from optical.c)
│   ├── data_converter.h                 
│   ├── tlv_protocol.c                   🆕 Type-Length-Value helper
│   ├── tlv_protocol.h                   
│   └── limit_utils.c                    🔄 Generic limit checking (from limit.c)
│       limit_utils.h                    
│
├── 📁 Examples/                          # Integration examples
│   ├── 01_simple_dio_test.c             🆕 Basic digital I/O test
│   ├── 02_adc_dac_loopback.c            🆕 Analog loopback
│   ├── 03_ethernet_echo_server.c        🆕 TCP echo server
│   ├── 04_fiber_optic_bridge.c          🆕 Multi-channel fiber comm
│   ├── 05_sync_signal_capture.c         🆕 Sync timing demo
│   └── 06_full_integration.c            🆕 Complete system demo
│
├── 📁 Docs/                              # Documentation (Phase 6)
│   ├── API_REFERENCE.md                 🆕 Complete API docs
│   ├── MIGRATION_GUIDE.md               🆕 How to migrate from old API
│   ├── INTEGRATION_GUIDE.md             🆕 How to integrate with apps
│   ├── TIMING_ANALYSIS.md               🆕 Real-time performance
│   ├── TROUBLESHOOTING.md               🆕 Common issues
│   └── PIN_MAPPING.md                   🆕 Complete pin assignments
│
├── REFACTORING_PLAN.md                  ✅ Detailed implementation plan
├── REFACTORING_QUICK_START.md           ✅ Quick reference guide
├── REFACTORING_FILE_CLASSIFICATION.md   ✅ File-by-file breakdown
└── REFACTORING_DIRECTORY_STRUCTURE.md   ✅ This file

Legend:
✅ = Keep as-is or minimal changes
🎯 = Improve API/interface
🔄 = Refactor (separate HW from app)
🆕 = Create new file
🔧 = Extract utility from existing file
```

---

## 📊 Statistics

### Before Refactoring
- **Total files**: 40 C/H pairs
- **Lines of code**: ~12,000
- **Flat structure**: All in one directory
- **Application code**: ~3,000 lines (25%)
- **Mixed code**: ~4,000 lines (33%)
- **Pure drivers**: ~5,000 lines (42%)

### After Refactoring
- **Total files**: ~60 C/H pairs (more organized)
- **Lines of code**: ~10,000 driver code
- **Directories**: 8 organized folders
- **Application code**: 0 lines (removed)
- **Pure drivers**: 100% clean
- **Examples**: 6 working demos

---

## 🗂️ Detailed Breakdown

### Drivers/Peripherals/ (12 files, ~3,000 LOC)
**Purpose**: Direct hardware peripheral drivers  
**Dependencies**: STM32 HAL only  
**Users**: Board-level drivers, HAL  

| File                    | From             | LOC | Purpose               |
| ----------------------- | ---------------- | --- | --------------------- |
| `ad7606_driver.*`       | AD7606.*         | 200 | 8-ch ADC via FSMC     |
| `ad57x4_driver.*`       | AD57x4.*         | 150 | 4-ch DAC via SPI      |
| `adc_internal_driver.*` | ADC.*            | 180 | Internal ADC          |
| `gpio_driver.*`         | GPIO.*           | 120 | GPIO utilities        |
| `timer_driver.*`        | TIM.*            | 250 | Timers/PWM            |
| `usart_driver.*`        | USART*.*         | 400 | UART (3 files merged) |
| `spi_flash_driver.*`    | M25P32.*         | 300 | SPI Flash             |
| `fsmc_sram_driver.*`    | nandflash_sram.* | 350 | External memory       |
| `watchdog_driver.*`     | WatchDog.*       | 100 | Watchdog              |
| `temperature_driver.*`  | temperature.*    | 150 | Temp sensors          |

---

### Drivers/Communication/w5200/ (5 files, ~800 LOC)
**Purpose**: W5200 Ethernet chip driver  
**Dependencies**: SPI peripheral  
**Users**: Ethernet transport layer  

| File             | Purpose              | LOC |
| ---------------- | -------------------- | --- |
| `w5200_hal.*`    | SPI register access  | 300 |
| `w5200_socket.*` | Socket operations    | 400 |
| `w5200_reg.h`    | Register definitions | 100 |

---

### Drivers/Communication/ethernet/ (4 files, ~600 LOC)
**Purpose**: TCP/IP transport layer  
**Dependencies**: W5200 driver  
**Users**: User applications  

| File              | Purpose                  | LOC |
| ----------------- | ------------------------ | --- |
| `eth_transport.*` | Socket management, TX/RX | 400 |
| `eth_config.h`    | Config structures        | 100 |
| `eth_callbacks.h` | Callback typedefs        | 100 |

---

### Drivers/Communication/fiber_optic/ (7 files, ~1,200 LOC)
**Purpose**: Multi-channel fiber optic via FPGA  
**Dependencies**: FSMC, Middleware/CRC  
**Users**: User applications  

| File                | Purpose                  | LOC |
| ------------------- | ------------------------ | --- |
| `fpga_fsmc_hal.*`   | FSMC memory-mapped I/O   | 200 |
| `fiber_transport.*` | Multi-channel management | 600 |
| `fiber_frame.*`     | Frame parsing/building   | 300 |
| `fiber_callbacks.h` | Event definitions        | 100 |

---

### Drivers/Board/ (6 files, ~1,500 LOC)
**Purpose**: Board-level I/O abstractions  
**Dependencies**: Peripheral drivers  
**Users**: User applications  

| File            | From   | Purpose                    | LOC |
| --------------- | ------ | -------------------------- | --- |
| `dio_driver.*`  | DIDO.c | Digital I/O with callbacks | 400 |
| `aio_driver.*`  | AI.c   | Analog I/O with scaling    | 600 |
| `sync_driver.*` | syn.c  | Sync signal capture        | 500 |

---

### HAL/ (5 files, ~800 LOC)
**Purpose**: Unified board initialization  
**Dependencies**: All drivers  
**Users**: Main application  

| File                | Purpose             | LOC |
| ------------------- | ------------------- | --- |
| `board_hal.*`       | Complete board init | 400 |
| `board_config.h`    | Master config       | 150 |
| `board_callbacks.h` | Master callbacks    | 150 |
| `board_types.h`     | Common types        | 100 |

---

### Middleware/ (6 files, ~600 LOC)
**Purpose**: Reusable utilities  
**Dependencies**: None (standalone)  
**Users**: Communication drivers, applications  

| File               | Purpose          | LOC |
| ------------------ | ---------------- | --- |
| `crc_utils.*`      | CRC16/CRC32      | 150 |
| `data_converter.*` | Float↔Byte       | 150 |
| `tlv_protocol.*`   | TLV parsing      | 200 |
| `limit_utils.*`    | Min/max checking | 100 |

---

### Examples/ (6 files, ~1,200 LOC)
**Purpose**: Integration examples  
**Dependencies**: All drivers  
**Users**: Developers learning API  

| File                        | Purpose         | LOC |
| --------------------------- | --------------- | --- |
| `01_simple_dio_test.c`      | Basic DIO       | 150 |
| `02_adc_dac_loopback.c`     | Analog test     | 200 |
| `03_ethernet_echo_server.c` | TCP server      | 250 |
| `04_fiber_optic_bridge.c`   | Multi-channel   | 300 |
| `05_sync_signal_capture.c`  | Timing demo     | 150 |
| `06_full_integration.c`     | Complete system | 350 |

---

## 🔀 Migration Map

### Files Moving Locations

```
OLD → NEW

AD7606.c/h                → Drivers/Peripherals/ad7606_driver.c/h
AD57x4.c/h                → Drivers/Peripherals/ad57x4_driver.c/h
ADC.c/h                   → Drivers/Peripherals/adc_internal_driver.c/h
GPIO.c/h                  → Drivers/Peripherals/gpio_driver.c/h
TIM.c/h                   → Drivers/Peripherals/timer_driver.c/h
USART*.c/h (3 files)      → Drivers/Peripherals/usart_driver.c/h (merged)
M25P32.c/h                → Drivers/Peripherals/spi_flash_driver.c/h
nandflash_sram.c/h        → Drivers/Peripherals/fsmc_sram_driver.c/h
WatchDog.c/h              → Drivers/Peripherals/watchdog_driver.c/h
temperature.c/h           → Drivers/Peripherals/temperature_driver.c/h

w5200.c                   → Drivers/Communication/w5200/w5200_hal.c
w5200.h                   → Drivers/Communication/w5200/w5200_hal.h
w5200Reg.h                → Drivers/Communication/w5200/w5200_reg.h
net.c (refactored)        → Drivers/Communication/ethernet/eth_transport.c
net.h (refactored)        → Drivers/Communication/ethernet/eth_transport.h

optical_fiber.c (partial) → Drivers/Communication/fiber_optic/fpga_fsmc_hal.c
optical.c (refactored)    → Drivers/Communication/fiber_optic/fiber_transport.c
optical.c (CRC extract)   → Middleware/crc_utils.c
optical.c (convert extr.) → Middleware/data_converter.c

AI.c (refactored)         → Drivers/Board/aio_driver.c
DIDO.c (refactored)       → Drivers/Board/dio_driver.c
syn.c (refactored)        → Drivers/Board/sync_driver.c
limit.c (refactored)      → Middleware/limit_utils.c

para.c (init only)        → HAL/board_hal.c

mode.c/h                  → [REMOVED]
eIfd.c/h                  → [REMOVED]
ref.c/h                   → [REMOVED]
command.c/h               → [REMOVED]
pub.c/h                   → [REMOVED]
```

---

## 📐 Include Path Changes

### Before:
```c
#include "pub.h"           // Global app variables
#include "AI.h"            // Mixed app/hw code
#include "net.h"           // Mixed protocol
#include "optical.h"       // 1800 line monolith
```

### After:
```c
// For drivers
#include "Drivers/Peripherals/ad7606_driver.h"
#include "Drivers/Communication/ethernet/eth_transport.h"
#include "Drivers/Board/aio_driver.h"

// For HAL
#include "HAL/board_hal.h"
#include "HAL/board_callbacks.h"

// For utilities
#include "Middleware/crc_utils.h"
```

Or with proper include paths:
```c
// Clean includes
#include "ad7606_driver.h"
#include "eth_transport.h"
#include "board_hal.h"
```

---

## 🔨 Build System Changes

### CMakeLists.txt Structure:

```cmake
# Peripheral drivers library
add_library(peripherals STATIC
    Drivers/Peripherals/ad7606_driver.c
    Drivers/Peripherals/ad57x4_driver.c
    # ... all peripheral drivers
)

# Communication drivers library
add_library(communication STATIC
    Drivers/Communication/w5200/w5200_hal.c
    Drivers/Communication/w5200/w5200_socket.c
    Drivers/Communication/ethernet/eth_transport.c
    Drivers/Communication/fiber_optic/fpga_fsmc_hal.c
    Drivers/Communication/fiber_optic/fiber_transport.c
    Drivers/Communication/fiber_optic/fiber_frame.c
)

# Board drivers library
add_library(board_drivers STATIC
    Drivers/Board/dio_driver.c
    Drivers/Board/aio_driver.c
    Drivers/Board/sync_driver.c
)

# Middleware library
add_library(middleware STATIC
    Middleware/crc_utils.c
    Middleware/data_converter.c
    Middleware/tlv_protocol.c
    Middleware/limit_utils.c
)

# HAL library (depends on all above)
add_library(board_hal STATIC
    HAL/board_hal.c
)
target_link_libraries(board_hal
    peripherals
    communication
    board_drivers
    middleware
)

# Complete driver package
add_library(cn_board_drivers INTERFACE)
target_link_libraries(cn_board_drivers INTERFACE
    peripherals
    communication
    board_drivers
    middleware
    board_hal
)

# User application links against this
target_link_libraries(yeet_embedded_app
    cn_board_drivers
    # ... other libs
)

# Examples
add_executable(example_01_dio Examples/01_simple_dio_test.c)
target_link_libraries(example_01_dio cn_board_drivers)
# ... more examples
```

---

## 🎯 Benefits of New Structure

### ✅ Organization
- Logical grouping by function
- Easy to find what you need
- Clear dependency hierarchy

### ✅ Modularity
- Each driver independent
- Can test in isolation
- Reusable in other projects

### ✅ Scalability
- Easy to add new peripherals
- Easy to add new protocols
- Examples show how to extend

### ✅ Maintainability
- Clear responsibilities
- No circular dependencies
- Application code separate

### ✅ Documentation
- Each directory has purpose
- API organized by layer
- Examples for each feature

---

## 📍 Navigation Guide

### "I want to..."

**...read analog inputs**
→ `Drivers/Board/aio_driver.h`

**...use Ethernet**
→ `Drivers/Communication/ethernet/eth_transport.h`

**...use fiber optic**
→ `Drivers/Communication/fiber_optic/fiber_transport.h`

**...control digital outputs**
→ `Drivers/Board/dio_driver.h`

**...initialize the board**
→ `HAL/board_hal.h`

**...see an example**
→ `Examples/` directory

**...understand the API**
→ `Docs/API_REFERENCE.md` (after Phase 6)

**...migrate old code**
→ `Docs/MIGRATION_GUIDE.md` (after Phase 6)

---

**Last Updated**: October 15, 2025  
**Status**: Target structure defined ✅  
**Next**: Phase 1 - Create directory structure
