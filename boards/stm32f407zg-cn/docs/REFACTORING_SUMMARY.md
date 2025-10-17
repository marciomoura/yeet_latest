/******************************************************************************
 * NSD DRIVER REFACTORING SUMMARY
 * 
 * This document explains the refactoring of the NSD (Nengshida Electric Co.)
 * legacy drivers for STM32F407ZG-CN board external memory interfaces.
 * 
 * DATE: 2025
 * AUTHOR: Refactored from original Chinese-commented code
 *****************************************************************************/

## OVERVIEW

The original NSD driver files contained hardware interfaces for three types of
external memory connected to the STM32F407ZG microcontroller:

1. **NAND Flash** - K9F2G08U0M (256 MB non-volatile storage)
2. **SRAM** - IS61WV25616BLL (512 KB fast volatile memory)
3. **SPI Flash** - M25P32 (4 MB non-volatile storage)

These drivers have been completely refactored from monolithic, Chinese-commented
legacy code into modern, well-documented, instance-based C drivers following
the same architecture pattern as the other refactored drivers in this project.

## WHAT WAS REFACTORED

### 1. NAND Flash Driver (K9F2G08U0M via FSMC Bank 2)

**Original Files:**
- `nandflash_sram.h` - Combined header for both NAND and SRAM
- `nandflash_sram.c` - Implementation with mixed Chinese/English comments

**New Files:**
- `nand_driver.h` - Standalone NAND Flash driver header
- `nand_driver.c` - Modern implementation with comprehensive documentation

**Key Improvements:**
- **Separated concerns:** NAND and SRAM now have independent drivers
- **English documentation:** Complete English comments explaining all operations
- **Hardware details:** Comprehensive pin mapping and timing specifications
- **Instance-based:** No global variables, supports multiple instances
- **Statistics tracking:** Monitors read/write/erase operations and errors
- **Address validation:** Prevents out-of-bounds access
- **Modern API:** Clear function naming (NAND_Init, NAND_ReadPage, etc.)

**Hardware Configuration:**
- Interface: FSMC Bank 2 NAND
- Memory: 2 Gbit (2048 blocks × 64 pages × 2048 bytes)
- Data bus: 8-bit (PD0-1, PD14-15, PE7-10)
- Control: PD11 (A16/CLE), PD12 (A17/ALE), PD4 (NOE), PD5 (NWE), PD7 (NCE2)
- Features: Hardware ECC enabled for 2KB pages

**What the NAND Flash is For:**
- Large-capacity non-volatile storage (256 MB)
- Ideal for data logging, firmware storage, file systems
- Requires block erase before writing (typical NAND Flash behavior)
- Suitable for sequential write applications
- Use case: Event logging, configuration backup, bulk data storage

---

### 2. SRAM Driver (IS61WV25616BLL via FSMC Bank 1)

**Original Files:**
- `nandflash_sram.h` - Combined header
- `nandflash_sram.c` - Implementation

**New Files:**
- `sram_driver.h` - Standalone SRAM driver header
- `sram_driver.c` - Modern implementation

**Key Improvements:**
- **Separated from NAND:** Independent driver with clear purpose
- **Complete documentation:** Hardware specs, timing, pin mapping
- **Simple API:** Direct memory access through clean interface
- **Memory testing:** Built-in function to verify SRAM integrity
- **Fast access:** Optimized timing for 10ns SRAM
- **Fill and test functions:** Convenience functions for common operations

**Hardware Configuration:**
- Interface: FSMC Bank 1, NOR/SRAM Region 1
- Memory: 256K × 16-bit (512 KB total)
- Data bus: 16-bit (PD0-1, PD8-10, PD14-15, PE7-15)
- Address bus: 18-bit (PF0-5, PF12-15, PG0-5, PD11-13, PE3)
- Control: PD4 (NOE), PD5 (NWE), PD7 (NE1), PE0-1 (NBL0-1)
- Timing: Configured for 10ns access time @ 168 MHz HCLK

**What the SRAM is For:**
- Fast temporary data storage (10ns access time)
- Ideal for buffers, large arrays, temporary computations
- No erase required - write directly to any location
- Random access friendly
- Use case: DMA buffers, image processing, real-time data acquisition buffers
- **Note:** Volatile - loses data when power is removed

---

### 3. SPI Flash Driver (M25P32 via SPI1)

**Original Files:**
- `flash.h` - Header with minimal documentation
- `flash.c` - Implementation with mixed comments and state machine

**New Files:**
- `spi_flash_driver.h` - Complete header with detailed documentation
- **Implementation pending:** Full implementation to be completed

**Key Improvements:**
- **Clear purpose:** Documented as M25P32 SPI NOR Flash driver
- **Pin mapping:** All SPI1 pins clearly documented
- **Operation guide:** Typical operation flow explained
- **Timing specs:** Sector erase (0.6-3s), page write (0.6-5ms) documented
- **High-level API planned:** Auto-erase write function with verification

**Hardware Configuration:**
- Interface: SPI1 (up to 75 MHz)
- Pins: PA5 (SCK), PA6 (MISO), PA7 (MOSI), PE5 (CS)
- Memory: 4 MB (64 sectors × 64 KB)
- Page size: 256 bytes (programming unit)
- Features: Sector erase required before write

**What the SPI Flash is For:**
- Non-volatile code and data storage (4 MB)
- Firmware updates, parameter storage, calibration data
- Slower than SRAM but retains data without power
- Requires sector erase before writing
- Use case: Firmware backup, configuration parameters, bootloader storage

---

## ARCHITECTURE IMPROVEMENTS

### Before Refactoring:
```c
// Global variables everywhere
unsigned char M25P32_Write_Flag;
unsigned char M25P32_Write_Result;
int M25P32_Write_Length;

// Mixed language comments
/*******************************************************************************
*��������	NAND_EraseBlock
*����˵��   ��ȫ����һ��NAND Flash��
*******************************************************************************/

// Unclear function names
void NandFlash_Init(void);
uint32_t NAND_WriteSmallPage(...);
```

### After Refactoring:
```c
// Instance-based, no globals
nand_instance_t nand_instance;
sram_instance_t sram_instance;
flash_instance_t flash_instance;

// Complete English documentation
/**
 * @brief Erase a NAND Flash block
 * 
 * Erases all pages in the specified block (sets all bits to 0xFF).
 * This must be done before writing to any page in the block.
 * 
 * @param instance Pointer to NAND driver instance
 * @param address Block address to erase
 * @return NAND_OK if successful, error code otherwise
 */
 
// Clear, consistent naming
nand_status_t NAND_Init(nand_instance_t* instance);
nand_status_t NAND_EraseBlock(nand_instance_t* instance, nand_address_t address);
nand_status_t NAND_WritePage(nand_instance_t* instance, ...);
```

### Key Architectural Changes:

1. **Instance-Based Design**
   - All drivers use instance structures
   - No global variables
   - Supports multiple instances if needed
   - Better testability and maintainability

2. **Statistics Tracking**
   - All drivers track operation counts
   - Error counting for diagnostics
   - Useful for wear monitoring and debugging

3. **Address Validation**
   - All operations validate addresses before access
   - Prevents buffer overruns and hardware errors
   - Clear error codes returned

4. **Comprehensive Documentation**
   - Every function has detailed header comments
   - Hardware specifications at file level
   - Pin mappings clearly documented
   - Typical use cases explained

5. **Consistent Error Handling**
   - Defined status enums (NAND_OK, SRAM_OK, FLASH_OK)
   - Clear error codes (TIMEOUT, INVALID_ADDRESS, ERROR)
   - No silent failures

## WHY THIS MATTERS

### For NAND Flash:
- **Purpose:** Large-capacity storage for logs, files, firmware images
- **Requirement:** Must erase blocks before writing (NAND Flash characteristic)
- **Use case:** "I need to store 100 MB of sensor data for later retrieval"
- **Performance:** Slower writes (block erase takes time), fast reads

### For SRAM:
- **Purpose:** Fast temporary storage for real-time operations
- **Requirement:** None - write anywhere, anytime
- **Use case:** "I need a 512 KB buffer for DMA image capture"
- **Performance:** Very fast (10ns access), but volatile (loses data on power-off)

### For SPI Flash:
- **Purpose:** Non-volatile storage for firmware and configuration
- **Requirement:** Must erase sectors before writing
- **Use case:** "I need to store firmware updates or calibration parameters"
- **Performance:** Medium speed, retains data without power

## USAGE EXAMPLE

```c
// Initialize NAND Flash
nand_instance_t nand;
if (NAND_Init(&nand) != NAND_OK) {
    // Handle error
}

// Erase a block before writing
nand_address_t addr = {.zone = 0, .block = 0, .page = 0};
NAND_EraseBlock(&nand, addr);

// Write data
uint8_t data[2048];
NAND_WritePage(&nand, data, addr, 1);

// Read back
uint8_t readback[2048];
NAND_ReadPage(&nand, readback, addr, 1);

// Get statistics
uint32_t writes, erases, errors;
NAND_GetStatistics(&nand, NULL, &writes, &erases, &errors);
```

## ORIGINAL CODE ISSUES RESOLVED

1. **Chinese Comments:** All converted to English
2. **Global State:** Removed, replaced with instance structures
3. **Mixed Responsibilities:** NAND and SRAM separated
4. **Unclear Naming:** Standardized naming convention
5. **No Error Handling:** Comprehensive error codes added
6. **No Documentation:** Extensive comments and documentation
7. **Magic Numbers:** Replaced with named constants
8. **Hard-Coded Addresses:** Address validation functions added

## BACKWARD COMPATIBILITY

**Option 1: Create Legacy Wrappers**
Similar to previous refactorings, legacy wrapper files can be created:
- `nandflash_sram_legacy.c` - Wraps new drivers with old API
- `flash_legacy.c` - Wraps SPI flash driver

**Option 2: Direct Migration**
Update calling code to use new API directly (recommended for new development).

## FILES SUMMARY

| Original File      | Status | New File(s)                      | Purpose                     |
| ------------------ | ------ | -------------------------------- | --------------------------- |
| `nandflash_sram.h` | LEGACY | `nand_driver.h`, `sram_driver.h` | Split into separate drivers |
| `nandflash_sram.c` | LEGACY | `nand_driver.c`, `sram_driver.c` | Modern implementations      |
| `flash.h`          | LEGACY | `spi_flash_driver.h`             | Comprehensive documentation |
| `flash.c`          | LEGACY | `spi_flash_driver.c` (pending)   | To be implemented           |

## NEXT STEPS

1. **Complete SPI Flash Implementation** - Finish `spi_flash_driver.c`
2. **Create Legacy Wrappers** - If backward compatibility needed
3. **Update Application Code** - Migrate to new APIs
4. **Add Unit Tests** - Test each driver independently
5. **Performance Testing** - Verify timing requirements met

## CONCLUSION

These refactored drivers provide clean, documented, and maintainable interfaces
to the external memory systems on the STM32F407ZG-CN board. The separation of
concerns, instance-based design, and comprehensive documentation make the code
easier to understand, debug, and extend.

The original NSD drivers served their purpose but were difficult to maintain
due to mixed languages, unclear structure, and lack of documentation. The new
drivers follow modern C practices and match the architecture of other drivers
in this project.
