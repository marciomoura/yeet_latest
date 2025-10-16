# NSD_Driver Refactoring - Progress Tracker

**Track implementation progress phase by phase**

---

## 📅 Project Timeline

**Start Date**: TBD  
**Estimated Duration**: 19 working days  
**Target Completion**: TBD  

**Current Phase**: 🟡 Phase 0 - Planning Complete  
**Overall Progress**: ⬜⬜⬜⬜⬜⬜⬜⬜⬜⬜ 0%

---

## 📋 Phase Checklist

### Phase 0: Planning ✅ COMPLETE
- [x] Create refactoring plan document
- [x] Create quick start guide
- [x] Create file classification document
- [x] Create directory structure document
- [x] Create progress tracker (this file)
- [x] Review and approve plan

**Status**: ✅ Complete  
**Date Completed**: October 15, 2025

---

### Phase 1: Setup and Planning (1 day) 🔲 NOT STARTED
**Target**: Day 1  
**Progress**: ⬜⬜⬜⬜⬜⬜⬜⬜⬜⬜ 0%

#### Tasks:
- [ ] Create feature branch `refactor/driver-separation`
- [ ] Create new directory structure
  - [ ] `Drivers/Peripherals/`
  - [ ] `Drivers/Communication/w5200/`
  - [ ] `Drivers/Communication/ethernet/`
  - [ ] `Drivers/Communication/fiber_optic/`
  - [ ] `Drivers/Board/`
  - [ ] `HAL/`
  - [ ] `Middleware/`
  - [ ] `Examples/`
  - [ ] `Docs/`
- [ ] Update CMakeLists.txt for new structure
- [ ] Create stub header files with API definitions
- [ ] Verify build system compiles (empty stubs)
- [ ] Document current API usage patterns
- [ ] Commit: "Phase 1: Setup directory structure"

#### Deliverables:
- [ ] Directory tree created
- [ ] Build system ready for incremental migration
- [ ] Git branch created and pushed

**Started**: ___________  
**Completed**: ___________

---

### Phase 2: Ethernet Communication Stack (3 days) 🔲 NOT STARTED
**Target**: Days 2-4  
**Progress**: ⬜⬜⬜⬜⬜⬜⬜⬜⬜⬜ 0%

#### Day 1: W5200 Hardware Layer
- [ ] Review existing `w5200.c/h` implementation
- [ ] Create `w5200_hal.c/h` - Low-level driver
  - [ ] SPI initialization
  - [ ] Register read/write functions
  - [ ] Buffer read/write functions
  - [ ] Chip detect/reset functions
- [ ] Create `w5200_socket.c/h` - Socket abstraction
  - [ ] Socket open/close/bind
  - [ ] Socket listen/connect
  - [ ] Socket send/recv
  - [ ] Socket state machine
- [ ] Update `w5200_reg.h` - Clean register definitions
- [ ] Add dual SPI support (SPI2/SPI3)
- [ ] Test: SPI communication verified
- [ ] Test: Socket state transitions work
- [ ] Commit: "Phase 2.1: W5200 hardware layer"

#### Day 2: Ethernet Transport Layer
- [ ] Analyze `net.c` to identify transport vs. application
- [ ] Create `eth_transport.c/h`
  - [ ] Network initialization
  - [ ] Socket configuration
  - [ ] TX/RX functions
  - [ ] Event processing loop
- [ ] Create `eth_config.h` - Configuration structures
- [ ] Create `eth_callbacks.h` - Callback definitions
- [ ] Remove application protocol (P, Q parsing)
- [ ] Implement callback dispatcher
- [ ] Test: Server mode works
- [ ] Test: Client mode works
- [ ] Test: Callbacks triggered correctly
- [ ] Commit: "Phase 2.2: Ethernet transport layer"

#### Day 3: Ethernet Examples
- [ ] Create `03_ethernet_echo_server.c`
  - [ ] TCP echo server implementation
  - [ ] Connection handling
  - [ ] Error handling
- [ ] Test: Echo server functional
- [ ] Test: Multiple clients supported
- [ ] Test: Connection recovery works
- [ ] Create `03_ethernet_client.c` (bonus)
- [ ] Document Ethernet API usage
- [ ] Commit: "Phase 2.3: Ethernet examples"

**Progress Check**:
- [ ] All tests passing
- [ ] API documented
- [ ] Examples working
- [ ] Code reviewed

**Started**: ___________  
**Completed**: ___________

---

### Phase 3: Fiber Optic Communication Stack (4 days) 🔲 NOT STARTED
**Target**: Days 5-8  
**Progress**: ⬜⬜⬜⬜⬜⬜⬜⬜⬜⬜ 0%

#### Day 1: FPGA FSMC Interface
- [ ] Review `optical_fiber.c` FSMC initialization
- [ ] Create `fpga_fsmc_hal.c/h`
  - [ ] FSMC initialization
  - [ ] GPIO pin configuration
  - [ ] Memory-mapped read/write
  - [ ] FPGA register access
- [ ] Test: FSMC timing correct
- [ ] Test: Read/write operations work
- [ ] Commit: "Phase 3.1: FPGA FSMC interface"

#### Day 2: Extract Protocol Utilities
- [ ] Analyze `optical.c` for utilities
- [ ] Create `Middleware/crc_utils.c/h`
  - [ ] CRC16 calculation
  - [ ] CRC16 verification
  - [ ] CRC16 append
  - [ ] Unit tests
- [ ] Create `Middleware/data_converter.c/h`
  - [ ] Float to bytes (big/little endian)
  - [ ] Bytes to float
  - [ ] Int32/Int16 conversions
  - [ ] Unit tests
- [ ] Create `Middleware/tlv_protocol.c/h` (optional)
- [ ] Test: CRC matches original
- [ ] Test: Float conversion accurate
- [ ] Commit: "Phase 3.2: Protocol utilities"

#### Day 3-4: Fiber Transport Layer
- [ ] Analyze `optical.c` communication structure (1800 lines!)
- [ ] Create `fiber_frame.c/h`
  - [ ] Frame structure definition
  - [ ] Frame building
  - [ ] Frame parsing
  - [ ] Frame validation
- [ ] Create `fiber_transport.c/h`
  - [ ] Multi-channel management (8 channels)
  - [ ] TX/RX state machines
  - [ ] Timeout handling
  - [ ] Connection management
  - [ ] Event callbacks
- [ ] Create `fiber_callbacks.h` - Event definitions
- [ ] Remove excitation-specific protocol commands
- [ ] Test: 8-channel communication
- [ ] Test: CRC validation
- [ ] Test: Timeout/reconnect logic
- [ ] Test: Broadcast functionality
- [ ] Create `04_fiber_optic_bridge.c` example
- [ ] Document Fiber API usage
- [ ] Commit: "Phase 3.3: Fiber optic transport"

**Progress Check**:
- [ ] All 8 channels working
- [ ] CRC validation reliable
- [ ] Callbacks triggered correctly
- [ ] Example demonstrates usage

**Started**: ___________  
**Completed**: ___________

---

### Phase 4: Hardware I/O Abstraction (3 days) 🔲 NOT STARTED
**Target**: Days 9-11  
**Progress**: ⬜⬜⬜⬜⬜⬜⬜⬜⬜⬜ 0%

#### Day 1: Analog Input Driver
- [ ] Analyze `AI.c` (570 lines)
- [ ] Create `aio_driver.c/h`
  - [ ] Channel configuration
  - [ ] ADC acquisition control
  - [ ] Scaling/offset application
  - [ ] Filtering (moving average)
  - [ ] Sample callbacks
  - [ ] Buffer callbacks
- [ ] Remove excitation calculations (Vfd, Ifd, P, Q)
- [ ] Test: 8-channel acquisition
- [ ] Test: Scaling applied correctly
- [ ] Test: Callbacks triggered
- [ ] Create `02_adc_dac_loopback.c` example
- [ ] Commit: "Phase 4.1: Analog I/O driver"

#### Day 2: Digital I/O Driver
- [ ] Analyze `DIDO.c`
- [ ] Create `dio_driver.c/h`
  - [ ] Input configuration
  - [ ] Output configuration
  - [ ] Pin read/write
  - [ ] Edge detection
  - [ ] Input callbacks
  - [ ] Edge callbacks
- [ ] Remove SCR checking logic
- [ ] Remove breaker/fault logic
- [ ] Test: Input reading works
- [ ] Test: Output setting works
- [ ] Test: Edge detection works
- [ ] Create `01_simple_dio_test.c` example
- [ ] Commit: "Phase 4.2: Digital I/O driver"

#### Day 3: Synchronization Signal Driver
- [ ] Analyze `syn.c` (471 lines)
- [ ] Create `sync_driver.c/h`
  - [ ] Source configuration (A/B/C/Ext)
  - [ ] Interrupt timing capture
  - [ ] Period measurement
  - [ ] Frequency calculation
  - [ ] Event callbacks
  - [ ] Period callbacks
- [ ] Remove SCR pulse generation
- [ ] Remove firing angle control
- [ ] Test: Period measurement accurate
- [ ] Test: Frequency calculation correct
- [ ] Test: Source switching works
- [ ] Create `05_sync_signal_capture.c` example
- [ ] Commit: "Phase 4.3: Sync signal driver"

**Progress Check**:
- [ ] All I/O drivers functional
- [ ] Examples demonstrate usage
- [ ] No application logic remaining

**Started**: ___________  
**Completed**: ___________

---

### Phase 5: Board-Level HAL (2 days) 🔲 NOT STARTED
**Target**: Days 12-13  
**Progress**: ⬜⬜⬜⬜⬜⬜⬜⬜⬜⬜ 0%

#### Day 1: Board HAL Implementation
- [ ] Analyze `para.c` initialization sequences
- [ ] Create `board_hal.c/h`
  - [ ] `Board_Init()` function
  - [ ] Clock configuration
  - [ ] Peripheral enables
  - [ ] Node ID setup
  - [ ] Status readback
  - [ ] Reset/low-power functions
- [ ] Create `board_config.h`
  - [ ] Master configuration structure
  - [ ] Peripheral enable flags
  - [ ] Network settings
- [ ] Create `board_types.h`
  - [ ] Common type definitions
  - [ ] Status structures
- [ ] Test: All peripherals initialize
- [ ] Test: Status readback works
- [ ] Commit: "Phase 5.1: Board HAL"

#### Day 2: Callback Registration System
- [ ] Create `board_callbacks.h`
  - [ ] Master callback structure
  - [ ] All driver callback types
  - [ ] System callbacks
- [ ] Implement callback registration API
- [ ] Implement callback dispatcher
- [ ] Add priority handling (if needed)
- [ ] Test: All callback paths verified
- [ ] Test: No callbacks missed
- [ ] Update `06_full_integration.c` example
- [ ] Commit: "Phase 5.2: Callback system"

**Progress Check**:
- [ ] Single `Board_Init()` call works
- [ ] All callbacks registered
- [ ] Full integration example functional

**Started**: ___________  
**Completed**: ___________

---

### Phase 6: Examples and Documentation (3 days) 🔲 NOT STARTED
**Target**: Days 14-16  
**Progress**: ⬜⬜⬜⬜⬜⬜⬜⬜⬜⬜ 0%

#### Day 1: Complete Examples
- [ ] Review all examples created so far
- [ ] Enhance `01_simple_dio_test.c`
- [ ] Enhance `02_adc_dac_loopback.c`
- [ ] Enhance `03_ethernet_echo_server.c`
- [ ] Enhance `04_fiber_optic_bridge.c`
- [ ] Enhance `05_sync_signal_capture.c`
- [ ] Complete `06_full_integration.c`
  - [ ] All drivers demonstrated
  - [ ] Proper error handling
  - [ ] Performance monitoring
- [ ] Test all examples on hardware
- [ ] Commit: "Phase 6.1: Complete examples"

#### Day 2: Advanced Integration Example
- [ ] Create comprehensive `06_full_integration.c`
  - [ ] Board initialization
  - [ ] All drivers configured
  - [ ] All callbacks registered
  - [ ] Event loop implementation
  - [ ] Error handling
  - [ ] Performance monitoring
  - [ ] Comments explaining usage
- [ ] Test: Integration example works
- [ ] Commit: "Phase 6.2: Advanced integration"

#### Day 3: Documentation
- [ ] Create `Docs/API_REFERENCE.md`
  - [ ] All driver APIs documented
  - [ ] Function descriptions
  - [ ] Parameter descriptions
  - [ ] Return values
  - [ ] Usage notes
- [ ] Create `Docs/MIGRATION_GUIDE.md`
  - [ ] Old vs new API comparison
  - [ ] Migration steps
  - [ ] Code examples
- [ ] Create `Docs/INTEGRATION_GUIDE.md`
  - [ ] How to integrate with applications
  - [ ] Callback patterns
  - [ ] Best practices
- [ ] Create `Docs/TIMING_ANALYSIS.md`
  - [ ] Real-time performance
  - [ ] Interrupt latencies
  - [ ] Loop timing
- [ ] Create `Docs/TROUBLESHOOTING.md`
  - [ ] Common issues
  - [ ] Debug tips
  - [ ] FAQ
- [ ] Create `Docs/PIN_MAPPING.md`
  - [ ] Complete pin assignments
  - [ ] Alternate functions
  - [ ] Conflicts
- [ ] Commit: "Phase 6.3: Documentation"

**Progress Check**:
- [ ] All examples working
- [ ] Documentation complete
- [ ] API fully documented

**Started**: ___________  
**Completed**: ___________

---

### Phase 7: Remove Application Code (1 day) 🔲 NOT STARTED
**Target**: Day 17  
**Progress**: ⬜⬜⬜⬜⬜⬜⬜⬜⬜⬜ 0%

#### Tasks:
- [ ] Create archive directory for old code
- [ ] Move old files to archive:
  - [ ] `mode.c/h`
  - [ ] `eIfd.c/h`
  - [ ] `ref.c/h`
  - [ ] `command.c/h`
  - [ ] `pub.c/h`
  - [ ] `para.c/h` (keep init only)
  - [ ] Old `net.c/h`
  - [ ] Old `optical.c/h`
  - [ ] Old `AI.c/h`
  - [ ] Old `DIDO.c/h`
  - [ ] Old `syn.c/h`
- [ ] Update CMakeLists.txt to use only new structure
- [ ] Remove old include paths
- [ ] Verify no broken dependencies
- [ ] Run all tests
- [ ] Clean up unused code
- [ ] Commit: "Phase 7: Remove application code"

**Progress Check**:
- [ ] Build succeeds with new structure only
- [ ] All tests passing
- [ ] No old code references remaining

**Started**: ___________  
**Completed**: ___________

---

### Phase 8: Integration with YEET (2 days) 🔲 NOT STARTED
**Target**: Days 18-19  
**Progress**: ⬜⬜⬜⬜⬜⬜⬜⬜⬜⬜ 0%

#### Day 1: YEET Integration
- [ ] Update YEET embedded app to use new driver API
- [ ] Implement YEET-specific callbacks
  - [ ] ADC sample callback
  - [ ] Sync event callback
  - [ ] Ethernet RX callback
  - [ ] Fiber RX callback (if used)
- [ ] Map YEET control signals to hardware
  - [ ] ADC channels for measurements
  - [ ] DAC channels for control outputs
  - [ ] DIO for status/control
- [ ] Update YEET main loop
- [ ] Test: Basic integration works
- [ ] Commit: "Phase 8.1: YEET integration"

#### Day 2: Testing and Benchmarking
- [ ] Test complete YEET system
  - [ ] State observer fed by ADC
  - [ ] Controller output to DAC
  - [ ] Sync timing correct
  - [ ] Communication working
- [ ] Performance benchmarking
  - [ ] Control loop timing
  - [ ] Interrupt latencies
  - [ ] CPU usage
  - [ ] Memory usage
- [ ] Optimization if needed
- [ ] Document YEET integration
- [ ] Commit: "Phase 8.2: YEET testing and optimization"

**Progress Check**:
- [ ] YEET control system functional
- [ ] Performance meets requirements
- [ ] Integration documented

**Started**: ___________  
**Completed**: ___________

---

## 📊 Overall Progress Summary

### Phases Completed: 1 / 9 (11%)

| Phase               | Status        | Days | Progress |
| ------------------- | ------------- | ---- | -------- |
| 0. Planning         | ✅ Complete    | -    | 100%     |
| 1. Setup            | 🔲 Not Started | 1    | 0%       |
| 2. Ethernet         | 🔲 Not Started | 3    | 0%       |
| 3. Fiber Optic      | 🔲 Not Started | 4    | 0%       |
| 4. Hardware I/O     | 🔲 Not Started | 3    | 0%       |
| 5. Board HAL        | 🔲 Not Started | 2    | 0%       |
| 6. Examples & Docs  | 🔲 Not Started | 3    | 0%       |
| 7. Cleanup          | 🔲 Not Started | 1    | 0%       |
| 8. YEET Integration | 🔲 Not Started | 2    | 0%       |

---

## 🎯 Key Milestones

- [ ] **Milestone 1**: Directory structure created (Phase 1)
- [ ] **Milestone 2**: Communication stacks refactored (Phase 2-3)
- [ ] **Milestone 3**: All I/O drivers working (Phase 4)
- [ ] **Milestone 4**: Unified board HAL functional (Phase 5)
- [ ] **Milestone 5**: Documentation complete (Phase 6)
- [ ] **Milestone 6**: Old code removed (Phase 7)
- [ ] **Milestone 7**: YEET integration successful (Phase 8)

---

## 🐛 Issues and Blockers

### Open Issues:
_None yet - project not started_

### Resolved Issues:
_To be filled during implementation_

---

## 📝 Notes and Decisions

### Design Decisions:
- Callback-based architecture for event handling
- Separate transport from application protocol
- Middleware for reusable utilities
- Examples for each major feature

### Technical Decisions:
- Keep dual W5200 support (SPI2/SPI3)
- Support 8 fiber optic channels
- Configuration via structures, not #defines
- Doxygen-compatible documentation

---

## ✅ Success Criteria Tracking

### Functional Requirements:
- [ ] All hardware peripherals accessible via clean API
- [ ] Ethernet stack supports TCP/UDP with callbacks
- [ ] Fiber optic supports 8 channels
- [ ] ADC sampling at required rate
- [ ] DAC output with precise timing
- [ ] Digital I/O with edge detection
- [ ] Sync signal capture with <1us accuracy

### Non-Functional Requirements:
- [ ] Real-time loop timing: <100us deterministic
- [ ] Memory footprint: <50KB RAM, <200KB Flash
- [ ] CPU usage: <50% at maximum throughput
- [ ] API documentation: 100% coverage
- [ ] Example code: 6+ working examples
- [ ] Zero application-specific code in drivers

### Quality Metrics:
- [ ] Code coverage: >80% for drivers
- [ ] No magic numbers
- [ ] Consistent naming conventions
- [ ] Doxygen-compatible comments
- [ ] MISRA-C compliance (where applicable)

---

## 📅 Schedule Tracking

| Date         | Phase | Activity       | Hours | Notes             |
| ------------ | ----- | -------------- | ----- | ----------------- |
| Oct 15, 2025 | 0     | Planning       | 4     | Documents created |
| _TBD_        | 1     | Setup          | 8     |                   |
| _TBD_        | 2     | Ethernet Day 1 | 8     |                   |
| _TBD_        | 2     | Ethernet Day 2 | 8     |                   |
| _TBD_        | 2     | Ethernet Day 3 | 8     |                   |
| _TBD_        | 3     | Fiber Day 1    | 8     |                   |
| _TBD_        | 3     | Fiber Day 2    | 8     |                   |
| _TBD_        | 3     | Fiber Day 3-4  | 16    |                   |
| _TBD_        | 4     | I/O Day 1      | 8     |                   |
| _TBD_        | 4     | I/O Day 2      | 8     |                   |
| _TBD_        | 4     | I/O Day 3      | 8     |                   |
| _TBD_        | 5     | HAL Day 1      | 8     |                   |
| _TBD_        | 5     | HAL Day 2      | 8     |                   |
| _TBD_        | 6     | Docs Day 1     | 8     |                   |
| _TBD_        | 6     | Docs Day 2     | 8     |                   |
| _TBD_        | 6     | Docs Day 3     | 8     |                   |
| _TBD_        | 7     | Cleanup        | 8     |                   |
| _TBD_        | 8     | YEET Day 1     | 8     |                   |
| _TBD_        | 8     | YEET Day 2     | 8     |                   |

**Total Hours Planned**: 152 hours (19 days × 8 hours)  
**Hours Completed**: 4 hours (planning)  
**Hours Remaining**: 148 hours

---

## 🎓 Lessons Learned

_To be filled during implementation_

---

## 📞 Contact and Resources

**Project Lead**: YEET Development Team  
**Documentation**: See `REFACTORING_PLAN.md` for details  
**Quick Start**: See `REFACTORING_QUICK_START.md`  

---

**Status Legend**:
- ✅ Complete
- 🔄 In Progress
- 🔲 Not Started
- ⚠️ Blocked
- ❌ Failed

**Last Updated**: October 15, 2025  
**Next Update**: When Phase 1 begins
