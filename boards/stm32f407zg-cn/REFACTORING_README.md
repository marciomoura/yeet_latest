# NSD_Driver Refactoring Documentation

**Complete documentation suite for refactoring NSD_Driver into a clean, reusable hardware driver library**

---

## 📚 Documentation Index

This directory contains comprehensive documentation for the NSD_Driver refactoring project:

### 📋 Planning Documents

1. **[REFACTORING_PLAN.md](./REFACTORING_PLAN.md)** ⭐ START HERE
   - **Detailed implementation plan** (19 days)
   - Phase-by-phase breakdown with timelines
   - API design specifications
   - Testing strategy
   - Success criteria
   - **Read this first for complete understanding**

2. **[REFACTORING_QUICK_START.md](./REFACTORING_QUICK_START.md)**
   - Executive summary and quick reference
   - Key API examples (before/after)
   - High-level component overview
   - Getting started guide
   - **Read this for a quick overview**

### 📊 Reference Documents

3. **[REFACTORING_FILE_CLASSIFICATION.md](./REFACTORING_FILE_CLASSIFICATION.md)**
   - File-by-file breakdown (40 files)
   - Classification: KEEP, REFACTOR, REMOVE
   - Migration map (old → new locations)
   - Quick decision tree
   - **Use this when working on specific files**

4. **[REFACTORING_DIRECTORY_STRUCTURE.md](./REFACTORING_DIRECTORY_STRUCTURE.md)**
   - Visual directory tree (before/after)
   - Detailed folder breakdown with LOC estimates
   - Include path changes
   - Build system structure
   - Navigation guide
   - **Use this to understand the new organization**

### 📈 Progress Tracking

5. **[REFACTORING_PROGRESS.md](./REFACTORING_PROGRESS.md)**
   - Phase-by-phase checklist
   - Progress tracking (0-100%)
   - Schedule and timeline
   - Issues and blockers
   - Success criteria tracking
   - **Update this as work progresses**

---

## 🎯 Quick Navigation

### For Project Managers:
- Read: [REFACTORING_QUICK_START.md](./REFACTORING_QUICK_START.md)
- Track: [REFACTORING_PROGRESS.md](./REFACTORING_PROGRESS.md)
- Timeline: 19 days, 8 phases

### For Developers Starting Refactoring:
1. Read: [REFACTORING_PLAN.md](./REFACTORING_PLAN.md) - Complete details
2. Review: [REFACTORING_FILE_CLASSIFICATION.md](./REFACTORING_FILE_CLASSIFICATION.md) - What to do with each file
3. Reference: [REFACTORING_DIRECTORY_STRUCTURE.md](./REFACTORING_DIRECTORY_STRUCTURE.md) - Where things go
4. Track: [REFACTORING_PROGRESS.md](./REFACTORING_PROGRESS.md) - Mark tasks complete

### For Users Integrating with New API:
- Wait for Phase 6 completion
- See: `Examples/` directory for working code
- See: `Docs/API_REFERENCE.md` (created in Phase 6)
- See: `Docs/MIGRATION_GUIDE.md` (created in Phase 6)

---

## 📖 Document Summary

| Document                           | Pages         | Purpose                      | Audience              |
| ---------------------------------- | ------------- | ---------------------------- | --------------------- |
| REFACTORING_PLAN.md                | ~25           | Complete implementation plan | All team members      |
| REFACTORING_QUICK_START.md         | ~4            | Quick overview               | Managers, new members |
| REFACTORING_FILE_CLASSIFICATION.md | ~8            | File-by-file guide           | Developers            |
| REFACTORING_DIRECTORY_STRUCTURE.md | ~10           | Organization reference       | Developers            |
| REFACTORING_PROGRESS.md            | ~12           | Progress tracking            | All team members      |
| **TOTAL**                          | **~60 pages** | **Complete documentation**   |                       |

---

## 🚀 Project Overview

### Goal
Transform NSD_Driver from an **excitation system control application** into a **clean, reusable hardware driver library**.

### Key Changes
- ✅ **Keep**: All hardware drivers, Ethernet stack, Fiber optic stack
- 🔄 **Refactor**: Separate hardware from application logic
- ❌ **Remove**: Excitation control logic (AVR/FCR/AER, PID, etc.)

### Benefits
- Clean, documented API
- Callback-based event system
- Easy integration with any application
- Reusable for other projects
- Maintainable architecture

---

## 📅 Timeline

| Phase               | Duration    | Key Deliverable             |
| ------------------- | ----------- | --------------------------- |
| 0. Planning         | ✅ Done      | This documentation          |
| 1. Setup            | 1 day       | Directory structure         |
| 2. Ethernet         | 3 days      | W5200 + TCP/IP API          |
| 3. Fiber Optic      | 4 days      | Multi-channel transport     |
| 4. Hardware I/O     | 3 days      | AIO, DIO, Sync drivers      |
| 5. Board HAL        | 2 days      | Unified initialization      |
| 6. Examples & Docs  | 3 days      | 6 examples + API docs       |
| 7. Cleanup          | 1 day       | Remove old code             |
| 8. YEET Integration | 2 days      | Working system              |
| **TOTAL**           | **19 days** | **Complete driver library** |

**Current Status**: 🟡 Phase 0 Complete - Ready to Begin Phase 1

---

## 🗂️ New Structure Preview

```
Core/
├── Drivers/              # Hardware drivers
│   ├── Peripherals/      # MCU peripherals (ADC, DAC, GPIO, etc.)
│   ├── Communication/    # Ethernet + Fiber Optic
│   └── Board/            # Board-level abstractions (AIO, DIO, Sync)
├── HAL/                  # Board initialization & callbacks
├── Middleware/           # Utilities (CRC, data conversion)
├── Examples/             # 6+ working examples
└── Docs/                 # API documentation (Phase 6)
```

---

## 🔍 Key Features of New API

### Ethernet (W5200)
```c
ETH_Init(SPI2, &config, false);
ETH_ConfigSocket(0, &sock_config, my_rx_callback);
ETH_ProcessEvents();  // In main loop
```

### Fiber Optic (FPGA/FSMC)
```c
FIBER_Init();
FIBER_ConfigChannel(&ch_config, my_rx_handler, my_event_handler);
FIBER_SendFrame(channel, data, len);
FIBER_ProcessChannels();  // In main loop
```

### Analog I/O
```c
AIO_Init();
AIO_ConfigChannel(&adc_config);
float value = AIO_ReadChannel(channel);
```

### Digital I/O
```c
DIO_Init();
DIO_ConfigInput(&di_config);
DIO_ConfigOutput(&do_config);
bool state = DIO_ReadInput(pin);
DIO_WriteOutput(pin, true);
```

### Sync Signals
```c
SYNC_Init();
SYNC_ConfigSource(&sync_config);
uint32_t period = SYNC_GetPeriod(source);
float freq = SYNC_GetFrequency(source);
```

---

## ✅ Success Criteria

By the end of refactoring:

- [x] ✅ Planning documentation complete
- [ ] All peripherals accessible via clean API
- [ ] Ethernet stack with TCP/UDP support
- [ ] Fiber optic 8-channel communication
- [ ] 6+ working examples
- [ ] Complete API documentation
- [ ] YEET integration successful
- [ ] Zero application-specific code in drivers
- [ ] Real-time performance <100us
- [ ] All tests passing

---

## 📝 How to Use This Documentation

### Phase 0-1: Before Starting
1. Read all 5 documents to understand scope
2. Get approval on plan
3. Create feature branch
4. Set up tracking system

### Phase 2-7: During Implementation
1. Consult relevant document for current phase
2. Follow checklist in REFACTORING_PROGRESS.md
3. Update progress after each task
4. Commit frequently with clear messages

### Phase 8: Integration
1. Review MIGRATION_GUIDE.md (created in Phase 6)
2. Update application code to use new API
3. Test thoroughly
4. Document integration

### Post-Refactoring: Maintenance
1. Update documentation for any changes
2. Keep examples working
3. Version control with semantic versioning

---

## 🆘 Support

### Questions?
- **About the plan**: See [REFACTORING_PLAN.md](./REFACTORING_PLAN.md) Appendix
- **About a specific file**: See [REFACTORING_FILE_CLASSIFICATION.md](./REFACTORING_FILE_CLASSIFICATION.md)
- **About organization**: See [REFACTORING_DIRECTORY_STRUCTURE.md](./REFACTORING_DIRECTORY_STRUCTURE.md)
- **About progress**: See [REFACTORING_PROGRESS.md](./REFACTORING_PROGRESS.md)

### Issues?
- Document in REFACTORING_PROGRESS.md "Issues and Blockers" section
- Discuss with team
- Update plan if needed

---

## 📊 Document Version History

| Version | Date         | Changes                             |
| ------- | ------------ | ----------------------------------- |
| 1.0     | Oct 15, 2025 | Initial documentation suite created |
|         |              | - Complete refactoring plan         |
|         |              | - Quick start guide                 |
|         |              | - File classification               |
|         |              | - Directory structure               |
|         |              | - Progress tracker                  |

---

## 🎓 Related Resources

### Within This Project
- `boards/stm32f407zg-cn/Core/Inc/ad7606_driver.h` - Example of refactored driver
- `boards/stm32f407zg-cn/Core/NSD_Driver/` - Original code to refactor

### External References
- STM32F407 Reference Manual
- W5200 Datasheet
- AD7606 Datasheet
- FSMC Application Notes

---

## 👥 Contributors

**Project**: YEET Real-Time Control System  
**Board**: STM32F407ZG-CN  
**Organization**: techperts  
**Repository**: yeet  
**Branch**: nsd_driver  

---

## 📄 License

_To be determined by project maintainers_

---

**Document Status**: ✅ Complete and Ready for Use  
**Last Updated**: October 15, 2025  
**Next Review**: After Phase 1 completion  

---

## 🎯 Next Steps

1. **Review all documentation** with team
2. **Get approval** on refactoring plan
3. **Schedule kickoff** meeting
4. **Assign responsibilities** for phases
5. **Create feature branch**: `refactor/driver-separation`
6. **Begin Phase 1**: Setup and directory structure

**Ready to begin! 🚀**
