# NSD_Driver Refactoring - Quick Start Guide

**For the complete detailed plan, see: [REFACTORING_PLAN.md](./REFACTORING_PLAN.md)**

---

## 📋 Quick Overview

**Goal**: Transform NSD_Driver from an excitation control application → clean, reusable hardware driver library

**Timeline**: 19 days  
**Status**: ⏸️ Ready to Begin

---

## 🎯 What We're Doing

### ✅ KEEPING (with improvements):
- All hardware peripheral drivers (ADC, DAC, GPIO, UART, SPI, etc.)
- **Ethernet stack** (W5200 chip + TCP/IP)
- **Fiber optic stack** (FPGA interface + multi-channel protocol)
- Digital/analog I/O capabilities
- Sync signal capture

### ❌ REMOVING:
- Excitation system control logic (AVR/FCR/AER modes)
- PID field current regulator
- Reference tracking/ramping
- Application-specific protocols
- 100+ global application variables

### 🔄 REFACTORING:
- Separate hardware drivers from application logic
- Create callback-based event system
- Add configuration structures
- Improve API naming and consistency

---

## 🗂️ New Structure (High-Level)

```
Core/
├── Drivers/              # Hardware drivers
│   ├── Peripherals/      # MCU peripherals (ADC, DAC, GPIO, etc.)
│   ├── Communication/    # Ethernet + Fiber Optic stacks
│   └── Board/            # Board-level abstractions (AIO, DIO, Sync)
│
├── HAL/                  # Board initialization & callbacks
├── Middleware/           # Utilities (CRC, data conversion)
└── Examples/             # 6+ working examples
```

---

## 🚀 Implementation Phases

| #   | Phase                 | Days | Key Deliverables                  |
| --- | --------------------- | ---- | --------------------------------- |
| 1   | Setup                 | 1    | Directory structure, build config |
| 2   | **Ethernet Stack**    | 3    | W5200 driver + transport API      |
| 3   | **Fiber Optic Stack** | 4    | FPGA interface + multi-channel    |
| 4   | Hardware I/O          | 3    | AIO, DIO, Sync drivers            |
| 5   | Board HAL             | 2    | Unified init + callbacks          |
| 6   | Examples & Docs       | 3    | 6 examples + API docs             |
| 7   | Cleanup               | 1    | Remove old code                   |
| 8   | YEET Integration      | 2    | Connect to control system         |

---

## 🔧 Quick API Examples

### Before (Old API):
```c
// Mixed application + hardware code
void Init_Net(void) {
    // Hard-coded IP, ports, sockets
    Self_IP[0] = 192;
    // ...
}

void Net_Check(void) {
    // Application protocol parsing mixed in
    Byte_To_Float(&P, Net_Rx_Buf[0]+14);
    Qav += (Q - Qav) * 0.03f;  // App-specific
}
```

### After (New API):
```c
// Clean hardware driver + user callbacks
void my_rx_handler(uint8_t socket, uint8_t* data, uint16_t len) {
    // User handles their protocol
    float p, q;
    memcpy(&p, data + 14, 4);
    memcpy(&q, data + 18, 4);
    // Feed to YOUR control algorithm
}

int main(void) {
    eth_network_config_t config = {
        .ip = {192, 168, 2, 100},
        .port = 7000
    };
    ETH_Init(SPI2, &config, false);
    ETH_ConfigSocket(0, &sock_config, my_rx_handler);
    
    while(1) {
        ETH_ProcessEvents();
        // Your app logic
    }
}
```

---

## 📦 Key Components

### 1. Ethernet (W5200)
- **Hardware**: W5200 chip driver (SPI)
- **Transport**: TCP/UDP socket management
- **Features**: Dual chip support (SPI2/SPI3), DMA, callbacks
- **API**: `ETH_Init()`, `ETH_ConfigSocket()`, `ETH_Send()`, `ETH_ProcessEvents()`

### 2. Fiber Optic (FPGA/FSMC)
- **Hardware**: FSMC interface to FPGA
- **Transport**: 8-channel multi-point communication
- **Features**: CRC validation, timeout/reconnect, broadcast
- **API**: `FIBER_Init()`, `FIBER_ConfigChannel()`, `FIBER_SendFrame()`, `FIBER_ProcessChannels()`

### 3. Analog I/O
- **Hardware**: AD7606 (8-ch ADC), AD57x4 (4-ch DAC)
- **Features**: Configurable scaling, filtering, callbacks
- **API**: `AIO_Init()`, `AIO_ConfigChannel()`, `AIO_ReadChannel()`, `AIO_WriteChannel()`

### 4. Digital I/O
- **Hardware**: GPIO pins (64+ DI/DO)
- **Features**: Edge detection, callbacks, debouncing
- **API**: `DIO_Init()`, `DIO_ReadInput()`, `DIO_WriteOutput()`, `DIO_ProcessInputs()`

### 5. Sync Signals
- **Hardware**: Timer capture inputs
- **Features**: Period measurement, frequency calculation, 3-phase support
- **API**: `SYNC_Init()`, `SYNC_GetPeriod()`, `SYNC_GetFrequency()`, callbacks

---

## 🎓 Learning Resources

1. **Read First**: [REFACTORING_PLAN.md](./REFACTORING_PLAN.md) - Complete detailed plan
2. **API Reference**: (TBD - Phase 6)
3. **Migration Guide**: (TBD - Phase 6)
4. **Examples**: Check `Examples/` directory after Phase 6

---

## 🏁 How to Get Started

### For Developers Working on Refactoring:

1. **Read the full plan**: [REFACTORING_PLAN.md](./REFACTORING_PLAN.md)
2. **Create feature branch**: `git checkout -b refactor/driver-separation`
3. **Start with Phase 1**: Create directory structure
4. **Follow phases sequentially**: Don't skip ahead
5. **Test continuously**: Each phase has test criteria
6. **Document as you go**: Update API docs

### For Users Waiting to Integrate:

1. **Monitor progress**: Check which phase is active
2. **Review examples**: Once Phase 6 completes
3. **Plan your integration**: Think about your callbacks
4. **Prepare test cases**: What will you test with new API?
5. **Provide feedback**: API usability, missing features

---

## ✅ Success Checklist

By the end of refactoring, we should have:

- [ ] All peripherals accessible via clean API
- [ ] Zero application-specific code in drivers
- [ ] Callback-based event system working
- [ ] Ethernet stack with TCP/UDP support
- [ ] Fiber optic multi-channel working
- [ ] 6+ working examples
- [ ] Complete API documentation
- [ ] YEET integration successful
- [ ] All tests passing
- [ ] Performance benchmarks met

---

## 🆘 Need Help?

- **Questions about the plan**: See [REFACTORING_PLAN.md](./REFACTORING_PLAN.md) Appendix
- **Technical issues**: Check Troubleshooting (TBD - Phase 6)
- **API usage**: See Examples/ directory (Phase 6+)

---

## 📊 Current Status

**Phase**: 0 - Planning Complete ✅  
**Next**: Phase 1 - Setup  
**ETA**: 19 days from start  

**Last Updated**: October 15, 2025

---

**Ready to begin! 🚀**
