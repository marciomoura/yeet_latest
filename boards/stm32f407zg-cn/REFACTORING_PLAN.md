# NSD_Driver Refactoring Plan

**Project**: YEET Real-Time Control System  
**Board**: STM32F407ZG-CN  
**Date**: October 15, 2025  
**Branch**: nsd_driver  

---

## Executive Summary

Refactor the NSD_Driver from an excitation system control application into a **clean, reusable hardware driver library** that separates:
- ✅ **Hardware drivers** and peripheral interfaces (KEEP & IMPROVE)
- ✅ **Communication infrastructure** (Ethernet, Fiber Optic) (KEEP & IMPROVE)
- ❌ **Application-specific control logic** (AVR/FCR/AER, PID, reference tracking) (REMOVE)

**Goal**: Create a callback-based driver API that allows easy integration of user applications (like YEET's state feedback controller) while maintaining robust communication and I/O capabilities.

---

## Current State Analysis

### File Inventory (40 files in NSD_Driver/)

#### Category 1: Pure Hardware Drivers (13 files) ✅ KEEP
- `AD7606.c/h` - 8-channel ADC via FSMC
- `AD57x4.c/h` - 4-channel DAC via SPI
- `ADC.c/h` - Internal STM32 ADC
- `GPIO.c/h` - GPIO operations
- `TIM.c/h` - Timer peripherals
- `USART_T.c/h`, `USARTx_1.c`, `USARTx.c/h` - Serial communication
- `M25P32.c/h` - SPI Flash memory driver
- `nandflash_sram.c/h` - External memory interface
- `WatchDog.c/h` - Watchdog timer
- `temperature.c/h` - Temperature monitoring

#### Category 2: Communication Infrastructure (8 files) ✅ KEEP & REFACTOR
**Ethernet Stack:**
- `w5200.c/h` - W5200 Ethernet chip driver (dual SPI)
- `w5200Reg.h` - Register definitions
- `net.c/h` - Network socket management + application protocol

**Fiber Optic Stack:**
- `optical_fiber.c/h` - FSMC interface to FPGA
- `optical.c/h` - Multi-channel fiber protocol (1800+ lines)

#### Category 3: Hardware I/O with Application Logic (8 files) 🔄 REFACTOR
- `DIDO.c/h` - Digital I/O + SCR fault checking + breaker logic
- `AI.c/h` - Analog input + excitation calculations (Ifd, Vfd, P, Q)
- `syn.c/h` - Sync signal capture + SCR firing control
- `limit.c/h` - Limit checking with excitation-specific ranges

#### Category 4: Application Control Logic (10 files) ❌ REMOVE
- `mode.c/h` - AVR/FCR/AER mode switching (excitation control modes)
- `eIfd.c/h` - PID field current controller
- `ref.c/h` - Reference tracking and ramping
- `command.c/h` - RS232 command protocol parser
- `para.c/h` - Application parameter initialization
- `pub.c/h` - 100+ global application variables (Ifd, Vfd, Alpha, etc.)

---

## Target Architecture

### New Directory Structure

```
boards/stm32f407zg-cn/Core/
│
├── Drivers/                           # Low-level hardware drivers
│   │
│   ├── Peripherals/                   # MCU peripheral drivers
│   │   ├── ad7606_driver.c/h          # 8-ch ADC (already refactored)
│   │   ├── ad57x4_driver.c/h          # 4-ch DAC
│   │   ├── adc_internal_driver.c/h    # Internal ADC
│   │   ├── gpio_driver.c/h            # GPIO utilities
│   │   ├── timer_driver.c/h           # Timer/PWM
│   │   ├── usart_driver.c/h           # Serial UART
│   │   ├── spi_flash_driver.c/h       # M25P32 Flash
│   │   ├── fsmc_sram_driver.c/h       # External memory
│   │   ├── watchdog_driver.c/h        # IWDG/WWDG
│   │   └── temperature_driver.c/h     # Temp sensors
│   │
│   ├── Communication/                 # Communication stacks
│   │   │
│   │   ├── w5200/                     # W5200 Ethernet chip
│   │   │   ├── w5200_hal.c/h          # Low-level SPI register access
│   │   │   ├── w5200_reg.h            # Register map definitions
│   │   │   └── w5200_socket.c/h       # Socket abstraction layer
│   │   │
│   │   ├── ethernet/                  # TCP/IP transport layer
│   │   │   ├── eth_transport.c/h      # Socket management, TX/RX
│   │   │   ├── eth_config.h           # Configuration structures
│   │   │   └── eth_callbacks.h        # Callback type definitions
│   │   │
│   │   └── fiber_optic/               # Fiber optic via FPGA
│   │       ├── fpga_fsmc_hal.c/h      # FSMC interface to FPGA
│   │       ├── fiber_transport.c/h    # Multi-channel transport
│   │       ├── fiber_frame.c/h        # Frame parsing, validation
│   │       └── fiber_callbacks.h      # Event callback definitions
│   │
│   └── Board/                         # Board-level abstractions
│       ├── dio_driver.c/h             # Digital I/O abstraction
│       ├── aio_driver.c/h             # Analog I/O abstraction
│       └── sync_driver.c/h            # Synchronization signals
│
├── HAL/                               # Hardware Abstraction Layer
│   ├── board_hal.c/h                  # Complete board initialization
│   ├── board_config.h                 # Configuration structures
│   ├── board_callbacks.h              # User callback interfaces
│   └── board_types.h                  # Common type definitions
│
├── Middleware/                        # Protocol utilities
│   ├── crc_utils.c/h                  # CRC16/CRC32 calculations
│   ├── data_converter.c/h             # Float<->Byte conversion
│   └── tlv_protocol.c/h               # Type-Length-Value parsing
│
└── Examples/                          # Integration examples
    ├── 01_simple_dio_test.c           # Basic digital I/O
    ├── 02_adc_dac_loopback.c          # Analog loopback test
    ├── 03_ethernet_echo_server.c      # TCP echo server
    ├── 04_fiber_optic_bridge.c        # Multi-channel fiber comm
    ├── 05_sync_signal_capture.c       # Sync signal timing
    └── 06_full_integration.c          # Complete system demo
```

---

## Phase-by-Phase Implementation Plan

### Phase 1: Setup and Planning (1 day)
**Goal**: Prepare the workspace and create the new structure

#### Tasks:
1. ✅ Create refactoring plan document (this file)
2. Create new directory structure (empty stub files)
3. Set up build system modifications (CMakeLists.txt)
4. Create version control strategy (git branches)
5. Document current API usage patterns

**Deliverables**:
- [ ] Directory structure created
- [ ] Build system ready for incremental migration
- [ ] Git branch: `refactor/driver-separation`

---

### Phase 2: Communication Stack - Ethernet (3 days)
**Goal**: Refactor W5200/Ethernet stack into clean transport API

#### 2.1 W5200 Hardware Layer (Day 1)
**Input**: `w5200.c/h`, `w5200Reg.h`  
**Output**: `Drivers/Communication/w5200/`

**Tasks**:
1. Review existing W5200 functions
2. Create `w5200_hal.c/h` - Low-level SPI operations
   - Register read/write
   - Initialization
   - Chip detect/reset
3. Create `w5200_socket.c/h` - Socket abstraction
   - Socket open/close/bind/listen/connect
   - Data send/receive primitives
   - Status checking
4. Update `w5200_reg.h` - Clean register definitions
5. Add support for dual W5200 (SPI2/SPI3)

**API Design**:
```c
// w5200_hal.h
bool W5200_Init(SPI_TypeDef* spi, GPIO_TypeDef* cs_port, uint16_t cs_pin);
uint8_t W5200_ReadReg(uint16_t addr);
void W5200_WriteReg(uint16_t addr, uint8_t data);
void W5200_ReadBuffer(uint16_t addr, uint8_t* buf, uint16_t len);
void W5200_WriteBuffer(uint16_t addr, uint8_t* buf, uint16_t len);

// w5200_socket.h
typedef enum {
    W5200_SOCK_CLOSED = 0,
    W5200_SOCK_INIT,
    W5200_SOCK_LISTEN,
    W5200_SOCK_ESTABLISHED,
    // ...
} w5200_socket_state_t;

bool W5200_Socket_Open(uint8_t sock, uint8_t protocol, uint16_t port);
bool W5200_Socket_Listen(uint8_t sock);
bool W5200_Socket_Connect(uint8_t sock, uint8_t* ip, uint16_t port);
uint16_t W5200_Socket_Send(uint8_t sock, uint8_t* data, uint16_t len);
uint16_t W5200_Socket_Recv(uint8_t sock, uint8_t* buf, uint16_t len);
w5200_socket_state_t W5200_Socket_GetState(uint8_t sock);
void W5200_Socket_Close(uint8_t sock);
```

**Testing**:
- [ ] SPI communication verified
- [ ] Socket state machine tested
- [ ] Dual chip instance verified

---

#### 2.2 Ethernet Transport Layer (Day 2)
**Input**: `net.c/h`  
**Output**: `Drivers/Communication/ethernet/`

**Tasks**:
1. Analyze `net.c` to separate transport from application
2. Create `eth_transport.c/h` - High-level API
   - Network initialization
   - Socket configuration
   - Callback registration
   - Event processing loop
3. Create `eth_config.h` - Configuration structures
4. Create `eth_callbacks.h` - Callback definitions
5. Remove application-specific protocol (P, Q parsing)

**API Design**:
```c
// eth_config.h
typedef struct {
    uint8_t  ip[4];           // IP address
    uint8_t  subnet[4];       // Subnet mask
    uint8_t  gateway[4];      // Gateway
    uint8_t  mac[6];          // MAC address
    uint16_t timeout_ms;      // Connection timeout
} eth_network_config_t;

typedef struct {
    uint8_t  socket_id;       // 0-7
    bool     enabled;
    bool     is_server;       // true=server, false=client
    uint16_t local_port;
    uint8_t  remote_ip[4];    // For client mode
    uint16_t remote_port;     // For client mode
    uint16_t rx_buffer_size;
    uint16_t tx_buffer_size;
} eth_socket_config_t;

// eth_callbacks.h
typedef void (*eth_rx_callback_t)(uint8_t socket, uint8_t* data, uint16_t len);
typedef void (*eth_tx_complete_callback_t)(uint8_t socket, uint16_t bytes_sent);
typedef void (*eth_connection_callback_t)(uint8_t socket, bool connected);
typedef void (*eth_error_callback_t)(uint8_t socket, uint8_t error_code);

typedef struct {
    eth_rx_callback_t           on_receive;
    eth_tx_complete_callback_t  on_tx_complete;
    eth_connection_callback_t   on_connection_change;
    eth_error_callback_t        on_error;
} eth_socket_callbacks_t;

// eth_transport.h
bool ETH_Init(SPI_TypeDef* spi, eth_network_config_t* config, bool enable_dma);
bool ETH_ConfigSocket(uint8_t socket_id, 
                      eth_socket_config_t* config,
                      eth_socket_callbacks_t* callbacks);
bool ETH_Send(uint8_t socket_id, uint8_t* data, uint16_t len);
bool ETH_IsConnected(uint8_t socket_id);
uint16_t ETH_GetRxAvailable(uint8_t socket_id);
void ETH_ProcessEvents(void);  // Call in main loop or timer
void ETH_SetNodeID(uint8_t node_id);  // For IP assignment
```

**Testing**:
- [ ] Server mode verified
- [ ] Client mode verified
- [ ] Callbacks triggered correctly
- [ ] Dual instance tested (SPI2/SPI3)

---

#### 2.3 Example Application (Day 3)
**Output**: `Examples/03_ethernet_echo_server.c`

**Tasks**:
1. Create echo server example
2. Create client example
3. Create dual-interface example
4. Document API usage

**Example Code**:
```c
// 03_ethernet_echo_server.c
void my_rx_handler(uint8_t socket, uint8_t* data, uint16_t len) {
    // Echo back received data
    ETH_Send(socket, data, len);
}

void my_connection_handler(uint8_t socket, bool connected) {
    if (connected) {
        printf("Client connected to socket %d\n", socket);
    }
}

int main(void) {
    // Network configuration
    eth_network_config_t net_config = {
        .ip = {192, 168, 2, 100},
        .subnet = {255, 255, 255, 0},
        .gateway = {192, 168, 2, 1},
        .mac = {0x00, 0x08, 0xDC, 0x01, 0x02, 0x03},
        .timeout_ms = 5000
    };
    
    // Initialize Ethernet
    ETH_Init(SPI2, &net_config, false);
    
    // Configure socket 0 as TCP server
    eth_socket_config_t sock_config = {
        .socket_id = 0,
        .enabled = true,
        .is_server = true,
        .local_port = 7000,
        .rx_buffer_size = 2048,
        .tx_buffer_size = 2048
    };
    
    eth_socket_callbacks_t callbacks = {
        .on_receive = my_rx_handler,
        .on_connection_change = my_connection_handler,
        .on_tx_complete = NULL,
        .on_error = NULL
    };
    
    ETH_ConfigSocket(0, &sock_config, &callbacks);
    
    while(1) {
        ETH_ProcessEvents();
        // User application code
    }
}
```

**Testing**:
- [ ] Echo server works
- [ ] Multiple clients supported
- [ ] Connection recovery tested

---

### Phase 3: Communication Stack - Fiber Optic (4 days)
**Goal**: Refactor fiber optic communication stack

#### 3.1 FPGA FSMC Interface (Day 1)
**Input**: `optical_fiber.c/h`  
**Output**: `Drivers/Communication/fiber_optic/fpga_fsmc_hal.c/h`

**Tasks**:
1. Extract FSMC initialization from `optical_fiber.c`
2. Create clean FPGA memory-mapped I/O interface
3. Add pin configuration for FSMC
4. Add FPGA read/write primitives

**API Design**:
```c
// fpga_fsmc_hal.h
#define FPGA_BASE_ADDR    0x6C000000U

typedef enum {
    FPGA_BANK_1 = 0,
    FPGA_BANK_2,
    FPGA_BANK_3,
    FPGA_BANK_4
} fpga_bank_t;

bool FPGA_Init(void);
void FPGA_WriteReg(uint32_t offset, uint8_t value);
uint8_t FPGA_ReadReg(uint32_t offset);
void FPGA_WriteBuffer(uint32_t offset, uint8_t* data, uint16_t len);
void FPGA_ReadBuffer(uint32_t offset, uint8_t* data, uint16_t len);
```

**Testing**:
- [ ] FSMC timing verified
- [ ] Read/write operations tested

---

#### 3.2 Extract Protocol Utilities (Day 2)
**Input**: `optical.c` (CRC, float conversion)  
**Output**: `Middleware/`

**Tasks**:
1. Extract CRC16 calculation to `crc_utils.c/h`
2. Extract float<->byte conversion to `data_converter.c/h`
3. Create TLV parser utilities (if needed)
4. Add unit tests for utilities

**API Design**:
```c
// crc_utils.h
uint16_t CRC16_Calculate(uint8_t* data, uint16_t length);
bool CRC16_Verify(uint8_t* frame, uint16_t length);  // CRC at end
void CRC16_Append(uint8_t* frame, uint16_t data_len);  // Add CRC to frame

// data_converter.h
void Float_To_Bytes(float value, uint8_t* bytes, bool big_endian);
void Bytes_To_Float(uint8_t* bytes, float* value, bool big_endian);
void Int32_To_Bytes(int32_t value, uint8_t* bytes, bool big_endian);
void Bytes_To_Int32(uint8_t* bytes, int32_t* value, bool big_endian);
void Int16_To_Bytes(int16_t value, uint8_t* bytes, bool big_endian);
void Bytes_To_Int16(uint8_t* bytes, int16_t* value, bool big_endian);
```

**Testing**:
- [ ] CRC matches original implementation
- [ ] Float conversion verified (endianness)
- [ ] Unit tests pass

---

#### 3.3 Fiber Transport Layer (Day 3-4)
**Input**: `optical.c/h` (1800+ lines)  
**Output**: `Drivers/Communication/fiber_optic/`

**Tasks**:
1. Analyze optical.c communication structure
2. Identify transport vs. application protocol
3. Create `fiber_transport.c/h` - Multi-channel management
4. Create `fiber_frame.c/h` - Frame parsing/building
5. Create `fiber_callbacks.h` - Event definitions
6. Remove excitation-specific protocol commands

**API Design**:
```c
// fiber_callbacks.h
typedef enum {
    FIBER_EVENT_RX_COMPLETE = 0,
    FIBER_EVENT_TX_COMPLETE,
    FIBER_EVENT_CHANNEL_ONLINE,
    FIBER_EVENT_CHANNEL_OFFLINE,
    FIBER_EVENT_COMM_ERROR,
    FIBER_EVENT_CRC_ERROR
} fiber_event_t;

typedef void (*fiber_rx_callback_t)(uint8_t channel, uint8_t* data, uint16_t len);
typedef void (*fiber_event_callback_t)(uint8_t channel, fiber_event_t event, void* context);

typedef struct {
    fiber_rx_callback_t     on_receive;
    fiber_event_callback_t  on_event;
    void*                   user_context;
} fiber_channel_callbacks_t;

// fiber_transport.h
#define FIBER_MAX_CHANNELS      8
#define FIBER_MAX_FRAME_LEN     128

typedef struct {
    uint8_t  channel_id;          // 0-7
    bool     enabled;
    uint16_t timeout_ms;          // No data timeout
    uint16_t heartbeat_interval;  // Periodic TX interval
    uint16_t retry_count;         // TX retry attempts
    bool     auto_reconnect;
} fiber_channel_config_t;

bool FIBER_Init(void);
bool FIBER_ConfigChannel(fiber_channel_config_t* config,
                         fiber_channel_callbacks_t* callbacks);
bool FIBER_SendFrame(uint8_t channel, uint8_t* data, uint16_t len);
bool FIBER_Broadcast(uint8_t* data, uint16_t len);
bool FIBER_IsChannelOnline(uint8_t channel);
uint32_t FIBER_GetRxPacketCount(uint8_t channel);
uint32_t FIBER_GetTxPacketCount(uint8_t channel);
uint16_t FIBER_GetLastCommDelay(uint8_t channel);
void FIBER_ProcessChannels(void);  // Call periodically

// fiber_frame.h
typedef struct {
    uint8_t  header;      // Frame start marker
    uint8_t  channel;     // Source/dest channel
    uint16_t length;      // Payload length
    uint8_t* payload;     // User data
    uint16_t crc;         // CRC16 checksum
} fiber_frame_t;

bool FIBER_BuildFrame(fiber_frame_t* frame, uint8_t* buffer, uint16_t* buf_len);
bool FIBER_ParseFrame(uint8_t* buffer, uint16_t buf_len, fiber_frame_t* frame);
bool FIBER_ValidateFrame(fiber_frame_t* frame);
```

**Testing**:
- [ ] 8-channel communication verified
- [ ] CRC validation works
- [ ] Timeout/reconnect logic tested
- [ ] Broadcast tested

---

### Phase 4: Hardware I/O Abstraction (3 days)
**Goal**: Refactor mixed hardware/application files

#### 4.1 Analog Input Driver (Day 1)
**Input**: `AI.c/h` (570 lines)  
**Output**: `Drivers/Board/aio_driver.c/h`

**Tasks**:
1. Extract ADC data acquisition (keep)
2. Remove excitation calculations: Vfd, Ifd, P, Q, Vabc, Iabc (remove)
3. Create generic channel reading with scaling callbacks
4. Move to board driver layer

**API Design**:
```c
// aio_driver.h
#define AIO_MAX_CHANNELS    8
#define AIO_BUFFER_DEPTH    300

typedef struct {
    uint8_t  channel_id;
    bool     enabled;
    float    scale_factor;      // User-defined scaling
    float    offset;            // User-defined offset
    uint16_t filter_samples;    // Moving average filter
} aio_channel_config_t;

typedef void (*aio_sample_callback_t)(uint8_t channel, float value);
typedef void (*aio_buffer_ready_callback_t)(uint8_t channel, float* buffer, uint16_t count);

bool AIO_Init(void);
bool AIO_ConfigChannel(aio_channel_config_t* config);
bool AIO_StartAcquisition(void);
bool AIO_StopAcquisition(void);
float AIO_ReadChannel(uint8_t channel);
void AIO_RegisterCallback(aio_sample_callback_t callback);
void AIO_RegisterBufferCallback(aio_buffer_ready_callback_t callback);
void AIO_ProcessSamples(void);  // Call in ISR or main loop
```

**Testing**:
- [ ] 8-channel acquisition works
- [ ] Scaling applied correctly
- [ ] Callbacks triggered
- [ ] Buffer management verified

---

#### 4.2 Digital I/O Driver (Day 2)
**Input**: `DIDO.c/h`  
**Output**: `Drivers/Board/dio_driver.c/h`

**Tasks**:
1. Extract pin read/write functions (keep)
2. Remove SCR checking, breaker logic, fault detection (remove)
3. Create generic DI/DO with edge detection callbacks
4. Support multiple DI/DO banks

**API Design**:
```c
// dio_driver.h
#define DIO_MAX_INPUTS    64
#define DIO_MAX_OUTPUTS   64

typedef enum {
    DIO_EDGE_NONE = 0,
    DIO_EDGE_RISING,
    DIO_EDGE_FALLING,
    DIO_EDGE_BOTH
} dio_edge_t;

typedef void (*dio_input_callback_t)(uint8_t input_id, bool state);
typedef void (*dio_edge_callback_t)(uint8_t input_id, dio_edge_t edge);

typedef struct {
    uint8_t     input_id;
    bool        enabled;
    bool        pull_up;
    bool        pull_down;
    dio_edge_t  edge_detect;
} dio_input_config_t;

typedef struct {
    uint8_t  output_id;
    bool     enabled;
    bool     initial_state;
    bool     open_drain;
} dio_output_config_t;

bool DIO_Init(void);
bool DIO_ConfigInput(dio_input_config_t* config);
bool DIO_ConfigOutput(dio_output_config_t* config);
bool DIO_ReadInput(uint8_t input_id);
void DIO_WriteOutput(uint8_t output_id, bool state);
void DIO_ToggleOutput(uint8_t output_id);
uint64_t DIO_ReadAllInputs(void);
void DIO_WriteAllOutputs(uint64_t states);
void DIO_RegisterInputCallback(dio_input_callback_t callback);
void DIO_RegisterEdgeCallback(dio_edge_callback_t callback);
void DIO_ProcessInputs(void);  // Call periodically for edge detection
```

**Testing**:
- [ ] Input reading works
- [ ] Output setting works
- [ ] Edge detection works
- [ ] Callbacks triggered

---

#### 4.3 Synchronization Signal Driver (Day 3)
**Input**: `syn.c/h`  
**Output**: `Drivers/Board/sync_driver.c/h`

**Tasks**:
1. Extract interrupt timing capture (keep)
2. Extract phase detection (keep)
3. Remove SCR pulse generation, firing angle control (remove)
4. Create generic sync event callbacks

**API Design**:
```c
// sync_driver.h
typedef enum {
    SYNC_SOURCE_A = 0,
    SYNC_SOURCE_B,
    SYNC_SOURCE_C,
    SYNC_SOURCE_EXTERNAL
} sync_source_t;

typedef enum {
    SYNC_EVENT_RISING_EDGE = 0,
    SYNC_EVENT_FALLING_EDGE,
    SYNC_EVENT_ZERO_CROSS,
    SYNC_EVENT_PERIOD_COMPLETE
} sync_event_t;

typedef void (*sync_event_callback_t)(sync_source_t source, 
                                      sync_event_t event, 
                                      uint32_t timestamp);
typedef void (*sync_period_callback_t)(sync_source_t source, 
                                       uint32_t period_us, 
                                       float frequency_hz);

typedef struct {
    sync_source_t source;
    bool          enabled;
    uint16_t      debounce_us;
    bool          measure_period;
    bool          detect_errors;
} sync_config_t;

bool SYNC_Init(void);
bool SYNC_ConfigSource(sync_config_t* config);
bool SYNC_SelectSource(sync_source_t source);
uint32_t SYNC_GetPeriod(sync_source_t source);
float SYNC_GetFrequency(sync_source_t source);
uint32_t SYNC_GetLastTimestamp(sync_source_t source);
bool SYNC_IsSourceValid(sync_source_t source);
void SYNC_RegisterEventCallback(sync_event_callback_t callback);
void SYNC_RegisterPeriodCallback(sync_period_callback_t callback);
```

**Testing**:
- [ ] Period measurement accurate
- [ ] Frequency calculation correct
- [ ] Source switching works
- [ ] Callbacks triggered

---

### Phase 5: Board-Level HAL (2 days)
**Goal**: Create unified board initialization and configuration

#### 5.1 Board HAL Implementation (Day 1)
**Input**: `para.c/h`, various init functions  
**Output**: `HAL/board_hal.c/h`

**Tasks**:
1. Collect all hardware initialization sequences
2. Create unified `Board_Init()` function
3. Create configuration structures for all peripherals
4. Remove application-specific initialization

**API Design**:
```c
// board_hal.h
typedef struct {
    // Clock configuration
    uint32_t sysclk_mhz;
    uint32_t hclk_mhz;
    uint32_t pclk1_mhz;
    uint32_t pclk2_mhz;
    
    // Peripheral enables
    bool enable_ethernet;
    bool enable_fiber_optic;
    bool enable_external_adc;
    bool enable_external_dac;
    bool enable_flash_memory;
    bool enable_watchdog;
    
    // Node identification
    uint8_t node_id;
} board_config_t;

typedef struct {
    uint32_t uptime_ms;
    float    cpu_temp_c;
    uint16_t vdd_mv;
    bool     watchdog_reset;
} board_status_t;

bool Board_Init(board_config_t* config);
bool Board_GetStatus(board_status_t* status);
void Board_Reset(void);
void Board_EnterLowPower(void);
void Board_FeedWatchdog(void);
uint32_t Board_GetUptimeMs(void);
```

**Testing**:
- [ ] All peripherals initialize correctly
- [ ] Status readback works
- [ ] Watchdog functions properly

---

#### 5.2 Callback Registration System (Day 2)
**Output**: `HAL/board_callbacks.h`

**Tasks**:
1. Define master callback structure
2. Create callback registration API
3. Implement callback dispatcher
4. Add priority handling if needed

**API Design**:
```c
// board_callbacks.h
typedef struct {
    // Analog I/O callbacks
    aio_sample_callback_t       aio_on_sample;
    aio_buffer_ready_callback_t aio_on_buffer_ready;
    
    // Digital I/O callbacks
    dio_input_callback_t        dio_on_input_change;
    dio_edge_callback_t         dio_on_edge_detect;
    
    // Sync signal callbacks
    sync_event_callback_t       sync_on_event;
    sync_period_callback_t      sync_on_period;
    
    // Ethernet callbacks
    eth_socket_callbacks_t      eth_callbacks[8];  // 8 sockets
    
    // Fiber optic callbacks
    fiber_channel_callbacks_t   fiber_callbacks[8];  // 8 channels
    
    // System callbacks
    void (*on_error)(const char* error_msg);
    void (*on_watchdog_warning)(void);
} board_callbacks_t;

void Board_RegisterCallbacks(board_callbacks_t* callbacks);
void Board_DispatchEvents(void);  // Main event loop processor
```

**Testing**:
- [ ] All callback paths verified
- [ ] Priority handling works
- [ ] No callback missed

---

### Phase 6: Examples and Documentation (3 days)
**Goal**: Create comprehensive examples and API documentation

#### 6.1 Basic Examples (Day 1)
**Output**: `Examples/01-05_*.c`

**Tasks**:
1. Create simple DIO test
2. Create ADC/DAC loopback
3. Create Ethernet echo server
4. Create fiber optic bridge
5. Create sync signal capture

---

#### 6.2 Advanced Integration Example (Day 2)
**Output**: `Examples/06_full_integration.c`

**Tasks**:
1. Demonstrate all drivers working together
2. Show proper callback usage
3. Include error handling
4. Add performance monitoring

**Example Structure**:
```c
// 06_full_integration.c
void aio_handler(uint8_t ch, float value) {
    // Process analog input
    // Update control algorithm
    // Generate control output
}

void fiber_rx_handler(uint8_t ch, uint8_t* data, uint16_t len) {
    // Process fiber optic message
    // Coordinate with other nodes
}

void eth_rx_handler(uint8_t sock, uint8_t* data, uint16_t len) {
    // Process Ethernet command
    // Send response
}

void sync_event_handler(sync_source_t src, sync_event_t evt, uint32_t ts) {
    // Trigger time-critical operations
    // Synchronize control loop
}

int main(void) {
    // Configure board
    board_config_t board_cfg = { /* ... */ };
    Board_Init(&board_cfg);
    
    // Configure all drivers
    // Register all callbacks
    
    // Main event loop
    while(1) {
        Board_DispatchEvents();
        // User application logic
    }
}
```

---

#### 6.3 Documentation (Day 3)
**Output**: API documentation and migration guide

**Tasks**:
1. Write API reference for each driver
2. Create migration guide from old to new API
3. Document callback patterns
4. Add timing diagrams for communication
5. Create troubleshooting guide

**Documents to Create**:
- `API_REFERENCE.md` - Complete API documentation
- `MIGRATION_GUIDE.md` - How to migrate from NSD_Driver
- `INTEGRATION_GUIDE.md` - How to integrate with YEET
- `TIMING_ANALYSIS.md` - Real-time performance analysis
- `TROUBLESHOOTING.md` - Common issues and solutions

---

### Phase 7: Remove Application Code (1 day)
**Goal**: Clean up old application-specific code

#### Tasks:
1. Archive original NSD_Driver files
2. Delete application control files:
   - `mode.c/h`
   - `eIfd.c/h`
   - `ref.c/h`
   - `command.c/h`
   - `para.c/h`
   - `pub.c/h`
3. Update build system to use new structure
4. Verify no broken dependencies
5. Run all tests

---

### Phase 8: Integration with YEET (2 days)
**Goal**: Integrate refactored drivers with YEET control system

#### Tasks:
1. Update YEET embedded app to use new driver API
2. Implement YEET-specific callbacks
3. Map YEET control signals to hardware
4. Test complete system
5. Performance benchmarking

**Integration Points**:
```c
// YEET application uses new driver API
void yeet_adc_callback(uint8_t ch, float value) {
    // Feed to state observer
    yeet_update_measurements(ch, value);
}

void yeet_sync_callback(sync_source_t src, sync_event_t evt, uint32_t ts) {
    // Trigger control loop at precise timing
    if (evt == SYNC_EVENT_ZERO_CROSS) {
        yeet_execute_control_step();
    }
}

void yeet_control_loop(void) {
    // Get state estimate
    float x[6];
    yeet_get_state_estimate(x);
    
    // Compute control output
    float u = yeet_compute_control(x);
    
    // Apply to hardware
    AIO_WriteChannel(DAC_CHANNEL, u);
}
```

---

## Testing Strategy

### Unit Tests
- [ ] CRC utilities
- [ ] Data conversion (float/byte)
- [ ] Frame parsing
- [ ] Configuration validation

### Integration Tests
- [ ] Ethernet stack (loopback, echo)
- [ ] Fiber optic (multi-channel)
- [ ] ADC/DAC (loopback)
- [ ] DIO (read/write)
- [ ] Sync signals (period measurement)

### System Tests
- [ ] All drivers running simultaneously
- [ ] Real-time performance (loop timing)
- [ ] Stress testing (max throughput)
- [ ] Error recovery (disconnect/reconnect)
- [ ] Long-duration stability (24hr+)

### Hardware Tests
- [ ] Verify on actual STM32F407ZG board
- [ ] Test with real sensors/actuators
- [ ] Verify EMI/EMC compliance
- [ ] Temperature testing
- [ ] Power consumption

---

## Success Criteria

### Functional Requirements
✅ All hardware peripherals accessible via clean API  
✅ Ethernet stack supports TCP/UDP with callbacks  
✅ Fiber optic supports 8 channels with robust error handling  
✅ ADC sampling at required rate (configurable)  
✅ DAC output with precise timing  
✅ Digital I/O with edge detection  
✅ Sync signal capture with <1us accuracy  

### Non-Functional Requirements
✅ Real-time loop timing: <100us deterministic  
✅ Memory footprint: <50KB RAM, <200KB Flash  
✅ CPU usage: <50% at maximum throughput  
✅ API documentation: 100% coverage  
✅ Example code: 6+ working examples  
✅ Zero application-specific code in drivers  

### Quality Metrics
✅ Code coverage: >80% for drivers  
✅ No magic numbers (all constants named)  
✅ Consistent naming conventions  
✅ Doxygen-compatible comments  
✅ MISRA-C compliance (where applicable)  

---

## Risk Mitigation

### Technical Risks
| Risk                              | Impact | Mitigation                               |
| --------------------------------- | ------ | ---------------------------------------- |
| Breaking YEET integration         | High   | Incremental migration, parallel branches |
| Real-time performance degradation | High   | Early profiling, optimization passes     |
| Communication stack instability   | Medium | Extensive testing, maintain old code     |
| Hardware incompatibilities        | Medium | Test on real hardware early              |

### Schedule Risks
| Risk                      | Impact | Mitigation                             |
| ------------------------- | ------ | -------------------------------------- |
| Underestimated complexity | Medium | Buffer time, prioritize critical paths |
| Scope creep               | Medium | Strict adherence to plan               |
| Testing bottlenecks       | Low    | Parallel test development              |

---

## Timeline Summary

| Phase               | Duration    | Deliverables                             |
| ------------------- | ----------- | ---------------------------------------- |
| 1. Setup            | 1 day       | Directory structure, build system        |
| 2. Ethernet         | 3 days      | W5200 driver, transport API, examples    |
| 3. Fiber Optic      | 4 days      | FPGA interface, transport API, utilities |
| 4. Hardware I/O     | 3 days      | AIO, DIO, Sync drivers                   |
| 5. Board HAL        | 2 days      | Unified init, callback system            |
| 6. Examples         | 3 days      | 6 examples, documentation                |
| 7. Cleanup          | 1 day       | Remove old code, verify build            |
| 8. YEET Integration | 2 days      | Connect to YEET control                  |
| **TOTAL**           | **19 days** | **Complete refactored driver library**   |

---

## Maintenance Plan

### Post-Refactoring
1. Establish code review process
2. Create regression test suite
3. Set up CI/CD pipeline
4. Version control strategy (semantic versioning)
5. Issue tracking system

### Future Enhancements
- [ ] Add DMA support for high-speed ADC
- [ ] Implement FreeRTOS integration
- [ ] Add USB device/host support
- [ ] Create Python test tools
- [ ] Add CAN bus support (if hardware available)

---

## Appendix

### A. Key API Changes Summary

| Old API               | New API                   | Notes             |
| --------------------- | ------------------------- | ----------------- |
| `Init_Net()`          | `ETH_Init()`              | More configurable |
| `Net_Check()`         | `ETH_ProcessEvents()`     | Callback-based    |
| `Init_Optical_Comm()` | `FIBER_Init()`            | Cleaner naming    |
| `Optical_Comm()`      | `FIBER_ProcessChannels()` | Multi-channel     |
| `Init_AD_Data()`      | `AIO_Init()`              | Generic naming    |
| `Cal_Avg()`           | User callback             | App logic removed |
| `Get_DI1()`           | `DIO_ReadInput()`         | Consistent API    |
| `Set_DO()`            | `DIO_WriteOutput()`       | Consistent API    |

### B. Memory Layout

```
Flash (1MB):
├── Bootloader         [0x08000000 - 0x08008000]  32KB
├── Driver Library     [0x08008000 - 0x08040000]  224KB
├── User Application   [0x08040000 - 0x080C0000]  512KB
└── Reserved           [0x080C0000 - 0x08100000]  256KB

RAM (192KB):
├── Stack              [0x20000000 - 0x20004000]  16KB
├── Heap               [0x20004000 - 0x20014000]  64KB
├── Driver BSS         [0x20014000 - 0x20020000]  48KB
├── User Application   [0x20020000 - 0x20030000]  64KB
└── CCM RAM            [0x10000000 - 0x10010000]  64KB (fast access)
```

### C. Pin Mapping Reference

See separate `PIN_MAPPING.md` document for complete pin assignments.

### D. Build System Changes

**CMakeLists.txt additions**:
```cmake
# Driver library
add_library(board_drivers STATIC
    Drivers/Peripherals/*.c
    Drivers/Communication/**/*.c
    Drivers/Board/*.c
    HAL/*.c
    Middleware/*.c
)

target_include_directories(board_drivers PUBLIC
    Drivers/Peripherals
    Drivers/Communication
    Drivers/Board
    HAL
    Middleware
)

# Link with user application
target_link_libraries(yeet_embedded_app
    board_drivers
    # ... other libs
)
```

---

**End of Refactoring Plan**

*Document Version: 1.0*  
*Last Updated: October 15, 2025*  
*Author: YEET Development Team*  
*Status: Ready for Implementation*
