# IO Abstraction Layer - Concept and Design

**Date:** October 12, 2025  
**Purpose:** Design a generic, embedded-friendly IO abstraction for slow supervisory signals  
**Scope:** Bridge between hardware (STM32 main.c) and application layer (sfc_integration)

---

## 1. Design Goals

### 1.1 Core Requirements

**Embedded-Friendly:**
- ✅ Zero dynamic memory allocation (no `new`, `malloc`, `std::vector`)
- ✅ Deterministic execution time
- ✅ Minimal RAM footprint
- ✅ No exceptions (embedded C++ constraints)
- ✅ Real-time safe (10-100 ms update rate for slow IO)

**Generic and Flexible:**
- ✅ Usable across different platforms (STM32, simulation, HIL test benches)
- ✅ Type-safe access to digital and analog signals
- ✅ Clear separation: Hardware layer ↔ Application layer
- ✅ Easy to mock/stub for unit testing

**Integration-Ready:**
- ✅ Compatible with existing `yeet::signal_input<T>` / `yeet::signal_output<T>` system
- ✅ Can be used by state machine runners, excitation controller, etc.
- ✅ Maps cleanly to IO definition (DI1-DI16, DO1-DO32, AI1-AI2, AO1-AO2)

---

## 2. Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────┐
│                        APPLICATION LAYER                             │
│  (sfc_integration: state_machine_runner, excitation_controller)     │
│                                                                       │
│  Uses: signal_input<bool>, signal_output<float>, etc.               │
└────────────────────┬────────────────────────────────────────────────┘
                     │
                     │ Connects via signals_connector
                     │
┌────────────────────▼────────────────────────────────────────────────┐
│                     IO ABSTRACTION LAYER                             │
│                  (sfc_slow_io_interface.hpp)                         │
│                                                                       │
│  • sfc_digital_inputs: Typed access to DI1-DI16                      │
│  • sfc_digital_outputs: Typed access to DO1-DO32                     │
│  • sfc_analog_inputs: Typed access to AI1-AI2                        │
│  • sfc_analog_outputs: Typed access to AO1-AO2                       │
│                                                                       │
│  Provides: High-level signal names (exciter_ready, start_command)   │
└────────────────────┬────────────────────────────────────────────────┘
                     │
                     │ Maps to/from
                     │
┌────────────────────▼────────────────────────────────────────────────┐
│                      HARDWARE LAYER                                  │
│              (STM32 main.c + hardware drivers)                       │
│                                                                       │
│  • FPGA digital I/O (48 DI, 48 DO)                                   │
│  • AD7606 external ADC (8 channels @ 16-bit)                         │
│  • STM32 internal ADC (4 channels @ 12-bit)                          │
│  • W5200 Ethernet (slow telemetry)                                   │
│                                                                       │
│  task_10000us(): 100 Hz update rate for slow IO                      │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 3. Proposed Class Structure

### 3.1 Digital Input Interface

```cpp
// File: sfc_slow_io_interface.hpp
namespace yeet::application::sfc {

/**
 * @brief Raw digital input state (hardware representation)
 * 
 * This structure directly maps to hardware registers or FPGA I/O.
 * Updated by embedded layer at 100 Hz in task_10000us().
 */
struct sfc_digital_input_raw {
    // Digital Input Module 1 (16 bits)
    bool di1_operation_on_cmd : 1;
    bool di2_operation_off_cmd : 1;
    bool di3_high_speed_spin_request : 1;
    bool di4_high_speed_keep_request : 1;
    bool di5_low_speed_keep_request : 1;
    bool di6_sfc_acceleration_request : 1;
    bool di7_cold_drag_start_command : 1;
    bool di8_acceleration_request : 1;
    bool di9_speed_power_control_switching : 1;
    bool di10_start_hsb_cmd : 1;
    bool di11_spare : 1;
    bool di12_spare : 1;
    bool di13_front_door_open : 1;
    bool di14_lv_supervision : 1;
    bool di15_supervision : 1;
    bool di16_mcb_open_alarm : 1;
    
    // Digital Input Module 2 (16 bits)
    bool di17_exc_run_stop_status : 1;
    bool di18_exc_active : 1;
    bool di19_spare : 1;
    bool di20_spare : 1;
    bool di21_icb_closed_fb : 1;
    bool di22_icb_opened_fb : 1;
    bool di23_icb_is_tripping : 1;
    bool di24_press_relief_trip : 1;
    bool di25_winding_temp_alarm : 1;
    bool di26_winding_temp_trip : 1;
    bool di27_buchholz_alarm : 1;
    bool di28_buchholz_trip : 1;
    bool di29_emergency_off_pressed : 1;
    bool di30_common_hw_trip : 1;
    bool di31_external_fault : 1;
    bool di32_spare : 1;
    
    // Power Cabinet Module (5 bits)
    bool di33_overvoltage_monit : 1;
    bool di34_fan_stop_alarm : 1;
    bool di35_mcb_monit : 1;
    bool di36_reactor_temp_a : 1;
    bool di37_reactor_temp_t : 1;
    
    // Padding to align to 8 bytes
    uint16_t _padding : 11;
};
static_assert(sizeof(sfc_digital_input_raw) == 8, "Digital inputs must be 8 bytes");

/**
 * @brief Semantic digital input interface (application-level names)
 * 
 * This class provides meaningful names for application logic.
 * Maps raw hardware signals to high-level concepts.
 */
class sfc_digital_inputs {
public:
    explicit sfc_digital_inputs(const sfc_digital_input_raw* raw_inputs) noexcept
        : _raw(raw_inputs)
    {
        assert(_raw != nullptr);
    }
    
    // ==================== TCS Commands ====================
    
    bool get_start_command() const noexcept { 
        return _raw->di1_operation_on_cmd; 
    }
    
    bool get_stop_command() const noexcept { 
        return _raw->di2_operation_off_cmd; 
    }
    
    bool get_high_speed_spin_request() const noexcept { 
        return _raw->di3_high_speed_spin_request; 
    }
    
    bool get_cold_drag_start_command() const noexcept { 
        return _raw->di7_cold_drag_start_command; 
    }
    
    // ==================== Safety Interlocks ====================
    
    bool get_front_door_open() const noexcept { 
        return _raw->di13_front_door_open; 
    }
    
    bool get_emergency_stop_pressed() const noexcept { 
        return _raw->di29_emergency_off_pressed; 
    }
    
    bool get_common_hardware_trip() const noexcept { 
        return _raw->di30_common_hw_trip; 
    }
    
    bool get_external_fault() const noexcept { 
        return _raw->di31_external_fault; 
    }
    
    // ==================== Auxiliary System Status ====================
    
    bool get_lv_supervision_ok() const noexcept { 
        return _raw->di14_lv_supervision; 
    }
    
    bool get_power_supervision_ok() const noexcept { 
        return _raw->di15_supervision; 
    }
    
    bool get_mcb_closed() const noexcept { 
        return !_raw->di16_mcb_open_alarm;  // Inverted logic
    }
    
    bool get_fan_running() const noexcept { 
        return !_raw->di34_fan_stop_alarm;  // Inverted logic
    }
    
    /**
     * @brief Composite auxiliary system health check
     * 
     * Returns true only if all auxiliary systems are healthy:
     * - Power supervision OK
     * - LV supervision OK
     * - MCB closed
     * - Fans running
     * - No door open
     * - No hardware trip
     */
    bool get_auxiliaries_are_healthy() const noexcept {
        return get_power_supervision_ok() &&
               get_lv_supervision_ok() &&
               get_mcb_closed() &&
               get_fan_running() &&
               !get_front_door_open() &&
               !get_common_hardware_trip();
    }
    
    // ==================== Circuit Breaker Status ====================
    
    bool get_breaker_closed() const noexcept { 
        return _raw->di21_icb_closed_fb; 
    }
    
    bool get_breaker_opened() const noexcept { 
        return _raw->di22_icb_opened_fb; 
    }
    
    bool get_breaker_tripping() const noexcept { 
        return _raw->di23_icb_is_tripping; 
    }
    
    // ==================== Excitation Control ====================
    
    bool get_exciter_running() const noexcept { 
        return _raw->di17_exc_run_stop_status; 
    }
    
    bool get_exciter_active() const noexcept { 
        return _raw->di18_exc_active; 
    }
    
    // ==================== Protection System ====================
    
    bool get_pressure_relief_trip() const noexcept { 
        return _raw->di24_press_relief_trip; 
    }
    
    bool get_winding_temp_alarm() const noexcept { 
        return _raw->di25_winding_temp_alarm; 
    }
    
    bool get_winding_temp_trip() const noexcept { 
        return _raw->di26_winding_temp_trip; 
    }
    
    bool get_buchholz_alarm() const noexcept { 
        return _raw->di27_buchholz_alarm; 
    }
    
    bool get_buchholz_trip() const noexcept { 
        return _raw->di28_buchholz_trip; 
    }
    
    bool get_overvoltage_fault() const noexcept { 
        return _raw->di33_overvoltage_monit; 
    }
    
    bool get_reactor_temp_alarm() const noexcept { 
        return _raw->di36_reactor_temp_a || _raw->di37_reactor_temp_t; 
    }
    
private:
    const sfc_digital_input_raw* _raw;
};

} // namespace yeet::application::sfc
```

---

### 3.2 Digital Output Interface

```cpp
/**
 * @brief Raw digital output state (hardware representation)
 */
struct sfc_digital_output_raw {
    // Fan control (DO1-DO4)
    bool do1_reactor_fan_cmd : 1;
    bool do2_grid_bridge_fan_cmd : 1;
    bool do3_machine_bridge_fan_cmd : 1;
    bool do4_control_cabinet_fan_cmd : 1;
    
    // Breaker control (DO5-DO6, DO27)
    bool do5_icb_close_cmd : 1;
    bool do6_icb_open_cmd : 1;
    
    // TCS feedback (DO17-DO26)
    bool do17_lci_is_healthy : 1;
    bool do18_mv_on_fb : 1;
    bool do19_operation_off_fb : 1;
    bool do20_operation_on_fb : 1;
    bool do21_sfc_is_ready : 1;
    bool do22_speed_control_selected : 1;
    bool do23_power_control_selected : 1;
    bool do24_start_profile_selected : 1;
    bool do25_common_alarm : 1;
    bool do26_common_trip : 1;
    bool do27_emergency_trip_icb : 1;
    
    // Excitation control (DO28-DO30)
    bool do28_exc_on_cmd : 1;
    bool do29_exc_off_cmd : 1;
    bool do30_exc_control_mode : 1;
    
    // System control (DO31-DO32)
    bool do31_sw_trip : 1;
    bool do32_emergency_off_reset : 1;
    
    uint16_t _padding : 9;  // Pad to 32 bits
};
static_assert(sizeof(sfc_digital_output_raw) == 4, "Digital outputs must be 4 bytes");

/**
 * @brief Semantic digital output interface
 */
class sfc_digital_outputs {
public:
    explicit sfc_digital_outputs(sfc_digital_output_raw* raw_outputs) noexcept
        : _raw(raw_outputs)
    {
        assert(_raw != nullptr);
    }
    
    // ==================== Fan Control ====================
    
    void set_reactor_fan(bool enable) noexcept { 
        _raw->do1_reactor_fan_cmd = enable; 
    }
    
    void set_grid_bridge_fan(bool enable) noexcept { 
        _raw->do2_grid_bridge_fan_cmd = enable; 
    }
    
    void set_machine_bridge_fan(bool enable) noexcept { 
        _raw->do3_machine_bridge_fan_cmd = enable; 
    }
    
    void set_control_cabinet_fan(bool enable) noexcept { 
        _raw->do4_control_cabinet_fan_cmd = enable; 
    }
    
    void set_all_fans(bool enable) noexcept {
        set_reactor_fan(enable);
        set_grid_bridge_fan(enable);
        set_machine_bridge_fan(enable);
        set_control_cabinet_fan(enable);
    }
    
    // ==================== Breaker Control ====================
    
    void set_breaker_close_command(bool cmd) noexcept { 
        _raw->do5_icb_close_cmd = cmd; 
    }
    
    void set_breaker_open_command(bool cmd) noexcept { 
        _raw->do6_icb_open_cmd = cmd; 
    }
    
    void set_emergency_breaker_trip(bool trip) noexcept { 
        _raw->do27_emergency_trip_icb = trip; 
    }
    
    // ==================== TCS Feedback ====================
    
    void set_lci_is_healthy(bool healthy) noexcept { 
        _raw->do17_lci_is_healthy = healthy; 
    }
    
    void set_mv_on_feedback(bool on) noexcept { 
        _raw->do18_mv_on_fb = on; 
    }
    
    void set_operation_off_feedback(bool off) noexcept { 
        _raw->do19_operation_off_fb = off; 
    }
    
    void set_operation_on_feedback(bool on) noexcept { 
        _raw->do20_operation_on_fb = on; 
    }
    
    void set_sfc_is_ready(bool ready) noexcept { 
        _raw->do21_sfc_is_ready = ready; 
    }
    
    void set_speed_control_selected(bool selected) noexcept { 
        _raw->do22_speed_control_selected = selected; 
    }
    
    void set_power_control_selected(bool selected) noexcept { 
        _raw->do23_power_control_selected = selected; 
    }
    
    void set_common_alarm(bool alarm) noexcept { 
        _raw->do25_common_alarm = alarm; 
    }
    
    void set_common_trip(bool trip) noexcept { 
        _raw->do26_common_trip = trip; 
    }
    
    // ==================== Excitation Control ====================
    
    void set_exciter_on_command(bool cmd) noexcept { 
        _raw->do28_exc_on_cmd = cmd; 
    }
    
    void set_exciter_off_command(bool cmd) noexcept { 
        _raw->do29_exc_off_cmd = cmd; 
    }
    
    void set_exciter_control_mode(bool mode) noexcept { 
        _raw->do30_exc_control_mode = mode; 
    }
    
    // ==================== System Control ====================
    
    void set_software_trip(bool trip) noexcept { 
        _raw->do31_sw_trip = trip; 
    }
    
    void set_emergency_off_reset(bool reset) noexcept { 
        _raw->do32_emergency_off_reset = reset; 
    }
    
private:
    sfc_digital_output_raw* _raw;
};
```

---

### 3.3 Analog Input Interface

```cpp
/**
 * @brief Raw analog input state (hardware representation)
 * 
 * Values are in engineering units after scaling/calibration.
 */
struct sfc_analog_input_raw {
    float ai1_exc_current_feedback;  // Amperes
    float ai2_exc_voltage_feedback;  // Volts
};

/**
 * @brief Semantic analog input interface
 */
class sfc_analog_inputs {
public:
    explicit sfc_analog_inputs(const sfc_analog_input_raw* raw_inputs) noexcept
        : _raw(raw_inputs)
    {
        assert(_raw != nullptr);
    }
    
    float get_excitation_current_a() const noexcept { 
        return _raw->ai1_exc_current_feedback; 
    }
    
    float get_excitation_voltage_v() const noexcept { 
        return _raw->ai2_exc_voltage_feedback; 
    }
    
    /**
     * @brief Get excitation current in per-unit
     * @param rated_current Rated field current in amperes
     */
    float get_excitation_current_pu(float rated_current) const noexcept {
        return _raw->ai1_exc_current_feedback / rated_current;
    }
    
    /**
     * @brief Get excitation voltage in per-unit
     * @param rated_voltage Rated field voltage in volts
     */
    float get_excitation_voltage_pu(float rated_voltage) const noexcept {
        return _raw->ai2_exc_voltage_feedback / rated_voltage;
    }
    
private:
    const sfc_analog_input_raw* _raw;
};
```

---

### 3.4 Analog Output Interface

```cpp
/**
 * @brief Raw analog output state (hardware representation)
 */
struct sfc_analog_output_raw {
    float ao1_exc_voltage_ref;  // Volts (TBD - may not be used)
    float ao2_exc_current_ref;  // Amperes
};

/**
 * @brief Semantic analog output interface
 */
class sfc_analog_outputs {
public:
    explicit sfc_analog_outputs(sfc_analog_output_raw* raw_outputs) noexcept
        : _raw(raw_outputs)
    {
        assert(_raw != nullptr);
    }
    
    void set_excitation_voltage_reference_v(float voltage) noexcept { 
        _raw->ao1_exc_voltage_ref = voltage; 
    }
    
    void set_excitation_current_reference_a(float current) noexcept { 
        _raw->ao2_exc_current_ref = current; 
    }
    
    /**
     * @brief Set excitation current from per-unit value
     * @param current_pu Per-unit current (0.0 to 1.0)
     * @param rated_current Rated field current in amperes
     */
    void set_excitation_current_reference_pu(float current_pu, float rated_current) noexcept {
        _raw->ao2_exc_current_ref = current_pu * rated_current;
    }
    
    /**
     * @brief Set excitation voltage from per-unit value
     * @param voltage_pu Per-unit voltage (0.0 to 1.0)
     * @param rated_voltage Rated field voltage in volts
     */
    void set_excitation_voltage_reference_pu(float voltage_pu, float rated_voltage) noexcept {
        _raw->ao1_exc_voltage_ref = voltage_pu * rated_voltage;
    }
    
private:
    sfc_analog_output_raw* _raw;
};
```

---

## 4. Embedded Layer Integration

### 4.1 Hardware Mapping (STM32 main.c)

```cpp
// In main.c or dedicated io_hardware_mapping.c

// Global IO state (shared between ISRs and application)
static sfc_digital_input_raw g_digital_inputs = {0};
static sfc_digital_output_raw g_digital_outputs = {0};
static sfc_analog_input_raw g_analog_inputs = {0};
static sfc_analog_output_raw g_analog_outputs = {0};

/**
 * @brief Update digital inputs from FPGA I/O
 * Called from task_10000us() at 100 Hz
 */
void update_digital_inputs_from_fpga(void)
{
    // Read FPGA digital input registers
    FPGA_IO_ReadInputs(&fpga_io);
    
    // Map FPGA bits to digital input structure
    // Digital Input Module 1
    g_digital_inputs.di1_operation_on_cmd = (fpga_io.inputs[0] & (1 << 0)) != 0;
    g_digital_inputs.di2_operation_off_cmd = (fpga_io.inputs[0] & (1 << 1)) != 0;
    g_digital_inputs.di3_high_speed_spin_request = (fpga_io.inputs[0] & (1 << 2)) != 0;
    // ... map remaining DI1-DI16
    
    // Digital Input Module 2
    g_digital_inputs.di17_exc_run_stop_status = (fpga_io.inputs[2] & (1 << 0)) != 0;
    g_digital_inputs.di18_exc_active = (fpga_io.inputs[2] & (1 << 1)) != 0;
    // ... map remaining DI17-DI32
    
    // Power Cabinet Module
    g_digital_inputs.di33_overvoltage_monit = (fpga_io.inputs[4] & (1 << 0)) != 0;
    g_digital_inputs.di34_fan_stop_alarm = (fpga_io.inputs[4] & (1 << 1)) != 0;
    // ... map remaining power cabinet inputs
}

/**
 * @brief Update analog inputs from AD7606
 * Called from task_10000us() at 100 Hz
 */
void update_analog_inputs_from_adc(void)
{
    // Read AD7606 external ADC
    AD7606_ReadData(&ad7606_data);
    
    // Apply calibration and convert to engineering units
    // AI1: Excitation current (e.g., channel 0, 0-10V → 0-100A)
    float ai1_voltage = ad7606_data.channels[0] * (10.0f / 65535.0f);
    g_analog_inputs.ai1_exc_current_feedback = ai1_voltage * 10.0f;  // 10A per volt
    
    // AI2: Excitation voltage (e.g., channel 1, 0-10V → 0-300V)
    float ai2_voltage = ad7606_data.channels[1] * (10.0f / 65535.0f);
    g_analog_inputs.ai2_exc_voltage_feedback = ai2_voltage * 30.0f;  // 30V per volt
}

/**
 * @brief Write digital outputs to FPGA I/O
 * Called from task_10000us() at 100 Hz
 */
void write_digital_outputs_to_fpga(void)
{
    // Map digital output structure to FPGA bits
    fpga_io.outputs[0] = 0;
    if (g_digital_outputs.do1_reactor_fan_cmd) fpga_io.outputs[0] |= (1 << 0);
    if (g_digital_outputs.do2_grid_bridge_fan_cmd) fpga_io.outputs[0] |= (1 << 1);
    if (g_digital_outputs.do3_machine_bridge_fan_cmd) fpga_io.outputs[0] |= (1 << 2);
    // ... map remaining DO1-DO32
    
    // Write to FPGA
    FPGA_IO_WriteOutputs(&fpga_io);
}

/**
 * @brief Write analog outputs to DAC
 * Called from task_10000us() at 100 Hz
 */
void write_analog_outputs_to_dac(void)
{
    // Convert engineering units to DAC counts
    // AO2: Excitation current reference (0-100A → 0-4095 counts)
    uint16_t ao2_counts = (uint16_t)(g_analog_outputs.ao2_exc_current_ref * 40.95f);
    if (ao2_counts > 4095) ao2_counts = 4095;
    
    // Write to DAC (via SPI or I2C depending on hardware)
    // DAC_WriteChannel(2, ao2_counts);
}

/**
 * @brief Task 3: 100 Hz slow IO update
 * Called from TIM4 interrupt (HAL_TIM_PeriodElapsedCallback)
 */
void isr_task_10000us()
{
    // Read inputs from hardware
    update_digital_inputs_from_fpga();
    update_analog_inputs_from_adc();
    
    // Call application layer (defined in yeet_application)
    extern void yeet_task_10000us(
        const sfc_digital_input_raw* di,
        const sfc_analog_input_raw* ai,
        sfc_digital_output_raw* do_out,
        sfc_analog_output_raw* ao_out
    );
    
    yeet_task_10000us(&g_digital_inputs, &g_analog_inputs, 
                      &g_digital_outputs, &g_analog_outputs);
    
    // Write outputs to hardware
    write_digital_outputs_to_fpga();
    write_analog_outputs_to_dac();
    
    // Toggle LED for visual feedback
    task3_counter++;
    if (task3_counter >= TASK3_TOGGLE_COUNT) {
        FPGA_LED_Toggle(2);
        task3_counter = 0;
    }
}
```

---

## 5. Application Layer Integration

### 5.1 SFC Integration Runner

```cpp
// File: sfc_slow_io_runner.hpp
namespace yeet::application::sfc::integration {

/**
 * @brief Runner for slow IO interface
 * 
 * Bridges raw IO structures to signal-based datamodel
 */
class sfc_slow_io_runner {
public:
    struct inputs {
        // Raw IO structures (from embedded layer)
        const sfc_digital_input_raw* digital_inputs_raw;
        const sfc_analog_input_raw* analog_inputs_raw;
    };
    
    struct outputs {
        // Raw IO structures (to embedded layer)
        sfc_digital_output_raw* digital_outputs_raw;
        sfc_analog_output_raw* analog_outputs_raw;
    };
    
    struct datamodel {
        // Semantic signal interface (for application layer)
        struct inputs {
            // TCS commands
            signal_input<bool> start_command;
            signal_input<bool> stop_command;
            signal_input<bool> emergency_stop;
            
            // Auxiliary system status
            signal_input<bool> auxiliaries_are_on;
            signal_input<bool> breakers_are_on;
            
            // Excitation feedback
            signal_input<float> excitation_current_a;
            signal_input<float> excitation_voltage_v;
            signal_input<bool> exciter_running;
            signal_input<bool> exciter_active;
            
            // Protection signals
            signal_input<bool> common_hardware_trip;
            signal_input<bool> external_fault;
            signal_input<bool> winding_temp_alarm;
        };
        
        struct outputs {
            // Fan control
            signal_output<bool> enable_all_fans;
            
            // Breaker control
            signal_output<bool> breaker_close_command;
            signal_output<bool> breaker_open_command;
            
            // TCS feedback
            signal_output<bool> lci_is_healthy;
            signal_output<bool> sfc_is_ready;
            signal_output<bool> operation_on_feedback;
            signal_output<bool> operation_off_feedback;
            
            // Excitation control
            signal_output<bool> exciter_on_command;
            signal_output<bool> exciter_off_command;
            signal_output<float> excitation_current_reference_a;
        };
    };
    
    explicit sfc_slow_io_runner(const inputs& in, outputs& out, 
                                 const datamodel::inputs& dm_in, datamodel::outputs& dm_out)
        : _inputs(in),
          _outputs(out),
          _datamodel_inputs(dm_in),
          _datamodel_outputs(dm_out),
          _digital_inputs(in.digital_inputs_raw),
          _analog_inputs(in.analog_inputs_raw),
          _digital_outputs(out.digital_outputs_raw),
          _analog_outputs(out.analog_outputs_raw)
    {
    }
    
    /**
     * @brief Update: Read hardware inputs → Write to signals
     */
    void update_inputs() noexcept {
        // TCS commands
        _datamodel_inputs.start_command.write(_digital_inputs.get_start_command());
        _datamodel_inputs.stop_command.write(_digital_inputs.get_stop_command());
        _datamodel_inputs.emergency_stop.write(_digital_inputs.get_emergency_stop_pressed());
        
        // Auxiliary system status
        _datamodel_inputs.auxiliaries_are_on.write(_digital_inputs.get_auxiliaries_are_healthy());
        _datamodel_inputs.breakers_are_on.write(_digital_inputs.get_breaker_closed());
        
        // Excitation feedback
        _datamodel_inputs.excitation_current_a.write(_analog_inputs.get_excitation_current_a());
        _datamodel_inputs.excitation_voltage_v.write(_analog_inputs.get_excitation_voltage_v());
        _datamodel_inputs.exciter_running.write(_digital_inputs.get_exciter_running());
        _datamodel_inputs.exciter_active.write(_digital_inputs.get_exciter_active());
        
        // Protection signals
        _datamodel_inputs.common_hardware_trip.write(_digital_inputs.get_common_hardware_trip());
        _datamodel_inputs.external_fault.write(_digital_inputs.get_external_fault());
        _datamodel_inputs.winding_temp_alarm.write(_digital_inputs.get_winding_temp_alarm());
    }
    
    /**
     * @brief Update: Read signals → Write to hardware outputs
     */
    void update_outputs() noexcept {
        // Fan control
        bool enable_fans = _datamodel_outputs.enable_all_fans.read();
        _digital_outputs.set_all_fans(enable_fans);
        
        // Breaker control
        _digital_outputs.set_breaker_close_command(_datamodel_outputs.breaker_close_command.read());
        _digital_outputs.set_breaker_open_command(_datamodel_outputs.breaker_open_command.read());
        
        // TCS feedback
        _digital_outputs.set_lci_is_healthy(_datamodel_outputs.lci_is_healthy.read());
        _digital_outputs.set_sfc_is_ready(_datamodel_outputs.sfc_is_ready.read());
        _digital_outputs.set_operation_on_feedback(_datamodel_outputs.operation_on_feedback.read());
        _digital_outputs.set_operation_off_feedback(_datamodel_outputs.operation_off_feedback.read());
        
        // Excitation control
        _digital_outputs.set_exciter_on_command(_datamodel_outputs.exciter_on_command.read());
        _digital_outputs.set_exciter_off_command(_datamodel_outputs.exciter_off_command.read());
        _analog_outputs.set_excitation_current_reference_a(_datamodel_outputs.excitation_current_reference_a.read());
    }
    
    /**
     * @brief Main run function (called at 100 Hz)
     */
    void run() noexcept {
        update_inputs();   // Hardware → Signals
        update_outputs();  // Signals → Hardware
    }
    
private:
    const inputs& _inputs;
    outputs& _outputs;
    const datamodel::inputs& _datamodel_inputs;
    datamodel::outputs& _datamodel_outputs;
    
    // Semantic interfaces
    sfc_digital_inputs _digital_inputs;
    sfc_analog_inputs _analog_inputs;
    sfc_digital_outputs _digital_outputs;
    sfc_analog_outputs _analog_outputs;
};

} // namespace yeet::application::sfc::integration
```

---

## 6. Usage in State Machine

```cpp
// Example: Connecting slow IO to state machine
void connect_slow_io_to_state_machine()
{
    // Create slow IO runner
    sfc_slow_io_runner::inputs io_in{&g_digital_inputs, &g_analog_inputs};
    sfc_slow_io_runner::outputs io_out{&g_digital_outputs, &g_analog_outputs};
    sfc_slow_io_runner::datamodel::inputs io_dm_in;
    sfc_slow_io_runner::datamodel::outputs io_dm_out;
    
    sfc_slow_io_runner io_runner(io_in, io_out, io_dm_in, io_dm_out);
    
    // Connect to state machine
    signals_connector::connect_output_to_input(
        io_dm_in.start_command,
        state_machine.get_inputs().start_command
    );
    
    signals_connector::connect_output_to_input(
        io_dm_in.stop_command,
        state_machine.get_inputs().stop_command
    );
    
    signals_connector::connect_output_to_input(
        io_dm_in.auxiliaries_are_on,
        state_machine.get_inputs().auxiliaries_are_on
    );
    
    signals_connector::connect_output_to_input(
        io_dm_in.breakers_are_on,
        state_machine.get_inputs().breakers_are_on
    );
    
    // Create excitation_is_ready event using analog inputs
    sfc_state_event excitation_ready_event("excitation_is_ready", [&io_dm_in]() {
        float current = io_dm_in.excitation_current_a.read();
        float voltage = io_dm_in.excitation_voltage_v.read();
        
        const float current_threshold = 10.0f;  // 10 A
        const float voltage_threshold = 240.0f; // 240 V
        
        return (current > current_threshold) && (voltage > voltage_threshold);
    });
    
    // Connect state machine outputs to IO
    signals_connector::connect_output_to_input(
        state_machine.get_outputs().enable_excitation,
        io_dm_out.exciter_on_command
    );
}
```

---

## 7. Testing Strategy

### 7.1 Unit Testing (Mock Hardware)

```cpp
// Test: Digital input reading
TEST(SlowIOTest, DigitalInputsReadCorrectly) {
    sfc_digital_input_raw raw_inputs = {0};
    raw_inputs.di1_operation_on_cmd = true;
    raw_inputs.di2_operation_off_cmd = false;
    
    sfc_digital_inputs inputs(&raw_inputs);
    
    EXPECT_TRUE(inputs.get_start_command());
    EXPECT_FALSE(inputs.get_stop_command());
}

// Test: Composite auxiliary health check
TEST(SlowIOTest, AuxiliariesHealthComposite) {
    sfc_digital_input_raw raw = {0};
    raw.di14_lv_supervision = true;
    raw.di15_supervision = true;
    raw.di16_mcb_open_alarm = false;  // MCB closed
    raw.di34_fan_stop_alarm = false;  // Fan running
    raw.di13_front_door_open = false;
    raw.di30_common_hw_trip = false;
    
    sfc_digital_inputs inputs(&raw);
    
    EXPECT_TRUE(inputs.get_auxiliaries_are_healthy());
    
    // Fail if door open
    raw.di13_front_door_open = true;
    EXPECT_FALSE(inputs.get_auxiliaries_are_healthy());
}

// Test: Analog input unit conversion
TEST(SlowIOTest, AnalogInputPerUnitConversion) {
    sfc_analog_input_raw raw = {0};
    raw.ai1_exc_current_feedback = 50.0f;  // 50 A
    
    sfc_analog_inputs inputs(&raw);
    
    float current_pu = inputs.get_excitation_current_pu(100.0f);  // Rated 100A
    EXPECT_NEAR(current_pu, 0.5f, 1e-6f);
}
```

---

## 8. Benefits of This Architecture

### 8.1 Separation of Concerns

| Layer           | Responsibility                          | Language/Platform       |
| --------------- | --------------------------------------- | ----------------------- |
| **Hardware**    | Map physical pins to raw structs        | C (STM32 HAL)           |
| **Abstraction** | Provide semantic names                  | C++ (embedded-safe)     |
| **Integration** | Bridge to signal system                 | C++ (application layer) |
| **Application** | Business logic (state machine, control) | C++ (generic)           |

### 8.2 Portability

- ✅ **Hardware layer** (main.c) is platform-specific: STM32, FPGA, AD7606
- ✅ **Abstraction layer** (sfc_slow_io_interface.hpp) is **portable**: Works on any platform
- ✅ **Integration layer** (sfc_slow_io_runner.hpp) is **generic**: No hardware dependencies
- ✅ **Application layer** (state_machine, excitation_controller) is **fully reusable**

### 8.3 Testability

- ✅ Unit tests can mock `sfc_digital_input_raw` / `sfc_analog_input_raw`
- ✅ No need for physical hardware to test application logic
- ✅ HIL tests can replace embedded layer with real-time simulator

### 8.4 Memory Safety

- ✅ All structs are **fixed-size** (no dynamic allocation)
- ✅ **Bitfields** minimize RAM usage (48 DI bits = 6 bytes)
- ✅ **Const-correctness** prevents unintended writes
- ✅ **Asserts** catch nullptr at initialization

---

## 9. Implementation Roadmap

### Phase 1: Core IO Abstraction (Week 1)
- [ ] Create `sfc_slow_io_interface.hpp` with raw structs and semantic classes
- [ ] Write unit tests for digital/analog input/output classes
- [ ] Validate memory layout and size constraints

### Phase 2: Embedded Integration (Week 2)
- [ ] Implement hardware mapping functions in `io_hardware_mapping.c`
- [ ] Integrate into `task_10000us()` in main.c
- [ ] Test with physical hardware (STM32 + FPGA)

### Phase 3: Application Integration (Week 3)
- [ ] Create `sfc_slow_io_runner.hpp` for signal bridging
- [ ] Connect to existing state machine runner
- [ ] Connect to excitation controller runner
- [ ] Add excitation_is_ready event using analog inputs

### Phase 4: Validation (Week 4)
- [ ] End-to-end test: TCS command → State machine → Exciter control
- [ ] Verify all IO definition signals are mapped correctly
- [ ] HIL testing with external excitation controller
- [ ] Documentation and code review

---

**End of Concept Document**

Would you like me to proceed with implementing any of these components?
