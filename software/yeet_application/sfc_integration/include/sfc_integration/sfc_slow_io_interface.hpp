// Copyright 2025 Techperts
// SFC Slow IO Interface - Generic IO Abstraction Layer
//
// This file provides a platform-independent abstraction for slow I/O signals
// (digital/analog inputs/outputs) that interface with TCS, excitation controller,
// and auxiliary systems. Update rate: 10-100 Hz (supervisory level).
//
// Architecture:
// - Raw structs use bitfields to minimize RAM (28 bytes total)
// - Semantic wrapper classes provide meaningful accessor methods
// - No dynamic allocation, noexcept, deterministic execution
// - Compatible with embedded systems and simulation environments

#pragma once

#include <cstdint>

#include "yeet_control/units.hpp"

namespace yeet {
namespace sfc {

// =============================================================================
// RAW DATA STRUCTURES (Layer 1: Hardware Interface)
// =============================================================================

/// @brief Raw digital input state from hardware (48 bits packed into bitfield)
/// Memory: 8 bytes (6 bytes data + 2 bytes padding for alignment)
struct sfc_digital_input_raw {
    // Module 1: TCS Commands (16 inputs)
    uint8_t operation_on_cmd : 1;   // DI1: Start command from TCS
    uint8_t operation_off_cmd : 1;  // DI2: Stop command from TCS
    uint8_t spare_m1_di3 : 1;       // DI3: Spare
    uint8_t spare_m1_di4 : 1;       // DI4: Spare
    uint8_t spare_m1_di5 : 1;       // DI5: Spare
    uint8_t spare_m1_di6 : 1;       // DI6: Spare
    uint8_t spare_m1_di7 : 1;       // DI7: Spare
    uint8_t spare_m1_di8 : 1;       // DI8: Spare

    uint8_t spare_m1_di9 : 1;     // DI9: Spare
    uint8_t spare_m1_di10 : 1;    // DI10: Spare
    uint8_t spare_m1_di11 : 1;    // DI11: Spare
    uint8_t spare_m1_di12 : 1;    // DI12: Spare
    uint8_t front_door_open : 1;  // DI13: Safety interlock
    uint8_t lv_supervision : 1;   // DI14: Low voltage power OK
    uint8_t supervision : 1;      // DI15: Main supervision OK
    uint8_t mcb_open_alarm : 1;   // DI16: Motor circuit breaker alarm

    // Module 2: Excitation & Protection (16 inputs)
    uint8_t exciter_run_stop_status : 1;     // DI1: Exciter is running
    uint8_t exciter_active : 1;              // DI2: Exciter voltage applied
    uint8_t spare_m2_di3 : 1;                // DI3: Spare (can be used for field established)
    uint8_t spare_m2_di4 : 1;                // DI4: Spare
    uint8_t icb_closed_fb : 1;               // DI5: Input circuit breaker closed
    uint8_t icb_open_fb : 1;                 // DI6: Input circuit breaker open
    uint8_t icb_tripping : 1;                // DI7: Breaker is tripping
    uint8_t transformer_pressure_alarm : 1;  // DI8: Transformer protection

    uint8_t overheat_alarm : 1;               // DI9: Temperature warning
    uint8_t overheat_trip : 1;                // DI10: Temperature fault
    uint8_t transformer_light_gas_alarm : 1;  // DI11: Buchholz relay
    uint8_t transformer_heavy_gas_trip : 1;   // DI12: Buchholz trip
    uint8_t emergency_off : 1;                // DI13: E-stop button
    uint8_t common_hw_trip : 1;               // DI14: Hardware protection trip
    uint8_t external_fault : 1;               // DI15: External system fault
    uint8_t spare_m2_di16 : 1;                // DI16: Spare

    // Module 3: Power Cabinet (16 inputs, using first 5)
    uint8_t overvoltage_fuse : 1;    // DI1: Fuse status
    uint8_t fan_stop_alarm : 1;      // DI2: Cooling fan fault
    uint8_t spare_m3_di3 : 1;        // DI3: Spare
    uint8_t reactor_temp_alarm : 1;  // DI4: Reactor overtemp warning
    uint8_t reactor_temp_trip : 1;   // DI5: Reactor overtemp fault
    uint8_t spare_m3_di6 : 1;        // DI6-16: Spare
    uint8_t spare_m3_di7 : 1;
    uint8_t spare_m3_di8 : 1;

    uint8_t spare_m3_di9 : 1;
    uint8_t spare_m3_di10 : 1;
    uint8_t spare_m3_di11 : 1;
    uint8_t spare_m3_di12 : 1;
    uint8_t spare_m3_di13 : 1;
    uint8_t spare_m3_di14 : 1;
    uint8_t spare_m3_di15 : 1;
    uint8_t spare_m3_di16 : 1;

    uint16_t _padding;  // Explicit padding to reach 8 bytes (6 bytes data + 2 bytes padding)
};
static_assert(sizeof(sfc_digital_input_raw) == 8, "Digital input struct must be 8 bytes");

/// @brief Raw digital output commands to hardware (32 bits packed into bitfield)
/// Memory: 4 bytes (23 used bits + padding)
struct sfc_digital_output_raw {
    // Auxiliary Control
    uint8_t fan1_on : 1;    // DO1: Cooling fan 1
    uint8_t fan2_on : 1;    // DO2: Cooling fan 2
    uint8_t fan3_on : 1;    // DO3: Cooling fan 3
    uint8_t fan4_on : 1;    // DO4: Cooling fan 4
    uint8_t icb_close : 1;  // DO5: Close input breaker
    uint8_t icb_open : 1;   // DO6: Open input breaker
    uint8_t spare_do7 : 1;  // DO7-18: Spare
    uint8_t spare_do8 : 1;

    uint8_t spare_do9 : 1;
    uint8_t spare_do10 : 1;
    uint8_t spare_do11 : 1;
    uint8_t spare_do12 : 1;
    uint8_t spare_do13 : 1;
    uint8_t spare_do14 : 1;
    uint8_t spare_do15 : 1;
    uint8_t spare_do16 : 1;

    uint8_t spare_do17 : 1;
    uint8_t spare_do18 : 1;
    uint8_t operation_off_fb : 1;  // DO19: System stopped feedback
    uint8_t operation_on_fb : 1;   // DO20: System running feedback
    uint8_t spare_do21 : 1;        // DO21-26: Spare
    uint8_t spare_do22 : 1;
    uint8_t spare_do23 : 1;
    uint8_t spare_do24 : 1;

    uint8_t spare_do25 : 1;
    uint8_t spare_do26 : 1;
    uint8_t icb_trip : 1;              // DO27: Trip breaker command
    uint8_t exciter_on_cmd : 1;        // DO28: Start excitation
    uint8_t exciter_off_cmd : 1;       // DO29: Stop excitation
    uint8_t exciter_control_mode : 1;  // DO30: Exciter mode selection (TBD)
    uint8_t spare_do31 : 1;            // DO31-32: Spare
    uint8_t spare_do32 : 1;
};
static_assert(sizeof(sfc_digital_output_raw) == 4, "Digital output struct must be 4 bytes");

/// @brief Raw analog input measurements from hardware
/// Memory: 8 bytes (2 floats)
struct sfc_analog_input_raw {
    float excitation_current_a;  // AI1: Field current [Amperes]
    float excitation_voltage_v;  // AI2: Field voltage [Volts]
};
static_assert(sizeof(sfc_analog_input_raw) == 8, "Analog input struct must be 8 bytes");

/// @brief Raw analog output commands to hardware
/// Memory: 8 bytes (2 floats)
struct sfc_analog_output_raw {
    float excitation_voltage_ref_v;  // AO1: Field voltage setpoint [Volts] (TBD)
    float excitation_current_ref_a;  // AO2: Field current setpoint [Amperes]
};
static_assert(sizeof(sfc_analog_output_raw) == 8, "Analog output struct must be 8 bytes");

// Total memory footprint: 8 + 4 + 8 + 8 = 28 bytes

// =============================================================================
// SEMANTIC WRAPPER CLASSES (Layer 2: Application Interface)
// =============================================================================

/// @brief Read-only interface to digital inputs with semantic accessor methods
/// Provides meaningful names for TCS commands, protection signals, and auxiliary status
class sfc_digital_inputs {
public:
    /// @brief Construct from raw hardware data
    explicit sfc_digital_inputs(const sfc_digital_input_raw& raw) noexcept : _raw(raw) {}

    // TCS Command Inputs (Module 1)
    bool get_start_command() const noexcept { return _raw.operation_on_cmd; }
    bool get_stop_command() const noexcept { return _raw.operation_off_cmd; }

    // Safety Interlocks (Module 1)
    bool get_front_door_open() const noexcept { return _raw.front_door_open; }
    bool get_lv_supervision_ok() const noexcept { return _raw.lv_supervision; }
    bool get_supervision_ok() const noexcept { return _raw.supervision; }
    bool get_mcb_open_alarm() const noexcept { return _raw.mcb_open_alarm; }

    // Excitation Controller Status (Module 2)
    bool get_exciter_is_running() const noexcept { return _raw.exciter_run_stop_status; }
    bool get_exciter_voltage_applied() const noexcept { return _raw.exciter_active; }

    // Circuit Breaker Status (Module 2)
    bool get_icb_is_closed() const noexcept { return _raw.icb_closed_fb; }
    bool get_icb_is_open() const noexcept { return _raw.icb_open_fb; }
    bool get_icb_is_tripping() const noexcept { return _raw.icb_tripping; }

    // Protection System Inputs (Module 2 & 3)
    bool get_transformer_pressure_alarm() const noexcept { return _raw.transformer_pressure_alarm; }
    bool get_overheat_alarm() const noexcept { return _raw.overheat_alarm; }
    bool get_overheat_trip() const noexcept { return _raw.overheat_trip; }
    bool get_transformer_light_gas_alarm() const noexcept { return _raw.transformer_light_gas_alarm; }
    bool get_transformer_heavy_gas_trip() const noexcept { return _raw.transformer_heavy_gas_trip; }
    bool get_emergency_stop() const noexcept { return _raw.emergency_off; }
    bool get_common_hardware_trip() const noexcept { return _raw.common_hw_trip; }
    bool get_external_fault() const noexcept { return _raw.external_fault; }
    bool get_overvoltage_fuse_blown() const noexcept { return _raw.overvoltage_fuse; }
    bool get_fan_stop_alarm() const noexcept { return _raw.fan_stop_alarm; }
    bool get_reactor_temp_alarm() const noexcept { return _raw.reactor_temp_alarm; }
    bool get_reactor_temp_trip() const noexcept { return _raw.reactor_temp_trip; }

    // Composite Health Checks
    /// @brief Check if all auxiliary systems are healthy (no faults)
    bool get_auxiliaries_are_healthy() const noexcept
    {
        return _raw.supervision &&       // Main power OK
               _raw.lv_supervision &&    // Low voltage OK
               !_raw.fan_stop_alarm &&   // Cooling operational
               !_raw.mcb_open_alarm &&   // MCB not alarming
               !_raw.front_door_open &&  // Safety interlock
               !_raw.common_hw_trip;     // No hardware trip
    }

    /// @brief Check if breakers are ready for operation
    bool get_breakers_are_ready() const noexcept { return _raw.icb_closed_fb && !_raw.icb_tripping; }

    /// @brief Check if any shutdown condition is present
    bool get_shutdown_requested() const noexcept
    {
        return _raw.emergency_off || _raw.common_hw_trip || _raw.external_fault || _raw.overheat_trip ||
               _raw.transformer_heavy_gas_trip || _raw.reactor_temp_trip;
    }

    /// @brief Access raw bitfield (for debugging or hardware layer)
    const sfc_digital_input_raw& get_raw() const noexcept { return _raw; }

private:
    const sfc_digital_input_raw& _raw;
};

/// @brief Write interface to digital outputs with semantic setter methods
class sfc_digital_outputs {
public:
    /// @brief Construct with reference to raw output struct
    explicit sfc_digital_outputs(sfc_digital_output_raw& raw) noexcept : _raw(raw) {}

    // Auxiliary Control Commands
    void set_fan1_on(bool on) noexcept { _raw.fan1_on = on; }
    void set_fan2_on(bool on) noexcept { _raw.fan2_on = on; }
    void set_fan3_on(bool on) noexcept { _raw.fan3_on = on; }
    void set_fan4_on(bool on) noexcept { _raw.fan4_on = on; }

    /// @brief Command all cooling fans simultaneously
    void set_all_fans(bool on) noexcept
    {
        _raw.fan1_on = on;
        _raw.fan2_on = on;
        _raw.fan3_on = on;
        _raw.fan4_on = on;
    }

    // Circuit Breaker Commands
    void set_icb_close_command(bool cmd) noexcept { _raw.icb_close = cmd; }
    void set_icb_open_command(bool cmd) noexcept { _raw.icb_open = cmd; }
    void set_icb_trip_command(bool cmd) noexcept { _raw.icb_trip = cmd; }

    // TCS Feedback Signals
    void set_operation_off_feedback(bool stopped) noexcept { _raw.operation_off_fb = stopped; }
    void set_operation_on_feedback(bool running) noexcept { _raw.operation_on_fb = running; }

    // Excitation Controller Commands
    void set_exciter_on_command(bool on) noexcept { _raw.exciter_on_cmd = on; }
    void set_exciter_off_command(bool off) noexcept { _raw.exciter_off_cmd = off; }
    void set_exciter_control_mode(bool mode) noexcept { _raw.exciter_control_mode = mode; }

    /// @brief Access raw bitfield (for hardware layer or debugging)
    sfc_digital_output_raw& get_raw() noexcept { return _raw; }
    const sfc_digital_output_raw& get_raw() const noexcept { return _raw; }

    /// @brief Reset all outputs to safe state (all OFF/false)
    void reset_all() noexcept
    {
        _raw = sfc_digital_output_raw{};  // Zero-initialize
    }

private:
    sfc_digital_output_raw& _raw;
};

/// @brief Read-only interface to analog inputs with unit conversion
class sfc_analog_inputs {
public:
    /// @brief Construct from raw hardware data
    explicit sfc_analog_inputs(const sfc_analog_input_raw& raw) noexcept : _raw(raw) {}

    // Raw Engineering Units
    float get_excitation_current_a() const noexcept { return _raw.excitation_current_a; }
    float get_excitation_voltage_v() const noexcept { return _raw.excitation_voltage_v; }

    // Per-Unit Conversions (requires base values from machine parameters)
    /// @brief Get excitation current in per-unit (rated current = 1.0 pu)
    /// @param base_current_a Rated field current in Amperes
    real_t get_excitation_current_pu(float base_current_a) const noexcept
    {
        return real_t(_raw.excitation_current_a / base_current_a);
    }

    /// @brief Get excitation voltage in per-unit (rated voltage = 1.0 pu)
    /// @param base_voltage_v Rated field voltage in Volts
    real_t get_excitation_voltage_pu(float base_voltage_v) const noexcept
    {
        return real_t(_raw.excitation_voltage_v / base_voltage_v);
    }

    /// @brief Access raw measurements (for debugging or hardware layer)
    const sfc_analog_input_raw& get_raw() const noexcept { return _raw; }

private:
    const sfc_analog_input_raw& _raw;
};

/// @brief Write interface to analog outputs with unit conversion
class sfc_analog_outputs {
public:
    /// @brief Construct with reference to raw output struct
    explicit sfc_analog_outputs(sfc_analog_output_raw& raw) noexcept : _raw(raw) {}

    // Raw Engineering Units
    void set_excitation_voltage_ref_v(float voltage_v) noexcept { _raw.excitation_voltage_ref_v = voltage_v; }

    void set_excitation_current_ref_a(float current_a) noexcept { _raw.excitation_current_ref_a = current_a; }

    // Per-Unit Conversions
    /// @brief Set excitation voltage reference from per-unit value
    /// @param voltage_pu Voltage setpoint in per-unit
    /// @param base_voltage_v Rated field voltage in Volts
    void set_excitation_voltage_ref_pu(real_t voltage_pu, float base_voltage_v) noexcept
    {
        _raw.excitation_voltage_ref_v = voltage_pu * base_voltage_v;
    }

    /// @brief Set excitation current reference from per-unit value
    /// @param current_pu Current setpoint in per-unit
    /// @param base_current_a Rated field current in Amperes
    void set_excitation_current_ref_pu(real_t current_pu, float base_current_a) noexcept
    {
        _raw.excitation_current_ref_a = current_pu * base_current_a;
    }

    /// @brief Access raw setpoints (for hardware layer or debugging)
    sfc_analog_output_raw& get_raw() noexcept { return _raw; }
    const sfc_analog_output_raw& get_raw() const noexcept { return _raw; }

    /// @brief Reset all outputs to zero
    void reset_all() noexcept
    {
        _raw = sfc_analog_output_raw{};  // Zero-initialize
    }

private:
    sfc_analog_output_raw& _raw;
};

}  // namespace sfc
}  // namespace yeet
