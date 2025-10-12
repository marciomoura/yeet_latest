// Copyright 2025 Techperts
// Datamodel specification for SFC Slow IO component

#pragma once

#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
#include "yeet/signal.hpp"

/**
 * @file sfc_slow_io_datamodel.hpp
 * @brief Datamodel for slow I/O interface (TCS, excitation controller, auxiliary systems)
 *
 * This component provides the interface between the embedded hardware (FPGA GPIO, external ADC)
 * and the application layer. It operates at 100 Hz (10 ms sampling) for supervisory-level
 * signals that don't require real-time control.
 *
 * Scope: Digital/analog I/O for TCS commands, excitation controller interface, and auxiliary
 * system monitoring. Does NOT include fast control signals (grid/DC/machine measurements,
 * firing pulses) which are handled by the real-time controller at 20 kHz.
 */

namespace yeet::application::sfc::integration {

class sfc_slow_io_datamodel {
public:
    /**
     * @brief Input signals TO the slow IO runner (commands from other components)
     * These are commands that other components send to this runner to control
     * hardware outputs (fans, breakers, excitation, etc.)
     */
    struct inputs {
        // ===== Auxiliary Control Commands (FROM application TO hardware) =====
        /// Command to turn cooling fan 1 on/off
        signal_input<bool> fan1_on;

        /// Command to turn cooling fan 2 on/off
        signal_input<bool> fan2_on;

        /// Command to turn cooling fan 3 on/off
        signal_input<bool> fan3_on;

        /// Command to turn cooling fan 4 on/off
        signal_input<bool> fan4_on;

        // ===== Circuit Breaker Commands (FROM application TO hardware) =====
        /// Command input circuit breaker to close
        signal_input<bool> icb_close_command;

        /// Command input circuit breaker to open
        signal_input<bool> icb_open_command;

        /// Command input circuit breaker to trip immediately
        signal_input<bool> icb_trip_command;

        // ===== TCS Feedback Signals (FROM application TO TCS) =====
        /// SFC system is stopped (OPERATION_OFF state)
        signal_input<bool> operation_off_feedback;

        /// SFC system is running (OPERATION_ON state)
        signal_input<bool> operation_on_feedback;

        // ===== Excitation Controller Commands (FROM application TO exciter) =====
        /// Command excitation controller to turn on
        signal_input<bool> exciter_on_command;

        /// Command excitation controller to turn off
        signal_input<bool> exciter_off_command;

        /// Excitation controller mode selection (vendor-specific)
        signal_input<bool> exciter_control_mode;

        /// Excitation voltage reference setpoint [Volts]
        signal_input<float> excitation_voltage_ref_v;

        /// Excitation current reference setpoint [Amperes]
        signal_input<float> excitation_current_ref_a;
    };

    /**
     * @brief Output signals FROM the slow IO runner (feedback to other components)
     * These signals are read from hardware and published for other components
     * to use (TCS commands, protection status, excitation feedback, etc.)
     */
    struct outputs {
        // ===== TCS Command Inputs (FROM hardware TO application) =====
        /// Start command from turbine control system
        signal_output<bool> start_command;

        /// Stop command from turbine control system
        signal_output<bool> stop_command;

        // ===== Safety and Interlock Inputs (FROM hardware TO application) =====
        /// Front access door is open (safety interlock)
        signal_output<bool> front_door_open;

        /// Low voltage supervision power OK
        signal_output<bool> lv_supervision_ok;

        /// Main supervision power OK
        signal_output<bool> supervision_ok;

        /// Motor circuit breaker alarm condition
        signal_output<bool> mcb_open_alarm;

        // ===== Excitation Controller Status (FROM hardware TO application) =====
        /// External excitation controller is running
        signal_output<bool> exciter_is_running;

        /// Excitation voltage is applied to field winding
        signal_output<bool> exciter_voltage_applied;

        /// Excitation field current feedback [Amperes]
        signal_output<float> excitation_current_a;

        /// Excitation field voltage feedback [Volts]
        signal_output<float> excitation_voltage_v;

        // ===== Circuit Breaker Status (FROM hardware TO application) =====
        /// Input circuit breaker is closed
        signal_output<bool> icb_is_closed;

        /// Input circuit breaker is open
        signal_output<bool> icb_is_open;

        /// Input circuit breaker is tripping (in process)
        signal_output<bool> icb_is_tripping;

        // ===== Protection System Inputs (FROM hardware TO application) =====
        /// Transformer pressure alarm
        signal_output<bool> transformer_pressure_alarm;

        /// Overheating alarm (warning level)
        signal_output<bool> overheat_alarm;

        /// Overheating trip (fault level)
        signal_output<bool> overheat_trip;

        /// Transformer light gas alarm (Buchholz relay)
        signal_output<bool> transformer_light_gas_alarm;

        /// Transformer heavy gas trip (Buchholz relay)
        signal_output<bool> transformer_heavy_gas_trip;

        /// Emergency stop button pressed
        signal_output<bool> emergency_stop;

        /// Common hardware protection trip
        signal_output<bool> common_hardware_trip;

        /// External fault from auxiliary systems
        signal_output<bool> external_fault;

        /// Overvoltage fuse blown
        signal_output<bool> overvoltage_fuse_blown;

        /// Cooling fan stop alarm
        signal_output<bool> fan_stop_alarm;

        /// Reactor temperature alarm
        signal_output<bool> reactor_temp_alarm;

        /// Reactor temperature trip
        signal_output<bool> reactor_temp_trip;
    }; /**
        * @brief Configuration parameters for slow IO operations
        */
    struct parameters {
        /// Rated field current for per-unit conversion [Amperes]
        parameter<float> rated_field_current_a{100.0f};

        /// Rated field voltage for per-unit conversion [Volts]
        parameter<float> rated_field_voltage_v{300.0f};

        /// Current threshold for field establishment detection (per-unit)
        /// Field is considered established when current exceeds this value
        parameter<float> field_current_threshold_pu{0.10f};  // 10% of rated

        /// Voltage threshold for field establishment detection (per-unit)
        /// Field is considered established when voltage exceeds this value
        parameter<float> field_voltage_threshold_pu{0.80f};  // 80% of rated
    };
};

}  // namespace yeet::application::sfc::integration
