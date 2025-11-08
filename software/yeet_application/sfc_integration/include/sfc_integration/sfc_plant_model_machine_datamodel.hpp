#pragma once

#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
#include "yeet/signal.hpp"
#include "yeet/utils/parameter_accessors.hpp"
#include "yeet_control/angle_wrapped.hpp"
#include "yeet_control/reference_frame.hpp"
#include "yeet_control/six_pulse_generator.hpp"
#include "yeet_control/units.hpp"

/**
 * @file sfc_plant_model_machine_datamodel.hpp
 * @brief Datamodel specification for SFC Plant Model with Complete Machine component
 */

namespace yeet::application::sfc::integration {

class sfc_plant_model_machine_datamodel {
public:
    /**
     * @brief Input signals for the SFC plant model machine component
     */
    struct inputs {
        /// Grid-side firing pulses (6-pulse rectifier)
        signal_input<control::six_pulse_generator::pulses> grid_firing_pulses;

        /// Machine-side firing pulses (6-pulse inverter)
        signal_input<control::six_pulse_generator::pulses> machine_firing_pulses;
    };

    /**
     * @brief Output signals from the SFC plant model machine component
     */
    struct outputs {
        /// DC link voltage [V]
        signal_output<voltage_t> dc_link_voltage;

        /// DC link current [A]
        signal_output<current_t> dc_link_current;

        /// Grid-side current measurements [A]
        signal_output<abc<current_t>> grid_current_abc;

        /// Grid-side voltage measurements (line-to-line) [V]
        signal_output<abc<voltage_t>> grid_voltage_abc;

        /// Grid voltage source angular position [rad]
        signal_output<angle_wrapped> grid_voltage_source_angle;

        /// Grid voltage source phase-to-neutral voltages [V]
        signal_output<abc<voltage_t>> grid_voltage_source_phase_to_neutral_abc;

        /// Grid voltage source currents [A]
        signal_output<abc<current_t>> grid_voltage_source_currents_abc;

        /// Machine-side current measurements [A]
        signal_output<abc<current_t>> machine_current_abc;

        /// Machine-side voltage measurements (line-to-line) [V]
        signal_output<abc<voltage_t>> machine_voltage_abc;

        /// Machine-side DC voltage [V]
        signal_output<voltage_t> machine_dc_voltage;

        /// Machine stator currents (abc) [A]
        signal_output<abc<current_t>> machine_stator_currents_abc;

        /// Machine mechanical speed [rad/s]
        signal_output<speed_t> machine_speed;

        /// Machine electrical torque [Nm]
        signal_output<torque_t> machine_electrical_torque;

        /// Machine mechanical angle [rad]
        signal_output<angle_wrapped> machine_mechanical_angle;

        /// Machine load torque [Nm]
        signal_output<torque_t> machine_load_torque;

        /// Machine electrical power [W]
        signal_output<power_t> machine_electrical_power;

        /// Machine stator current d-axis [A]
        signal_output<current_t> machine_stator_current_d;

        /// Machine stator current q-axis [A]
        signal_output<current_t> machine_stator_current_q;

        /// Machine stator voltage d-axis [V]
        signal_output<voltage_t> machine_stator_voltage_d;

        /// Machine stator voltage q-axis [V]
        signal_output<voltage_t> machine_stator_voltage_q;

        /// Machine rotor flux d-axis [Wb]
        signal_output<magnetic_flux_t> machine_flux_d;

        /// Machine rotor flux q-axis [Wb]
        signal_output<magnetic_flux_t> machine_flux_q;

        /// Machine field current d-axis [A]
        signal_output<current_t> machine_field_current_d;

        /// Machine damper current q1 [A]
        signal_output<current_t> machine_damper_current_q1;

        /// Machine damper current q2 [A]
        signal_output<current_t> machine_damper_current_q2;

        /// Machine damper current d [A]
        signal_output<current_t> machine_damper_current_d;

        /// Source voltage phase [rad]
        signal_output<angle_wrapped> source_voltage_phase;

        /// Grid frequency [Hz]
        signal_output<frequency_t> grid_frequency;
    };

    /**
     * @brief Configuration parameters for the SFC plant model machine
     */
    struct parameters {
        /// Field excitation voltage [V]
        parameter<voltage_t> field_voltage = voltage_t(100.0);

        /// Forced machine speed in per-unit (used when forced_speed_enabled = true)
        parameter<real_t> forced_speed_pu = real_t(1.0);

        /// Enable forced speed mode (true) or mechanical dynamics (false)
        parameter<bool> forced_speed_enabled = true;

        /// DC isolator state (true = closed, false = open)
        parameter<bool> dc_isolator_closed = false;

        /// Grid voltage magnitude phase A [V]
        /// Peak voltage: nominal is 2090 V rms line-to-line
        parameter<voltage_t> grid_voltage_magnitude_a = voltage_t(1707.0);

        /// Grid voltage magnitude phase B [V]
        /// Peak voltage: nominal is 2090 V rms line-to-line
        parameter<voltage_t> grid_voltage_magnitude_b = voltage_t(1707.0);

        /// Grid voltage magnitude phase C [V]
        /// Peak voltage: nominal is 2090 V rms line-to-line
        parameter<voltage_t> grid_voltage_magnitude_c = voltage_t(1707.0);

        /// Grid frequency [Hz]
        parameter<frequency_t> grid_frequency_param = frequency_t(50.0);

        /// Load torque coefficient c0 (constant term)
        parameter<real_t> load_coeff_c0 = real_t(0.0);

        /// Load torque coefficient c1 (linear term)
        parameter<real_t> load_coeff_c1 = real_t(0.0);

        /// Load torque coefficient c2 (quadratic term)
        parameter<real_t> load_coeff_c2 = real_t(0.0);
    };

private:
    inputs* _model_inputs;
    outputs* _model_outputs;
    parameters* _model_parameters;

public:
    sfc_plant_model_machine_datamodel(inputs* model_inputs, outputs* model_outputs, parameters* model_parameters)
        : _model_inputs(model_inputs), _model_outputs(model_outputs), _model_parameters(model_parameters)
    {
        assert(_model_inputs != nullptr);
        assert(_model_outputs != nullptr);
        assert(_model_parameters != nullptr);
    }

    void register_data_model(model_registry& registry)
    {
        model_registry_entry entry{signal_registry{}, parameter_registry{}};

        entry.parameters.insert("field_voltage", yeet::utils::make_units_accessor(_model_parameters->field_voltage));
        entry.parameters.insert(
            "forced_speed_pu", yeet::utils::make_units_accessor(_model_parameters->forced_speed_pu));
        entry.parameters.insert(
            "forced_speed_enabled", yeet::utils::make_units_accessor(_model_parameters->forced_speed_enabled));
        entry.parameters.insert(
            "dc_isolator_closed", yeet::utils::make_units_accessor(_model_parameters->dc_isolator_closed));
        entry.parameters.insert(
            "grid_voltage_magnitude_a", yeet::utils::make_units_accessor(_model_parameters->grid_voltage_magnitude_a));
        entry.parameters.insert(
            "grid_voltage_magnitude_b", yeet::utils::make_units_accessor(_model_parameters->grid_voltage_magnitude_b));
        entry.parameters.insert(
            "grid_voltage_magnitude_c", yeet::utils::make_units_accessor(_model_parameters->grid_voltage_magnitude_c));
        entry.parameters.insert(
            "grid_frequency", yeet::utils::make_units_accessor(_model_parameters->grid_frequency_param));
        entry.parameters.insert("load_coeff_c0", yeet::utils::make_units_accessor(_model_parameters->load_coeff_c0));
        entry.parameters.insert("load_coeff_c1", yeet::utils::make_units_accessor(_model_parameters->load_coeff_c1));
        entry.parameters.insert("load_coeff_c2", yeet::utils::make_units_accessor(_model_parameters->load_coeff_c2));

        registry.insert("plant_model_machine", std::move(entry));
    }

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }
};

}  // namespace yeet::application::sfc::integration
