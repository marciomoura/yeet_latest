/**
 * @file sfc_per_unit_system_datamodel.hpp
 * @brief Per-unit system parameters as a datamodel
 *
 * Provides a datamodel interface for the SFC per-unit system base quantities,
 * allowing these fundamental system parameters to be accessed, monitored, and
 * optionally tuned through the network parameter interface.
 *
 * The per-unit system is used throughout the SFC control system to normalize
 * all signals and parameters. These base values are essential for understanding
 * the per-unit scales used in all other components.
 */

#pragma once

#include <memory>

#include "sfc/sfc_per_unit_system.hpp"
#include "sfc/sfc_types.hpp"
#include "yeet/model_registry.hpp"
#include "yeet/signal.hpp"
#include "yeet_control/adc_value.hpp"
#include "yeet_control/six_pulse_generator.hpp"

namespace yeet::application::sfc::integration {

/**
 * @brief Per-unit system datamodel for network parameter access
 *
 * This datamodel exposes the base quantities used in the SFC per-unit system
 * to the network parameter interface. All values are read-only after initialization
 * but are registered so they can be queried via the parameter protocol.
 *
 * The per-unit system defines base quantities for:
 * - Grid-side AC quantities (voltage, current, frequency)
 * - DC link quantities (current base)
 * - Machine-side AC quantities (voltage, frequency)
 * - Machine mechanical parameters (pole pairs)
 */
class sfc_machine_side_external_ios_datamodel {
public:
    /**
     * @brief Parameters structure containing all per-unit system base values
     *
     * These are initialized from the sfc_per_unit_system object during construction
     * and exposed through the datamodel interface.
     */
    struct parameters {
        yeet::parameter<bool> override_excitation_inputs{false};
        yeet::parameter<bool> exciter_ready{false};
        yeet::parameter<bool> excitation_start_command{false};
        yeet::parameter<bool> excitation_stop_command{false};

        // Speed generator profile selection bridge signals (turbine starting sequence)
        yeet::parameter<bool> override_speed_generator_profiles{false};
        yeet::parameter<bool> run_creep_speed{false};     // Profile 1: Creep speed (~5% rated)
        yeet::parameter<bool> run_purge_accel{false};     // Profile 2: Purge acceleration (~20% rated)
        yeet::parameter<bool> run_ignition_accel{false};  // Profile 3: Ignition acceleration (~55% rated)
        yeet::parameter<bool> run_assist_accel{false};    // Profile 4: Assist to self-sustaining (~95% rated)
        yeet::parameter<bool> hold_speed{false};          // Hold current speed setpoint
        yeet::parameter<bool> decelerate{false};          // Controlled deceleration to zero
        yeet::parameter<bool> idle{false};                // Idle mode (zero torque/speed outputs)

        // Speed controller cascade bridge signals from grid-side
        parameter<bool> override_fiber_optics_inputs{false};         ///< Acknowledgement of forced commutation
        parameter<bool> force_commutation_acknowledged{false};       ///< Acknowledgement of forced commutation
        parameter<bool> is_current_zero{false};                      ///< Indicator that machine current is zero
        parameter<bool> current_controller_is_saturated{false};      ///< Indicator that current controller is saturated
        parameter<bool> pulse_releases{true};                        ///< Current controller saturation flag
        parameter<current_pu_t> dc_link_current{current_pu_t{0.4}};  ///< DC link current in per-unit
    };

    /**
     * @brief Output signals for the machine-side pulse generator
     */
    struct outputs {
        // Excitation control bridge signals
        yeet::signal_output<bool> exciter_ready;
        yeet::signal_output<bool> excitation_start_command;
        yeet::signal_output<bool> excitation_stop_command;
        // Speed generator profile selection bridge signals (turbine starting sequence)
        yeet::signal_output<bool> run_creep_speed;     // Profile 1: Creep speed (~5% rated)
        yeet::signal_output<bool> run_purge_accel;     // Profile 2: Purge acceleration (~20% rated)
        yeet::signal_output<bool> run_ignition_accel;  // Profile 3: Ignition acceleration (~55% rated)
        yeet::signal_output<bool> run_assist_accel;    // Profile 4: Assist to self-sustaining (~95% rated)
        yeet::signal_output<bool> hold_speed;          // Hold current speed setpoint
        yeet::signal_output<bool> decelerate;          // Controlled deceleration to zero
        yeet::signal_output<bool> idle;                // Idle mode (zero torque/speed outputs)
    };

    struct control_inputs {
        signal_output<control::adc_counts> v_ab_counts{};  ///< Machine voltage AB differential (ADC counts)
        signal_output<control::adc_counts> v_bc_counts{};  ///< Machine voltage BC differential (ADC counts)

        ///< These are comming from the fiber optics...
        signal_output<sfc_operating_mode_t> operating_mode{
            sfc_operating_mode_t::natural_commutation};              ///< SFC operating mode
        signal_output<bool> force_commutation_acknowledged{false};   ///< Acknowledgement of forced commutation
        signal_output<bool> is_current_zero{false};                  ///< Indicator that machine current is zero
        signal_output<bool> current_controller_is_saturated{false};  ///< Indicator that current controller is saturated
        signal_output<bool> pulse_releases{true};                    ///< Current controller saturation flag
        signal_output<current_pu_t> dc_link_current{current_pu_t{0.4}};  ///< DC link current in per-unit
    };

    struct control_outputs {
        signal_input<control::six_pulse_generator::pulses> firing_pulses;
    };

    sfc_machine_side_external_ios_datamodel() { register_callbacks(); }

    /**
     * @brief Get parameters structure (const)
     * @return Const reference to parameters
     */
    const parameters& get_parameters() const { return _parameters; }
    const outputs& get_outputs() const { return _outputs; }
    const control_outputs& get_control_outputs() const { return _control_outputs; }
    const control_inputs& get_control_inputs() const { return _control_inputs; }
    parameters& get_parameters() { return _parameters; }
    outputs& get_outputs() { return _outputs; }
    control_outputs& get_control_outputs() { return _control_outputs; }
    control_inputs& get_control_inputs() { return _control_inputs; }

    /**
     * @brief Register datamodel with model registry
     */
    void register_datamodel(yeet::model_registry& registry);

    void register_callbacks()
    {
        auto excitation_inputs_callback = [this]() { write_excitation_overrides(); };

        auto speed_generator_profiles_callback = [this]() { write_speed_generator_overrides(); };

        auto fiber_optics_inputs_callback = [this]() { write_fiber_optics_input_overrides(); };

        _parameters.override_excitation_inputs.register_callback(excitation_inputs_callback);
        _parameters.exciter_ready.register_callback(excitation_inputs_callback);
        _parameters.excitation_start_command.register_callback(excitation_inputs_callback);
        _parameters.excitation_stop_command.register_callback(excitation_inputs_callback);

        _parameters.run_creep_speed.register_callback(excitation_inputs_callback);
        _parameters.run_purge_accel.register_callback(excitation_inputs_callback);
        _parameters.run_ignition_accel.register_callback(excitation_inputs_callback);
        _parameters.run_assist_accel.register_callback(excitation_inputs_callback);
        _parameters.hold_speed.register_callback(excitation_inputs_callback);
        _parameters.decelerate.register_callback(excitation_inputs_callback);
        _parameters.idle.register_callback(excitation_inputs_callback);

        _parameters.override_fiber_optics_inputs.register_callback(fiber_optics_inputs_callback);
        _parameters.force_commutation_acknowledged.register_callback(fiber_optics_inputs_callback);
        _parameters.is_current_zero.register_callback(fiber_optics_inputs_callback);
        _parameters.current_controller_is_saturated.register_callback(fiber_optics_inputs_callback);
        _parameters.pulse_releases.register_callback(fiber_optics_inputs_callback);
        _parameters.dc_link_current.register_callback(fiber_optics_inputs_callback);
    }

    void update_analog_inputs(std::int16_t v_ab_counts, std::int16_t v_bc_counts)
    {
        _control_inputs.v_ab_counts.write(control::adc_counts{v_ab_counts});
        _control_inputs.v_bc_counts.write(control::adc_counts{v_bc_counts});
    }

    control::six_pulse_generator::pulses get_thyristor_pulse_outputs() const
    {
        return _control_outputs.firing_pulses.read();
    }

    void update_grid_side_control_inputs(sfc_operating_mode_t operating_mode,
        current_pu_t dc_link_current,
        bool force_commutation_acknowledged,
        bool is_current_zero,
        bool current_controller_is_saturated,
        bool pulse_releases)
    {
        // _control_inputs.operating_mode.write(operating_mode);
        // _control_inputs.dc_link_current.write(dc_link_current);
        // _control_inputs.is_current_zero.write(is_current_zero);
        // _control_inputs.force_commutation_acknowledged.write(force_commutation_acknowledged);
        // _control_inputs.current_controller_is_saturated.write(current_controller_is_saturated);
        // _control_inputs.pulse_releases.write(pulse_releases);

        // _control_inputs.operating_mode.write();
        _control_inputs.dc_link_current.write(current_pu_t{0.4f});
        _control_inputs.is_current_zero.write(false);
        _control_inputs.force_commutation_acknowledged.write(false);
        _control_inputs.current_controller_is_saturated.write(false);
        _control_inputs.pulse_releases.write(true);
    }

    void update_digital_inputs(bool exciter_ready,
        bool excitation_start_command,
        bool excitation_stop_command,
        bool run_creep_speed,
        bool run_purge_accel,
        bool run_ignition_accel,
        bool run_assist_accel,
        bool hold_speed,
        bool decelerate,
        bool idle,
        bool enable_speed_control,
        bool freeze_integrator,
        bool modulator_freeze_integrator)
    {
        const bool excitation_overrides_written = write_excitation_overrides();
        if (!excitation_overrides_written) {
            _outputs.exciter_ready.write(exciter_ready);
            _outputs.excitation_start_command.write(excitation_start_command);
            _outputs.excitation_stop_command.write(excitation_stop_command);
        }

        const bool speed_generator_overrides_written = write_speed_generator_overrides();
        if (!speed_generator_overrides_written) {
            _outputs.run_creep_speed.write(run_creep_speed);
            _outputs.run_purge_accel.write(run_purge_accel);
            _outputs.run_ignition_accel.write(run_ignition_accel);
            _outputs.run_assist_accel.write(run_assist_accel);
            _outputs.hold_speed.write(hold_speed);
            _outputs.decelerate.write(decelerate);
            _outputs.idle.write(idle);
        }

        // const bool grid_side_inputs_overrides_written = write_grid_side_input_overrides();
        // if (!grid_side_inputs_overrides_written) {
        //     _control_inputs.pulse_releases.write(enable_speed_control);
        //     _control_inputs..write(freeze_integrator);
        //     _control_inputs.modulator_freeze_integrator.write(modulator_freeze_integrator);
        // }
    }

private:
    bool write_excitation_overrides()
    {
        if (!_parameters.override_excitation_inputs.read()) {
            return false;
        }

        _outputs.exciter_ready.write(_parameters.exciter_ready.read());
        _outputs.excitation_start_command.write(_parameters.excitation_start_command.read());
        _outputs.excitation_stop_command.write(_parameters.excitation_stop_command.read());
        return true;
    }

    bool write_speed_generator_overrides()
    {
        if (!_parameters.override_speed_generator_profiles.read()) {
            return false;
        }

        _outputs.run_creep_speed.write(_parameters.run_creep_speed.read());
        _outputs.run_purge_accel.write(_parameters.run_purge_accel.read());
        _outputs.run_ignition_accel.write(_parameters.run_ignition_accel.read());
        _outputs.run_assist_accel.write(_parameters.run_assist_accel.read());
        _outputs.hold_speed.write(_parameters.hold_speed.read());
        _outputs.decelerate.write(_parameters.decelerate.read());
        _outputs.idle.write(_parameters.idle.read());
        return true;
    }

    bool write_fiber_optics_input_overrides()
    {
        if (!_parameters.override_fiber_optics_inputs.read()) {
            return false;
        }

        _control_inputs.force_commutation_acknowledged.write(_parameters.force_commutation_acknowledged.read());
        _control_inputs.is_current_zero.write(_parameters.is_current_zero.read());
        _control_inputs.current_controller_is_saturated.write(_parameters.current_controller_is_saturated.read());
        _control_inputs.pulse_releases.write(_parameters.pulse_releases.read());
        _control_inputs.dc_link_current.write(_parameters.dc_link_current.read());
        return true;
    }

    parameters _parameters;  ///< Per-unit system base quantity parameters
    outputs _outputs;
    control_outputs _control_outputs;  ///< Control output signals
    control_inputs _control_inputs;    ///< Control input signals
};

}  // namespace yeet::application::sfc::integration
