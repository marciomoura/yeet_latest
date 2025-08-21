#include "sfc_integration/sfc_grid_side_modulator_runner.hpp"

namespace yeet::application::sfc::integration {

void sfc_grid_side_modulator_runner::initialize()
{
    set_parameters();
    set_parameter_callback();
    _sfc_grid_side_modulator.reset();
}

void sfc_grid_side_modulator_runner::run()
{
    // Extract inputs for the simplified grid-side modulator interface

    // TODO: Connect to machine-side modulator for force commutation request
    bool request_force_commutation = false;

    // TODO: Connect to current sensor for DC link current status
    bool dc_link_current_is_zero = false;

    // Convert firing angle reference from angle_wrapped to angle_t
    angle_wrapped firing_angle_wrapped = _inputs.firing_angle_reference;
    angle_t firing_angle_from_main_pi = angle_t(firing_angle_wrapped.get_radians());

    // Update the modulator with simplified interface
    _sfc_grid_side_modulator.update(request_force_commutation, dc_link_current_is_zero, firing_angle_from_main_pi);

    // Get outputs from modulator
    angle_t firing_angle_rad = _sfc_grid_side_modulator.get_firing_angle_rad();
    _outputs.actual_firing_angle = angle_wrapped::from_radians(firing_angle_rad);

    // For now, generate simple firing pulses based on grid angle and firing angle
    // In a full implementation, this would use a six_pulse_generator
    control::six_pulse_generator::pulses default_pulses{};
    _outputs.firing_pulses = default_pulses;

    // Set status outputs
    _outputs.is_enabled = true;       // TODO: Connect from system controller
    _outputs.is_synchronized = true;  // Assume synchronized for now

    // TODO: Export the force commutation acknowledgment and PI reset request signals
    // bool force_commutation_acknowledged = _sfc_grid_side_modulator.get_force_commutation_acknowledged();
    // bool pi_reset_requested = _sfc_grid_side_modulator.get_is_requesting_main_pi_reset();
}

void sfc_grid_side_modulator_runner::stop() { _sfc_grid_side_modulator.reset(); }

void sfc_grid_side_modulator_runner::set_parameters()
{
    // Configure force commutation angle from max firing angle parameter
    angle_wrapped max_firing_angle_wrapped = _parameters.max_firing_angle;
    angle_t force_commutation_angle = angle_t(max_firing_angle_wrapped.get_radians());
    _sfc_grid_side_modulator.configure_force_commutation_angle_rad(force_commutation_angle);
}

void sfc_grid_side_modulator_runner::set_parameter_callback()
{
    _parameters.min_firing_angle.register_callback([&]() { set_parameters(); });
    _parameters.max_firing_angle.register_callback([&]() { set_parameters(); });
    _parameters.pulse_width.register_callback([&]() { set_parameters(); });
    _parameters.deadtime.register_callback([&]() { set_parameters(); });
    _parameters.nominal_grid_frequency.register_callback([&]() { set_parameters(); });
}

}  // namespace yeet::application::sfc::integration
