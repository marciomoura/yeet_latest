#include "sfc_integration/sfc_machine_side_external_ios_datamodel.hpp"

#include <cmath>

#include "yeet/utils/parameter_accessors.hpp"

namespace yeet::application::sfc::integration {

void sfc_machine_side_external_ios_datamodel::register_datamodel(yeet::model_registry& registry)
{
    yeet::model_registry_entry entry{yeet::signal_registry{}, yeet::parameter_registry{}};

    // Register grid-side base quantities
    entry.parameters.insert(
        "exc_override_excitation_inputs", yeet::utils::make_units_accessor(_parameters.override_excitation_inputs));
    entry.parameters.insert("exc_exciter_ready", yeet::utils::make_units_accessor(_parameters.exciter_ready));
    entry.parameters.insert(
        "exc_excitation_start_command", yeet::utils::make_units_accessor(_parameters.excitation_start_command));
    entry.parameters.insert(
        "exc_excitation_stop_command", yeet::utils::make_units_accessor(_parameters.excitation_stop_command));

    entry.parameters.insert("speed_ref_override_speed_generator_profiles",
        yeet::utils::make_units_accessor(_parameters.override_speed_generator_profiles));
    entry.parameters.insert("speed_ref_run_creep_speed", yeet::utils::make_units_accessor(_parameters.run_creep_speed));
    entry.parameters.insert("speed_ref_run_purge_accel", yeet::utils::make_units_accessor(_parameters.run_purge_accel));
    entry.parameters.insert(
        "speed_ref_run_ignition_accel", yeet::utils::make_units_accessor(_parameters.run_ignition_accel));
    entry.parameters.insert(
        "speed_ref_run_assist_accel", yeet::utils::make_units_accessor(_parameters.run_assist_accel));
    entry.parameters.insert("speed_ref_hold_speed", yeet::utils::make_units_accessor(_parameters.hold_speed));
    entry.parameters.insert("speed_ref_decelerate", yeet::utils::make_units_accessor(_parameters.decelerate));
    entry.parameters.insert("speed_ref_idle", yeet::utils::make_units_accessor(_parameters.idle));

    entry.parameters.insert(
        "fo_override_fiber_optics_inputs", yeet::utils::make_units_accessor(_parameters.override_fiber_optics_inputs));
    entry.parameters.insert("fo_force_commutation_acknowledged",
        yeet::utils::make_units_accessor(_parameters.force_commutation_acknowledged));
    entry.parameters.insert("fo_is_current_zero", yeet::utils::make_units_accessor(_parameters.is_current_zero));
    entry.parameters.insert("fo_current_controller_is_saturated",
        yeet::utils::make_units_accessor(_parameters.current_controller_is_saturated));
    entry.parameters.insert("fo_pulse_releases", yeet::utils::make_units_accessor(_parameters.pulse_releases));
    entry.parameters.insert("fo_dc_link_current", yeet::utils::make_units_accessor(_parameters.dc_link_current));
    entry.parameters.insert("fo_freeze_integrator", yeet::utils::make_units_accessor(_parameters.freeze_integrator));
    entry.parameters.insert(
        "fo_is_requesting_main_pi_reset", yeet::utils::make_units_accessor(_parameters.is_requesting_main_pi_reset));
    entry.parameters.insert("fo_current_state_id", yeet::utils::make_units_accessor(_parameters.current_state_id));

    // Register the entry with the model registry
    registry.insert("machine_side_external_ios", std::move(entry));
}

}  // namespace yeet::application::sfc::integration
