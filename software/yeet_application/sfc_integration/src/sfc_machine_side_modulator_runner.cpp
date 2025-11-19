#include "sfc_integration/sfc_machine_side_modulator_runner.hpp"

#include <cmath>

namespace yeet::application::sfc::integration {

void sfc_machine_side_modulator_runner::register_datamodel(yeet::model_registry& registry)
{
    // Delegate to the datamodel's registration method
    sfc_machine_side_modulator_datamodel datamodel(&_inputs, &_outputs, &_parameters);
    datamodel.register_data_model(registry);
}

void sfc_machine_side_modulator_runner::initialize()
{
    set_parameters();
    set_parameter_callback();
    _sfc_machine_side_modulator.reset();
}

void sfc_machine_side_modulator_runner::run()
{
    // Read inputs from the datamodel
    const bool enable = _inputs.release_pulses.read();
    const sfc_operating_mode_t operating_mode = _inputs.operating_mode.read();
    const bool force_commutation_acknowledged = _inputs.force_commutation_acknowledged.read();
    const angle_t estimated_rotor_angle = _inputs.machine_reference_angle.read().get_radians();
    const angle_t virtual_rotor_angle = _inputs.virtual_rotor_angle.read().get_radians();
    const reference_frame_source_t frame_source = _inputs.reference_frame_source.read();
    const angle_t firing_angle_from_lut = _inputs.firing_angle.read();

    // Select rotor angle based on reference frame source from startup sequencer
    const angle_t selected_rotor_angle =
        (frame_source == reference_frame_source_t::VIRTUAL) ? virtual_rotor_angle : estimated_rotor_angle;

    // Update the modulator - enable controlled via release_pulses input
    _sfc_machine_side_modulator.update(
        enable, operating_mode, force_commutation_acknowledged, selected_rotor_angle, firing_angle_from_lut);

    // Write outputs to the datamodel
    _outputs.firing_angle = _sfc_machine_side_modulator.get_firing_angle_rad();
    _outputs.release_pulses = _sfc_machine_side_modulator.get_release_pulses();
    _outputs.request_force_commutation = _sfc_machine_side_modulator.get_request_force_commutation();
    _outputs.commutation_state = _sfc_machine_side_modulator.get_commutation_state();

    // Get current rotor sector from modulator (0-5), convert to 1-6 for diagnostics
    const int sector_0_indexed = _sfc_machine_side_modulator.get_current_rotor_sector();
    _outputs.current_rotor_sector = (sector_0_indexed == -1) ? 0 : static_cast<uint8_t>(sector_0_indexed + 1);
}

void sfc_machine_side_modulator_runner::stop() { _sfc_machine_side_modulator.reset(); }

void sfc_machine_side_modulator_runner::set_parameters()
{
    // Note: forced_commutation_firing_angle parameter exists in datamodel
    // but is not yet configurable in the core sfc_machine_side_modulator component.
    // It is currently hardcoded to 180° in the core implementation.
    // Future enhancement: Add configure method to the core component.
}

void sfc_machine_side_modulator_runner::set_parameter_callback()
{
    _parameters.forced_commutation_firing_angle.register_callback([&]() { set_parameters(); });
}

}  // namespace yeet::application::sfc::integration
