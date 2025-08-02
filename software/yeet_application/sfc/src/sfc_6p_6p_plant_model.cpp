#include "sfc/sfc_6p_6p_plant_model.hpp"

#include <algorithm>
#include <array>

#include "sfc_6p_6p_topology.h"  // Include the Simulink-generated header
#include "yeet_control/reference_frame.hpp"
#include "yeet_control/units.hpp"

using namespace yeet;

namespace yeet::application::sfc {

sfc_6p_6p_plant_model::sfc_6p_6p_plant_model() : _model(std::make_unique<generated::sfc::sfc_6p_6p_topology>()) {}

sfc_6p_6p_plant_model::~sfc_6p_6p_plant_model() {
    if (_model) {
        _model->terminate();
    }
}

void sfc_6p_6p_plant_model::initialize() { _model->initialize(); }

void sfc_6p_6p_plant_model::step() { _model->step(); }

void sfc_6p_6p_plant_model::terminate() { _model->terminate(); }

void sfc_6p_6p_plant_model::set_grid_side_pulses(const std::array<bool, 6>& pulses) {
    std::array<double, 6> double_pulses{};
    std::transform(pulses.cbegin(), pulses.cend(), double_pulses.begin(),
                   [](bool b) { return static_cast<double>(b); });
    _model->sfc_6p_6p_topology_U.grid_side_converter_pulses = double_pulses;
}

void sfc_6p_6p_plant_model::set_machine_side_pulses(const std::array<bool, 6>& pulses) {
    std::array<double, 6> double_pulses{};
    std::transform(pulses.cbegin(), pulses.cend(), double_pulses.begin(),
                   [](bool b) { return static_cast<double>(b); });
    _model->sfc_6p_6p_topology_U.machine_side_converter_pulses = double_pulses;
}

void sfc_6p_6p_plant_model::set_grid_parameters(frequency_t frequency_hz, voltage_t voltage_rms) {
    _model->sfc_6p_6p_topology_U.grid_voltage_source_frequency_hz = frequency_hz;
    _model->sfc_6p_6p_topology_U.grid_voltage_source_line_voltage_rms = voltage_rms;
}

void sfc_6p_6p_plant_model::set_machine_parameters(frequency_t frequency_hz, voltage_t voltage_rms) {
    _model->sfc_6p_6p_topology_U.machine_voltage_source_frequency_hz = frequency_hz;
    _model->sfc_6p_6p_topology_U.machine_voltage_source_line_voltage_rms = voltage_rms;
}

frequency_t sfc_6p_6p_plant_model::get_grid_frequency() const {
    return frequency_t(_model->sfc_6p_6p_topology_U.grid_voltage_source_frequency_hz);
}

frequency_t sfc_6p_6p_plant_model::get_machine_frequency() const {
    return frequency_t(_model->sfc_6p_6p_topology_U.machine_voltage_source_frequency_hz);
}

voltage_t sfc_6p_6p_plant_model::get_rms_grid_voltage() const {
    return voltage_t(_model->sfc_6p_6p_topology_U.grid_voltage_source_line_voltage_rms);
}

voltage_t sfc_6p_6p_plant_model::get_rms_machine_voltage() const {
    return voltage_t(_model->sfc_6p_6p_topology_U.machine_voltage_source_line_voltage_rms);
}

abc<voltage_t> sfc_6p_6p_plant_model::get_grid_side_voltages_abc() const {
    const auto& voltages = _model->sfc_6p_6p_topology_Y.grid_side_converter_voltage_phase_si;
    return to_dimension_abc<voltage_t>(abc<real_t>{voltages});
}

abc<current_t> sfc_6p_6p_plant_model::get_grid_side_currents_abc() const {
    const auto& currents = _model->sfc_6p_6p_topology_Y.grid_side_converter_current_si;
    return to_dimension_abc<current_t>(abc<real_t>{currents});
}

abc<voltage_t> sfc_6p_6p_plant_model::get_machine_side_voltages_abc() const {
    const auto& voltages = _model->sfc_6p_6p_topology_Y.machine_side_converter_voltage_phase_si;
    return to_dimension_abc<voltage_t>(abc<real_t>{voltages});
}

abc<current_t> sfc_6p_6p_plant_model::get_machine_side_currents_abc() const {
    const auto& currents = _model->sfc_6p_6p_topology_Y.machine_side_converter_current_si;
    return to_dimension_abc<current_t>(abc<real_t>{currents});
}

angle<zero_to_two_pi> sfc_6p_6p_plant_model::get_grid_voltage_source_angle() const {
    return angle<zero_to_two_pi>(_model->sfc_6p_6p_topology_Y.grid_voltage_source_angle_wt);
}

angle<zero_to_two_pi> sfc_6p_6p_plant_model::get_machine_voltage_source_angle() const {
    return angle<zero_to_two_pi>(_model->sfc_6p_6p_topology_Y.machine_voltage_source_angle_wt);
}

current_t sfc_6p_6p_plant_model::get_dc_link_current() const {
    return current_t(_model->sfc_6p_6p_topology_Y.dc_link_current_si);
}

}  // namespace yeet::application::sfc
