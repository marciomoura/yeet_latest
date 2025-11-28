#pragma once

#include "sfc_topology_interface.hpp"
#include "yeet_control/three_phase_waveform_generator.hpp"
#include "yeet_control/thyristor_back_to_back.hpp"

namespace yeet::application::sfc {

/**
 * @brief Physics-based implementation of the SFC topology (Internal Model).
 *
 * Uses `yeet_control::thyristor_back_to_back` and `yeet_control::three_phase_waveform_generator`
 * to simulate the electrical behavior of the SFC.
 */
class sfc_plant_internal_model : public sfc_topology_interface {
public:
    static constexpr double kSamplingTime = 10e-6;

    explicit sfc_plant_internal_model(const sfc_plant_model_electrical_parameters& params);
    ~sfc_plant_internal_model() override = default;

    void configure_electrical_parameters(const sfc_plant_model_electrical_parameters& params) override;
    void initialize() override;
    void reset() override;
    void terminate() override;

    void update(const std::array<bool, 6>& grid_pulses,
        const std::array<bool, 6>& machine_pulses,
        frequency_t grid_freq,
        voltage_t grid_volt_rms,
        frequency_t machine_freq,
        voltage_t machine_volt_rms) override;

    abc<voltage_t> get_grid_voltages() const noexcept override;
    abc<current_t> get_grid_currents() const noexcept override;
    abc<voltage_t> get_machine_voltages() const noexcept override;
    abc<current_t> get_machine_currents() const noexcept override;
    current_t get_dc_link_current() const noexcept override;

    angle_wrapped get_grid_source_angle() const noexcept override;
    angle_wrapped get_machine_source_angle() const noexcept override;

    voltage_t get_rectifier_dc_voltage() const noexcept override;
    voltage_t get_inverter_dc_voltage() const noexcept override;

    std::array<bool, 6> get_grid_bridge_conduction_states() const noexcept override;
    std::array<bool, 6> get_machine_bridge_conduction_states() const noexcept override;

private:
    void configure_waveform_generator(
        control::three_phase_waveform_generator& generator, frequency_t frequency, voltage_t voltage_rms) noexcept;

private:
    control::thyristor_back_to_back _converter;
    control::three_phase_waveform_generator _grid_waveform;
    control::three_phase_waveform_generator _machine_waveform;

    abc<voltage_t> _grid_source_voltages{voltage_t{0.0}, voltage_t{0.0}, voltage_t{0.0}};
    abc<voltage_t> _machine_source_voltages{voltage_t{0.0}, voltage_t{0.0}, voltage_t{0.0}};
    angle_wrapped _grid_source_angle{0.0};
    angle_wrapped _machine_source_angle{0.0};
};

}  // namespace yeet::application::sfc
