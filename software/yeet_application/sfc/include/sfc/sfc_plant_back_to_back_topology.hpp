#pragma once

#include <array>
#include <memory>

#include "sfc_plant_back_to_back_topology_interface.hpp"
#include "sfc_plant_model_parameters.hpp"
#include "yeet_control/angle_wrapped.hpp"
#include "yeet_control/reference_frame.hpp"
#include "yeet_control/three_phase_waveform_generator.hpp"
#include "yeet_control/thyristor_back_to_back.hpp"
#include "yeet_control/units.hpp"

namespace yeet::application::sfc {

/**
 * @brief Internal C++ implementation of a back-to-back converter plant model.
 *
 * This class provides a native C++ implementation using the thyristor_back_to_back
 * model from yeet_control. It includes internal three-phase voltage source generators
 * for both grid and machine sides.
 */
class sfc_plant_back_to_back_topology : public sfc_plant_back_to_back_topology_interface {
public:
    /**
     * @brief Constructs the plant model with configurable parameters.
     * @param electrical_params Electrical configuration parameters
     */
    explicit sfc_plant_back_to_back_topology(const sfc_plant_model_electrical_parameters& electrical_params);

    /**
     * @brief Constructs the plant model with default configuration.
     */
    sfc_plant_back_to_back_topology();

    ~sfc_plant_back_to_back_topology() override;

    // Interface implementation
    void configure_electrical_parameters(const sfc_plant_model_electrical_parameters& electrical_params) override;
    void configure_mechanical_parameters(const sfc_plant_model_mechanical_parameters& mechanical_params) override;
    void configure_voltage_control_parameters(
        const sfc_plant_model_voltage_control_parameters& voltage_control_params) override;

    void initialize() override;
    void update() override;
    void update(const std::array<bool, 6>& grid_side_pulses,
        const std::array<bool, 6>& machine_side_pulses,
        frequency_t grid_frequency_hz,
        frequency_t machine_frequency_hz,
        voltage_t grid_voltage_rms,
        voltage_t machine_voltage_rms) override;
    void terminate() override;

    void set_grid_side_pulses(const std::array<bool, 6>& pulses) override;
    void set_machine_side_pulses(const std::array<bool, 6>& pulses) override;
    void set_grid_parameters(frequency_t frequency_hz, voltage_t voltage_rms) override;
    void set_machine_parameters(frequency_t frequency_hz, voltage_t voltage_rms) override;

    // Output getters
    frequency_t get_grid_frequency() const noexcept override;
    frequency_t get_machine_frequency() const noexcept override;
    voltage_t get_rms_grid_voltage() const noexcept override;
    voltage_t get_rms_machine_voltage() const noexcept override;
    abc<voltage_t> get_grid_side_voltages_abc() const noexcept override;
    abc<current_t> get_grid_side_currents_abc() const noexcept override;
    abc<voltage_t> get_machine_side_voltages_abc() const noexcept override;
    abc<current_t> get_machine_side_currents_abc() const noexcept override;
    angle_wrapped get_grid_voltage_source_angle() const noexcept override;
    angle_wrapped get_machine_voltage_source_angle() const noexcept override;
    current_t get_dc_link_current() const noexcept override;

    // Parameter getters
    const sfc_plant_model_electrical_parameters& get_electrical_parameters() const noexcept override;
    const sfc_plant_model_mechanical_parameters& get_mechanical_parameters() const noexcept override;
    const sfc_plant_model_voltage_control_parameters& get_voltage_control_parameters() const noexcept override;

    /**
     * @brief Get access to the underlying thyristor back-to-back model.
     * @return Const reference to the internal thyristor model
     */
    const control::thyristor_back_to_back& get_thyristor_model() const noexcept { return *_converter; }

private:
    static sfc_plant_model_electrical_parameters create_default_electrical_parameters() noexcept;
    static sfc_plant_model_mechanical_parameters create_default_mechanical_parameters() noexcept;
    static sfc_plant_model_voltage_control_parameters create_default_voltage_control_parameters() noexcept;

    void update_voltage_generators();
    void rebuild_converter();
    real_t rms_to_peak_phase(voltage_t v_rms_ll) const noexcept;

private:
    // Parameters
    sfc_plant_model_electrical_parameters _electrical_params;
    sfc_plant_model_mechanical_parameters _mechanical_params;
    sfc_plant_model_voltage_control_parameters _voltage_control_params;

    // Three-phase voltage source generators
    std::unique_ptr<control::three_phase_waveform_generator> _grid_voltage_generator;
    std::unique_ptr<control::three_phase_waveform_generator> _machine_voltage_generator;

    // Back-to-back converter model
    std::unique_ptr<control::thyristor_back_to_back> _converter;

    // Current firing pulses
    std::array<bool, 6> _grid_pulses{};
    std::array<bool, 6> _machine_pulses{};

    // Cached voltages for output
    abc<voltage_t> _grid_voltages{voltage_t{0.0}, voltage_t{0.0}, voltage_t{0.0}};
    abc<voltage_t> _machine_voltages{voltage_t{0.0}, voltage_t{0.0}, voltage_t{0.0}};
};

}  // namespace yeet::application::sfc
