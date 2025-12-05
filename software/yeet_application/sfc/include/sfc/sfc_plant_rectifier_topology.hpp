#pragma once

#include <array>
#include <memory>

#include "sfc_plant_back_to_back_topology_interface.hpp"
#include "sfc_plant_model_parameters.hpp"
#include "yeet_control/angle_wrapped.hpp"
#include "yeet_control/reference_frame.hpp"
#include "yeet_control/three_phase_waveform_generator.hpp"
#include "yeet_control/thyristor_rectifier.hpp"
#include "yeet_control/units.hpp"

namespace yeet::application::sfc {

/**
 * @brief Rectifier-only plant topology for testing current control.
 *
 * This class implements the back-to-back topology interface but operates only
 * the grid-side rectifier with the machine-side effectively short-circuited
 * (zero back-EMF voltage). This allows testing of:
 *   - Grid-side current controller tuning
 *   - Rectifier firing angle control
 *   - DC link current regulation
 *
 * System topology:
 *   V_grid_abc → [R_grid, L_grid] → Thyristor Rectifier → [R_dc, L_dc] → Short circuit (V=0)
 *
 * The machine-side pulses and voltages are ignored. Machine-side outputs return
 * zero values. This provides a clean test environment for the rectifier half of
 * the back-to-back converter without machine dynamics.
 */
class sfc_plant_rectifier_topology : public sfc_plant_back_to_back_topology_interface {
public:
    /**
     * @brief Constructs the rectifier topology with configurable parameters.
     * @param electrical_params Electrical configuration parameters
     */
    explicit sfc_plant_rectifier_topology(const sfc_plant_model_electrical_parameters& electrical_params);

    /**
     * @brief Constructs the rectifier topology with default configuration.
     */
    sfc_plant_rectifier_topology();

    ~sfc_plant_rectifier_topology() override;

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

    // Circuit breaker control
    void set_grid_circuit_breaker_closed(bool closed) override;
    bool is_grid_circuit_breaker_closed() const noexcept override;

    /**
     * @brief Get access to the underlying thyristor rectifier model.
     * @return Const reference to the internal rectifier model
     */
    const control::thyristor_rectifier& get_rectifier() const noexcept { return *_rectifier; }

private:
    static sfc_plant_model_electrical_parameters create_default_electrical_parameters() noexcept;
    static sfc_plant_model_mechanical_parameters create_default_mechanical_parameters() noexcept;
    static sfc_plant_model_voltage_control_parameters create_default_voltage_control_parameters() noexcept;

    void rebuild_rectifier();

private:
    // Parameters
    sfc_plant_model_electrical_parameters _electrical_params;
    sfc_plant_model_mechanical_parameters _mechanical_params;
    sfc_plant_model_voltage_control_parameters _voltage_control_params;

    // Three-phase voltage source generator (grid side only)
    std::unique_ptr<control::three_phase_waveform_generator> _grid_voltage_generator;

    // Rectifier model (no machine-side inverter)
    std::unique_ptr<control::thyristor_rectifier> _rectifier;

    // Current firing pulses (grid side only - machine side ignored)
    std::array<bool, 6> _grid_pulses{};

    // Cached voltages for output
    abc<voltage_t> _grid_voltages{voltage_t{0.0}, voltage_t{0.0}, voltage_t{0.0}};

    // Machine-side angle tracking (for interface compatibility)
    angle_wrapped _machine_angle{0.0};

    // Circuit breaker state (true = closed/conducting, false = open/isolated)
    bool _grid_circuit_breaker_closed{true};
};

}  // namespace yeet::application::sfc
