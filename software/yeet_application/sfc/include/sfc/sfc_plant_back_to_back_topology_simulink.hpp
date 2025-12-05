#pragma once

#include <array>
#include <memory>

#include "sfc_plant_back_to_back_topology_interface.hpp"
#include "sfc_plant_model_parameters.hpp"
#include "yeet_control/angle_wrapped.hpp"
#include "yeet_control/reference_frame.hpp"
#include "yeet_control/units.hpp"

namespace generated::sfc {
class sfc_6p_6p_topology;
}

namespace yeet::application::sfc {

/**
 * @brief Wrapper class for the Simulink-generated 6-pulse to 6-pulse SFC topology model
 *
 * This class provides a clean C++ interface to the auto-generated Simulink model,
 * handling type conversions and providing accessor methods for all electrical quantities.
 * Parameters are configurable and not tied to specific default values.
 */
class sfc_plant_back_to_back_topology_simulink : public sfc_plant_back_to_back_topology_interface {
public:
    static constexpr double kSamplingTime = 10e-6;  // 10 µs sampling time (100 kHz)

    /**
     * @brief Constructs the plant model wrapper with configurable parameters.
     * @param electrical_params Electrical configuration parameters
     */
    explicit sfc_plant_back_to_back_topology_simulink(const sfc_plant_model_electrical_parameters& electrical_params);

    /**
     * @brief Constructs the plant model wrapper with default configuration for 6p-6p topology.
     */
    sfc_plant_back_to_back_topology_simulink();

    /**
     * @brief Destructor.
     */
    ~sfc_plant_back_to_back_topology_simulink() override;

    // Prevent copying and moving
    sfc_plant_back_to_back_topology_simulink(const sfc_plant_back_to_back_topology_simulink&) = delete;
    sfc_plant_back_to_back_topology_simulink& operator=(const sfc_plant_back_to_back_topology_simulink&) = delete;
    sfc_plant_back_to_back_topology_simulink(sfc_plant_back_to_back_topology_simulink&&) = delete;
    sfc_plant_back_to_back_topology_simulink& operator=(sfc_plant_back_to_back_topology_simulink&&) = delete;

    /**
     * @brief Configures the electrical parameters of the plant model.
     * @param electrical_params New electrical configuration parameters
     */
    void configure_electrical_parameters(const sfc_plant_model_electrical_parameters& electrical_params) override;

    /**
     * @brief Configures the mechanical parameters of the plant model.
     * @param mechanical_params New mechanical configuration parameters
     */
    void configure_mechanical_parameters(const sfc_plant_model_mechanical_parameters& mechanical_params) override;

    /**
     * @brief Configures the voltage control parameters of the plant model.
     * @param voltage_control_params New voltage control configuration parameters
     */
    void configure_voltage_control_parameters(
        const sfc_plant_model_voltage_control_parameters& voltage_control_params) override;

    /**
     * @brief Initializes the underlying Simulink model.
     */
    void initialize() override;

    /**
     * @brief Executes one step of the simulation.
     */
    void update() override;

    /**
     * @brief Updates the model with new firing pulses and voltage parameters.
     * @param grid_side_pulses The 6-pulse firing signals for the grid side.
     * @param machine_side_pulses The 6-pulse firing signals for the machine side.
     * @param grid_frequency_hz The grid frequency in Hertz.
     * @param machine_frequency_hz The machine frequency in Hertz.
     * @param grid_voltage_rms The RMS line-to-line voltage on the grid side.
     * @param machine_voltage_rms The RMS line-to-line voltage on the machine side.
     */
    void update(const std::array<bool, 6>& grid_side_pulses,
        const std::array<bool, 6>& machine_side_pulses,
        frequency_t grid_frequency_hz,
        frequency_t machine_frequency_hz,
        voltage_t grid_voltage_rms,
        voltage_t machine_voltage_rms) override;

    /**
     * @brief Terminates the underlying Simulink model.
     */
    void terminate() override;

    /**
     * @brief Sets the firing pulses for the grid-side converter.
     * @param pulses An array of 6 pulse values (true for on, false for off).
     */
    void set_grid_side_pulses(const std::array<bool, 6>& pulses) override;

    /**
     * @brief Sets the firing pulses for the machine-side converter.
     * @param pulses An array of 6 pulse values (true for on, false for off).
     */
    void set_machine_side_pulses(const std::array<bool, 6>& pulses) override;

    /**
     * @brief Sets the grid-side voltage source parameters.
     * @param frequency_hz The frequency in Hertz.
     * @param voltage_rms The line-to-line RMS voltage.
     */
    void set_grid_parameters(frequency_t frequency_hz, voltage_t voltage_rms) override;

    /**
     * @brief Sets the machine-side voltage source parameters.
     * @param frequency_hz The frequency in Hertz.
     * @param voltage_rms The line-to-line RMS voltage.
     */
    void set_machine_parameters(frequency_t frequency_hz, voltage_t voltage_rms) override;

    // Output getters - Current model state
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

    // Parameter getters - Current configuration values
    const sfc_plant_model_electrical_parameters& get_electrical_parameters() const noexcept override;
    const sfc_plant_model_mechanical_parameters& get_mechanical_parameters() const noexcept override;
    const sfc_plant_model_voltage_control_parameters& get_voltage_control_parameters() const noexcept override;

    // Circuit breaker control
    void set_grid_circuit_breaker_closed(bool closed) override;
    bool is_grid_circuit_breaker_closed() const noexcept override;

private:
    /**
     * @brief Creates default electrical parameters for 6p-6p topology.
     * @return Default electrical configuration parameters
     */
    static sfc_plant_model_electrical_parameters create_default_6p_6p_electrical_parameters() noexcept;

    /**
     * @brief Creates default mechanical parameters.
     * @return Default mechanical configuration parameters
     */
    static sfc_plant_model_mechanical_parameters create_default_mechanical_parameters() noexcept;

    /**
     * @brief Creates default voltage control parameters.
     * @return Default voltage control configuration parameters
     */
    static sfc_plant_model_voltage_control_parameters create_default_voltage_control_parameters() noexcept;

private:
    std::unique_ptr<generated::sfc::sfc_6p_6p_topology> _model;
    sfc_plant_model_electrical_parameters _electrical_params;
    sfc_plant_model_mechanical_parameters _mechanical_params;
    sfc_plant_model_voltage_control_parameters _voltage_control_params;

    // Circuit breaker state (true = closed/conducting, false = open/isolated)
    bool _grid_circuit_breaker_closed{true};
};

}  // namespace yeet::application::sfc
