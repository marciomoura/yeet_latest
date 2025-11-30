#pragma once

#include <array>

#include "sfc_plant_model_parameters.hpp"
#include "yeet_control/angle_wrapped.hpp"
#include "yeet_control/reference_frame.hpp"
#include "yeet_control/units.hpp"

namespace yeet::application::sfc {

/**
 * @brief Abstract interface for back-to-back converter plant models.
 *
 * This interface defines the common API for different implementations of
 * back-to-back converter models used in SFC applications. Implementations may
 * include Simulink-generated models or internal C++ thyristor models.
 */
class sfc_plant_back_to_back_topology_interface {
public:
    static constexpr double kSamplingTime = 10e-6;  // 10 µs sampling time (100 kHz)

    virtual ~sfc_plant_back_to_back_topology_interface() = default;

    // Prevent copying and moving
    sfc_plant_back_to_back_topology_interface(const sfc_plant_back_to_back_topology_interface&) = delete;
    sfc_plant_back_to_back_topology_interface& operator=(const sfc_plant_back_to_back_topology_interface&) = delete;
    sfc_plant_back_to_back_topology_interface(sfc_plant_back_to_back_topology_interface&&) = delete;
    sfc_plant_back_to_back_topology_interface& operator=(sfc_plant_back_to_back_topology_interface&&) = delete;

    /**
     * @brief Configures the electrical parameters of the plant model.
     * @param electrical_params New electrical configuration parameters
     */
    virtual void configure_electrical_parameters(const sfc_plant_model_electrical_parameters& electrical_params) = 0;

    /**
     * @brief Configures the mechanical parameters of the plant model.
     * @param mechanical_params New mechanical configuration parameters
     */
    virtual void configure_mechanical_parameters(const sfc_plant_model_mechanical_parameters& mechanical_params) = 0;

    /**
     * @brief Configures the voltage control parameters of the plant model.
     * @param voltage_control_params New voltage control configuration parameters
     */
    virtual void configure_voltage_control_parameters(
        const sfc_plant_model_voltage_control_parameters& voltage_control_params) = 0;

    /**
     * @brief Initializes the underlying plant model.
     */
    virtual void initialize() = 0;

    /**
     * @brief Executes one step of the simulation.
     */
    virtual void update() = 0;

    /**
     * @brief Updates the model with new firing pulses and voltage parameters.
     * @param grid_side_pulses The 6-pulse firing signals for the grid side.
     * @param machine_side_pulses The 6-pulse firing signals for the machine side.
     * @param grid_frequency_hz The grid frequency in Hertz.
     * @param machine_frequency_hz The machine frequency in Hertz.
     * @param grid_voltage_rms The RMS line-to-line voltage on the grid side.
     * @param machine_voltage_rms The RMS line-to-line voltage on the machine side.
     */
    virtual void update(const std::array<bool, 6>& grid_side_pulses,
        const std::array<bool, 6>& machine_side_pulses,
        frequency_t grid_frequency_hz,
        frequency_t machine_frequency_hz,
        voltage_t grid_voltage_rms,
        voltage_t machine_voltage_rms) = 0;

    /**
     * @brief Terminates the underlying plant model.
     */
    virtual void terminate() = 0;

    /**
     * @brief Sets the firing pulses for the grid-side converter.
     * @param pulses An array of 6 pulse values (true for on, false for off).
     */
    virtual void set_grid_side_pulses(const std::array<bool, 6>& pulses) = 0;

    /**
     * @brief Sets the firing pulses for the machine-side converter.
     * @param pulses An array of 6 pulse values (true for on, false for off).
     */
    virtual void set_machine_side_pulses(const std::array<bool, 6>& pulses) = 0;

    /**
     * @brief Sets the grid-side voltage source parameters.
     * @param frequency_hz The frequency in Hertz.
     * @param voltage_rms The line-to-line RMS voltage.
     */
    virtual void set_grid_parameters(frequency_t frequency_hz, voltage_t voltage_rms) = 0;

    /**
     * @brief Sets the machine-side voltage source parameters.
     * @param frequency_hz The frequency in Hertz.
     * @param voltage_rms The line-to-line RMS voltage.
     */
    virtual void set_machine_parameters(frequency_t frequency_hz, voltage_t voltage_rms) = 0;

    // Output getters - Current model state
    virtual frequency_t get_grid_frequency() const noexcept = 0;
    virtual frequency_t get_machine_frequency() const noexcept = 0;
    virtual voltage_t get_rms_grid_voltage() const noexcept = 0;
    virtual voltage_t get_rms_machine_voltage() const noexcept = 0;
    virtual abc<voltage_t> get_grid_side_voltages_abc() const noexcept = 0;
    virtual abc<current_t> get_grid_side_currents_abc() const noexcept = 0;
    virtual abc<voltage_t> get_machine_side_voltages_abc() const noexcept = 0;
    virtual abc<current_t> get_machine_side_currents_abc() const noexcept = 0;
    virtual angle_wrapped get_grid_voltage_source_angle() const noexcept = 0;
    virtual angle_wrapped get_machine_voltage_source_angle() const noexcept = 0;
    virtual current_t get_dc_link_current() const noexcept = 0;

    // Parameter getters - Current configuration values
    virtual const sfc_plant_model_electrical_parameters& get_electrical_parameters() const noexcept = 0;
    virtual const sfc_plant_model_mechanical_parameters& get_mechanical_parameters() const noexcept = 0;
    virtual const sfc_plant_model_voltage_control_parameters& get_voltage_control_parameters() const noexcept = 0;

protected:
    sfc_plant_back_to_back_topology_interface() = default;
};

}  // namespace yeet::application::sfc
