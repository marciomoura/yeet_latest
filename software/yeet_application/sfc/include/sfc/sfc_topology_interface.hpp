#pragma once

#include <array>

#include "sfc_plant_model_parameters.hpp"
#include "yeet_control/angle_wrapped.hpp"
#include "yeet_control/reference_frame.hpp"
#include "yeet_control/units.hpp"

namespace yeet::application::sfc {

/**
 * @brief Interface for SFC electrical topology models.
 *
 * This interface abstracts the underlying implementation of the electrical plant,
 * allowing switching between physics-based models (C++) and code-generated models (Simulink).
 */
class sfc_topology_interface {
public:
    virtual ~sfc_topology_interface() = default;

    /**
     * @brief Configures the electrical parameters of the topology.
     * @param params Electrical configuration parameters
     */
    virtual void configure_electrical_parameters(const sfc_plant_model_electrical_parameters& params) = 0;

    /**
     * @brief Initializes the topology model.
     */
    virtual void initialize() = 0;

    /**
     * @brief Resets the topology model to initial state.
     */
    virtual void reset() = 0;

    /**
     * @brief Terminates the topology model.
     */
    virtual void terminate() = 0;

    /**
     * @brief Updates the topology state.
     * @param grid_pulses Firing pulses for the grid-side converter.
     * @param machine_pulses Firing pulses for the machine-side converter.
     * @param grid_freq Grid frequency.
     * @param grid_volt_rms Grid line-to-line RMS voltage.
     * @param machine_freq Machine frequency.
     * @param machine_volt_rms Machine line-to-line RMS voltage.
     */
    virtual void update(const std::array<bool, 6>& grid_pulses,
        const std::array<bool, 6>& machine_pulses,
        frequency_t grid_freq,
        voltage_t grid_volt_rms,
        frequency_t machine_freq,
        voltage_t machine_volt_rms) = 0;

    // ==================== Outputs ====================

    // Electrical Quantities
    virtual abc<voltage_t> get_grid_voltages() const noexcept = 0;
    virtual abc<current_t> get_grid_currents() const noexcept = 0;
    virtual abc<voltage_t> get_machine_voltages() const noexcept = 0;
    virtual abc<current_t> get_machine_currents() const noexcept = 0;
    virtual current_t get_dc_link_current() const noexcept = 0;

    // Source Angles
    virtual angle_wrapped get_grid_source_angle() const noexcept = 0;
    virtual angle_wrapped get_machine_source_angle() const noexcept = 0;

    // DC Voltages (Optional/Specific to some models)
    virtual voltage_t get_rectifier_dc_voltage() const noexcept = 0;
    virtual voltage_t get_inverter_dc_voltage() const noexcept = 0;

    // Conduction States (Optional/Specific to some models)
    virtual std::array<bool, 6> get_grid_bridge_conduction_states() const noexcept = 0;
    virtual std::array<bool, 6> get_machine_bridge_conduction_states() const noexcept = 0;
};

}  // namespace yeet::application::sfc
