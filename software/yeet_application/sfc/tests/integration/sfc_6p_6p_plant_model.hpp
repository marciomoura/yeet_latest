#pragma once

#include <array>
#include <memory>

#include "yeet_control/angle.hpp"
#include "yeet_control/reference_frame.hpp"
#include "yeet_control/units.hpp"

using namespace yeet;

// Forward-declare the code-generated class to hide implementation details
namespace generated::sfc {
class sfc_6p_6p_topology;
}

namespace yeet::application::sfc {

/**
 * @class sfc_6p_6p_plant_model
 * @brief A wrapper for the sfc_6p_6p_topology Simulink model.
 *
 * This class provides a high-level interface to interact with the generated
 * plant model, offering type-safe accessors for physical quantities using
 * components from the yeet_control library. It is designed for use in
 * testing environments.
 */
class sfc_6p_6p_plant_model {
public:
    static constexpr double kSamplingTime = 10e-6;  // 100 µs sampling time (10 kHz)

    /**
     * @brief Constructs the plant model wrapper.
     */
    sfc_6p_6p_plant_model();

    /**
     * @brief Destructor.
     */
    ~sfc_6p_6p_plant_model();

    // Prevent copying and moving
    sfc_6p_6p_plant_model(const sfc_6p_6p_plant_model&) = delete;
    sfc_6p_6p_plant_model& operator=(const sfc_6p_6p_plant_model&) = delete;
    sfc_6p_6p_plant_model(sfc_6p_6p_plant_model&&) = delete;
    sfc_6p_6p_plant_model& operator=(sfc_6p_6p_plant_model&&) = delete;

    /**
     * @brief Initializes the underlying Simulink model.
     */
    void initialize();

    /**
     * @brief Executes one step of the simulation.
     */
    void step();

    /**
     * @brief Terminates the underlying Simulink model.
     */
    void terminate();

    /**
     * @brief Sets the firing pulses for the grid-side converter.
     * @param pulses An array of 6 pulse values (1.0 for on, 0.0 for off).
     */
    void set_grid_side_pulses(const std::array<bool, 6>& pulses);

    /**
     * @brief Sets the firing pulses for the machine-side converter.
     * @param pulses An array of 6 pulse values (1.0 for on, 0.0 for off).
     */
    void set_machine_side_pulses(const std::array<bool, 6>& pulses);

    /**
     * @brief Sets the grid-side voltage source parameters.
     * @param frequency_hz The frequency in Hertz.
     * @param voltage_rms The line-to-line RMS voltage.
     */
    void set_grid_parameters(frequency_t frequency_hz, voltage_t voltage_rms);

    /**
     * @brief Sets the machine-side voltage source parameters.
     * @param frequency_hz The frequency in Hertz.
     * @param voltage_rms The line-to-line RMS voltage.
     */
    void set_machine_parameters(frequency_t frequency_hz, voltage_t voltage_rms);

    /**
     * @brief Gets the grid-side voltage source frequency.
     * @return The grid frequency in Hertz.
     */
    frequency_t get_grid_frequency() const;

    /**
     * @brief Gets the machine-side voltage source frequency.
     * @return The machine frequency in Hertz.
     */
    frequency_t get_machine_frequency() const;

    /**
     * @brief Gets the grid-side RMS line-to-line voltage.
     * @return The RMS voltage in volts.
     */
    voltage_t get_rms_grid_voltage() const;

    /**
     * @brief Gets the machine-side RMS line-to-line voltage.
     * @return The RMS voltage in volts.
     */
    voltage_t get_rms_machine_voltage() const;

    /**
     * @brief Gets the three-phase voltages of the grid-side converter.
     * @return Voltages in the abc reference frame.
     */
    abc<voltage_t> get_grid_side_voltages_abc() const;

    /**
     * @brief Gets the three-phase currents of the grid-side converter.
     * @return Currents in the abc reference frame.
     */
    abc<current_t> get_grid_side_currents_abc() const;

    /**
     * @brief Gets the three-phase voltages of the machine-side converter.
     * @return Voltages in the abc reference frame.
     */
    abc<voltage_t> get_machine_side_voltages_abc() const;

    /**
     * @brief Gets the three-phase currents of the machine-side converter.
     * @return Currents in the abc reference frame.
     */
    abc<current_t> get_machine_side_currents_abc() const;

    /**
     * @brief Gets the angle of the grid-side voltage source.
     * @return The angle, wrapped between 0 and 2*pi.
     */
    angle<zero_to_two_pi> get_grid_voltage_source_angle() const;

    /**
     * @brief Gets the angle of the machine-side voltage source.
     * @return The angle, wrapped between 0 and 2*pi.
     */
    angle<zero_to_two_pi> get_machine_voltage_source_angle() const;

    /**
     * @brief Gets the DC link current.
     * @return The current in Amperes.
     */
    current_t get_dc_link_current() const;

private:
    std::unique_ptr<generated::sfc::sfc_6p_6p_topology> _model;
};

}  // namespace yeet::application::sfc
