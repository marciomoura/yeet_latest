#pragma once

#include <array>
#include <memory>

#include "sfc_plant_model_parameters.hpp"
#include "yeet_control/angle_wrapped.hpp"
#include "yeet_control/reference_frame.hpp"
#include "yeet_control/units.hpp"

// Forward declaration for Simulink-generated class only
// The full header will be included in the .cpp file
class LCITopology6x6MinimalModelClass;

namespace yeet::application::sfc {
/**
 * @brief Wrapper class for the Simulink-generated 6-pulse to 6-pulse SFC topology with complete machine model
 *
 * This class provides a clean C++ interface to the auto-generated Simulink model that includes:
 * - Grid-side 6-pulse converter with input transformer
 * - DC link with realistic impedance
 * - Machine-side 6-pulse converter
 * - Complete synchronous machine model (stator, field, dampers)
 * - Mechanical load dynamics
 * - Field excitation control
 */
class sfc_plant_model_6p_6p_machine {
public:
    static constexpr double kSamplingTime = 12.5e-6;  // 12.5 µs sampling time (80 kHz)

    /**
     * @brief Constructs the plant model wrapper with configurable parameters.
     * @param transformer_params Input transformer configuration
     * @param dc_link_params DC link configuration
     * @param machine_electrical_params Machine electrical parameters
     * @param machine_mechanical_params Machine mechanical parameters
     * @param measurement_params Measurement scaling parameters
     */
    explicit sfc_plant_model_6p_6p_machine(const sfc_plant_model_transformer_parameters& transformer_params,
        const sfc_plant_model_dc_link_parameters& dc_link_params,
        const sfc_plant_model_machine_electrical_parameters& machine_electrical_params,
        const sfc_plant_model_machine_mechanical_parameters& machine_mechanical_params,
        const sfc_plant_model_measurement_parameters& measurement_params);

    /**
     * @brief Constructs the plant model wrapper with default configuration.
     */
    sfc_plant_model_6p_6p_machine();

    /**
     * @brief Destructor.
     */
    ~sfc_plant_model_6p_6p_machine();

    // Prevent copying and moving
    sfc_plant_model_6p_6p_machine(const sfc_plant_model_6p_6p_machine&) = delete;
    sfc_plant_model_6p_6p_machine& operator=(const sfc_plant_model_6p_6p_machine&) = delete;
    sfc_plant_model_6p_6p_machine(sfc_plant_model_6p_6p_machine&&) = delete;
    sfc_plant_model_6p_6p_machine& operator=(sfc_plant_model_6p_6p_machine&&) = delete;

    /**
     * @brief Configures the transformer parameters of the plant model.
     * @param transformer_params New transformer configuration parameters
     */
    void configure_transformer_parameters(const sfc_plant_model_transformer_parameters& transformer_params);

    /**
     * @brief Configures the DC link parameters of the plant model.
     * @param dc_link_params New DC link configuration parameters
     */
    void configure_dc_link_parameters(const sfc_plant_model_dc_link_parameters& dc_link_params);

    /**
     * @brief Configures the machine electrical parameters of the plant model.
     * @param machine_electrical_params New machine electrical configuration parameters
     */
    void configure_machine_electrical_parameters(
        const sfc_plant_model_machine_electrical_parameters& machine_electrical_params);

    /**
     * @brief Configures the machine mechanical parameters of the plant model.
     * @param machine_mechanical_params New machine mechanical configuration parameters
     */
    void configure_machine_mechanical_parameters(
        const sfc_plant_model_machine_mechanical_parameters& machine_mechanical_params);

    /**
     * @brief Configures the measurement scaling parameters of the plant model.
     * @param measurement_params New measurement scaling parameters
     */
    void configure_measurement_parameters(const sfc_plant_model_measurement_parameters& measurement_params);

    /**
     * @brief Initializes the underlying Simulink model.
     */
    void initialize();

    /**
     * @brief Executes one step of the simulation with current input values.
     */
    void update();

    /**
     * @brief Executes one step of the simulation with all input parameters.
     * @param grid_side_pulses The 6-pulse firing signals for the grid side converter
     * @param machine_side_pulses The 6-pulse firing signals for the machine side converter
     * @param grid_voltage_mag_a Grid voltage magnitude phase A
     * @param grid_voltage_mag_b Grid voltage magnitude phase B
     * @param grid_voltage_mag_c Grid voltage magnitude phase C
     * @param grid_frequency Grid frequency in Hz
     * @param field_voltage Machine field excitation voltage
     * @param load_coefficients Load torque polynomial coefficients [c0, c1, c2] for T = c0 + c1*w + c2*w^2
     * @param forced_speed_pu Forced speed in per-unit (if enabled)
     * @param enable_forced_speed Enable forced speed mode
     * @param close_dc_isolator Close the DC link isolator switch
     */
    void update(const std::array<bool, 6>& grid_side_pulses,
        const std::array<bool, 6>& machine_side_pulses,
        voltage_t grid_voltage_mag_a,
        voltage_t grid_voltage_mag_b,
        voltage_t grid_voltage_mag_c,
        frequency_t grid_frequency,
        voltage_t field_voltage,
        const std::array<real_t, 3>& load_coefficients,
        real_t forced_speed_pu,
        bool enable_forced_speed,
        bool close_dc_isolator);

    /**
     * @brief Terminates the underlying Simulink model.
     */
    void terminate();

    /**
     * @brief Sets the firing pulses for the grid-side converter.
     * @param pulses An array of 6 pulse values (true for on, false for off)
     */
    void set_grid_side_pulses(const std::array<bool, 6>& pulses);

    /**
     * @brief Sets the firing pulses for the machine-side converter.
     * @param pulses An array of 6 pulse values (true for on, false for off)
     */
    void set_machine_side_pulses(const std::array<bool, 6>& pulses);

    /**
     * @brief Sets the grid-side voltage source parameters.
     * @param voltage_mag_a Phase A voltage magnitude
     * @param voltage_mag_b Phase B voltage magnitude
     * @param voltage_mag_c Phase C voltage magnitude
     * @param frequency Grid frequency in Hz
     */
    void set_grid_voltage_source(
        voltage_t voltage_mag_a, voltage_t voltage_mag_b, voltage_t voltage_mag_c, frequency_t frequency);

    /**
     * @brief Sets the machine field excitation voltage.
     * @param field_voltage Field voltage
     */
    void set_field_voltage(voltage_t field_voltage);

    /**
     * @brief Sets the mechanical load coefficients.
     * @param load_coefficients Polynomial coefficients [c0, c1, c2] for load torque T = c0 + c1*w + c2*w^2
     */
    void set_load_coefficients(const std::array<real_t, 3>& load_coefficients);

    /**
     * @brief Sets the measurement scaling ratios (PT/CT).
     * @param grid_pt_ratio Grid-side potential transformer ratio
     * @param grid_ct_ratio Grid-side current transformer ratio
     * @param motor_pt_ratio Motor-side potential transformer ratio
     * @param motor_ct_ratio Motor-side current transformer ratio
     */
    void set_measurement_ratios(
        real_t grid_pt_ratio, real_t grid_ct_ratio, real_t motor_pt_ratio, real_t motor_ct_ratio);

    /**
     * @brief Sets the forced speed mode.
     * @param speed_pu Speed in per-unit
     * @param enable Enable forced speed mode (true) or mechanical dynamics (false)
     */
    void set_forced_speed(real_t speed_pu, bool enable);

    /**
     * @brief Sets the DC isolator state.
     * @param closed True to close the isolator, false to open
     */
    void set_dc_isolator(bool closed);

    // Output getters - DC link
    voltage_t get_dc_link_voltage() const noexcept;
    current_t get_dc_link_current() const noexcept;

    // Output getters - Grid side
    abc<current_t> get_grid_side_currents_abc() const noexcept;
    abc<voltage_t> get_grid_side_voltages_line_to_line_abc() const noexcept;
    angle_wrapped get_grid_voltage_source_angle() const noexcept;
    abc<voltage_t> get_grid_voltage_source_phase_to_neutral_abc() const noexcept;
    abc<current_t> get_grid_voltage_source_currents_abc() const noexcept;

    // Output getters - Machine side converter
    abc<current_t> get_machine_side_currents_abc() const noexcept;
    abc<voltage_t> get_machine_side_voltages_line_to_line_abc() const noexcept;
    voltage_t get_machine_side_dc_voltage() const noexcept;

    // Output getters - Machine mechanical state (grouped)
    const machine_mechanical_state& get_machine_mechanical_state() const noexcept;

    // Output getters - Machine stator state (grouped)
    const machine_stator_state_dq& get_machine_stator_state_dq() const noexcept;
    abc<current_t> get_machine_stator_currents_abc() const noexcept;

    // Output getters - Machine rotor state (grouped)
    const machine_rotor_state& get_machine_rotor_state() const noexcept;

    // Output getters - Source voltage phase
    angle_wrapped get_source_voltage_phase() const noexcept;

    // Parameter getters - Current configuration values
    const sfc_plant_model_transformer_parameters& get_transformer_parameters() const noexcept;
    const sfc_plant_model_dc_link_parameters& get_dc_link_parameters() const noexcept;
    const sfc_plant_model_machine_electrical_parameters& get_machine_electrical_parameters() const noexcept;
    const sfc_plant_model_machine_mechanical_parameters& get_machine_mechanical_parameters() const noexcept;
    const sfc_plant_model_measurement_parameters& get_measurement_parameters() const noexcept;

private:
    /**
     * @brief Creates default transformer parameters.
     * @return Default transformer configuration parameters
     */
    static sfc_plant_model_transformer_parameters create_default_transformer_parameters() noexcept;

    /**
     * @brief Creates default DC link parameters.
     * @return Default DC link configuration parameters
     */
    static sfc_plant_model_dc_link_parameters create_default_dc_link_parameters() noexcept;

    /**
     * @brief Creates default machine electrical parameters.
     * @return Default machine electrical configuration parameters
     */
    static sfc_plant_model_machine_electrical_parameters create_default_machine_electrical_parameters() noexcept;

    /**
     * @brief Creates default machine mechanical parameters.
     * @return Default machine mechanical configuration parameters
     */
    static sfc_plant_model_machine_mechanical_parameters create_default_machine_mechanical_parameters() noexcept;

    /**
     * @brief Creates default measurement parameters.
     * @return Default measurement scaling parameters
     */
    static sfc_plant_model_measurement_parameters create_default_measurement_parameters() noexcept;

    /**
     * @brief Updates the cached machine state values from the Simulink model outputs.
     */
    void update_cached_machine_states();

private:
    std::unique_ptr<LCITopology6x6MinimalModelClass> _model;
    sfc_plant_model_transformer_parameters _transformer_params;
    sfc_plant_model_dc_link_parameters _dc_link_params;
    sfc_plant_model_machine_electrical_parameters _machine_electrical_params;
    sfc_plant_model_machine_mechanical_parameters _machine_mechanical_params;
    sfc_plant_model_measurement_parameters _measurement_params;

    // Cached machine state outputs
    mutable machine_mechanical_state _cached_mechanical_state;
    mutable machine_stator_state_dq _cached_stator_state_dq;
    mutable machine_rotor_state _cached_rotor_state;
};
}  // namespace yeet::application::sfc