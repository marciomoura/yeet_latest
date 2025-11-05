#pragma once

#include "sfc_per_unit_system.hpp"
#include "sfc_plant_model_6p_6p.hpp"
#include "sfc_plant_model_parameters.hpp"
#include "yeet_control/angle_wrapped.hpp"
#include "yeet_control/mechanical_load.hpp"
#include "yeet_control/reference_frame.hpp"
#include "yeet_control/speed_voltage_relationship.hpp"
#include "yeet_control/units.hpp"

namespace yeet::application::sfc {

/**
 * @brief Augmented 6-pulse to 6-pulse SFC plant model with mechanical dynamics and voltage control
 *
 * This class wraps the basic sfc_plant_model_6p_6p and adds mechanical load dynamics
 * and speed-voltage relationship control. It provides configurable parameters without
 * exposing topology-specific default values in the public interface.
 */
class sfc_plant_model_augmented_6p_6p {
public:
    static constexpr double kSamplingTime = 10e-6;  // 10 µs sampling time (100 kHz)

    /**
     * @brief Constructs the augmented plant model with configurable parameters.
     * @param electrical_params Electrical configuration parameters
     * @param mechanical_params Mechanical configuration parameters
     * @param voltage_control_params Voltage control configuration parameters
     */
    sfc_plant_model_augmented_6p_6p(const sfc_plant_model_electrical_parameters& electrical_params,
        const sfc_plant_model_mechanical_parameters& mechanical_params,
        const sfc_plant_model_voltage_control_parameters& voltage_control_params);

    /**
     * @brief Constructs the augmented plant model with default 6p-6p topology parameters.
     */
    sfc_plant_model_augmented_6p_6p();

    /**
     * @brief Destructor.
     */
    ~sfc_plant_model_augmented_6p_6p();

    // Prevent copying and moving
    sfc_plant_model_augmented_6p_6p(const sfc_plant_model_augmented_6p_6p&) = delete;
    sfc_plant_model_augmented_6p_6p& operator=(const sfc_plant_model_augmented_6p_6p&) = delete;
    sfc_plant_model_augmented_6p_6p(sfc_plant_model_augmented_6p_6p&&) = delete;
    sfc_plant_model_augmented_6p_6p& operator=(sfc_plant_model_augmented_6p_6p&&) = delete;

    /**
     * @brief Configures the electrical parameters of the plant model.
     * @param electrical_params New electrical configuration parameters
     */
    void configure_electrical_parameters(const sfc_plant_model_electrical_parameters& electrical_params);

    /**
     * @brief Configures the parameters of the mechanical load model.
     * @param mechanical_params New mechanical configuration parameters
     */
    void configure_mechanical_parameters(const sfc_plant_model_mechanical_parameters& mechanical_params);

    /**
     * @brief Configures the speed-voltage relationship parameters.
     * @param voltage_control_params New voltage control configuration parameters
     */
    void configure_voltage_control_parameters(const sfc_plant_model_voltage_control_parameters& voltage_control_params);

    /**
     * @brief Configures the parameters of the mechanical load model.
     * @param inertia_constant_seconds The inertia constant in seconds.
     * @param damping_coefficient The damping factor in per-unit.
     */
    void configure_mechanical_load(
        inertia_constant_t inertia_constant_seconds, damping_coefficient_pu_t damping_coefficient);

    /**
     * @brief Configures the speed-voltage relationship parameters.
     * @param coefficient The voltage-speed coefficient [pu/pu].
     * @param offset_pu The voltage offset in per-unit.
     * @param time_constant_seconds The time constant for dynamics [s]. Use 0.0 for algebraic response.
     */
    void configure_speed_voltage_relationship(real_t coefficient, real_t offset_pu, real_t time_constant_seconds = 0.0);

    /**
     * @brief Configures the voltage limits for the speed-voltage relationship.
     * @param min_voltage_pu Minimum voltage magnitude [pu].
     * @param max_voltage_pu Maximum voltage magnitude [pu].
     */
    void configure_voltage_limits_pu(real_t min_voltage_pu, real_t max_voltage_pu);

    /**
     * @brief Initializes the underlying Simulink model.
     */
    void initialize();

    /**
     * @brief Executes one step of the simulation.
     */
    void update();

    /**
     * @brief Updates the model with new firing pulses and voltage parameters.
     * @param grid_side_pulses The 6-pulse firing signals for the grid side.
     * @param machine_side_pulses The 6-pulse firing signals for the machine side.
     * @param load_torque The load torque in per-unit.
     * @param grid_frequency_hz The grid frequency in Hertz.
     * @param machine_frequency_hz The machine frequency in Hertz.
     * @param grid_voltage_rms The RMS line-to-line voltage on the grid side.
     * @param machine_voltage_rms The RMS line-to-line voltage on the machine side.
     */
    void update(const std::array<bool, 6>& grid_side_pulses,
        const std::array<bool, 6>& machine_side_pulses,
        torque_pu_t load_torque,
        frequency_t grid_frequency_hz,
        frequency_t machine_frequency_hz,
        voltage_t grid_voltage_rms,
        voltage_t machine_voltage_rms);

    /**
     * @brief Terminates the underlying Simulink model.
     */
    void terminate();

    /**
     * @brief Sets the firing pulses for the grid-side converter.
     * @param pulses An array of 6 pulse values (true for on, false for off).
     */
    void set_grid_side_pulses(const std::array<bool, 6>& pulses);

    /**
     * @brief Sets the firing pulses for the machine-side converter.
     * @param pulses An array of 6 pulse values (true for on, false for off).
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

    // ==================== Per-Unit Runtime Setters (Grid) ====================
    void set_grid_voltage_pu(voltage_pu_t v_pu) noexcept;
    void set_grid_frequency_pu(frequency_pu_t f_pu) noexcept;
    void set_grid_operating_point(voltage_pu_t v_pu, frequency_pu_t f_pu) noexcept;

    // ==================== Per-Unit Runtime Setters (Machine) ====================
    void set_machine_voltage_pu(voltage_pu_t v_pu) noexcept;
    void set_machine_frequency_pu(frequency_pu_t f_pu) noexcept;
    void set_machine_operating_point(voltage_pu_t v_pu, frequency_pu_t f_pu) noexcept;

    // ==================== Granular Frequency / Voltage Setters (New) ====================
    /**
     * @brief Sets only the grid frequency (voltage preserved / or override value if enabled).
     */
    void set_grid_frequency(frequency_t frequency_hz);

    /**
     * @brief Sets only the grid RMS line voltage (frequency preserved / or override value if enabled).
     */
    void set_grid_voltage(voltage_t voltage_rms);

    /**
     * @brief Sets only the machine electrical frequency (subject to override semantics).
     * If machine frequency override is enabled this value persists; otherwise it may be
     * overwritten by variable-speed mode updates.
     */
    void set_machine_frequency(frequency_t frequency_hz);

    /**
     * @brief Sets only the machine RMS line voltage (subject to override semantics).
     * If machine voltage override is enabled this value persists; otherwise it may be
     * overwritten by speed-dependent voltage control each update.
     */
    void set_machine_voltage(voltage_t voltage_rms);

    /**
     * @brief Reset dynamic sub-models (mechanical + voltage relationship) and clear overrides.
     * Equivalent to initialize() for internal state but keeps configured parameters.
     */
    void reset();

    /**
     * @brief Sets the machine's operational mode (fixed or variable speed).
     * @param mode The desired operation mode.
     */
    void set_operation_mode(plant_model_machine_operation_mode mode);

    /**
     * @brief Sets the voltage control mode for the machine side.
     * @param mode The desired voltage control mode.
     */
    void set_voltage_control_mode(plant_model_voltage_control_mode mode);

    /**
     * @brief Sets the excitation voltage setpoint for external control mode.
     *
     * This method allows direct control of the machine-side voltage magnitude from an external
     * excitation controller. The setpoint is only used when voltage_control_mode is set to
     * external_control. This simulates a "perfect exciter" with instantaneous tracking.
     *
     * @param excitation_voltage_setpoint The excitation voltage setpoint in per-unit [pu].
     *        This value is multiplied by the nominal machine voltage to produce the actual voltage.
     */
    void set_excitation_voltage_setpoint_pu(voltage_pu_t excitation_voltage_setpoint);

    /**
     * @brief Sets the initial mechanical speed for the load.
     * @param initial_speed_pu The initial mechanical speed in per-unit.
     */
    void set_initial_mechanical_speed_pu(speed_pu_t initial_speed_pu);

    /**
     * @brief Sets the load torque in per-unit for the mechanical load model.
     * @param load_torque The load torque in per-unit.
     */
    void set_load_torque_pu(torque_pu_t load_torque);

    // Output getters - Current model state
    frequency_t get_grid_frequency() const noexcept;
    frequency_t get_machine_frequency() const noexcept;
    voltage_t get_rms_grid_voltage() const noexcept;
    voltage_t get_rms_machine_voltage() const noexcept;
    abc<voltage_t> get_grid_side_voltages_abc() const noexcept;
    abc<current_t> get_grid_side_currents_abc() const noexcept;
    abc<voltage_t> get_machine_side_voltages_abc() const noexcept;
    abc<current_t> get_machine_side_currents_abc() const noexcept;
    angle_wrapped get_grid_voltage_source_angle() const noexcept;
    angle_wrapped get_machine_voltage_source_angle() const noexcept;
    current_t get_dc_link_current() const noexcept;

    // Mechanical dynamics getters
    speed_pu_t get_mechanical_speed_pu() const noexcept;
    torque_pu_t get_electromagnetic_torque_pu() const noexcept;
    torque_pu_t get_load_torque_pu() const noexcept;

    // Voltage control getters
    voltage_t get_machine_voltage() const noexcept;
    real_t get_voltage_magnitude_pu() const noexcept;

    // Parameter getters - Current configuration
    const sfc_plant_model_electrical_parameters& get_electrical_parameters() const noexcept;
    const sfc_plant_model_mechanical_parameters& get_mechanical_parameters() const noexcept;
    const sfc_plant_model_voltage_control_parameters& get_voltage_control_parameters() const noexcept;

    // Component access getters
    const control::mechanical_load& get_mechanical_load() const noexcept;
    control::mechanical_load& get_mechanical_load() noexcept;
    const control::speed_voltage_relationship& get_speed_voltage_relationship() const noexcept;
    control::speed_voltage_relationship& get_speed_voltage_relationship() noexcept;
    const sfc_per_unit_system& get_per_unit_system() const noexcept;

    // Operation mode getters
    plant_model_machine_operation_mode get_operation_mode() const noexcept;
    plant_model_voltage_control_mode get_voltage_control_mode() const noexcept;

    // ==================== Per-Unit Runtime Getters ====================
    voltage_pu_t get_grid_voltage_pu() const noexcept;
    voltage_pu_t get_machine_voltage_pu() const noexcept;
    frequency_pu_t get_grid_frequency_pu() const noexcept;
    frequency_pu_t get_machine_frequency_pu() const noexcept;

private:
    /**
     * @brief Creates default electrical parameters for 6p-6p topology.
     * @return Default electrical configuration parameters
     */
    static sfc_plant_model_electrical_parameters create_default_electrical_parameters() noexcept;

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

    /**
     * @brief Steps the mechanical load dynamics if in variable speed mode.
     */
    void step_mechanical_dynamics();

    /**
     * @brief Steps the speed-voltage relationship if voltage control is enabled.
     */
    void step_voltage_control();

    /**
     * @brief Updates the plant model parameters with computed frequency and voltage.
     */
    void update_plant_parameters();

    // ==================== Resolution Helpers ====================
    frequency_t resolve_machine_frequency() const noexcept;  // effective machine electrical frequency
    voltage_t resolve_machine_voltage() const noexcept;      // effective machine RMS volts
    frequency_t resolve_grid_frequency() const noexcept;     // effective grid electrical frequency
    voltage_t resolve_grid_voltage() const noexcept;         // effective grid RMS volts

    // ==================== Nominal Bases (immutable after construct unless reconfigured) ====================
    voltage_t _nominal_grid_voltage_rms{};
    voltage_t _nominal_machine_voltage_rms{};
    frequency_t _nominal_grid_frequency{};
    frequency_t _nominal_machine_frequency{};

    // ==================== Runtime Per-Unit Scalars ====================
    voltage_pu_t _grid_voltage_pu{voltage_pu_t{1.0f}};
    voltage_pu_t _machine_voltage_pu{voltage_pu_t{1.0f}};
    frequency_pu_t _grid_frequency_pu{frequency_pu_t{1.0f}};
    frequency_pu_t _machine_frequency_pu{frequency_pu_t{1.0f}};  // used only when not variable_speed

    voltage_pu_t _external_excitation_voltage_setpoint_pu{voltage_pu_t{1.0f}};  // Used in external_control mode

    torque_pu_t _load_torque_pu = torque_pu_t{0.0};  // Current load torque in per-unit

    sfc_plant_model_6p_6p _plant;                                     /// The underlying plant model instance
    control::mechanical_load _mechanical_load;                        /// The mechanical load model instance
    control::speed_voltage_relationship _speed_voltage_relationship;  /// The speed-voltage relationship model instance
    sfc_per_unit_system _pu_system;                                   /// The per-unit system instance

    // Configuration parameters
    sfc_plant_model_electrical_parameters _electrical_params;
    sfc_plant_model_mechanical_parameters _mechanical_params;
    sfc_plant_model_voltage_control_parameters _voltage_control_params;

    // Operation modes
    plant_model_machine_operation_mode _mode = plant_model_machine_operation_mode::fixed_speed;
    plant_model_voltage_control_mode _voltage_mode = plant_model_voltage_control_mode::fixed_voltage;
};

}  // namespace yeet::application::sfc