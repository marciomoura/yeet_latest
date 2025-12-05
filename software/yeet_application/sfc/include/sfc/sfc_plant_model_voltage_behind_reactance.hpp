#pragma once

#include <memory>

#include "sfc_per_unit_system.hpp"
#include "sfc_plant_back_to_back_topology_interface.hpp"
#include "sfc_plant_model_parameters.hpp"
#include "yeet_control/angle_wrapped.hpp"
#include "yeet_control/field_current_regulator.hpp"
#include "yeet_control/first_order_low_pass_filter.hpp"
#include "yeet_control/mechanical_load.hpp"
#include "yeet_control/reference_frame.hpp"
#include "yeet_control/units.hpp"

namespace yeet::application::sfc {

/**
 * @brief Enumeration to select the back-to-back topology implementation.
 */
enum class plant_topology_type {
    simulink,  ///< Use Simulink-generated model wrapper
    internal,  ///< Use internal C++ thyristor model
    rectifier  ///< Use rectifier-only topology (machine-side short-circuited)
};

/**
 * @brief Augmented 6-pulse to 6-pulse SFC plant model with mechanical dynamics and voltage control
 *
 * This class wraps a back-to-back topology model (selectable via interface) and adds mechanical load
 * dynamics and speed-voltage relationship control. It provides configurable parameters without
 * exposing topology-specific default values in the public interface.
 */
class sfc_plant_model_voltage_behind_reactance {
public:
    static constexpr double kSamplingTime = 10e-6;  // 10 µs sampling time (100 kHz)

    /**
     * @brief Constructs the augmented plant model with configurable parameters.
     * @param electrical_params Electrical configuration parameters
     * @param mechanical_params Mechanical configuration parameters
     * @param voltage_control_params Voltage control configuration parameters
     * @param topology The topology implementation to use (default: simulink)
     */
    sfc_plant_model_voltage_behind_reactance(const sfc_plant_model_electrical_parameters& electrical_params,
        const sfc_plant_model_mechanical_parameters& mechanical_params,
        const sfc_plant_model_voltage_control_parameters& voltage_control_params,
        plant_topology_type topology = plant_topology_type::simulink);

    /**
     * @brief Constructs the augmented plant model with default 6p-6p topology parameters.
     * @param topology The topology implementation to use (default: simulink)
     */
    explicit sfc_plant_model_voltage_behind_reactance(plant_topology_type topology = plant_topology_type::simulink);

    /**
     * @brief Destructor.
     */
    ~sfc_plant_model_voltage_behind_reactance();

    // Prevent copying and moving
    sfc_plant_model_voltage_behind_reactance(const sfc_plant_model_voltage_behind_reactance&) = delete;
    sfc_plant_model_voltage_behind_reactance& operator=(const sfc_plant_model_voltage_behind_reactance&) = delete;
    sfc_plant_model_voltage_behind_reactance(sfc_plant_model_voltage_behind_reactance&&) = delete;
    sfc_plant_model_voltage_behind_reactance& operator=(sfc_plant_model_voltage_behind_reactance&&) = delete;

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
     *
     * This method directly configures the mechanical load's inertia, damping, and
     * speed-dependent load coefficients. For gas turbine applications, typical values:
     * - Inertia: 2.0-5.0 s (large rotational mass)
     * - Damping: 0.01-0.05 pu (low friction)
     * - Linear load: 0.0-0.1 pu/pu (minimal viscous losses)
     * - Quadratic load: 0.2-0.4 pu/pu^2 (compressor aerodynamic drag)
     *
     * @param inertia_constant_seconds The inertia constant in seconds.
     * @param damping_coefficient The damping factor in per-unit.
     * @param linear_load_coeff Linear load coefficient k1 [pu/pu]. Defaults to 0.0.
     * @param quadratic_load_coeff Quadratic load coefficient k2 [pu/pu^2]. Defaults to 0.0.
     */
    void configure_mechanical_load(inertia_constant_t inertia_constant_seconds,
        damping_coefficient_pu_t damping_coefficient,
        real_t linear_load_coeff = 0.0,
        real_t quadratic_load_coeff = 0.0);

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
     * @brief Sets the field current reference for external control mode.
     *
     * This method allows direct control of the machine-side field current from an external
     * excitation controller. The field current is regulated through a PI controller with
     * realistic field winding dynamics (L/R time constant), then converted to terminal voltage
     * using the proper synchronous machine relationship: E = k × Φ × ω.
     *
     * The field current regulator accounts for:
     * - Field winding resistance and inductance (typical τ_f = 0.5-5 seconds)
     * - PI control with anti-windup
     * - Exciter voltage limits
     * - Flux-speed relationship for back-EMF generation
     *
     * @param field_current_reference The field current reference in per-unit [pu].
     */
    void set_field_current_reference_pu(current_pu_t field_current_reference);

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

    // Field current and excitation getters
    current_pu_t get_field_current() const noexcept;
    voltage_pu_t get_exciter_voltage() const noexcept;
    real_t get_flux_pu() const noexcept;

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
    const sfc_per_unit_system& get_per_unit_system() const noexcept;

    // Operation mode getters
    plant_model_machine_operation_mode get_operation_mode() const noexcept;
    plant_model_voltage_control_mode get_voltage_control_mode() const noexcept;
    plant_topology_type get_topology_type() const noexcept;

    // ==================== Per-Unit Runtime Getters ====================
    voltage_pu_t get_grid_voltage_pu() const noexcept;
    voltage_pu_t get_machine_voltage_pu() const noexcept;
    frequency_pu_t get_grid_frequency_pu() const noexcept;
    frequency_pu_t get_machine_frequency_pu() const noexcept;

    // ==================== Circuit Breaker Control ====================
    /**
     * @brief Sets the state of the grid-side main circuit breaker.
     * @param closed True to close the breaker (normal operation), false to open (disconnect from grid).
     *
     * When open, the grid voltage seen by the rectifier becomes zero, causing DC link
     * current to decay through the system impedances. This models the physical behavior
     * of disconnecting the converter from the grid supply.
     */
    void set_grid_circuit_breaker_closed(bool closed);

    /**
     * @brief Gets the current state of the grid-side main circuit breaker.
     * @return True if the breaker is closed, false if open.
     */
    bool is_grid_circuit_breaker_closed() const noexcept;

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

    current_pu_t _field_current_reference_pu{current_pu_t{1.0f}};  // Field current reference for external control

    torque_pu_t _load_torque_pu = torque_pu_t{0.0};  // Current load torque in per-unit

    voltage_pu_t _machine_back_emf_pu{voltage_pu_t{0.0}};  // Calculated back-EMF: E = Φ × ω (variable_speed mode)

    std::unique_ptr<sfc_plant_back_to_back_topology_interface>
        _plant;                                                 ///< The underlying plant model instance (polymorphic)
    plant_topology_type _topology_type;                         ///< The selected topology type
    control::mechanical_load _mechanical_load;                  ///< The mechanical load model instance
    control::field_current_regulator _field_current_regulator;  /// The field current regulator instance
    control::first_order_low_pass_filter<speed_pu_t> _speed_filter;  /// Speed filter to smooth back-EMF oscillations
    sfc_per_unit_system _pu_system;                                  /// The per-unit system instance

    // Configuration parameters
    sfc_plant_model_electrical_parameters _electrical_params;
    sfc_plant_model_mechanical_parameters _mechanical_params;
    sfc_plant_model_voltage_control_parameters _voltage_control_params;

    // Operation modes
    plant_model_machine_operation_mode _mode = plant_model_machine_operation_mode::fixed_speed;
    plant_model_voltage_control_mode _voltage_mode = plant_model_voltage_control_mode::fixed_voltage;
};

}  // namespace yeet::application::sfc
