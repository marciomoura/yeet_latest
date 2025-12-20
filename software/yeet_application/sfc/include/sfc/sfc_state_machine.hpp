#pragma once

#include <cassert>
#include <functional>
#include <string_view>

#include "yeet_control/state_machine.hpp"
#include "yeet_control/units.hpp"

namespace yeet::application::sfc {

/**
 * @brief State identifiers for the sfc state machine
 */
enum class sfc_state_id {
    IDLE = 0,
    AUXILIARIES_ON = 1,
    BREAKERS_OFF = 2,
    BREAKERS_ON = 3,
    OPERATION_OFF = 4,
    OPERATION_ON = 5,
    SHUTDOWN_REQUESTED = 6
};

class sfc_state_event {
public:
    sfc_state_event(std::string_view name, std::function<bool()> condition) noexcept
        : _name(name), _condition(condition)
    {
    }

    bool is_true() const noexcept
    {
        assert(_condition != nullptr && "Condition function must be set");
        return _condition();
    }

    std::string_view get_name() const noexcept { return _name; }

private:
    std::string_view _name;
    std::function<bool()> _condition;
};

// return a "mock event" for testing purposes
inline sfc_state_event mock_event(std::string_view name) noexcept
{
    return sfc_state_event(name, []() { return true; });
}

inline sfc_state_event mock_event(std::string_view name, bool& status) noexcept
{
    return sfc_state_event(name, [&]() { return status; });
}

/**
 * @brief Input signals and commands for the sfc state machine
 */
struct sfc_state_inputs {
    // Commands from operator/control system
    sfc_state_event turn_on_command{mock_event("turn_on_command")};  ///< Command to energize system (IDLE->AUX_ON)
    sfc_state_event start_command{mock_event("start_command")};      ///< Command to start operation (OP_OFF->OP_ON)
    sfc_state_event stop_command{mock_event("stop_command")};        ///< Command to stop operation
    sfc_state_event shutdown_request{mock_event("shutdown_request")};

    // Feedback signals from hardware
    sfc_state_event auxiliaries_are_on_event{mock_event("auxiliaries_are_on")};  ///< Feedback: auxiliaries energized
    sfc_state_event mcb_is_closed_event{mock_event("mcb_is_closed")};            ///< Feedback: MV breaker closed
    sfc_state_event is_current_extinguished{mock_event("is_current_extinguished")};

    // Protection and safety inputs (consolidated)
    sfc_state_event any_fault_active_event{
        mock_event("any_fault_active")};  // Emergency off, breaker tripping, or any active fault
    sfc_state_event is_inhibited_from_starting_event{
        mock_event("is_inhibited")};  // Door open, protection inhibit, prevents startup
    sfc_state_event protection_trip_active_event{
        mock_event("protection_trip")};  // Combined protection trip status (inhibit OR fault)
};

/**
 * @brief Output signals from the sfc state machine
 */
struct sfc_state_outputs {
    // Control coordination signals
    bool pulse_releases{false};                 ///< Enable thyristor pulse generation
    bool freeze_integrator{false};              ///< Freeze controller integrators
    bool request_extinguish_dc_current{false};  ///< Request DC current extinguish mode
    bool reset_controllers{false};              ///< Command to reset all controllers to initial state

    // Physical output commands (with safety interlocks applied)
    bool auxiliaries_on_command{false};  ///< Command to energize auxiliary systems
    bool mv_breaker_close{false};        ///< MV breaker close command (with safety interlocks)
    bool mv_breaker_open{false};         ///< MV breaker open command

    // Protection coordination
    bool release_grid_protections{false};     ///< Release grid-side protection system (enable operation)
    bool release_machine_protections{false};  ///< Release machine-side protection system (via fiber optics)
};

// Type aliases for convenience
using sfc_state_machine_type = yeet_control::state_machine<sfc_state_id, sfc_state_inputs, sfc_state_outputs>;
using sfc_state_type = sfc_state_machine_type::state_type;

// Forward declarations
class idle_state;
class auxiliaries_on_state;
class breakers_off_state;
class breakers_on_state;
class operation_off_state;
class operation_on_state;
class shutdown_requested_state;

/**
 * @brief IDLE state - system de-energized, initial state
 */
class idle_state : public sfc_state_type {
public:
    explicit idle_state(auxiliaries_on_state* auxiliaries_on) noexcept : _auxiliaries_on_state(auxiliaries_on) {}

    sfc_state_type* update(const sfc_state_inputs& inputs) noexcept override;
    sfc_state_outputs get_outputs() const noexcept override;
    sfc_state_id get_id() const noexcept override { return sfc_state_id::IDLE; }

    /// Set trip latch - when true, requires turn_on to be released before restart
    void set_trip_latch(bool latch) noexcept { _trip_latch = latch; }
    /// Check if trip latch is active
    bool is_trip_latched() const noexcept { return _trip_latch; }

private:
    auxiliaries_on_state* _auxiliaries_on_state{nullptr};
    bool _trip_latch{false};  ///< When true, requires turn_on toggle before restart
};

/**
 * @brief AUXILIARIES_ON state - auxiliary systems activated
 */
class auxiliaries_on_state : public sfc_state_type {
public:
    auxiliaries_on_state(breakers_off_state* breakers_off, idle_state* idle) noexcept
        : _breakers_off_state(breakers_off), _idle_state(idle)
    {
    }

    sfc_state_type* update(const sfc_state_inputs& inputs) noexcept override;
    sfc_state_outputs get_outputs() const noexcept override;
    sfc_state_id get_id() const noexcept override { return sfc_state_id::AUXILIARIES_ON; }

private:
    breakers_off_state* _breakers_off_state{nullptr};
    idle_state* _idle_state{nullptr};
};

/**
 * @brief BREAKERS_OFF state - ready for breaker closure
 */
class breakers_off_state : public sfc_state_type {
public:
    explicit breakers_off_state(
        idle_state* idle, breakers_on_state* breakers_on, auxiliaries_on_state* auxiliaries_on) noexcept
        : _idle_state(idle), _breakers_on_state(breakers_on), _auxiliaries_on_state(auxiliaries_on)
    {
    }

    sfc_state_type* update(const sfc_state_inputs& inputs) noexcept override;
    sfc_state_outputs get_outputs() const noexcept override;
    sfc_state_id get_id() const noexcept override { return sfc_state_id::BREAKERS_OFF; }

private:
    idle_state* _idle_state{nullptr};
    breakers_on_state* _breakers_on_state{nullptr};
    auxiliaries_on_state* _auxiliaries_on_state{nullptr};
};

/**
 * @brief BREAKERS_ON state - main circuit energized, standby mode
 */
class breakers_on_state : public sfc_state_type {
public:
    explicit breakers_on_state(operation_off_state* operation_off, breakers_off_state* breakers_off_state) noexcept
        : _operation_off_state(operation_off), _breakers_off_state(breakers_off_state)
    {
    }

    sfc_state_type* update(const sfc_state_inputs& inputs) noexcept override;
    sfc_state_outputs get_outputs() const noexcept override;
    sfc_state_id get_id() const noexcept override { return sfc_state_id::BREAKERS_ON; }

private:
    operation_off_state* _operation_off_state{nullptr};
    breakers_off_state* _breakers_off_state{nullptr};
};

/**
 * @brief OPERATION_OFF state - final standby before operation
 */
class operation_off_state : public sfc_state_type {
public:
    explicit operation_off_state(operation_on_state* operation_on, breakers_on_state* breakers_on_state) noexcept
        : _operation_on_state(operation_on), _breakers_on_state(breakers_on_state)
    {
    }

    sfc_state_type* update(const sfc_state_inputs& inputs) noexcept override;
    sfc_state_outputs get_outputs() const noexcept override;
    sfc_state_id get_id() const noexcept override { return sfc_state_id::OPERATION_OFF; }

private:
    operation_on_state* _operation_on_state{nullptr};
    breakers_on_state* _breakers_on_state{nullptr};
};

/**
 * @brief OPERATION_ON state - active converter operation
 */
class operation_on_state : public sfc_state_type {
public:
    explicit operation_on_state(shutdown_requested_state* shutdown_requested) noexcept
        : _shutdown_requested_state(shutdown_requested)
    {
    }

    sfc_state_type* update(const sfc_state_inputs& inputs) noexcept override;
    sfc_state_outputs get_outputs() const noexcept override;
    sfc_state_id get_id() const noexcept override { return sfc_state_id::OPERATION_ON; }

private:
    shutdown_requested_state* _shutdown_requested_state{nullptr};
    bool _is_first_cycle{true};  ///< Flag to detect state entry
};

/**
 * @brief SHUTDOWN_REQUESTED state - controlled shutdown sequence
 */
class shutdown_requested_state : public sfc_state_type {
public:
    shutdown_requested_state(operation_off_state* operation_off, idle_state* idle) noexcept
        : _operation_off_state(operation_off), _idle_state(idle)
    {
    }

    sfc_state_type* update(const sfc_state_inputs& inputs) noexcept override;
    sfc_state_outputs get_outputs() const noexcept override;
    sfc_state_id get_id() const noexcept override { return sfc_state_id::SHUTDOWN_REQUESTED; }
    void reset() noexcept { _acknowledgment_sent = false; }

private:
    operation_off_state* _operation_off_state{nullptr};
    idle_state* _idle_state{nullptr};  ///< For direct trip path to IDLE
    bool _acknowledgment_sent{false};  ///< Flag to ensure trip acknowledgment is held for one cycle
};

/**
 * @brief sfc operational state machine
 *
 * Manages the complete lifecycle of an sfc converter from power-up
 * through operation to controlled shutdown, ensuring safe transitions
 * and proper sequencing of auxiliary systems and circuit breakers.
 */
class sfc_state_machine {
public:
    /**
     * @brief Constructor - initializes state machine in IDLE state
     */
    sfc_state_machine() noexcept;

    /**
     * @brief Main update function - executes current state logic
     * @param inputs Current input signals and commands
     */
    void update(const sfc_state_inputs& inputs) noexcept;

    /**
     * @brief Reset state machine to initial IDLE state
     */
    void reset() noexcept;

    /**
     * @brief Get current state outputs
     * @return Current output signals
     */
    sfc_state_outputs get_outputs() const noexcept;

    /**
     * @brief Get current state identifier
     * @return Current state ID
     */
    sfc_state_id get_current_state() const noexcept;

    /**
     * @brief Check if a state transition occurred in last update
     * @return True if state changed in last update cycle
     */
    bool has_transitioned() const noexcept;

    /**
     * @brief Apply safety interlocks to state machine outputs
     *
     * Applies hardware safety interlocks to outputs before they are used:
     * - MV breaker close is blocked if door is open, emergency off, or protection trip
     * - Auxiliaries are turned off on any trip or safety shutdown condition
     *
     * @param outputs State machine outputs (modified in place)
     * @param inputs Current input signals for safety checks
     */
    static void apply_safety_interlocks(sfc_state_outputs& outputs, const sfc_state_inputs& inputs) noexcept;

private:
    // Initialize state transition graph
    void initialize_transitions() noexcept;

    idle_state _idle_state;
    auxiliaries_on_state _auxiliaries_on_state;
    breakers_off_state _breakers_off_state;
    breakers_on_state _breakers_on_state;
    operation_off_state _operation_off_state;
    operation_on_state _operation_on_state;
    shutdown_requested_state _shutdown_requested_state;

    sfc_state_machine_type _state_machine;
};

}  // namespace yeet::application::sfc
