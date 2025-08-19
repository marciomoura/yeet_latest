#pragma once

namespace yeet_control {

/**
 * @brief Generic base class for state machine states
 * @tparam StateEnum Enumeration type defining state identifiers
 * @tparam InputType Type of input data structure
 * @tparam OutputType Type of output data structure
 */
template <typename StateEnum, typename InputType, typename OutputType>
class state_base {
public:
    /**
     * @brief Called when entering this state
     */
    virtual void on_entry() noexcept {}

    /**
     * @brief Called when exiting this state
     */
    virtual void on_exit() noexcept {}

    /**
     * @brief Main state update function
     * @param inputs Current input signals and commands
     * @return Pointer to next state (nullptr if no transition)
     */
    virtual state_base* update(const InputType& inputs) noexcept = 0;

    /**
     * @brief Get the current state outputs
     * @return State output signals
     */
    virtual OutputType get_outputs() const noexcept = 0;

    /**
     * @brief Get the state identifier
     * @return State ID for debugging/logging
     */
    virtual StateEnum get_id() const noexcept = 0;

    virtual ~state_base() = default;
};

/**
 * @brief Generic state machine implementation
 * @tparam StateEnum Enumeration type defining state identifiers
 * @tparam InputType Type of input data structure
 * @tparam OutputType Type of output data structure
 */
template <typename StateEnum, typename InputType, typename OutputType>
class state_machine {
public:
    using state_type = state_base<StateEnum, InputType, OutputType>;

    /**
     * @brief Constructor with initial state
     * @param initial_state Pointer to the initial state
     */
    explicit state_machine(state_type* initial_state) noexcept : _current_state(initial_state), _previous_state(nullptr)
    {
    }
    /**
     * @brief Main update function - executes current state logic
     * @param inputs Current input signals and commands
     */
    void update(const InputType& inputs) noexcept
    {
        if (_current_state == nullptr) return;

        // Execute current state logic and check for transition
        state_type* next_state = _current_state->update(inputs);

        // Handle state transition if needed
        if (next_state != nullptr && next_state != _current_state) {
            _current_state->on_exit();
            _previous_state = _current_state;
            _current_state = next_state;
            _current_state->on_entry();
        }

        // Update outputs from current state
        if (_current_state != nullptr) {
            _outputs = _current_state->get_outputs();
        }
    }

    /**
     * @brief Reset state machine to specified state
     * @param reset_state Pointer to the state to reset to
     */
    void reset(state_type* reset_state) noexcept
    {
        if (_current_state != nullptr) {
            _current_state->on_exit();
        }
        _current_state = reset_state;
        _previous_state = nullptr;
        if (_current_state != nullptr) {
            _current_state->on_entry();
            _outputs = _current_state->get_outputs();
        }
    }

    /**
     * @brief Get current state outputs
     * @return Current output signals
     */
    OutputType get_outputs() const noexcept { return _outputs; }

    /**
     * @brief Get current state identifier
     * @return Current state ID
     */
    StateEnum get_current_state() const noexcept { return _current_state ? _current_state->get_id() : StateEnum{}; }

    /**
     * @brief Check if a state transition occurred in last update
     * @return True if state changed in last update cycle
     */
    bool has_transitioned() const noexcept { return _previous_state != _current_state; }

    /**
     * @brief Get pointer to current state (for advanced usage)
     * @return Pointer to current state object
     */
    state_type* get_current_state_ptr() const noexcept { return _current_state; }

private:
    state_type* _current_state{nullptr};
    state_type* _previous_state{nullptr};
    OutputType _outputs{};
};

}  // namespace yeet_control