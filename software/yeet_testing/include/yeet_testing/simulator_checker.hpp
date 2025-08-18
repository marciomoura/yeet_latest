#ifndef YEET_TESTING_SIMULATOR_CHECKER_HPP
#define YEET_TESTING_SIMULATOR_CHECKER_HPP

#include <functional>
#include <memory>
#include <shared_mutex>
#include <string>

namespace yeet {

class user_checker_deactivation {
public:
    user_checker_deactivation() = default;

    /**
     * @brief Deactivate the checker
     *
     * This method allows the user to deactivate the checker, preventing it from running checks.
     */
    void deactivate() { _deactivated = true; }

    /**
     * @brief Check if the checker is deactivated
     * @return true if the checker is deactivated, false otherwise
     */
    bool is_deactivated() const { return _deactivated; }

private:
    bool _deactivated{false};
};

/**
 * @brief Base class for all simulation checkers
 *
 * Provides core functionality for time-based condition checking during simulation.
 * Checkers can be active during specific time windows and verify conditions.
 *
 * Example:
 * @code
 * // Create a checker active from t=0 to t=1
 * auto checker = make_range_checker<double>(
 *     "voltage_check",
 *     []() { return voltage; },
 *     -10.0,
 *     10.0,
 *     checker_activation_time_range{0.0, 1.0}
 * );
 * @endcode
 */
class simulator_checker {
public:
    explicit simulator_checker(std::string name, double start_time = 0.0, double end_time = -1.0)
        : _name(std::move(name)), _start_time(start_time), _end_time(end_time)
    {
    }

    virtual ~simulator_checker() = default;

    virtual bool check() = 0;
    virtual std::string get_failure_message() const = 0;

    bool is_active(double current_time) const
    {
        // Check if the checker is deactivated by the user
        if (_user_deactivation->is_deactivated()) {
            return false;
        }

        // If both start and end times are negative, the checker is always active
        if (_start_time < 0.0 && _end_time < 0.0) {
            return true;
        }

        return current_time >= _start_time && (_end_time < 0.0 || current_time < _end_time);
    }

    const std::string& get_name() const { return _name; }

    std::shared_ptr<user_checker_deactivation> get_user_deactivation_control() { return _user_deactivation; }

protected:
    std::string _name;
    double _start_time;
    double _end_time;
    std::shared_ptr<user_checker_deactivation> _user_deactivation{std::make_shared<user_checker_deactivation>()};
};

/**
 * @brief Convenient checker implementation using lambda functions
 *
 * Allows creating checkers with custom check logic and failure messages using lambdas.
 *
 * Example:
 * @code
 * auto lambda_check = lambda_checker(
 *     "custom_check",
 *     []() { return voltage < 10.0 && current < 5.0; },
 *     []() { return "Voltage or current exceeded limits"; }
 * );
 * @endcode
 */
class lambda_checker : public simulator_checker {
public:
    lambda_checker(std::string name,
        std::function<bool()> check_fn,
        std::function<std::string()> failure_message_fn = nullptr,
        double start_time = 0.0,
        double end_time = -1.0)
        : simulator_checker(std::move(name), start_time, end_time),
          _check_fn(std::move(check_fn)),
          _failure_message_fn(std::move(failure_message_fn))
    {
    }

    bool check() override { return _check_fn(); }

    std::string get_failure_message() const override
    {
        if (_failure_message_fn) {
            return _failure_message_fn();
        }
        return "Check '" + _name + "' failed";
    }

private:
    std::function<bool()> _check_fn;
    std::function<std::string()> _failure_message_fn;
};

/**
 * @brief Interface for type-specific checkers
 *
 * Base interface for implementing checkers that work with specific value types.
 * All concrete checker types must implement check() and get_failure_message().
 */
template <typename T>
class checker_type_interface {
public:
    virtual ~checker_type_interface() = default;

    virtual bool check(T value) = 0;
    virtual std::string get_failure_message(std::string name) const = 0;
};

/**
 * @brief Checks if values are below a maximum threshold
 *
 * Example:
 * @code
 * auto max_check = make_max_checker<double>(
 *     "max_voltage",
 *     []() { return voltage; },
 *     12.0,
 *     checker_activation_time_range{0.0, 1.0}
 * );
 * @endcode
 */
template <typename T>
class checker_type_max : public checker_type_interface<T> {
public:
    using value_type = T;

    explicit checker_type_max(T max) : _max(max) {}

    bool check(T value) override
    {
        _last_value = value;
        return value <= _max;
    }

    std::string get_failure_message(std::string name) const override
    {
        return "Value " + std::to_string(_last_value) + " for '" + name + "' exceeds maximum " + std::to_string(_max);
    }

private:
    T _max{};
    T _last_value{};
};

/**
 * @brief Checks if values are below a minimum threshold
 *
 * Example:
 * @code
 * auto max_check = make_min_checker<double>(
 *     "min_voltage",
 *     []() { return voltage; },
 *     12.0,
 *     checker_activation_time_range{0.0, 1.0}
 * );
 * @endcode
 */
template <typename T>
class checker_type_min : public checker_type_interface<T> {
public:
    using value_type = T;

    explicit checker_type_min(T min) : _min(min) {}

    bool check(T value) override
    {
        _last_value = value;
        return value >= _min;
    }

    std::string get_failure_message(std::string name) const override
    {
        return "Value " + std::to_string(_last_value) + " for '" + name + "' is below minimum " + std::to_string(_min);
    }

private:
    T _min{};
    T _last_value{};
};

/**
 * @brief Checks if values are within a specified range
 *
 * Example:
 * @code
 * auto range_check = make_range_checker<double>(
 *     "voltage_range",
 *     []() { return voltage; },
 *     -12.0,
 *     12.0,
 *     checker_activation_exact_time{0.5, 0.001}
 * );
 * @endcode
 */
template <typename T>
class checker_type_range : public checker_type_interface<T> {
public:
    using value_type = T;

    explicit checker_type_range(T min, T max) : _min(min), _max(max) {}

    bool check(T value) override
    {
        _last_value = value;
        return value >= _min && value <= _max;
    }

    std::string get_failure_message(std::string name) const override
    {
        return "Value " + std::to_string(_last_value) + " for '" + name + "' is outside range [" +
               std::to_string(_min) + ", " + std::to_string(_max) + "]";
    }

private:
    T _min{};
    T _max{};
    T _last_value{};
};

/**
 * @brief Checks if values are equal to a reference value
 *
 * Example:
 * @code
 * auto equal_check = make_equal_checker<int>(
 *     "state_check",
 *     []() { return state; },
 *     READY_STATE,
 *     checker_activation_exact_time{1.0, 0.001}
 * );
 * @endcode
 */
template <typename T>
class checker_type_equal : public checker_type_interface<T> {
public:
    using value_type = T;

    explicit checker_type_equal(T value) : _value(value) {}

    bool check(T value) override
    {
        _last_value = value;
        return value == _value;
    }

    std::string get_failure_message(std::string name) const override
    {
        return "Value " + std::to_string(_last_value) + " for '" + name + "' is not equal to " + std::to_string(_value);
    }

private:
    T _value{};
    T _last_value{};
};

/**
 * @brief Checks if values are within a tolerance of a reference value
 *
 * Useful for floating-point comparisons where exact equality is impractical.
 *
 * Example:
 * @code
 * checker_type_near<double> near_check(1.0, 0.001);
 * bool result = near_check.check(1.0001);  // true if within tolerance
 * @endcode
 */
template <typename T>
class checker_type_near : public checker_type_interface<T> {
public:
    using value_type = T;

    explicit checker_type_near(T value, T tolerance) : _value(value), _tolerance(tolerance) {}

    bool check(T value) override
    {
        _last_value = value;
        return std::abs(value - _value) <= _tolerance;
    }

    std::string get_failure_message(std::string name) const override
    {
        return "Value " + std::to_string(_last_value) + " for '" + name + "' is not near " + std::to_string(_value) +
               " within tolerance " + std::to_string(_tolerance);
    }

private:
    T _value{};
    T _tolerance{};
    T _last_value{};
};

template <typename T>
using checker_type_floating_near = checker_type_near<T>;

/**
 * @brief Activation configuration for time range checking
 *
 * Specifies a time window during which a checker should be active.
 * If time range is not specified, the checker is active indefinitely from the start time.
 *
 * Example:
 * @code
 * checker_activation_time_range range{0.0, 1.0};  // Active from t=0 to t=1
 * @endcode
 */
struct checker_activation_time_range {
    explicit checker_activation_time_range(double start_time, double end_time)
        : start_time(start_time), end_time(end_time)
    {
    }

    explicit checker_activation_time_range(double start_time) : start_time(start_time), end_time(-1.0f) {}

    double start_time;
    double end_time;
};

/**
 * @brief Activation configuration for point-in-time checking
 *
 * Specifies a single time point (with sampling window) when a checker should be active.
 *
 * Example:
 * @code
 * checker_activation_exact_time point{0.5, 0.001};  // Active at t=0.5 ±0.0005
 * @endcode
 */
struct checker_activation_exact_time {
    explicit checker_activation_exact_time(double time, double sampling_time) : time(time), sampling_time(sampling_time)
    {
    }

    double time;
    double sampling_time;
};

/**
 * @brief Activation configuration for always-active checking
 *
 * Specifies that a checker should always be active (unless manually deactivated).
 * The checker will run indefinitely and can only be controlled via user_checker_deactivation.
 *
 * Example:
 * @code
 * checker_activation_always always{};  // Always active
 * @endcode
 */
struct checker_activation_always {
    checker_activation_always() = default;
};

/**
 * @brief Template class combining checker types with activation timing
 *
 * Provides a unified interface for all checker types with time-based activation.
 *
 * Example:
 * @code
 * // Check voltage is within ±10V from t=0 to t=1
 * checker_type_range<double> range_checker(-10.0, 10.0);
 * auto templated_checker = checker_type_templated(
 *     "voltage_check",
 *     []() { return voltage; },
 *     range_checker,
 *     checker_activation_time_range{0.0, 1.0}
 * );
 * @endcode
 */
template <typename CheckerType>
class checker_type_templated : public simulator_checker {
public:
    using value_type = typename CheckerType::value_type;

    // Constructor for time range activation
    checker_type_templated(std::string name,
        std::function<value_type()> value_fn,
        CheckerType checker,
        checker_activation_time_range activation)
        : simulator_checker(std::move(name), activation.start_time, activation.end_time),
          _value_fn(std::move(value_fn)),
          _checker(std::move(checker))
    {
    }

    // Constructor for exact time activation
    checker_type_templated(std::string name,
        std::function<value_type()> value_fn,
        CheckerType checker,
        checker_activation_exact_time activation)
        : simulator_checker(std::move(name),
              activation.time - activation.sampling_time / 2.0,
              activation.time + activation.sampling_time / 2.0),
          _value_fn(std::move(value_fn)),
          _checker(std::move(checker))
    {
    }

    // Constructor for always active
    checker_type_templated(std::string name,
        std::function<value_type()> value_fn,
        CheckerType checker,
        checker_activation_always activation)
        : simulator_checker(std::move(name), -1.0, -1.0),  // Both negative means always active
          _value_fn(std::move(value_fn)),
          _checker(std::move(checker))
    {
    }

    // Default time range constructor
    checker_type_templated(std::string name,
        std::function<value_type()> value_fn,
        CheckerType checker,
        double start_time = 0.0,
        double end_time = -1.0)
        : simulator_checker(std::move(name), start_time, end_time),
          _value_fn(std::move(value_fn)),
          _checker(std::move(checker))
    {
    }

    bool check() override
    {
        value_type const value = _value_fn();
        return _checker.check(value);
    }

    std::string get_failure_message() const override { return _checker.get_failure_message(_name); }

private:
    std::function<value_type()> _value_fn;
    CheckerType _checker;
};

// Helper functions for creating checkers with different types - Time Range version
template <typename T>
auto make_range_checker(
    std::string name, std::function<T()> value_fn, T min_value, T max_value, checker_activation_time_range activation)
{
    checker_type_range<T> checker{min_value, max_value};
    return checker_type_templated<checker_type_range<T>>(
        std::move(name), std::move(value_fn), std::move(checker), activation);
}

// Point in time version
template <typename T>
auto make_range_checker(
    std::string name, std::function<T()> value_fn, T min_value, T max_value, checker_activation_exact_time activation)
{
    checker_type_range<T> checker{min_value, max_value};
    return checker_type_templated<checker_type_range<T>>(
        std::move(name), std::move(value_fn), std::move(checker), activation);
}

// Equal checker overloads
template <typename T>
auto make_equal_checker(
    std::string name, std::function<T()> value_fn, T expected_value, checker_activation_time_range activation)
{
    checker_type_equal<T> checker{expected_value};
    return checker_type_templated<checker_type_equal<T>>(
        std::move(name), std::move(value_fn), std::move(checker), activation);
}

template <typename T>
auto make_equal_checker(
    std::string name, std::function<T()> value_fn, T expected_value, checker_activation_exact_time activation)
{
    checker_type_equal<T> checker{expected_value};
    return checker_type_templated<checker_type_equal<T>>(
        std::move(name), std::move(value_fn), std::move(checker), activation);
}

// Min checker overloads
template <typename T>
auto make_min_checker(
    std::string name, std::function<T()> value_fn, T min_value, checker_activation_time_range activation)
{
    checker_type_min<T> checker{min_value};
    return checker_type_templated<checker_type_min<T>>(
        std::move(name), std::move(value_fn), std::move(checker), activation);
}

template <typename T>
auto make_min_checker(
    std::string name, std::function<T()> value_fn, T min_value, checker_activation_exact_time activation)
{
    checker_type_min<T> checker{min_value};
    return checker_type_templated<checker_type_min<T>>(
        std::move(name), std::move(value_fn), std::move(checker), activation);
}

// Max checker overloads
template <typename T>
auto make_max_checker(
    std::string name, std::function<T()> value_fn, T max_value, checker_activation_time_range activation)
{
    checker_type_max<T> checker{max_value};
    return checker_type_templated<checker_type_max<T>>(
        std::move(name), std::move(value_fn), std::move(checker), activation);
}

template <typename T>
auto make_max_checker(
    std::string name, std::function<T()> value_fn, T max_value, checker_activation_exact_time activation)
{
    checker_type_max<T> checker{max_value};
    return checker_type_templated<checker_type_max<T>>(
        std::move(name), std::move(value_fn), std::move(checker), activation);
}

// Helper functions for creating checkers with different types - Always active version
template <typename T>
auto make_range_checker(
    std::string name, std::function<T()> value_fn, T min_value, T max_value, checker_activation_always activation)
{
    checker_type_range<T> checker{min_value, max_value};
    return checker_type_templated<checker_type_range<T>>(
        std::move(name), std::move(value_fn), std::move(checker), activation);
}

template <typename T>
auto make_equal_checker(
    std::string name, std::function<T()> value_fn, T expected_value, checker_activation_always activation)
{
    checker_type_equal<T> checker{expected_value};
    return checker_type_templated<checker_type_equal<T>>(
        std::move(name), std::move(value_fn), std::move(checker), activation);
}

template <typename T>
auto make_min_checker(std::string name, std::function<T()> value_fn, T min_value, checker_activation_always activation)
{
    checker_type_min<T> checker{min_value};
    return checker_type_templated<checker_type_min<T>>(
        std::move(name), std::move(value_fn), std::move(checker), activation);
}

template <typename T>
auto make_max_checker(std::string name, std::function<T()> value_fn, T max_value, checker_activation_always activation)
{
    checker_type_max<T> checker{max_value};
    return checker_type_templated<checker_type_max<T>>(
        std::move(name), std::move(value_fn), std::move(checker), activation);
}

// Near checker overloads
template <typename T>
auto make_near_checker(std::string name,
    std::function<T()> value_fn,
    T expected_value,
    T tolerance,
    checker_activation_time_range activation)
{
    checker_type_near<T> checker{expected_value, tolerance};
    return checker_type_templated<checker_type_near<T>>(
        std::move(name), std::move(value_fn), std::move(checker), activation);
}

template <typename T>
auto make_near_checker(std::string name,
    std::function<T()> value_fn,
    T expected_value,
    T tolerance,
    checker_activation_exact_time activation)
{
    checker_type_near<T> checker{expected_value, tolerance};
    return checker_type_templated<checker_type_near<T>>(
        std::move(name), std::move(value_fn), std::move(checker), activation);
}

template <typename T>
auto make_near_checker(
    std::string name, std::function<T()> value_fn, T expected_value, T tolerance, checker_activation_always activation)
{
    checker_type_near<T> checker{expected_value, tolerance};
    return checker_type_templated<checker_type_near<T>>(
        std::move(name), std::move(value_fn), std::move(checker), activation);
}

template <typename CheckerType>
std::unique_ptr<simulator_checker> make_unique_checker(CheckerType checker)
{
    return std::make_unique<CheckerType>(checker);
}

}  // namespace yeet

#endif  // YEET_TESTING_SIMULATOR_CHECKER_HPP
