#pragma once

#include <algorithm>
#include <cassert>
#include <optional>
#include <random>  // white noise
#include <vector>

#include "angle.hpp"
#include "constants.hpp"
#include "reference_frame.hpp"
#include "yeet_control/angle.hpp"

namespace yeet::control {

template <typename T>
class white_noise_generator {
public:
    explicit white_noise_generator(double stddev = 0.0, double mean = 0.0)
        : m_rng(std::random_device{}()), m_mean(mean), m_stddev(stddev) {
        update_distribution();
    }

    void set_parameters(double stddev, double mean = 0.0) {
        m_mean = mean;
        m_stddev = stddev;
        update_distribution();
    }

    // Specialization for scalar types
    template <typename U = T>
    std::enable_if_t<std::is_arithmetic_v<U>, T> generate() {
        if (m_stddev <= 0.0) {
            return static_cast<T>(m_mean);  // Return mean when no noise (stddev = 0)
        }
        return static_cast<T>(m_distribution(m_rng));
    }

    // Specialization for abc<T>
    template <typename U = T>
    std::enable_if_t<std::is_same_v<U, abc<double>>, T> generate() {
        if (m_stddev <= 0.0) {
            return yeet::abc<double>{m_mean, m_mean, m_mean};  // Return mean when no noise
        }
        return yeet::abc<double>{m_distribution(m_rng), m_distribution(m_rng), m_distribution(m_rng)};
    }

    // Specialization for alphabeta<T>
    template <typename U = T>
    std::enable_if_t<std::is_same_v<U, alphabeta<double>>, T> generate() {
        if (m_stddev <= 0.0) {
            return alphabeta<double>{m_mean, m_mean};  // Return mean when no noise
        }
        return alphabeta<double>{m_distribution(m_rng), m_distribution(m_rng)};
    }

    // Specialization for dq<T>
    template <typename U = T>
    std::enable_if_t<std::is_same_v<U, dq<double>>, T> generate() {
        if (m_stddev <= 0.0) {
            return dq<double>{m_mean, m_mean};  // Return mean when no noise
        }
        return dq<double>{m_distribution(m_rng), m_distribution(m_rng)};
    }

private:
    void update_distribution() {
        // Only create distribution if stddev > 0 to avoid assertion failure
        if (m_stddev > 0.0) {
            m_distribution = std::normal_distribution<double>(m_mean, m_stddev);
        }
        // If stddev <= 0, we'll return the mean value directly in generate() methods
    }

    std::mt19937 m_rng;
    std::normal_distribution<double> m_distribution;
    double m_mean{0.0};
    double m_stddev{0.0};
};

template <typename T>
struct generator_step {
    double time{};
    T value{};
};

template <typename T>
struct generator_step_rate_of_change {
    double time_start{};
    double time_end{};
    T value_delta{};

    T get_incremental_delta(double sampling_time) const {
        double const delta_time = time_end - time_start;

        assert(delta_time > 0.0f && "Time delta must be greater than 0");

        return value_delta * sampling_time / delta_time;
    }
};

/**
 * @brief A class to generate a three-phase waveform signal with harmonics and noise.
 *        The signal is generated based on the positive and negative sequence components of the fundamental frequency
 *        as well as the harmonics that are added to the signal.
 *        The signal can be configured to have a specific initial angle, amplitude, and frequency.
 *        The signal can be configured to have specific harmonics with their respective magnitudes.
 *        The signal can be updated with noise and configured to have a specific noise standard deviation.
 *        The signal can be updated with steps in amplitude, frequency, and angle.
 *        The signal can also be updated with rate of change in amplitude and frequency.
 *
 */
class three_phase_waveform_generator {
private:
    struct signal_harmonics_configuration {
        std::size_t harmonic_order{};
        double magnitude{};
    };

    struct signal_harmonics_data {
        abc<angle<zero_to_two_pi>> signal_angle{};
        abc<double> signal{};
    };

    struct Signal_Harmonics {
        signal_harmonics_configuration configuration;
        signal_harmonics_data data;
    };

public:
    three_phase_waveform_generator(double sampling_time) : m_sampling_time(sampling_time) {
        assert(m_sampling_time > 0.0f && "Sampling time must be greater than 0");
    }

    /**
     * @brief Steps the generator to update the signal. The signal can be retrieved with the available getter methods.
     *
     */
    void update() {
        update_amplitude_frequency_and_angle_steps();

        update_rate_of_change_steps();

        m_fundamental_positive_sequence_signal_abc = generate_positive_sequence_signal_abc(m_fundamental_signal_angle);

        m_fundamental_negative_sequence_signal_abc = generate_negative_sequence_signal_abc(m_fundamental_signal_angle);

        m_signal_noise_abc = generate_signal_noise();

        for (Signal_Harmonics& n : m_harmonics) {
            n.data.signal = update_harmonics_signals(n);
        }

        m_simulation_time = increment_simulation_time(m_simulation_time);

        m_fundamental_signal_angle = increment_fundamental_angle(m_fundamental_signal_angle);

        for (auto& n : m_harmonics) {
            n.data.signal_angle = increment_harmonics_angles(n);
        }
    }

    /**
     * @brief Retrieves the fundamental signal, which is the sum of the positive and negative sequence signals.
     *
     * @return abc<double> fundamental signal
     */
    abc<double> get_fundamental_signal_abc() const {
        abc<double> const full_fundamental_signal =
            m_fundamental_positive_sequence_signal_abc + m_fundamental_negative_sequence_signal_abc;

        return full_fundamental_signal;
    }

    /**
     * @brief Retrieves the fundamental signal angle.
     *
     * @return abc<angle<zero_to_two_pi>> fundamental signal angle
     */
    abc<angle<zero_to_two_pi>> get_fundamental_signal_angle() const { return m_fundamental_signal_angle; }

    /**
     * @brief Retrieves the fundamental positive sequence signal.
     *
     * @return abc<double> fundamental positive sequence signal
     */

    abc<double> get_fundamental_positive_sequence_signal_abc() const {
        return m_fundamental_positive_sequence_signal_abc;
    }

    /**
     * @brief Retrieves the fundamental negative sequence signal.
     *
     * @return abc<double> fundamental negative sequence signal
     */
    abc<double> get_fundamental_negative_sequence_signal_abc() const {
        return m_fundamental_negative_sequence_signal_abc;
    }

    /**
     * @brief Retrieves the harmonic signal per harmonic.
     *
     * @param harmonic harmonic order
     * @return abc<double> harmonic signal if the harmonic exists, otherwise an empty signal
     */
    abc<double> get_harmonic_signal_abc(std::size_t harmonic) const {
        if (harmonic == 1) {
            return get_fundamental_signal_abc();
        }

        const auto it = std::find_if(m_harmonics.begin(), m_harmonics.end(), [harmonic](Signal_Harmonics const& n) {
            return n.configuration.harmonic_order == harmonic;
        });

        if (it == m_harmonics.end()) {
            return {};
        }

        return it->data.signal;
    }

    /**
     * @brief Retrieves the full signal, including the fundamental, harmonic signals and noise.
     *
     * @return abc<double> full signal
     */
    abc<double> get_signal_abc() const {
        abc<double> full_signal = get_fundamental_signal_abc();

        for (Signal_Harmonics const& n : m_harmonics) {
            full_signal += n.data.signal;
        }

        return full_signal + m_signal_noise_abc;
    }

    abc<double> get_signal_noise_abc() const { return m_signal_noise_abc; }

    double get_signal_frequency() const { return m_signal_frequency; }

    void enable_noise(double noise_standard_deviation = 0.1f) {
        set_noise_standard_deviation(noise_standard_deviation);
    }

    void disable_noise() { set_noise_standard_deviation(0.0f); }

    void set_noise_standard_deviation(double noise_standard_deviation) {
        m_noise_generator.set_parameters(noise_standard_deviation, 0.0);
    }

    /**
     * @brief Sets a symmetric angle for the fundamental and harmonics signal.
     *
     * @param angle three-phase angle
     */
    void set_angle(angle<zero_to_two_pi> angle) { set_angle({angle, angle, angle}); }

    /**
     * @brief Sets the angle for the fundamental and harmonics signal.
     *
     * @param angle three-phase angle
     */
    void set_angle(abc<angle<zero_to_two_pi>> angle) {
        m_fundamental_signal_angle = angle;

        for (auto& n : m_harmonics) {
            for (std::size_t phase = 0; phase < 3; ++phase) {
                n.data.signal_angle[phase] = angle[phase];
            }
        }
    }

    /**
     * @brief Configures the initial angle for a specific harmonic.
     *
     * @param harmonic harmonic order
     * @param angle three-phase angle
     */
    void set_initial_harmonics_angle(std::size_t harmonic, angle<zero_to_two_pi> angle) {
        set_initial_harmonics_angle(harmonic, {angle, angle, angle});
    }

    /**
     * @brief Configures the initial angle for a specific harmonic.
     *
     * @param harmonic harmonic order
     * @param angle three-phase angle
     */
    void set_initial_harmonics_angle(std::size_t harmonic, abc<angle<zero_to_two_pi>> angle) {
        for (auto& n : m_harmonics) {
            if (n.configuration.harmonic_order == harmonic) {
                n.data.signal_angle = angle;
            }
        }
    }

    /**
     * @brief Configures the fundamental signal amplitude.
     *
     * @param symmetric_amplitude symmetric amplitude for the fundamental signal
     */
    void set_fundamental_positive_sequence_signal_amplitude(double symmetric_positive_sequence_amplitude) {
        set_fundamental_positive_sequence_signal_amplitude({symmetric_positive_sequence_amplitude,
                                                            symmetric_positive_sequence_amplitude,
                                                            symmetric_positive_sequence_amplitude});
    }

    /**
     * @brief Configures the fundamental signal amplitude for the positive-sequence component.
     *
     * @param positive_sequence_amplitude three-phase amplitude for the positive sequence component
     */
    void set_fundamental_positive_sequence_signal_amplitude(abc<double> positive_sequence_amplitude) {
        m_fundamental_positive_sequence_signal_amplitude = positive_sequence_amplitude;
    }

    /**
     * @brief Configures the fundamental signal amplitude for the negative-sequence component.
     *
     * @param symmetric_amplitude symmetric amplitude for the negative sequence component
     */
    void set_fundamental_negative_sequence_signal_amplitude(double symmetric_negative_sequence_amplitude) {
        set_fundamental_negative_sequence_signal_amplitude({symmetric_negative_sequence_amplitude,
                                                            symmetric_negative_sequence_amplitude,
                                                            symmetric_negative_sequence_amplitude});
    }

    /**
     * @brief Configures the fundamental signal amplitude for the negative-sequence component.
     *
     * @param negative_sequence_amplitude three-phase amplitude for the negative sequence component
     */
    void set_fundamental_negative_sequence_signal_amplitude(abc<double> negative_sequence_amplitude) {
        m_fundamental_negative_sequence_signal_amplitude = negative_sequence_amplitude;
    }

    /**
     * @brief Configures the fundamental signal frequency.
     *
     * @param frequency fundamental signal frequency
     */
    void set_signal_frequency(double frequency) {
        assert(frequency > 0.0f && "Frequency must be greater than 0");

        m_signal_frequency = frequency;
    }

    /**
     * @brief Adds a harmonic of the fundamental frequency to the signal. Be aware that this function uses dynamic
     * memory. If the harmonic order already exists, the function will update the harmonic magnitude.
     *
     * @param harmonic_order harmonic order, greater than 1
     * @param harmonic_magnitude harmonic magnitude
     */
    void add_signal_harmonics(std::size_t harmonic_order, double harmonic_magnitude) {
        assert(harmonic_order > 1 && "Harmonic order must be greater than 1");

        Signal_Harmonics new_harmonics{};
        new_harmonics.configuration = signal_harmonics_configuration{harmonic_order, harmonic_magnitude};

        auto const it = std::find_if(m_harmonics.begin(), m_harmonics.end(), [&](Signal_Harmonics& n) {
            return n.configuration.harmonic_order == harmonic_order;
        });

        if (it != m_harmonics.end()) {
            *it = new_harmonics;
            return;
        }

        m_harmonics.emplace_back(new_harmonics);
    }

    /**
     * @brief Configures a step in the fundamental signal amplitude.
     *
     * @param amplitude three-phase step magnitude
     * @param time simulation time when the step will occur
     */
    void set_amplitude_step(abc<double> amplitude, double time) {
        generator_step<abc<double>> const step{time, amplitude};

        m_step_amplitude = step;
    }

    /**
     * @brief Configures a symmetric step in the fundamental signal amplitude.
     *
     * @param amplitude step magnitude
     * @param time simulation time when the step will occur
     */
    void set_amplitude_step(double amplitude, double time) {
        set_amplitude_step({amplitude, amplitude, amplitude}, time);
    }

    /**
     * @brief Configures a step in the signal frequency.
     *
     * @param frequency step magnitude
     * @param time simulation time when the step will occur
     */
    void set_frequency_step(double frequency, double time) {
        generator_step<double> const step{time, frequency};

        m_frequency_step = step;
    }

    /**
     * @brief Configures a step in the signal angle.
     *
     * @param angle step magnitude
     * @param time simulation time when the step will occur
     */
    void set_angle_step(abc<angle<zero_to_two_pi>> angle_step, double time) {
        generator_step<abc<angle<zero_to_two_pi>>> const step{time, angle_step};

        m_angle_step = step;
    }

    /**
     * @brief Configures a symmetric step in the signal angle.
     *
     * @param angle step magnitude
     * @param time simulation time when the step will occur
     */
    void set_angle_step(angle<zero_to_two_pi> angle_step, double time) {
        generator_step<abc<angle<zero_to_two_pi>>> const step{time, {angle_step, angle_step, angle_step}};

        m_angle_step = step;
    }

    /**
     * @brief Configures a rate of change in the fundamental signal amplitude.
     *
     * @param value_delta three-phase rate of change magnitude
     * @param time_start simulation time when the rate of change will start
     * @param time_end simulation time when the rate of change will end
     */
    void set_amplitude_rate_of_change(abc<double> value_delta, double time_start, double time_end) {
        generator_step_rate_of_change<abc<double>> const rate_of_change_step{time_start, time_end, value_delta};

        m_rate_of_change_amplitude = rate_of_change_step;
    }

    /**
     * @brief Configures a symmetric rate of change in the fundamental signal amplitude.
     *
     * @param value_delta rate of change magnitude
     * @param time_start simulation time when the rate of change will start
     * @param time_end simulation time when the rate of change will end
     */
    void set_frequency_rate_of_change(double value_delta, double time_start, double time_end) {
        generator_step_rate_of_change<double> const rate_of_change_step{time_start, time_end, value_delta};

        m_rate_of_change_frequency = rate_of_change_step;
    }

private:
    static abc<double> generate_abc_signal(std::size_t harmonic_index, abc<double> const& harmonic_amplitude,
                                           abc<angle<zero_to_two_pi>> const& signal_angle,
                                           abc<angle<zero_to_two_pi>> const& phase_shift) {
        // signal_angle + static_cast<double>(harmonic_index) * phase_shift;

        return abc<double>{harmonic_amplitude[0] *
                               std::sin(angle<zero_to_two_pi>{signal_angle[0].get_radians() +
                                                              static_cast<double>(harmonic_index) * phase_shift[0]}),
                           harmonic_amplitude[1] *
                               std::sin(angle<zero_to_two_pi>{signal_angle[1].get_radians() +
                                                              static_cast<double>(harmonic_index) * phase_shift[1]}),
                           harmonic_amplitude[2] *
                               std::sin(angle<zero_to_two_pi>{signal_angle[2].get_radians() +
                                                              static_cast<double>(harmonic_index) * phase_shift[2]})};
    }

    angle<zero_to_two_pi> get_signal_angle_increment(std::size_t harmonic_order, double signal_frequency) const {
        return angle<zero_to_two_pi>(static_cast<double>(harmonic_order) * 2.0 * pi * signal_frequency *
                                     m_sampling_time);
    }

    abc<angle<zero_to_two_pi>> increment_fundamental_angle(abc<angle<zero_to_two_pi>> fundamental_angle) const {
        for (std::size_t phase = 0; phase < 3; ++phase) {
            fundamental_angle[phase] += get_signal_angle_increment(1, m_signal_frequency);
        }

        return fundamental_angle;
    }

    abc<angle<zero_to_two_pi>> increment_harmonics_angles(Signal_Harmonics const& harmonic) const {
        abc<angle<zero_to_two_pi>> harmonics_angle = harmonic.data.signal_angle;

        for (std::size_t phase = 0; phase < 3; ++phase) {
            harmonics_angle[phase] +=
                get_signal_angle_increment(harmonic.configuration.harmonic_order, m_signal_frequency);
        }

        return harmonics_angle;
    }

    abc<double> generate_positive_sequence_signal_abc(
        abc<angle<zero_to_two_pi>> const& fundamental_signal_angle) const {
        return generate_abc_signal(1, m_fundamental_positive_sequence_signal_amplitude, fundamental_signal_angle,
                                   m_phase_shift);
    }

    abc<double> generate_negative_sequence_signal_abc(abc<angle<zero_to_two_pi>> fundamental_signal_angle) const {
        return generate_abc_signal(1, m_fundamental_negative_sequence_signal_amplitude, fundamental_signal_angle,
                                   -m_phase_shift);
    }

    bool is_at_step_time(const double& step_time) const {
        return ((m_simulation_time <= step_time + m_sampling_time / 2.0f) &&
                (m_simulation_time >= step_time - m_sampling_time / 2.0f));
    }

    void update_amplitude_frequency_and_angle_steps() {
        if (m_step_amplitude.has_value() && is_at_step_time(m_step_amplitude->time)) {
            m_fundamental_positive_sequence_signal_amplitude += m_step_amplitude->value;
        }

        if (m_frequency_step.has_value() && is_at_step_time(m_frequency_step->time)) {
            m_signal_frequency += m_frequency_step->value;
        }

        if (m_angle_step.has_value() && is_at_step_time(m_angle_step->time)) {
            m_fundamental_signal_angle += m_angle_step->value;

            for (Signal_Harmonics& n : m_harmonics) {
                for (std::size_t phase = 0; phase < 3; ++phase) {
                    n.data.signal_angle[phase] +=
                        (static_cast<double>(n.configuration.harmonic_order) * m_angle_step->value[phase]);
                }
            }
        }
    }

    void update_rate_of_change_steps() {
        if (m_rate_of_change_amplitude.has_value() && (m_simulation_time >= m_rate_of_change_amplitude->time_start) &&
            (m_simulation_time <= m_rate_of_change_amplitude->time_end)) {
            for (std::size_t phase = 0; phase < 3; ++phase) {
                m_fundamental_positive_sequence_signal_amplitude[phase] +=
                    m_rate_of_change_amplitude->get_incremental_delta(m_sampling_time)[phase];
            }
        }

        if (m_rate_of_change_frequency.has_value() && (m_simulation_time >= m_rate_of_change_frequency->time_start) &&
            (m_simulation_time <= m_rate_of_change_frequency->time_end)) {
            m_signal_frequency += m_rate_of_change_frequency->get_incremental_delta(m_sampling_time);
        }
    }

    abc<double> generate_signal_noise() { return m_noise_generator.generate(); }

    abc<double> update_harmonics_signals(Signal_Harmonics const& harmonic) {
        return generate_abc_signal(
            harmonic.configuration.harmonic_order,
            {harmonic.configuration.magnitude, harmonic.configuration.magnitude, harmonic.configuration.magnitude},
            harmonic.data.signal_angle, m_phase_shift);
    }

    double increment_simulation_time(double simulation_time) const {
        simulation_time += m_sampling_time;

        return simulation_time;
    }

    abc<angle<zero_to_two_pi>> m_phase_shift{angle<zero_to_two_pi>{}, angle<zero_to_two_pi>{-2.0 * pi / 3.0f},
                                             angle<zero_to_two_pi>{2.0 * pi / 3.0f}};

    double m_sampling_time{0.0f};

    double m_simulation_time{0.0f};

    double m_signal_frequency{50.0f};

    abc<angle<zero_to_two_pi>> m_fundamental_signal_angle = {};

    abc<double> m_signal_noise_abc{};
    abc<double> m_fundamental_positive_sequence_signal_abc{};
    abc<double> m_fundamental_negative_sequence_signal_abc{};
    abc<double> m_fundamental_positive_sequence_signal_amplitude{1.0f, 1.0f, 1.0f};
    abc<double> m_fundamental_negative_sequence_signal_amplitude{0.0f, 0.0f, 0.0f};

    std::vector<Signal_Harmonics> m_harmonics{};

    std::optional<generator_step<abc<double>>> m_step_amplitude{};
    std::optional<generator_step<double>> m_frequency_step{};
    std::optional<generator_step<abc<angle<zero_to_two_pi>>>> m_angle_step{};
    std::optional<generator_step_rate_of_change<double>> m_rate_of_change_frequency{};
    std::optional<generator_step_rate_of_change<abc<double>>> m_rate_of_change_amplitude{};

    white_noise_generator<abc<double>> m_noise_generator;
};

}  // namespace yeet::control
