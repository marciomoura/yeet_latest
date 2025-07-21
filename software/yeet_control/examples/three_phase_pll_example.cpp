/**
 * @file three_phase_pll_example.cpp
 * @brief Example demonstrating the usage of the three-phase PLL
 *
 * This example shows how to:
 * 1. Configure a three-phase PLL
 * 2. Generate test signals using the three_phase_waveform_generator
 * 3. Process signals through the PLL
 * 4. Monitor frequency and phase tracking performance
 */

#include <iomanip>
#include <iostream>

#include "yeet_control/angle.hpp"
#include "yeet_control/reference_frame.hpp"
#include "yeet_control/three_phase_pll.hpp"
#include "yeet_control/three_phase_waveform_generator.hpp"

using namespace yeet::control;

void print_header() {
    std::cout << std::setw(8) << "Time[s]" << std::setw(12) << "Input_Freq" << std::setw(12) << "Est_Freq"
              << std::setw(12) << "Freq_Error" << std::setw(12) << "Phase[deg]" << std::setw(12) << "Q_Error"
              << std::setw(8) << "Locked" << std::endl;

    std::cout << std::string(80, '-') << std::endl;
}

void print_data(double time, double input_freq, const three_phase_pll<>::Outputs& pll_output, bool is_locked) {
    double freq_error = pll_output.estimated_frequency - input_freq;
    double phase_deg = pll_output.estimated_angle.get_degrees();

    std::cout << std::fixed << std::setprecision(3) << std::setw(8) << time << std::setw(12) << input_freq
              << std::setw(12) << pll_output.estimated_frequency << std::setw(12) << freq_error << std::setw(12)
              << phase_deg << std::setw(12) << pll_output.q_error << std::setw(8) << (is_locked ? "YES" : "NO")
              << std::endl;
}

int main() {
    std::cout << "=== Three-Phase PLL Demonstration ===" << std::endl;
    std::cout << std::endl;

    // Configuration parameters
    constexpr double sampling_time = 1e-4;       // 100 µs (10 kHz sampling)
    constexpr double nominal_frequency = 50.0;   // 50 Hz nominal frequency
    constexpr double simulation_duration = 1.0;  // 1 second total simulation

    // Configure the PLL
    three_phase_pll<>::Config pll_config = {
        .sampling_time = sampling_time,
        .kp = 100.0,    // Proportional gain
        .ki = 10000.0,  // Integral gain
        .nominal_frequency = nominal_frequency,
        .frequency_limit_min = 45.0,  // Minimum frequency limit
        .frequency_limit_max = 55.0   // Maximum frequency limit
    };

    three_phase_pll<double> pll(pll_config);

    // Configure signal generator for test signals
    three_phase_waveform_generator signal_generator(sampling_time);
    signal_generator.set_fundamental_positive_sequence_signal_amplitude(1.0);  // 1.0 pu amplitude
    signal_generator.set_signal_frequency(nominal_frequency);

    std::cout << "PLL Configuration:" << std::endl;
    std::cout << "  Sampling Time: " << sampling_time * 1e6 << " µs" << std::endl;
    std::cout << "  Nominal Frequency: " << nominal_frequency << " Hz" << std::endl;
    std::cout << "  Proportional Gain: " << pll_config.kp << std::endl;
    std::cout << "  Integral Gain: " << pll_config.ki << std::endl;
    std::cout << "  Frequency Limits: [" << pll_config.frequency_limit_min << ", " << pll_config.frequency_limit_max
              << "] Hz" << std::endl;
    std::cout << std::endl;

    // Scenario 1: Nominal frequency tracking
    std::cout << "=== Scenario 1: Nominal Frequency Tracking (50 Hz) ===" << std::endl;
    print_header();

    pll.reset();

    for (int i = 0; i < 2000; ++i) {  // 0.2 seconds
        signal_generator.update();
        abc<double> signal = signal_generator.get_fundamental_signal_abc();

        three_phase_pll<>::Inputs inputs{signal};
        auto pll_output = pll.update(inputs);

        double time = i * sampling_time;
        bool is_locked = pll.is_locked(0.05);

        // Print every 50 samples (5 ms intervals)
        if (i % 50 == 0) {
            print_data(time, nominal_frequency, pll_output, is_locked);
        }
    }

    std::cout << std::endl;

    // Scenario 2: Frequency step response
    std::cout << "=== Scenario 2: Frequency Step Response (50 Hz → 52 Hz) ===" << std::endl;
    print_header();

    pll.reset();

    double step_time = 0.1;  // Step occurs at 100 ms
    double new_frequency = 52.0;

    for (int i = 0; i < 3000; ++i) {  // 0.3 seconds
        double time = i * sampling_time;

        // Apply frequency step at specified time
        if (time >= step_time) {
            signal_generator.set_signal_frequency(new_frequency);
        }

        signal_generator.update();
        abc<double> signal = signal_generator.get_fundamental_signal_abc();

        three_phase_pll<>::Inputs inputs{signal};
        auto pll_output = pll.update(inputs);

        bool is_locked = pll.is_locked(0.05);
        double input_freq = (time >= step_time) ? new_frequency : nominal_frequency;

        // Print every 50 samples (5 ms intervals)
        if (i % 50 == 0) {
            print_data(time, input_freq, pll_output, is_locked);
        }
    }

    std::cout << std::endl;

    // Scenario 3: Voltage unbalance test
    std::cout << "=== Scenario 3: Voltage Unbalance Test (20% unbalance on phase B) ===" << std::endl;
    print_header();

    pll.reset();
    signal_generator.set_signal_frequency(nominal_frequency);  // Reset to nominal

    for (int i = 0; i < 2000; ++i) {  // 0.2 seconds
        signal_generator.update();
        abc<double> signal = signal_generator.get_fundamental_signal_abc();

        // Apply 20% unbalance to phase B
        signal.b() *= 0.8;

        three_phase_pll<>::Inputs inputs{signal};
        auto pll_output = pll.update(inputs);

        double time = i * sampling_time;
        bool is_locked = pll.is_locked(0.1);  // Slightly relaxed threshold for unbalanced case

        // Print every 50 samples (5 ms intervals)
        if (i % 50 == 0) {
            print_data(time, nominal_frequency, pll_output, is_locked);
        }
    }

    std::cout << std::endl;

    // Performance summary
    std::cout << "=== Performance Summary ===" << std::endl;
    std::cout << "The three-phase PLL demonstrates:" << std::endl;
    std::cout << "1. Fast convergence to nominal frequency" << std::endl;
    std::cout << "2. Good step response with minimal overshoot" << std::endl;
    std::cout << "3. Robustness to voltage unbalance" << std::endl;
    std::cout << "4. Accurate phase tracking" << std::endl;
    std::cout << std::endl;

    std::cout << "Key features:" << std::endl;
    std::cout << "- Synchronous Reference Frame (SRF) based algorithm" << std::endl;
    std::cout << "- PI controller for frequency estimation" << std::endl;
    std::cout << "- Low-pass filtering for noise rejection" << std::endl;
    std::cout << "- Automatic lock detection" << std::endl;
    std::cout << "- Configurable frequency limits" << std::endl;

    return 0;
}
