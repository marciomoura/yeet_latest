#pragma once

#include <cmath>

#include "constants.hpp"

namespace yeet::control {

// Structure for nominal values configuration with flexible voltage specification
struct per_unit_definition {
    double voltage;    // [V] peak phase voltage - internal base voltage
    double power;      // [VA] 3-phase power
    double frequency;  // [Hz] base frequency

    // Factory methods for different voltage specifications
    static per_unit_definition from_peak_phase_voltage(double peak_phase_voltage, double power, double frequency) {
        return {peak_phase_voltage, power, frequency};
    }

    static per_unit_definition from_rms_phase_voltage(double rms_phase_voltage, double power, double frequency) {
        // Convert RMS to peak: Vpeak = √2 * Vrms
        double peak_phase_voltage = rms_phase_voltage * std::sqrt(2.0);
        return {peak_phase_voltage, power, frequency};
    }

    static per_unit_definition from_rms_line_voltage(double rms_line_voltage, double power, double frequency) {
        // Convert line voltage to phase voltage: Vphase_rms = Vline_rms / √3
        // Then convert to peak: Vpeak = √2 * Vrms
        double peak_phase_voltage = rms_line_voltage * std::sqrt(2.0 / 3.0);
        return {peak_phase_voltage, power, frequency};
    }
};

// Structure to hold all derived base values for per-unit calculations
struct per_unit_base_values {
    double voltage;    // [V] peak phase
    double current;    // [A] peak phase
    double impedance;  // [ohm]
    double flux;       // [Wb]
    double speed;      // [rad/s]
    double torque;     // [N·m]
    double frequency;  // [Hz]
    double power;      // [VA] 3-phase

    // Constructor to calculate base values from nominal values
    explicit per_unit_base_values(const per_unit_definition& definition) {
        // Store the original nominal values
        voltage = definition.voltage;
        power = definition.power;
        frequency = definition.frequency;

        // Calculate derived base values
        current = power / (3.0 * voltage);
        impedance = voltage / current;
        flux = voltage / (2.0 * pi<double> * frequency);
        speed = 2.0 * pi<double> * frequency;
        torque = power / speed;
    }
};

}  // namespace yeet::control