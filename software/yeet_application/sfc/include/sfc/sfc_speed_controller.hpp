#pragma once

#include "yeet_control/first_order_low_pass_filter.hpp"
#include "yeet_control/pi_controller.hpp"

namespace yeet::application::sfc {

/**
 * @brief Implements the outer speed control loop for an LCI drive.
 *
 * This class encapsulates the functionality of a standard PI speed controller.
 * It operates on per-unit values and generates a per-unit torque reference,
 * which serves as the setpoint for the inner current control loop. The output
 * of this controller is therefore the DC-link current reference.
 *
 * It includes a low-pass filter on the speed reference to ensure smooth ramps.
 */
class sfc_speed_controller {
public:
    static constexpr double kDefaultReferenceFilterCutoffFreqHz =
        10.0;  // Default cutoff frequency for speed reference filter

    /**
     * @brief Constructs the speed controller.
     * @param sampling_time The sampling time of the control loop [s].
     */
    explicit sfc_speed_controller(double sampling_time);

    /**
     * @brief Sets the gains for the internal PI controller.
     * @param kp The proportional gain (dimensionless).
     * @param ti The integral time constant [s].
     */
    void set_pi_gains(double kp, double ti);

    /**
     * @brief Sets the operational limits for the output torque reference.
     * @param min_pu The minimum allowed torque reference [pu].
     * @param max_pu The maximum allowed torque reference [pu].
     */
    void set_torque_limits_pu(double min_pu, double max_pu);

    /**
     * @brief Configures the low-pass filter for the speed reference.
     * @param cutoff_freq_hz The cutoff frequency of the filter [Hz].
     */
    void configure_speed_reference_filter(double cutoff_freq_hz);

    /**
     * @brief Executes one update cycle of the speed controller.
     * @param speed_ref_pu The per-unit speed reference.
     * @param speed_measured_pu The measured per-unit speed.
     * @return The calculated torque reference [pu] for the current controller.
     */
    double update(double speed_ref_pu, double speed_measured_pu);

    /**
     * @brief Gets the most recently calculated torque reference.
     * @return The torque reference [pu].
     */
    double get_torque_ref_pu() const;

    /**
     * @brief Gets the filtered speed reference used in the last update cycle.
     * @return The filtered speed reference [pu].
     */
    double get_filtered_speed_ref_pu() const;

    /**
     * @brief Resets the internal state of the PI controller and filter.
     */
    void reset();

private:
    yeet::control::pi_controller<double> _pi_controller;
    yeet::control::first_order_low_pass_filter<double> _speed_ref_filter;

    // Outputs
    double _torque_ref_pu{};
    double _filtered_speed_ref_pu{};
};

}  // namespace yeet::application::sfc
