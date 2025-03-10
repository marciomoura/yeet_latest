#pragma once

#include "first_order_low_pass_filter.hpp"
#include "pi_controller.hpp"

namespace yeet::control {

class machine_speed_controller {
public:
    explicit machine_speed_controller(double sampling_period)
        : _ts(sampling_period), _speed_controller(), _speed_ref_filter(sampling_period) {}

    /**
     * @brief Configure the speed controller
     *
     * @param inertia System inertia [p.u.]
     * @param desired_bandwidth Desired closed-loop bandwidth [rad/s]
     * @param damping_ratio Desired damping ratio
     * @param torque_limit Maximum torque limit [p.u.]
     */
    void configure(double inertia, double desired_bandwidth, double damping_ratio, double torque_limit) {
        // For specified bandwidth and damping:
        // Kp = J * (2*zeta*wn)
        // Ki = J * wn^2
        const double kp = inertia * (double(2.0) * damping_ratio * desired_bandwidth);
        const double ti = double(2.0) * damping_ratio / desired_bandwidth;

        _speed_controller.configure_with_ti(kp, ti, _ts);
        _speed_controller.set_output_limits(-torque_limit, torque_limit);
        _torque_limit = torque_limit;
    }

    /**
     * @brief Configure speed reference filter.
     *
     * @param cutoff_freq Filter cutoff frequency [Hz]
     */
    void configure_filter(double cutoff_freq) { _speed_ref_filter.configure(cutoff_freq); }

    /**
     * @brief Update the speed controller
     *
     * @param speed_ref Speed reference [p.u.]
     * @param speed_measured Measured speed [p.u.]
     * @return T Torque reference [p.u.]
     */
    double update(double speed_ref, double speed_measured) {
        // Filter speed reference
        double speed_ref_filtered = _speed_ref_filter.update(speed_ref);

        // Calculate speed error
        double speed_error = speed_ref_filtered - speed_measured;

        // PI control to generate torque reference
        return _speed_controller.update(speed_error);
    }

    /**
     * @brief Get the filtered speed reference
     */
    double get_filtered_speed_reference() const { return _speed_ref_filter.get_output(); }

    /**
     * @brief Apply custom torque limit (useful for different machine doubles)
     */
    void set_torque_limit(double torque_limit) {
        _speed_controller.set_output_limits(-torque_limit, torque_limit);
        _torque_limit = torque_limit;
    }

private:
    double _ts{};
    pi_controller<double> _speed_controller;
    first_order_low_pass_filter<double> _speed_ref_filter;
    double _torque_limit{1.0};
};

}  // namespace yeet::control