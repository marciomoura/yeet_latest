#pragma once

#include "yeet_control/pi_controller.hpp"

namespace yeet::application::sfc {

/**
 * @brief Implements the inner current control loop for an LCI drive.
 *
 * This class encapsulates the full functionality of a high-performance current
 * controller. It is suitable for both high-performance LCI drives and
 * soft-starter applications. The controller operates on per-unit values.
 */
class sfc_current_controller {
public:
    /**
     * @brief Constructs the current controller.
     * @param sampling_time The sampling time of the control loop [s].
     */
    explicit sfc_current_controller(double sampling_time);

    /**
     * @brief Sets the gains for the internal PI controller.
     * @param kp The proportional gain (dimensionless).
     * @param ti The integral time constant [s].
     */
    void set_pi_gains(double kp, double ti);

    /**
     * @brief Sets the operational limits for the DC link current reference.
     * @param min_pu The minimum allowed current reference [pu].
     * @param max_pu The maximum allowed current reference [pu].
     */
    void set_current_limits_pu(double min_pu, double max_pu);

    /**
     * @brief Sets the operational limits for the grid-side converter firing angle.
     * @param min_radians The minimum allowed firing angle [rad].
     * @param max_radians The maximum allowed firing angle [rad].
     */
    void set_firing_angle_limits_radians(double min_radians, double max_radians);

    /**
     * @brief Executes one update cycle of the current controller.
     * @param dc_link_current_ref_pu The per-unit current reference from the outer loop.
     * @param dc_link_current_measured_pu The measured per-unit DC link current.
     * @param dc_link_voltage_inverter_estimated_pu Estimated per-unit inverter back-EMF for feedforward.
     * @return The calculated firing angle reference for the rectifier [rad].
     */
    double update(double dc_link_current_ref_pu, double dc_link_current_measured_pu,
                  double dc_link_voltage_inverter_estimated_pu);

    /**
     * @brief Gets the most recently calculated firing angle reference.
     * @return The firing angle reference [rad].
     */
    double get_firing_angle_ref_radians() const;

    /**
     * @brief Gets the limited current reference used in the last update cycle.
     * @return The limited DC link current reference [pu].
     */
    double get_current_ref_limited_pu() const;

    /*
     * @brief Gets the clamped cosine of the firing angle.
     * @return The clamped cosine value.
     */
    double get_cos_alpha_clamped() const;

private:
    yeet::control::pi_controller<double> _pi_controller;

    // Configuration
    double _dc_link_current_min_pu{};
    double _dc_link_current_max_pu{};
    double _grid_firing_angle_min_radians{};
    double _grid_firing_angle_max_radians{};

    // Outputs
    double _cos_alpha_clamped{};
    double _firing_angle_ref_radians{};
    double _current_ref_limited_pu{};
};

}  // namespace yeet::application::sfc
