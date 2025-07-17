#include "sfc/sfc_speed_controller.hpp"

#include <algorithm>

namespace yeet::application::sfc {

sfc_speed_controller::sfc_speed_controller(double sampling_time)
    : _pi_controller(sampling_time), _speed_ref_filter(sampling_time) {
    _pi_controller.enable_integrator_clamping(true);
    _speed_ref_filter.configure(kDefaultReferenceFilterCutoffFreqHz);
}

void sfc_speed_controller::set_pi_gains(double kp, double ti) { _pi_controller.configure_with_ti(kp, ti); }

void sfc_speed_controller::set_torque_limits_pu(double min_pu, double max_pu) {
    _pi_controller.set_output_limits(min_pu, max_pu);
}

void sfc_speed_controller::configure_speed_reference_filter(double cutoff_freq_hz) {
    _speed_ref_filter.configure(cutoff_freq_hz);
}

double sfc_speed_controller::update(double speed_ref_pu, double speed_measured_pu) {
    // Step 1: Filter the speed reference to ensure a smooth setpoint ramp
    _filtered_speed_ref_pu = _speed_ref_filter.update(speed_ref_pu);

    // Step 2: Calculate the speed error using the filtered reference
    const double speed_error_pu = _filtered_speed_ref_pu - speed_measured_pu;

    // Step 3: Update the PI controller to get the torque reference
    _torque_ref_pu = _pi_controller.update(speed_error_pu);

    return _torque_ref_pu;
}

double sfc_speed_controller::get_torque_ref_pu() const { return _torque_ref_pu; }

double sfc_speed_controller::get_filtered_speed_ref_pu() const { return _filtered_speed_ref_pu; }

void sfc_speed_controller::reset(double initial_speed_ref_pu) {
    _pi_controller.reset(initial_speed_ref_pu);
    _speed_ref_filter.reset(initial_speed_ref_pu);
    _torque_ref_pu = update(initial_speed_ref_pu, initial_speed_ref_pu);
    _filtered_speed_ref_pu = initial_speed_ref_pu;
}

}  // namespace yeet::application::sfc