#include "yeet_control/sogi_filter.hpp"

#include <cassert>

namespace yeet::control {

sogi_filter::sogi_filter(real_t sampling_time) : _sampling_time(sampling_time) {
    assert(sampling_time > real_t{0} && "Sampling time must be positive");

    // Configure all integrators with the system sampling time
    _alpha_integrator.configure(sampling_time);
    _beta_integrator.configure(sampling_time);
    _dc_integrator.configure(sampling_time);
}

void sogi_filter::configure(real_t k) {
    assert(k > real_t{0} && "SOGI gain must be positive");
    _k = k;
}

void sogi_filter::configure_dc_rejection(real_t time_constant) {
    assert(time_constant > real_t{0} && "Time constant must be positive");
    // The gain is the inverse of the time constant, defining the filter's pole
    _k_dc = real_t(1.0) / time_constant;
}

void sogi_filter::reset() {
    _alpha_integrator.reset();
    _beta_integrator.reset();
    _dc_integrator.reset();
}

alphabeta<real_t> sogi_filter::get_output() const { return _output; }

real_t sogi_filter::get_alpha() const { return _output.alpha(); }

real_t sogi_filter::get_beta() const { return _output.beta(); }

real_t sogi_filter::get_dc_offset() const { return _dc_integrator.get_output(); }

void sogi_filter::update(real_t input_voltage, real_t omega) {
    // Get the state from the end of the previous time step
    real_t const v_alpha_prev = _alpha_integrator.get_output();
    real_t const v_beta_prev = _beta_integrator.get_output();
    real_t const dc_offset_prev = _dc_integrator.get_output();

    // Calculate the main SOGI error signal
    real_t const error_sogi = input_voltage - v_alpha_prev - dc_offset_prev;

    // Update the DC integrator first
    // The input to the DC filter is the SOGI error itself
    real_t const dc_integrator_input = _k_dc * error_sogi;
    _dc_integrator.update(dc_integrator_input);

    // Calculate the inputs for the main SOGI integrators
    // d(v_alpha)/dt = ω * (k * error - v_beta)
    real_t const alpha_integrator_input = omega * ((_k * error_sogi) - v_beta_prev);

    // Update the alpha integrator to get the NEW alpha value for this time step
    _alpha_integrator.update(alpha_integrator_input);
    real_t const v_alpha_new = _alpha_integrator.get_output();

    // Use the newly computed alpha value (v_alpha_new) as the input to the beta
    // integrator. This significantly reduces internal phase delay and improves accuracy.
    // d(v_beta)/dt = ω * v_alpha
    real_t const beta_integrator_input = omega * v_alpha_new;

    // Update the beta integrator
    _beta_integrator.update(beta_integrator_input);

    // The output is the current alpha and beta values, rotated by half the time step
    // to account for the phase shift introduced by the SOGI structure
    _output = alphabeta<real_t>{_alpha_integrator.get_output(), _beta_integrator.get_output()}.rotate_clockwise(
        omega * _sampling_time);
}

}  // namespace yeet::control