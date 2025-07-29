#include "yeet_control/sogi_filter.hpp"

#include <cassert>

#include "yeet_control/units.hpp"

namespace yeet::control {

sogi_filter::sogi_filter(duration_t sampling_time) : _sampling_time(sampling_time) {
    assert(sampling_time > duration_t{0} && "Sampling time must be positive");

    // Configure all integrators with the system sampling time
    _alpha_integrator.configure(sampling_time.value());
    _beta_integrator.configure(sampling_time.value());
    _dc_integrator.configure(sampling_time.value());

    configure(real_t{default_k});
    configure_dc_rejection(duration_t{default_dc_rejection_time_constant});
}

void sogi_filter::configure(real_t k) {
    assert(k > real_t{0} && "SOGI gain must be positive");
    _k = k;
}

void sogi_filter::configure_dc_rejection(duration_t time_constant) {
    assert(time_constant > duration_t{0} && "Time constant must be positive");
    // The gain is the inverse of the time constant, defining the filter's pole
    _k_dc = real_t(1.0) / time_constant;
}

void sogi_filter::reset() {
    _alpha_integrator.reset();
    _beta_integrator.reset();
    _dc_integrator.reset();
}

alphabeta<voltage_pu_t> sogi_filter::get_output() const { return _output; }

voltage_pu_t sogi_filter::get_alpha() const { return _output.alpha(); }

voltage_pu_t sogi_filter::get_beta() const { return _output.beta(); }

voltage_pu_t sogi_filter::get_dc_offset() const { return voltage_pu_t{_dc_integrator.get_output()}; }

void sogi_filter::update(voltage_pu_t input_voltage, angular_frequency_t omega) {
    // Get the state from the end of the previous time step
    voltage_pu_t const v_alpha_prev = voltage_pu_t{_alpha_integrator.get_output()};
    voltage_pu_t const v_beta_prev = voltage_pu_t{_beta_integrator.get_output()};
    voltage_pu_t const dc_offset_prev = voltage_pu_t{_dc_integrator.get_output()};

    // Calculate the main SOGI error signal
    voltage_pu_t const error_sogi = input_voltage - v_alpha_prev - dc_offset_prev;

    // Update the DC integrator first
    // The input to the DC filter is the SOGI error itself
    voltage_pu_t const dc_integrator_input = _k_dc * error_sogi;
    _dc_integrator.update(dc_integrator_input);

    // Calculate the inputs for the main SOGI integrators
    // d(v_alpha)/dt = ω * (k * error - v_beta)
    auto const alpha_integrator_input = omega * ((_k * error_sogi) - v_beta_prev);

    // Update the alpha integrator to get the NEW alpha value for this time step
    _alpha_integrator.update(alpha_integrator_input);
    auto const v_alpha_new = _alpha_integrator.get_output();

    // Use the newly computed alpha value (v_alpha_new) as the input to the beta
    // integrator. This significantly reduces internal phase delay and improves accuracy.
    // d(v_beta)/dt = ω * v_alpha
    auto const beta_integrator_input = omega * v_alpha_new;

    // Update the beta integrator
    _beta_integrator.update(beta_integrator_input);

    // The output is the current alpha and beta values, rotated by half the time step
    // to account for the phase shift introduced by the SOGI structure
    _output = alphabeta<voltage_pu_t>{voltage_pu_t{_alpha_integrator.get_output()},
                                      voltage_pu_t{_beta_integrator.get_output()}}
                  .rotate_clockwise({omega * _sampling_time});
}

}  // namespace yeet::control