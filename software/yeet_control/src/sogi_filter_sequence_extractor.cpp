#include "yeet_control/sogi_filter_sequence_extractor.hpp"

#include <cassert>

namespace yeet::control {

sogi_filter_sequence_extractor::sogi_filter_sequence_extractor(duration_t sampling_time)
    : _sampling_time(sampling_time), _sogi_alpha(sampling_time), _sogi_beta(sampling_time) {
    assert(sampling_time > duration_t{0} && "Sampling time must be positive");

    // _sogi_alpha.configure_dc_rejection(duration_t{1.0f});
    // _sogi_beta.configure_dc_rejection(duration_t{1.0f});

    // Configure sequence extractor for testing
    // sogi_filter.configure(1.414, 1.0 / 50.0, true);  // SOGI gain and DC rejection time constant
}

void sogi_filter_sequence_extractor::configure(real_t k, duration_t time_constant_dc_rejection_seconds,
                                               bool scale_output) {
    assert(k > real_t{0} && "SOGI gain must be positive");
    assert(time_constant_dc_rejection_seconds > duration_t{0} && "DC rejection gain must be positive");

    _scale_output = scale_output;

    // Configure both SOGI filters with the same parameters
    _sogi_alpha.configure(k);
    _sogi_alpha.configure_dc_rejection(time_constant_dc_rejection_seconds);
    _sogi_beta.configure(k);
    _sogi_beta.configure_dc_rejection(time_constant_dc_rejection_seconds);

    reset();
}

void sogi_filter_sequence_extractor::reset() {
    _sogi_alpha.reset();
    _sogi_beta.reset();
    _positive_sequence = {};
}

void sogi_filter_sequence_extractor::update(const alphabeta<voltage_pu_t>& v_in, angular_frequency_t omega) {
    // 1. Run the alpha component through the first SOGI filter
    _sogi_alpha.update(v_in.alpha(), omega);
    const alphabeta<voltage_pu_t>& sogi_alpha_out = _sogi_alpha.get_output();
    const voltage_pu_t u_alpha_prime = sogi_alpha_out.alpha();        // In-phase output from SOGI-alpha (v')
    const voltage_pu_t u_alpha_double_prime = sogi_alpha_out.beta();  // Quadrature output from SOGI-alpha (qv')

    // 2. Run the beta component through the second SOGI filter
    _sogi_beta.update(v_in.beta(), omega);
    const alphabeta<voltage_pu_t>& sogi_beta_out = _sogi_beta.get_output();
    const voltage_pu_t u_beta_prime = sogi_beta_out.alpha();        // In-phase output from SOGI-beta (v')
    const voltage_pu_t u_beta_double_prime = sogi_beta_out.beta();  // Quadrature output from SOGI-beta (qv')

    // 3. Apply the cross-coupling logic to isolate the positive sequence
    // According to the documentation, the cross-coupling is:
    // Uα_out = Uα' - Uβ''  (alpha in-phase minus beta quadrature)
    // Uβ_out = Uβ' + Uα''  (beta in-phase plus alpha quadrature)
    // This extracts positive sequence and doubles the amplitude
    voltage_pu_t v_alpha_pos = u_alpha_prime - u_beta_double_prime;
    voltage_pu_t v_beta_pos = u_beta_prime + u_alpha_double_prime;

    // 4. Optionally scale the output by 0.5 to compensate for the doubling in amplitude
    if (_scale_output) {
        v_alpha_pos = real_t{0.5} * v_alpha_pos;
        v_beta_pos = real_t{0.5} * v_beta_pos;
    }

    _positive_sequence = alphabeta<voltage_pu_t>{v_alpha_pos, v_beta_pos};
}

const alphabeta<voltage_pu_t>& sogi_filter_sequence_extractor::get_positive_sequence() const {
    return _positive_sequence;
}

voltage_pu_t sogi_filter_sequence_extractor::get_positive_sequence_alpha() const { return _positive_sequence.alpha(); }

voltage_pu_t sogi_filter_sequence_extractor::get_positive_sequence_beta() const { return _positive_sequence.beta(); }

alphabeta<voltage_pu_t> sogi_filter_sequence_extractor::get_alpha_sogi_output() const {
    return _sogi_alpha.get_output();
}

alphabeta<voltage_pu_t> sogi_filter_sequence_extractor::get_beta_sogi_output() const { return _sogi_beta.get_output(); }

alphabeta<voltage_pu_t> sogi_filter_sequence_extractor::get_dc_offset() const {
    return alphabeta<voltage_pu_t>{_sogi_alpha.get_dc_offset(), _sogi_beta.get_dc_offset()};
}

}  // namespace yeet::control