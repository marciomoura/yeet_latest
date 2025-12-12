#include "yeet_control/field_current_regulator.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace yeet::control {

TEST(field_current_regulator_test, field_current_decays_with_configured_time_constant_when_exciter_voltage_is_zero)
{
    // Use a relatively large sampling time so the expected discrete-time response is easy to reason about.
    const duration_t ts{0.01};  // 10 ms
    field_current_regulator regulator{ts};

    const resistance_pu_t r_f{0.01};
    const duration_t tau_f{2.5};
    regulator.configure_field_circuit(r_f, tau_f);
    regulator.configure_voltage_limits(voltage_pu_t{0.0}, voltage_pu_t{2.0});

    // Force the PI to command 0V for negative current error by saturating at the minimum limit.
    regulator.configure_pi_gains(100.0, 1.0);

    // Start from 1.0 pu field current.
    regulator.reset(current_pu_t{1.0});

    // Command 0.0 pu field current so the controller drives exciter voltage to 0.
    const int steps = static_cast<int>(tau_f.value() / ts.value());
    for (int i = 0; i < steps; ++i) {
        regulator.update(current_pu_t{0.0});
    }

    const double a_prev = tau_f.value() / (tau_f.value() + ts.value());
    const double expected = std::pow(a_prev, static_cast<double>(steps)) * 1.0;

    EXPECT_NEAR(regulator.get_exciter_voltage().value(), 0.0, 1e-6);
    EXPECT_NEAR(regulator.get_field_current().value(), expected, 1e-3);
}

}  // namespace yeet::control
