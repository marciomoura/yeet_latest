/**
 * @file synchronous_machine_backemf_test.cpp
 * @brief Isolated test to debug back-EMF calculation in synchronous_machine
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <iomanip>

#include "yeet_control/synchronous_machine.hpp"
#include "yeet_control/types.hpp"

using namespace yeet::control;
using namespace yeet;

TEST(SynchronousMachineBackEMFTest, OpenCircuitBackEMF)
{
    // Create machine with same parameters as main test
    synchronous_machine machine(resistance_t(0.5),  // R_s
        resistance_t(0.5),                          // R_fD
        resistance_t(0.01),                         // R_kD
        resistance_t(0.01),                         // R_kQ
        inductance_t(0.015),                        // L_d
        inductance_t(0.010),                        // L_q
        inductance_t(0.012),                        // L_ad
        inductance_t(0.010),                        // L_aq
        inductance_t(5.0),                          // L_fD
        inductance_t(0.01),                         // L_kD (use larger value to avoid numerical issues)
        inductance_t(0.008),                        // L_kQ
        moment_of_inertia_t(100.0),                 // J
        4                                           // P (total poles = 2 pole pairs)
    );

    // Create machine state with known values
    synchronous_machine_state state{};
    state.psi_fD = 15.0;  // 15 Wb field flux

    // CRITICAL: For zero damper current in steady-state, damper flux must be:
    // psi_kD = L_ad / (L_fD + L_ad) × psi_fD
    // This accounts for mutual coupling - damper needs flux to balance field mutual flux
    constexpr double L_ad_val = 0.012;
    constexpr double L_fD_val = 5.0;
    state.psi_kD = (L_ad_val / (L_fD_val + L_ad_val)) * state.psi_fD;  // Correct initial damper flux

    state.psi_kQ = 0.0;     // No q-axis damper flux (no q-axis excitation)
    state.omega_r = 150.0;  // 150 rad/s mechanical = 300 rad/s electrical
    state.theta_r = 0.0;    // Aligned at 0 degrees

    // Zero current (open circuit)
    abc<current_t> i_zero(current_t(0.0), current_t(0.0), current_t(0.0));

    // Get terminal voltages
    abc<voltage_t> v_terminal = machine.get_terminal_voltages_abc(state, i_zero);

    // Manual calculation of expected back-EMF
    double omega_elec = state.omega_r * 2.0;  // 300 rad/s
    double L_ad = 0.012;
    double L_fD = 5.0;
    double L_kD = 0.005;

    // Calculate field current from flux
    double det = (L_fD + L_ad) * (L_kD + L_ad) - L_ad * L_ad;
    double i_f = ((L_kD + L_ad) * state.psi_fD - L_ad * state.psi_kD) / det;

    // Stator d-axis flux from field
    double psi_stator_d = L_ad * i_f;

    // q-axis voltage (perpendicular to field flux)
    double expected_emf_q = omega_elec * psi_stator_d;

    std::cout << "\n=== Back-EMF Debug Test ===" << std::endl;
    std::cout << "Field flux: " << state.psi_fD << " Wb" << std::endl;
    std::cout << "Speed: " << state.omega_r << " rad/s mech = " << omega_elec << " rad/s elec" << std::endl;
    std::cout << "Field current (calculated): " << i_f << " A" << std::endl;
    std::cout << "Stator d-axis flux: " << psi_stator_d << " Wb" << std::endl;
    std::cout << "Expected EMF (q-axis): " << expected_emf_q << " V" << std::endl;
    std::cout << "\nActual terminal voltages from machine model:" << std::endl;
    std::cout << "  V_a = " << v_terminal.a().value() << " V" << std::endl;
    std::cout << "  V_b = " << v_terminal.b().value() << " V" << std::endl;
    std::cout << "  V_c = " << v_terminal.c().value() << " V" << std::endl;

    // Calculate peak from abc values
    double v_peak = std::max(
        {std::abs(v_terminal.a().value()), std::abs(v_terminal.b().value()), std::abs(v_terminal.c().value())});
    std::cout << "Peak voltage: " << v_peak << " V" << std::endl;
    std::cout << "Expected peak: " << expected_emf_q << " V (dq frame)" << std::endl;

    // The αβ→abc transformation uses: [α, -0.5α + √3/2 β, -0.5α - √3/2 β]
    // With only β component (v_q), peak abc = √3/2 × v_q
    double expected_abc_peak = expected_emf_q * (std::sqrt(3.0) / 2.0);
    std::cout << "Expected ABC peak (with transform scaling): " << expected_abc_peak << " V" << std::endl;
    std::cout << "Ratio (actual/expected ABC): " << (v_peak / expected_abc_peak) << std::endl;

    // The back-EMF should be close to expected value
    // Using 10% tolerance for now to see how far off we are
    EXPECT_NEAR(v_peak, expected_abc_peak, expected_abc_peak * 0.05) << "Back-EMF calculation is wrong!";
}

TEST(SynchronousMachineBackEMFTest, BackEMFAtDifferentAngles)
{
    // Same machine setup
    synchronous_machine machine(resistance_t(0.5), resistance_t(0.5), resistance_t(0.01), resistance_t(0.01),
        inductance_t(0.015), inductance_t(0.010), inductance_t(0.012), inductance_t(0.010), inductance_t(5.0),
        inductance_t(0.005), inductance_t(0.005), moment_of_inertia_t(100.0), 4  // Total poles = 2 pole pairs
    );

    synchronous_machine_state state{};
    state.psi_fD = 15.0;

    // Initialize damper flux to steady-state value for zero damper current
    double L_ad_val = 0.012, L_fD_val = 5.0;
    state.psi_kD = (L_ad_val / (L_fD_val + L_ad_val)) * state.psi_fD;
    state.psi_kQ = 0.0;
    state.omega_r = 150.0;

    abc<current_t> i_zero(current_t(0.0), current_t(0.0), current_t(0.0));

    std::cout << "\n=== Back-EMF at Different Rotor Angles ===" << std::endl;

    // Test at several angles
    for (double theta_deg = 0.0; theta_deg <= 360.0; theta_deg += 60.0) {
        state.theta_r = theta_deg * pi_double / 180.0;  // Convert to radians (mechanical)

        abc<voltage_t> v_terminal = machine.get_terminal_voltages_abc(state, i_zero);

        double v_peak = std::max(
            {std::abs(v_terminal.a().value()), std::abs(v_terminal.b().value()), std::abs(v_terminal.c().value())});

        std::cout << "θ=" << std::setw(5) << theta_deg << "°: V_abc=(" << std::setw(8) << v_terminal.a().value() << ", "
                  << std::setw(8) << v_terminal.b().value() << ", " << std::setw(8) << v_terminal.c().value()
                  << ") V, peak=" << v_peak << " V" << std::endl;
    }
}
