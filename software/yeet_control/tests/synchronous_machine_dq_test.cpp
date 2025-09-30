#include "yeet_control/synchronous_machine_dq.hpp"

#include <gtest/gtest.h>

#include "yeet_control/reference_frame.hpp"

using namespace yeet;
using namespace yeet::control;

TEST(synchronous_machine_dq, zero_initial_conditions_produce_zero_outputs)
{
    synchronous_machine_dq machine(1.0e-4f);
    machine.configure_stator_inductances(inductance_pu_t{1.2f}, inductance_pu_t{1.0f});
    machine.configure_field_coupling(inductance_pu_t{0.9f}, inductance_pu_t{0.7f}, resistance_pu_t{0.02f});
    machine.configure_pole_pairs(2);
    machine.configure_base_electrical_frequency_hz(50.0f);
    machine.reset();

    abc<current_pu_t> currents{current_pu_t{0.0f}, current_pu_t{0.0f}, current_pu_t{0.0f}};
    auto torque = machine.update(currents, voltage_pu_t{0.0f}, speed_pu_t{0.0f}, angle_wrapped{0.0});

    auto emf = machine.get_back_emf_abc();
    auto induced_dq = machine.get_induced_voltage_dq();
    EXPECT_NEAR(emf.a().value(), 0.0, 1e-6);
    EXPECT_NEAR(emf.b().value(), 0.0, 1e-6);
    EXPECT_NEAR(emf.c().value(), 0.0, 1e-6);
    EXPECT_NEAR(induced_dq.d().value(), 0.0, 1e-6);
    EXPECT_NEAR(induced_dq.q().value(), 0.0, 1e-6);
    EXPECT_NEAR(torque.value(), 0.0, 1e-6);
    EXPECT_NEAR(machine.get_field_current_pu().value(), 0.0, 1e-6);
}

TEST(synchronous_machine_dq, field_voltage_generates_field_flux_and_current)
{
    synchronous_machine_dq machine(1.0e-4f);
    machine.configure_stator_inductances(inductance_pu_t{1.1f}, inductance_pu_t{1.0f});
    machine.configure_field_coupling(inductance_pu_t{0.8f}, inductance_pu_t{0.6f}, resistance_pu_t{0.05f});
    machine.configure_pole_pairs(2);
    machine.configure_base_electrical_frequency_hz(50.0f);
    machine.reset();

    abc<current_pu_t> currents{current_pu_t{0.0f}, current_pu_t{0.0f}, current_pu_t{0.0f}};
    for (int k = 0; k < 5000; ++k) {
        auto torque = machine.update(currents, voltage_pu_t{1.0f}, speed_pu_t{0.0f}, angle_wrapped{0.0});
        (void)torque;  // suppress unused variable warning
    }
    EXPECT_GT(machine.get_field_flux_linkage_pu().value(), 0.0f);
    EXPECT_GT(machine.get_field_current_pu().value(), 0.0f);
}

TEST(synchronous_machine_dq, torque_sign_changes_with_q_axis_current)
{
    synchronous_machine_dq machine(1.0e-4f);
    machine.configure_stator_inductances(inductance_pu_t{1.2f}, inductance_pu_t{1.0f});
    machine.configure_field_coupling(inductance_pu_t{0.9f}, inductance_pu_t{0.7f}, resistance_pu_t{0.02f});
    machine.configure_pole_pairs(2);
    machine.configure_base_electrical_frequency_hz(50.0f);
    machine.reset();

    // establish some field flux first
    abc<current_pu_t> zero_currents{current_pu_t{0.0f}, current_pu_t{0.0f}, current_pu_t{0.0f}};
    for (int k = 0; k < 2000; ++k) {
        auto torque = machine.update(zero_currents, voltage_pu_t{1.0f}, speed_pu_t{0.0f}, angle_wrapped{0.0});
        (void)torque;  // suppress unused variable warning
    }
    auto base_flux = machine.get_field_flux_linkage_pu().value();
    EXPECT_GT(base_flux, 0.0f);

    // apply balanced q-axis producing current set via abc injection at fixed angle ~0
    // choose currents that approximate pure q-axis: shift phase currents by 90 deg relative to d-axis angle 0
    // with theta ~0, pure q negative sine component so: i_a=0, i_b=+I, i_c=-I approximates some q component.
    abc<current_pu_t> currents_positive_q{current_pu_t{0.0f}, current_pu_t{0.5f}, current_pu_t{-0.5f}};
    auto torque_pos_result =
        machine.update(currents_positive_q, voltage_pu_t{0.0f}, speed_pu_t{0.0f}, angle_wrapped{0.0});
    auto torque_pos = torque_pos_result.value();

    abc<current_pu_t> currents_negative_q{current_pu_t{0.0f}, current_pu_t{-0.5f}, current_pu_t{0.5f}};
    auto torque_neg_result =
        machine.update(currents_negative_q, voltage_pu_t{0.0f}, speed_pu_t{0.0f}, angle_wrapped{0.0});
    auto torque_neg = torque_neg_result.value();

    EXPECT_TRUE(torque_pos * torque_neg <= 0.0f);  // sign change or zero
}
