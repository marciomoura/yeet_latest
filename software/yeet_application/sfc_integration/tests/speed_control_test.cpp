// Unified typed speed‑control test (no per‑type traits; identical criteria for all builders)
#include <gtest/gtest.h>

#include <variant>

#include "sfc_integration/plant_sensorless_speed_control_mocked_hardware_simulation_builder.hpp"
#include "sfc_integration/plant_sensorless_speed_control_simulation_builder.hpp"
#include "sfc_integration/plant_speed_control_simulation_builder.hpp"
#include "sfc_loggers.hpp"
#include "sfc_plant_configuration.hpp"
#include "yeet_control/units.hpp"
#include "yeet_testing/simulator_checker.hpp"
#include "yeet_testing/simulator_gtest.hpp"

using yeet::application::sfc::integration::plant_sensorless_speed_control_mocked_hardware_simulation_builder;
using yeet::application::sfc::integration::plant_sensorless_speed_control_simulation_builder;
using yeet::application::sfc::integration::plant_speed_control_simulation_builder;
using namespace yeet;

// ---------- helper ----------
template <class... Ts>
struct overloaded : Ts... {
    using Ts::operator()...;
};
template <class... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

using speed_system_variant_t = std::variant<plant_speed_control_simulation_builder*,
    plant_sensorless_speed_control_simulation_builder*,
    plant_sensorless_speed_control_mocked_hardware_simulation_builder*>;

// ---------- typed fixture ----------
template <typename BuilderT>
class SpeedControlTypedTest : public ::testing::Test {
public:
    void SetUp() override
    {
        // Direct literal for sampler period
        sim.configure_sampler_sampling_rate(5.0e-4);
        system.register_applications(sim);
        configure_variable_speed_plant_if_supported();
        system.initialize();
        post_initialize_configuration();
        register_loggers();
        initial_settling_sequence();
    }

    // Make speed‑error margin configurable (default ±0.05 pu)
    std::shared_ptr<checker_user_activation> create_speed_error_checker(speed_pu_t margin = speed_pu_t{0.05f})
    {
        return sim.register_checker(yeet::make_range_checker<speed_pu_t>(
            "speed error checker", [&]() -> speed_pu_t { return read_speed_error(); }, speed_pu_t{-margin.value()},
            speed_pu_t{+margin.value()}, yeet::checker_activation_always{}));
    }

    // ---- control wrappers ----
    void set_speed_reference(speed_pu_t ref) noexcept
    {
        std::visit(overloaded{[&](auto* sys) { sys->speed_reference_pu = ref; }}, speed_system_variant_t{&system});
    }

    void set_load_torque(torque_pu_t tq) noexcept
    {
        std::visit(overloaded{[&](auto* sys) { sys->load_torque_pu = tq; }}, speed_system_variant_t{&system});
    }

    void enable_speed_control(bool en = true) noexcept
    {
        std::visit(overloaded{[&](auto* sys) { sys->enable_speed_control = en; }}, speed_system_variant_t{&system});
    }

    // Added: configure speed reference rate limiter (pu/s)
    void set_speed_reference_rate_limit(real_t rate_limit) noexcept
    {
        std::visit(overloaded{[&](auto* sys) {
            sys->speed_controller.get_parameters().speed_reference_rate_limit.write(rate_limit);
        }},
            speed_system_variant_t{&system});
    }

    speed_pu_t read_speed_error() const noexcept
    {
        return std::visit(
            overloaded{[&](auto const* sys) { return sys->speed_controller.get_outputs().speed_error.read(); }},
            speed_system_variant_t{const_cast<BuilderT*>(&system)});
    }

protected:
    void configure_variable_speed_plant_if_supported()
    {
        // Inline configuration (no temporary config objects kept around)
        std::visit(overloaded{[&](plant_speed_control_simulation_builder* sys) {
                                  application::sfc::integration::test::variable_speed_config cfg;
                                  cfg.initial_speed = speed_pu_t{0.0f};
                                  cfg.voltage_at_base_speed = 0.9f;
                                  application::sfc::integration::test::configure_variable_speed_plant(sys->plant, cfg);
                              },
                       [&](plant_sensorless_speed_control_simulation_builder* sys) {
                           application::sfc::integration::test::variable_speed_config cfg;
                           cfg.initial_speed = speed_pu_t{0.0f};
                           cfg.voltage_at_base_speed = 0.90f;
                           application::sfc::integration::test::configure_variable_speed_plant(sys->plant, cfg);
                       },
                       [&](plant_sensorless_speed_control_mocked_hardware_simulation_builder* sys) {
                           application::sfc::integration::test::variable_speed_config cfg;
                           cfg.initial_speed = speed_pu_t{0.0f};
                           cfg.voltage_at_base_speed = 0.90f;
                           application::sfc::integration::test::configure_variable_speed_plant(sys->plant, cfg);
                       }},
            speed_system_variant_t{&system});
    }

    void post_initialize_configuration()
    {
        std::visit(overloaded{[&](plant_speed_control_simulation_builder* /*sys*/) {},
                       [&](plant_sensorless_speed_control_simulation_builder* sys) {
                           sys->speed_controller.get_parameters().speed_reference_rate_limit.write(2.0);
                           sys->plant.get_plant_model().set_voltage_control_mode(
                               yeet::application::sfc::plant_model_voltage_control_mode::speed_dependent);
                       },
                       [&](plant_sensorless_speed_control_mocked_hardware_simulation_builder* sys) {
                           sys->speed_controller.get_parameters().speed_reference_rate_limit.write(2.0);
                           sys->plant.get_plant_model().set_voltage_control_mode(
                               yeet::application::sfc::plant_model_voltage_control_mode::speed_dependent);
                       }},
            speed_system_variant_t{&system});
    }

    void register_loggers()
    {
        std::visit(overloaded{[&](plant_speed_control_simulation_builder* sys) {
                                  register_complete_logger(sim, sys->speed_controller);
                                  register_complete_logger(sim, sys->current_controller);
                                  register_complete_logger(sim, sys->machine_side_controller);
                                  register_complete_logger(sim, sys->dc_link_measurements);
                              },
                       [&](plant_sensorless_speed_control_simulation_builder* sys) {
                           register_complete_logger(sim, sys->speed_controller);
                           register_complete_logger(sim, sys->current_controller);
                           register_complete_logger(sim, sys->dc_link_measurements);
                           register_complete_logger(sim, sys->grid_side_observer);
                       },
                       [&](plant_sensorless_speed_control_mocked_hardware_simulation_builder* sys) {
                           register_complete_logger(sim, sys->speed_controller);
                           register_complete_logger(sim, sys->current_controller);
                           register_complete_logger(sim, sys->dc_link_measurements);
                           register_complete_logger(sim, sys->grid_side_observer);
                           register_complete_logger(sim, sys->machine_side_observer);
                       }},
            speed_system_variant_t{&system});
    }

    void initial_settling_sequence()
    {
        sim.simulate_for(0.1);
        enable_speed_control(false);
        set_load_torque(torque_pu_t{0.3f});
        enable_speed_control(true);
    }

protected:
    BuilderT system;
    simulator sim = yeet::make_gtest_simulator();
};

// ---- Name generator (builders provide static constexpr name internally) ----
struct SpeedTypedTestNameGenerator {
    template <typename T>
    static std::string GetName(int)
    {
        return std::string{T::name.data()};
    }
};

using SpeedTestedBuilders = ::testing::Types<plant_speed_control_simulation_builder,
    plant_sensorless_speed_control_simulation_builder,
    plant_sensorless_speed_control_mocked_hardware_simulation_builder>;

TYPED_TEST_SUITE(SpeedControlTypedTest, SpeedTestedBuilders, SpeedTypedTestNameGenerator);

// ============================================================================
// Test 1: Two sequential speed references with checker gating
// ============================================================================
TYPED_TEST(SpeedControlTypedTest, SpeedRegulation)
{
    auto checker = this->create_speed_error_checker();

    checker->deactivate();
    this->set_speed_reference(speed_pu_t{1.0f});
    ASSERT_TRUE(this->sim.simulate_for(2.0));
    checker->activate();
    ASSERT_TRUE(this->sim.simulate_for(3.0));

    // Phase 2
    checker->deactivate();
    this->set_speed_reference(speed_pu_t{1.05f});
    ASSERT_TRUE(this->sim.simulate_for(1.0));
    checker->activate();
    ASSERT_TRUE(this->sim.simulate_for(3.0));
}

// ============================================================================
// Test 2: Load torque disturbance recovery
// ============================================================================
TYPED_TEST(SpeedControlTypedTest, SpeedRegulationWithTorqueDisturbance)
{
    auto checker = this->create_speed_error_checker();

    // Phase 1: Settle at initial speed reference and load
    checker->deactivate();
    this->set_speed_reference(speed_pu_t{1.0f});
    ASSERT_TRUE(this->sim.simulate_for(2.0));
    checker->activate();
    ASSERT_TRUE(this->sim.simulate_for(2.5));

    // Phase 2: Step-up torque disturbance
    checker->deactivate();
    this->set_load_torque(torque_pu_t{0.60f});
    ASSERT_TRUE(this->sim.simulate_for(1.0));  // Disturbance transient
    checker->activate();
    ASSERT_TRUE(this->sim.simulate_for(3.0));  // Recovery steady-state after step-up

    // Phase 3: Step-down torque disturbance (return to initial load)
    checker->deactivate();
    this->set_load_torque(torque_pu_t{0.30f});  // Return to initial load from SetUp
    ASSERT_TRUE(this->sim.simulate_for(1.0));   // Disturbance transient
    checker->activate();
    ASSERT_TRUE(this->sim.simulate_for(3.0));  // Final recovery steady-state
}

// ============================================================================
// Test 3: Reference ramp tracking (incremental steps)
// ============================================================================
TYPED_TEST(SpeedControlTypedTest, SpeedReferenceRampTracking)
{
    auto checker = this->create_speed_error_checker();

    checker->deactivate();
    this->set_speed_reference(speed_pu_t{0.80f});
    ASSERT_TRUE(this->sim.simulate_for(1.5));
    checker->activate();
    ASSERT_TRUE(this->sim.simulate_for(1.0));

    const float targets[] = {0.85f, 0.90f, 1.00f, 1.05f};
    for (float r : targets) {
        checker->deactivate();
        this->set_speed_reference(speed_pu_t{r});
        ASSERT_TRUE(this->sim.simulate_for(0.5));
        checker->activate();
        ASSERT_TRUE(this->sim.simulate_for(0.8));
    }
}

// ============================================================================
// Test 4: Full speed ramp-up from very low speed using rate limiter
// ============================================================================
TYPED_TEST(SpeedControlTypedTest, SpeedRampUpFromLowSpeedNaturalCommutation)
{
    auto checker = this->create_speed_error_checker();

    // Phase 1: Initialize at 0.2 speed reference and settle with nominal load torque
    checker->deactivate();
    this->set_speed_reference(speed_pu_t{0.25f});
    this->set_load_torque(torque_pu_t{0.2f});
    ASSERT_TRUE(this->sim.simulate_for(2.0));  // low-speed settling

    // Phase 2: Configure a 50 s linear ramp to 1.0 pu via rate limiter
    const real_t ramp_time_s = 50.0;
    this->set_speed_reference_rate_limit(real_t{1.0} / ramp_time_s);  // ~0.02 pu/s
    this->set_speed_reference(speed_pu_t{1.0f});

    // Monitor entire ramp (slow ramp keeps error within ±0.05 pu band)
    checker->activate();
    bool success = this->sim.simulate_for(ramp_time_s);
    checker->deactivate();
    ASSERT_TRUE(success);

    // Phase 3: Final steady-state hold at 1.0 pu
    checker->activate();
    ASSERT_TRUE(this->sim.simulate_for(3.0));
}
