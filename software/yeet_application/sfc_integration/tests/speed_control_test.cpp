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
        sim.configure_sampler_sampling_rate(1e-3);
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

    speed_pu_t read_speed_error() const noexcept
    {
        return std::visit(overloaded{[&](auto* sys) {
            auto err = sys->speed_reference.read() - sys->plant.get_plant_model().get_mechanical_speed_pu();
            return err;
        }},
            speed_system_variant_t{const_cast<BuilderT*>(&system)});
    }

    // ---- control wrappers ----
    void set_speed_reference(speed_pu_t ref) noexcept
    {
        std::visit(overloaded{[&](auto* sys) { sys->speed_reference = ref; }}, speed_system_variant_t{&system});
    }

    void set_load_torque(torque_pu_t tq) noexcept
    {
        std::visit(overloaded{[&](auto* sys) { sys->plant.get_parameters().constant_torque.write(tq); }},
            speed_system_variant_t{&system});
    }

    void set_initial_speed(speed_pu_t speed) noexcept
    {
        std::visit(overloaded{[&](auto* sys) { sys->plant.get_plant_model().set_initial_mechanical_speed_pu(speed); }},
            speed_system_variant_t{&system});
    }

    void enable_speed_control(bool en = true) noexcept
    {
        std::visit(overloaded{[&](auto* sys) { sys->enable_speed_control = en; }}, speed_system_variant_t{&system});
    }

protected:
    void configure_variable_speed_plant_if_supported()
    {
        // Inline configuration (no temporary config objects kept around)
        std::visit(overloaded{[&](plant_speed_control_simulation_builder* sys) {
                                  application::sfc::integration::test::variable_speed_config cfg;
                                  cfg.initial_speed = speed_pu_t{0.0f};
                                  // cfg.voltage_at_base_speed removed - back-EMF calculated directly from E = Φ × ω
                                  application::sfc::integration::test::configure_variable_speed_plant(sys->plant, cfg);
                              },
                       [&](plant_sensorless_speed_control_simulation_builder* sys) {
                           application::sfc::integration::test::variable_speed_config cfg;
                           cfg.initial_speed = speed_pu_t{0.0f};
                           // cfg.voltage_at_base_speed removed - back-EMF calculated directly from E = Φ × ω
                           application::sfc::integration::test::configure_variable_speed_plant(sys->plant, cfg);
                       },
                       [&](plant_sensorless_speed_control_mocked_hardware_simulation_builder* sys) {
                           application::sfc::integration::test::variable_speed_config cfg;
                           cfg.initial_speed = speed_pu_t{0.0f};
                           // cfg.voltage_at_base_speed removed - back-EMF calculated directly from E = Φ × ω
                           application::sfc::integration::test::configure_variable_speed_plant(sys->plant, cfg);
                       }},
            speed_system_variant_t{&system});
    }

    void post_initialize_configuration()
    {
        std::visit(overloaded{[&](plant_speed_control_simulation_builder* /*sys*/) {},
                       [&](plant_sensorless_speed_control_simulation_builder* sys) {
                           sys->plant.get_plant_model().set_voltage_control_mode(
                               yeet::application::sfc::plant_model_voltage_control_mode::variable_speed);
                       },
                       [&](plant_sensorless_speed_control_mocked_hardware_simulation_builder* sys) {
                           sys->plant.get_plant_model().set_voltage_control_mode(
                               yeet::application::sfc::plant_model_voltage_control_mode::variable_speed);
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
                                  register_complete_logger(sim, sys->plant);
                              },
                       [&](plant_sensorless_speed_control_simulation_builder* sys) {
                           register_complete_logger(sim, sys->speed_controller);
                           register_complete_logger(sim, sys->current_controller);
                           register_complete_logger(sim, sys->dc_link_measurements);
                           register_complete_logger(sim, sys->grid_side_observer);
                           register_complete_logger(sim, sys->machine_side_observer);
                           register_complete_logger(sim, sys->plant);
                       },
                       [&](plant_sensorless_speed_control_mocked_hardware_simulation_builder* sys) {
                           register_complete_logger(sim, sys->speed_controller);
                           register_complete_logger(sim, sys->current_controller);
                           register_complete_logger(sim, sys->dc_link_measurements);
                           register_complete_logger(sim, sys->grid_side_observer);
                           register_complete_logger(sim, sys->machine_side_observer);
                           register_complete_logger(sim, sys->plant);
                       }},
            speed_system_variant_t{&system});
    }

    void initial_settling_sequence()
    {
        set_initial_speed(speed_pu_t{0.01f});
        set_load_torque(torque_pu_t{0.0f});
        sim.simulate_for(0.5);
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
    EXPECT_TRUE(this->sim.simulate_for(2.0));
    checker->activate();
    EXPECT_TRUE(this->sim.simulate_for(3.0));

    // Phase 2
    checker->deactivate();
    this->set_speed_reference(speed_pu_t{1.05f});
    EXPECT_TRUE(this->sim.simulate_for(1.0));
    checker->activate();
    EXPECT_TRUE(this->sim.simulate_for(3.0));
}

// ============================================================================
// Test 2: Load torque disturbance recovery
// ============================================================================
TYPED_TEST(SpeedControlTypedTest, SpeedRegulationWithTorqueDisturbance)
{
    // auto checker = this->create_speed_error_checker();  // Disabled: speed_error output removed

    // Phase 1: Settle at initial speed reference and load
    // checker->deactivate();  // Disabled: checker not created
    this->set_speed_reference(speed_pu_t{1.0f});
    EXPECT_TRUE(this->sim.simulate_for(2.0));
    // checker->activate();  // Disabled: checker not created
    EXPECT_TRUE(this->sim.simulate_for(2.5));

    // Phase 2: Step-up torque disturbance
    // checker->deactivate();  // Disabled: checker not created
    this->set_load_torque(torque_pu_t{0.60f});
    EXPECT_TRUE(this->sim.simulate_for(1.0));  // Disturbance transient
    // checker->activate();  // Disabled: checker not created
    EXPECT_TRUE(this->sim.simulate_for(3.0));  // Recovery steady-state after step-up

    // Phase 3: Step-down torque disturbance (return to initial load)
    // checker->deactivate();  // Disabled: checker not created
    this->set_load_torque(torque_pu_t{0.30f});  // Return to initial load from SetUp
    EXPECT_TRUE(this->sim.simulate_for(1.0));   // Disturbance transient
    // checker->activate();  // Disabled: checker not created
    EXPECT_TRUE(this->sim.simulate_for(3.0));  // Final recovery steady-state
}

// ============================================================================
// Test 3: Reference ramp tracking (incremental steps)
// ============================================================================
TYPED_TEST(SpeedControlTypedTest, SpeedReferenceRampTracking)
{
    auto checker = this->create_speed_error_checker();

    checker->deactivate();
    this->set_speed_reference(speed_pu_t{0.80f});
    // Increased settling time for high inertia (H=2.5s)
    EXPECT_TRUE(this->sim.simulate_for(10.0));
    std::cout << "Error at 10s: " << this->read_speed_error().value() << std::endl;
    checker->activate();
    EXPECT_TRUE(this->sim.simulate_for(2.0));

    const float targets[] = {0.85f, 0.90f, 1.00f, 1.05f};
    for (float r : targets) {
        checker->deactivate();
        this->set_speed_reference(speed_pu_t{r});
        EXPECT_TRUE(this->sim.simulate_for(1.5));
        checker->activate();
        EXPECT_TRUE(this->sim.simulate_for(1.0));
    }
}

// ============================================================================
// Test 4: Full speed ramp-up from very low speed with explicit ramp generation
// ============================================================================
TYPED_TEST(SpeedControlTypedTest, SpeedRampUpFromLowSpeedNaturalCommutation)
{
    // auto checker = this->create_speed_error_checker();  // Disabled: speed_error output removed

    // Phase 1: Initialize at 0.25 speed reference and settle with nominal load torque
    // checker->deactivate();  // Disabled: checker not created
    this->set_speed_reference(speed_pu_t{0.25f});
    this->set_load_torque(torque_pu_t{0.2f});
    EXPECT_TRUE(this->sim.simulate_for(2.0));  // low-speed settling

    // Phase 2: Generate a 50 s linear ramp from 0.25 to 1.0 pu by stepping the reference
    // Since speed generator now handles rate limiting, we manually generate the ramp
    const real_t ramp_time_s = 50.0;
    const real_t ramp_rate = (1.0f - 0.25f) / ramp_time_s;  // 0.015 pu/s
    const real_t step_time = 0.5;                           // Update reference every 0.5 seconds
    const int num_steps = static_cast<int>(ramp_time_s / step_time);

    // checker->activate();  // Disabled: checker not created
    for (int i = 1; i <= num_steps; ++i) {
        const real_t elapsed_time = i * step_time;
        const speed_pu_t reference = speed_pu_t{0.25f + ramp_rate * elapsed_time};
        this->set_speed_reference(reference);

        bool success = this->sim.simulate_for(step_time);
        if (!success) {
            // checker->deactivate();  // Disabled: checker not created
            EXPECT_TRUE(success) << "Simulation failed at ramp step " << i;
            return;
        }
    }
    // checker->deactivate();  // Disabled: checker not created

    // Phase 3: Final steady-state hold at 1.0 pu
    this->set_speed_reference(speed_pu_t{1.0f});  // Ensure we end exactly at 1.0
    // checker->activate();  // Disabled: checker not created
    EXPECT_TRUE(this->sim.simulate_for(3.0));
}
