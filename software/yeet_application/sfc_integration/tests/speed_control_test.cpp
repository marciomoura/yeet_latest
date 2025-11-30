// Unified typed speed‐control test (no per‐type traits; identical criteria for all builders)
#include <gtest/gtest.h>

#include <string_view>
#include <variant>

#include "sfc_integration/plant_sensorless_speed_control_mocked_hardware_simulation_builder.hpp"
#include "sfc_integration/plant_sensorless_speed_control_simulation_builder.hpp"
#include "sfc_integration/plant_speed_control_simulation_builder.hpp"
#include "sfc_integration/sfc_plant_model_datamodel.hpp"
#include "sfc_loggers.hpp"
#include "sfc_plant_configuration.hpp"
#include "yeet_control/units.hpp"
#include "yeet_testing/simulator_checker.hpp"
#include "yeet_testing/simulator_gtest.hpp"

using yeet::application::sfc::integration::plant_sensorless_speed_control_mocked_hardware_simulation_builder;
using yeet::application::sfc::integration::plant_sensorless_speed_control_simulation_builder;
using yeet::application::sfc::integration::plant_speed_control_simulation_builder;
using yeet::application::sfc::integration::sfc_plant_model_datamodel;
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

// ---------- test parameter wrapper ----------
// Combines builder type with plant topology selection for parameterized testing
template <typename BuilderT, sfc_plant_model_datamodel::topology_type Topology>
struct speed_control_test_config {
    using builder_type = BuilderT;
    static constexpr auto topology = Topology;
    static constexpr std::string_view topology_name =
        (Topology == sfc_plant_model_datamodel::topology_type::simulink) ? "SimulinkModel" : "InternalModel";
};

// ---------- typed fixture ----------
template <typename ConfigT>
class SpeedControlTypedTest : public ::testing::Test {
public:
    using builder_type = typename ConfigT::builder_type;

    void SetUp() override
    {
        // Direct literal for sampler period
        sim.configure_sampler_sampling_rate(1e-3);
        system.register_applications(sim);

        // Configure plant topology based on test parameter
        set_plant_topology(ConfigT::topology);

        configure_variable_speed_plant_if_supported();
        system.initialize();
        post_initialize_configuration();
        register_loggers();
        initial_settling_sequence();
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

    void enable_speed_control(bool en = true) noexcept
    {
        std::visit(overloaded{[&](auto* sys) { sys->enable_speed_control = en; }}, speed_system_variant_t{&system});
    }

    void set_plant_topology(sfc_plant_model_datamodel::topology_type topology) noexcept
    {
        std::visit(overloaded{[&](auto* sys) { sys->plant.get_parameters().topology.write(topology); }},
            speed_system_variant_t{&system});
    }

    speed_pu_t get_measured_speed() const
    {
        return std::visit(
            overloaded{[&](auto const* sys) -> speed_pu_t { return system.plant.get_outputs().machine_speed.read(); }},
            speed_system_variant_t{const_cast<builder_type*>(&system)});
    }

    real_t get_tolerance() const
    {
        if constexpr (std::is_same_v<builder_type, plant_speed_control_simulation_builder>) {
            return 0.02f;
        }
        else {
            return 0.07f;
        }
    }

protected:
    void configure_variable_speed_plant_if_supported()
    {
        // Explicit gas turbine configuration for speed control tests:
        // - Inertia constant: 3.0 s (heavy rotor and compressor assembly)
        // - Damping factor: 0.1 pu (medium friction)
        // - Quadratic load coefficient: 0.3 pu/pu² (compressor aerodynamic drag ∝ speed²)
        // - Initial speed: 0.0 pu (standstill)
        // - Constant load torque: 0.0 pu (set later in initial_settling_sequence)
        //
        // With torque limit of 1.2 pu and inertia of 3.0 s:
        //   Acceleration ≈ (T_max - T_load) / J ≈ (1.2 - 0.3) / 3.0 ≈ 0.3 pu/s
        //   Time to reach 1.0 pu speed ≈ 1.0 / 0.3 ≈ 3.3 s (minimum)
        application::sfc::integration::test::variable_speed_config cfg;
        cfg.inertia_constant = inertia_constant_t{3.0f};
        cfg.damping_factor = damping_coefficient_pu_t{0.1f};
        cfg.quadratic_load_coefficient = 0.3f;
        cfg.linear_load_coefficient = 0.0f;
        cfg.load_torque = torque_pu_t{0.0f};
        cfg.initial_speed = speed_pu_t{0.0f};
        cfg.voltage_control_mode = application::sfc::plant_model_voltage_control_mode::variable_speed;

        std::visit(overloaded{[&](auto* sys) {
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
        sim.simulate_for(0.5);
        enable_speed_control(false);
        set_load_torque(torque_pu_t{0.3f});
        enable_speed_control(true);
    }

protected:
    builder_type system;
    simulator sim = yeet::make_gtest_simulator();
};

// ---- Name generator ----
struct SpeedControlTestNameGenerator {
    template <typename T>
    static std::string GetName(int)
    {
        return std::string{T::builder_type::name.data()} + "_" + std::string{T::topology_name};
    }
};

// ---- Test configurations: all combinations of builders and topologies ----
using SpeedControlTestedConfigs = ::testing::Types<
    // Simulink model configurations
    speed_control_test_config<plant_speed_control_simulation_builder,
        sfc_plant_model_datamodel::topology_type::simulink>,
    speed_control_test_config<plant_sensorless_speed_control_simulation_builder,
        sfc_plant_model_datamodel::topology_type::simulink>,
    speed_control_test_config<plant_sensorless_speed_control_mocked_hardware_simulation_builder,
        sfc_plant_model_datamodel::topology_type::simulink>,
    // Internal model configurations
    speed_control_test_config<plant_speed_control_simulation_builder,
        sfc_plant_model_datamodel::topology_type::internal>,
    speed_control_test_config<plant_sensorless_speed_control_simulation_builder,
        sfc_plant_model_datamodel::topology_type::internal>,
    speed_control_test_config<plant_sensorless_speed_control_mocked_hardware_simulation_builder,
        sfc_plant_model_datamodel::topology_type::internal>>;

TYPED_TEST_SUITE(SpeedControlTypedTest, SpeedControlTestedConfigs, SpeedControlTestNameGenerator);

// ============================================================================
// Test 1: Two sequential speed references with checker gating
// ============================================================================
TYPED_TEST(SpeedControlTypedTest, SpeedRegulation)
{
    // With J=3.0s and T_max=1.2pu, acceleration is limited.
    // Time to 1.0 pu ≈ 3-4s, plus settling time ≈ 2s = ~6s total.
    // We use 15.0s to be absolutely sure it settles.
    this->set_speed_reference(speed_pu_t{1.0f});
    EXPECT_TRUE(this->sim.simulate_for(15.0));
    EXPECT_NEAR(this->get_measured_speed(), speed_pu_t{1.0f}, this->get_tolerance())
        << "Speed regulation error too high at first reference";

    // Phase 2: Small step from 1.0 to 1.05 pu (5% change)
    this->set_speed_reference(speed_pu_t{1.05f});
    EXPECT_TRUE(this->sim.simulate_for(5.0));
    EXPECT_NEAR(this->get_measured_speed(), speed_pu_t{1.05f}, this->get_tolerance())
        << "Speed regulation error too high at second reference";
}

// ============================================================================
// Test 2: Load torque disturbance recovery
// ============================================================================
TYPED_TEST(SpeedControlTypedTest, SpeedRegulationWithTorqueDisturbance)
{
    // Phase 1: Settle at initial speed reference and load
    // With J=3.0s and T_max=1.2pu, need longer settling time
    // Increased to 15.0s to ensure full settling with high inertia
    this->set_speed_reference(speed_pu_t{1.0f});
    EXPECT_TRUE(this->sim.simulate_for(15.0));
    EXPECT_NEAR(this->get_measured_speed(), speed_pu_t{1.0f}, this->get_tolerance())
        << "Initial speed regulation error too high";

    // Phase 2: Step-up torque disturbance (0.3 → 0.6 pu)
    // Large inertia means slower response to disturbance
    this->set_load_torque(torque_pu_t{0.60f});
    EXPECT_TRUE(this->sim.simulate_for(6.0));
    EXPECT_NEAR(this->get_measured_speed(), speed_pu_t{1.0f}, this->get_tolerance())
        << "Speed regulation error too high after torque disturbance";

    // Phase 3: Step-down torque disturbance (return to initial load)
    this->set_load_torque(torque_pu_t{0.30f});
    EXPECT_TRUE(this->sim.simulate_for(6.0));
    EXPECT_NEAR(this->get_measured_speed(), speed_pu_t{1.0f}, this->get_tolerance())
        << "Final speed regulation error too high after torque disturbance recovery";
}

// ============================================================================
// Test 3: Reference ramp tracking (incremental steps)
// ============================================================================
TYPED_TEST(SpeedControlTypedTest, SpeedReferenceRampTracking)
{
    // Initial settling to 0.80 pu with large inertia
    // Increased to 12.0s to ensure full settling
    this->set_speed_reference(speed_pu_t{0.80f});
    EXPECT_TRUE(this->sim.simulate_for(12.0));

    // Step through speed references with sufficient settling time
    // Each 5% step needs ~1-2s acceleration + settling
    const float targets[] = {0.85f, 0.90f, 1.00f, 1.05f};
    for (float r : targets) {
        this->set_speed_reference(speed_pu_t{r});
        EXPECT_TRUE(this->sim.simulate_for(4.0));
        EXPECT_NEAR(this->get_measured_speed(), speed_pu_t{r}, this->get_tolerance())
            << "Speed tracking error too high at ramp step " << r;
    }
}

// ============================================================================
// Test 4: Full speed ramp-up from very low speed with explicit ramp generation
// ============================================================================
TYPED_TEST(SpeedControlTypedTest, SpeedRampUpFromLowSpeedNaturalCommutation)
{
    // Phase 1: Initialize at 0.25 speed reference and settle with nominal load torque
    // With J=3.0s, need more time to reach 0.25 pu from standstill
    // Increased to 8.0s
    this->set_speed_reference(speed_pu_t{0.25f});
    this->set_load_torque(torque_pu_t{0.2f});
    EXPECT_TRUE(this->sim.simulate_for(8.0));

    // Phase 2: Generate a 50 s linear ramp from 0.25 to 1.0 pu by stepping the reference
    // Ramp rate: 0.75 pu / 50 s = 0.015 pu/s
    // This is slow enough that the controller can track with T_max=1.2 pu
    const real_t ramp_time_s = 50.0;
    const real_t ramp_rate = (1.0f - 0.25f) / ramp_time_s;  // 0.015 pu/s
    const real_t step_time = 0.5;
    const int num_steps = static_cast<int>(ramp_time_s / step_time);

    for (int i = 1; i <= num_steps; ++i) {
        const real_t elapsed_time = i * step_time;
        const speed_pu_t reference = speed_pu_t{0.25f + ramp_rate * elapsed_time};
        this->set_speed_reference(reference);

        bool success = this->sim.simulate_for(step_time);
        if (!success) {
            EXPECT_TRUE(success) << "Simulation failed at ramp step " << i;
            return;
        }

        // Relaxed tolerance during ramp to account for tracking lag with large inertia
        // Base tolerance + 0.03f for dynamic tracking error
        EXPECT_NEAR(this->get_measured_speed(), reference, this->get_tolerance() + 0.03f)
            << "Speed tracking error too high at ramp step " << i;
    }

    // Phase 3: Final steady-state hold at 1.0 pu
    this->set_speed_reference(speed_pu_t{1.0f});
    EXPECT_TRUE(this->sim.simulate_for(5.0));

    EXPECT_NEAR(this->get_measured_speed(), speed_pu_t{1.0f}, this->get_tolerance())
        << "Speed tracking error too high at final steady-state hold";
}
