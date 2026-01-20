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

    voltage_pu_t get_excitation_voltage_reference() const
    {
        return std::visit(
            overloaded{[&](plant_speed_control_simulation_builder const* sys) -> voltage_pu_t {
                           return sys->excitation_controller.get_outputs().machine_voltage_reference_pu.read();
                       },
                [&](auto const* /*sys*/) -> voltage_pu_t { return voltage_pu_t{0.0}; }},
            speed_system_variant_t{const_cast<builder_type*>(&system)});
    }

    voltage_pu_t get_excitation_voltage_measured() const
    {
        return std::visit(
            overloaded{[&](plant_speed_control_simulation_builder const* sys) -> voltage_pu_t {
                           return sys->excitation_controller.get_outputs().machine_voltage_magnitude_pu.read();
                       },
                [&](auto const* /*sys*/) -> voltage_pu_t { return voltage_pu_t{0.0}; }},
            speed_system_variant_t{const_cast<builder_type*>(&system)});
    }

    voltage_pu_t get_excitation_voltage_error() const
    {
        return std::visit(
            overloaded{[&](plant_speed_control_simulation_builder const* sys) -> voltage_pu_t {
                           return sys->excitation_controller.get_outputs().machine_voltage_error_pu.read();
                       },
                [&](auto const* /*sys*/) -> voltage_pu_t { return voltage_pu_t{0.0}; }},
            speed_system_variant_t{const_cast<builder_type*>(&system)});
    }

    current_pu_t get_field_current_reference() const
    {
        return std::visit(overloaded{[&](plant_speed_control_simulation_builder const* sys) -> current_pu_t {
                                         return sys->excitation_controller.get_outputs().field_current_reference.read();
                                     },
                              [&](auto const* /*sys*/) -> current_pu_t { return current_pu_t{0.0}; }},
            speed_system_variant_t{const_cast<builder_type*>(&system)});
    }

    current_pu_t get_actual_field_current_from_plant() const
    {
        return std::visit(overloaded{[&](auto const* sys) -> current_pu_t {
            return system.plant.get_outputs().field_current_pu.read();
        }},
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
        // Make the plant's runtime behavior unambiguous:
        // - Plant runner gating uses datamodel parameters.
        // - Plant physics uses the underlying plant model's internal modes.
        // We set both here (after initialize, callbacks are registered).
        std::visit(overloaded{[&](auto* sys) {
            sys->plant.get_parameters().operation_mode.write(
                sfc_plant_model_datamodel::machine_operation_mode::variable_speed);
            sys->plant.get_parameters().voltage_control_mode.write(
                sfc_plant_model_datamodel::machine_voltage_control_mode::variable_speed);
            sys->plant.get_plant_model().set_operation_mode(
                yeet::application::sfc::plant_model_machine_operation_mode::variable_speed);
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
                                  register_complete_logger(sim, sys->excitation_controller);
                                  register_complete_logger(sim, sys->machine_side_controller);
                                  register_complete_logger(sim, sys->dc_link_measurements);
                                  register_complete_logger(sim, sys->torque_controller);
                                  register_complete_logger(sim, sys->plant);
                              },
                       [&](plant_sensorless_speed_control_simulation_builder* sys) {
                           register_complete_logger(sim, sys->speed_controller);
                           register_complete_logger(sim, sys->current_controller);
                           register_complete_logger(sim, sys->excitation_controller);
                           register_complete_logger(sim, sys->dc_link_measurements);
                           register_complete_logger(sim, sys->grid_side_observer);
                           register_complete_logger(sim, sys->machine_side_observer);
                           register_complete_logger(sim, sys->plant);
                       },
                       [&](plant_sensorless_speed_control_mocked_hardware_simulation_builder* sys) {
                           register_complete_logger(sim, sys->speed_controller);
                           register_complete_logger(sim, sys->current_controller);
                           register_complete_logger(sim, sys->excitation_controller);
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

    // Voltage control checks - only for builders with excitation controller
    if constexpr (std::is_same_v<typename TestFixture::builder_type, plant_speed_control_simulation_builder>) {
        const auto voltage_measured = this->get_excitation_voltage_measured();
        const auto voltage_reference = this->get_excitation_voltage_reference();
        const auto voltage_error = this->get_excitation_voltage_error();
        const auto field_current = this->get_field_current_reference();

        // At 1.0 pu speed, voltage reference should be 1.0 pu (default LUT)
        EXPECT_NEAR(voltage_reference.value(), 1.0f, 0.01f)
            << "Excitation voltage reference should be near 1.0 pu at rated speed";

        // Machine voltage magnitude should track the reference within 5%
        EXPECT_NEAR(voltage_measured.value(), voltage_reference.value(), 0.05f)
            << "Machine voltage measured (" << voltage_measured.value() << " pu) should track reference ("
            << voltage_reference.value() << " pu) within 5%. Voltage error: " << voltage_error.value() << " pu";

        // Voltage error should be small (closed-loop regulation)
        EXPECT_NEAR(voltage_error.value(), 0.0f, 0.05f)
            << "Excitation voltage control error should be near zero at steady state";
    }

    // Phase 2: Small step from 1.0 to 1.05 pu (5% change)
    this->set_speed_reference(speed_pu_t{1.05f});

    EXPECT_TRUE(this->sim.simulate_for(10.0));

    EXPECT_NEAR(this->get_measured_speed(), speed_pu_t{1.05f}, this->get_tolerance())
        << "Speed regulation error too high at second reference";

    // Voltage control checks - only for builders with excitation controller
    if constexpr (std::is_same_v<typename TestFixture::builder_type, plant_speed_control_simulation_builder>) {
        auto voltage_measured = this->get_excitation_voltage_measured();
        auto voltage_reference = this->get_excitation_voltage_reference();
        auto voltage_error = this->get_excitation_voltage_error();
        auto field_current = this->get_field_current_reference();

        // At 1.05 pu speed, voltage reference should be ~1.04 pu (V/Hz continues linearly: 0.2 + 0.8*1.05 = 1.04)
        EXPECT_NEAR(voltage_reference.value(), 1.04f, 0.02f)
            << "Excitation voltage reference should scale with speed to maintain constant flux";

        // Machine voltage magnitude should track the reference within 5%
        EXPECT_NEAR(voltage_measured.value(), voltage_reference.value(), 0.05f)
            << "Machine voltage measured (" << voltage_measured.value() << " pu) should track reference ("
            << voltage_reference.value() << " pu) within 5%. Voltage error: " << voltage_error.value() << " pu";

        // Voltage error should be small (closed-loop regulation)
        EXPECT_NEAR(voltage_error.value(), 0.0f, 0.05f)
            << "Excitation voltage control error should be near zero at steady state";
    }
}

// ============================================================================
// Test 1a: Speed-based torque limiting (only builders with torque controller)
// ============================================================================
TYPED_TEST(SpeedControlTypedTest, SpeedBasedTorqueLimitSlowsAcceleration)
{
    if constexpr (!std::is_same_v<typename TestFixture::builder_type, plant_speed_control_simulation_builder>) {
        GTEST_SKIP() << "This builder does not include the torque controller.";
    }
    else {
        // Configure speed-based torque max schedule to a constant 0.6 pu.
        // With the existing load model, this should noticeably slow acceleration.
        auto& tq_params = this->system.torque_controller.get_parameters();

        auto speed_axis = tq_params.speed_torque_limit_speed_axis_pu.read();
        auto min_torque = tq_params.speed_torque_limit_min_torque_pu.read();
        auto max_torque = tq_params.speed_torque_limit_max_torque_pu.read();

        for (auto& v : min_torque) {
            v = torque_pu_t{0.0f};
        }
        for (auto& v : max_torque) {
            v = torque_pu_t{0.6f};
        }

        tq_params.speed_torque_limit_speed_axis_pu.write(speed_axis);
        tq_params.speed_torque_limit_min_torque_pu.write(min_torque);
        tq_params.speed_torque_limit_max_torque_pu.write(max_torque);
        tq_params.enable_speed_based_torque_limits.write(true);

        this->set_speed_reference(speed_pu_t{1.0f});
        EXPECT_TRUE(this->sim.simulate_for(5.0));

        const auto measured_speed = this->get_measured_speed();
        EXPECT_LT(measured_speed.value(), 0.85f) << "Acceleration should be limited by 0.6 pu torque cap";

        const auto evaluated_max = this->system.torque_controller.get_outputs().speed_based_max_torque_limit.read();
        EXPECT_NEAR(evaluated_max.value(), 0.6f, 0.01f);

        const auto applied_torque = this->system.torque_controller.get_outputs().torque_reference.read();
        EXPECT_LE(applied_torque.value(), 0.6f + 0.02f) << "Applied torque must respect speed-based max";
    }
}

// ============================================================================
// Test 1b: Voltage limiting test (0.9 pu limit)
// ============================================================================
TYPED_TEST(SpeedControlTypedTest, VoltageRegulationLimit)
{
    // This test verifies that the excitation controller can regulate to a voltage below rated
    // We configure the voltage reference LUT to target 0.9 pu at 1.0 pu speed

    // Only run this test for builders with excitation controller
    if constexpr (std::is_same_v<typename TestFixture::builder_type, plant_speed_control_simulation_builder>) {
        // Configure custom voltage reference LUT: linear ramp to 0.9 pu at 1.0 pu speed
        // V = 0.2 + 0.7*speed (0.2 pu at standstill, 0.9 pu at rated speed)
        std::array<speed_pu_t, 7> speed_axis = {speed_pu_t{0.0}, speed_pu_t{0.2}, speed_pu_t{0.4}, speed_pu_t{0.6},
            speed_pu_t{0.8}, speed_pu_t{1.0}, speed_pu_t{1.1}};
        std::array<voltage_pu_t, 7> voltage_ref = {voltage_pu_t{0.2}, voltage_pu_t{0.34}, voltage_pu_t{0.48},
            voltage_pu_t{0.62}, voltage_pu_t{0.76}, voltage_pu_t{0.9}, voltage_pu_t{0.97}};

        this->system.excitation_controller.get_parameters().voltage_reference_speed_axis_pu.write(speed_axis);
        this->system.excitation_controller.get_parameters().voltage_reference_pu.write(voltage_ref);

        // Verify plant is in variable_speed voltage control mode
        auto plant_voltage_mode = this->system.plant.get_parameters().voltage_control_mode.read();
        EXPECT_EQ(plant_voltage_mode, sfc_plant_model_datamodel::machine_voltage_control_mode::variable_speed)
            << "Plant must be in variable_speed mode for excitation control to work";

        // Run to 1.0 pu speed - need longer settling time for voltage to stabilize at new reference
        this->set_speed_reference(speed_pu_t{1.0f});

        EXPECT_TRUE(this->sim.simulate_for(30.0));

        EXPECT_NEAR(this->get_measured_speed(), speed_pu_t{1.0f}, this->get_tolerance())
            << "Speed regulation error too high at 1.0 pu reference";

        // Check voltage regulation to 0.9 pu
        const auto voltage_measured = this->get_excitation_voltage_measured();
        const auto voltage_reference = this->get_excitation_voltage_reference();
        const auto voltage_error = this->get_excitation_voltage_error();
        const auto field_current = this->get_field_current_reference();

        // Voltage reference should be near 0.9 pu
        EXPECT_NEAR(voltage_reference.value(), 0.9f, 0.01f)
            << "Excitation voltage reference should be 0.9 pu (custom limit)";

        // Machine voltage should track the 0.9 pu reference
        EXPECT_NEAR(voltage_measured.value(), 0.9f, 0.05f) << "Machine voltage should regulate to 0.9 pu limit";

        // Voltage error should be small
        EXPECT_NEAR(voltage_error.value(), 0.0f, 0.05f)
            << "Voltage control error should be near zero with 0.9 pu limit";

        // Field current should be lower than rated (approximately 0.9 pu for 0.9 pu voltage)
        EXPECT_LT(field_current.value(), 0.95f) << "Field current should be reduced for 0.9 pu voltage limit";
    }
}

// ============================================================================
// Test 1c: Voltage reference saturation at 1.0 pu above rated speed
// ============================================================================
TYPED_TEST(SpeedControlTypedTest, VoltageRegulationSaturatesAtOnePuAboveRatedSpeed)
{
    // This test verifies that the excitation controller can clamp the commanded machine voltage
    // at 1.0 pu while speed increases beyond rated (e.g. 1.1 pu).

    // Only run this test for builders with excitation controller
    if constexpr (std::is_same_v<typename TestFixture::builder_type, plant_speed_control_simulation_builder>) {
        // Configure custom voltage reference LUT:
        // - Linear V/Hz up to rated speed (1.0 pu)
        // - Saturate voltage at 1.0 pu for speeds above rated
        std::array<speed_pu_t, 7> speed_axis = {speed_pu_t{0.0}, speed_pu_t{0.2}, speed_pu_t{0.4}, speed_pu_t{0.6},
            speed_pu_t{0.8}, speed_pu_t{1.0}, speed_pu_t{1.1}};
        std::array<voltage_pu_t, 7> voltage_ref = {voltage_pu_t{0.2}, voltage_pu_t{0.36}, voltage_pu_t{0.52},
            voltage_pu_t{0.68}, voltage_pu_t{0.84}, voltage_pu_t{1.0}, voltage_pu_t{1.0}};

        this->system.excitation_controller.get_parameters().voltage_reference_speed_axis_pu.write(speed_axis);
        this->system.excitation_controller.get_parameters().voltage_reference_pu.write(voltage_ref);

        // Verify plant is in variable_speed voltage control mode
        const auto plant_voltage_mode = this->system.plant.get_parameters().voltage_control_mode.read();
        EXPECT_EQ(plant_voltage_mode, sfc_plant_model_datamodel::machine_voltage_control_mode::variable_speed)
            << "Plant must be in variable_speed mode for excitation control to work";

        // Command speed beyond rated
        this->set_speed_reference(speed_pu_t{1.1f});

        // Let the speed and voltage loop settle (inertia is large; voltage is slower due to field dynamics)
        EXPECT_TRUE(this->sim.simulate_for(25.0));

        EXPECT_NEAR(this->get_measured_speed(), speed_pu_t{1.1f}, this->get_tolerance())
            << "Speed regulation error too high at 1.1 pu reference";

        const auto voltage_reference = this->get_excitation_voltage_reference();
        const auto voltage_measured = this->get_excitation_voltage_measured();
        const auto voltage_error = this->get_excitation_voltage_error();
        const auto field_current_ref = this->get_field_current_reference();

        // Voltage reference should clamp to 1.0 pu
        EXPECT_NEAR(voltage_reference.value(), 1.0f, 0.01f)
            << "Voltage reference should saturate at 1.0 pu above rated speed";

        // Measured voltage should be held near 1.0 pu (and not rise to ~1.1 pu)
        EXPECT_NEAR(voltage_measured.value(), 1.0f, 0.05f)
            << "Machine voltage should be regulated near 1.0 pu at 1.1 pu speed";
        EXPECT_LT(voltage_measured.value(), 1.05f) << "Machine voltage should remain below 1.05 pu";

        // Closed-loop regulation: error should be small
        EXPECT_NEAR(voltage_error.value(), 0.0f, 0.05f)
            << "Voltage control error should be near zero with 1.0 pu saturation";

        // To keep voltage at 1.0 pu at 1.1 pu speed, flux (and thus field current) must reduce below 1.0 pu.
        EXPECT_LT(field_current_ref.value(), 0.98f)
            << "Field current should reduce when clamping voltage above rated speed";
    }
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

// ============================================================================
// Test 5: Speed-based torque limiting with custom LUT
// ============================================================================
TYPED_TEST(SpeedControlTypedTest, SpeedBasedTorqueLimitingReducesTorqueAtHighSpeed)
{
    // Only plant_speed_control_simulation_builder has a torque_controller
    if constexpr (!std::is_same_v<typename TestFixture::builder_type, plant_speed_control_simulation_builder>) {
        GTEST_SKIP() << "This builder does not include the torque controller.";
    }
    else {
        // This test validates that the speed-based torque limiter correctly restricts
        // torque commands based on the speed reference. We configure a LUT that reduces
        // maximum torque at higher speeds to simulate field weakening or power limitations.

        // Configure speed-based torque limit LUT:
        // - At low speed (0.0 pu): full torque (1.2 pu)
        // - At mid speed (0.5 pu): moderate torque (0.8 pu)
        // - At high speed (1.0 pu): reduced torque (0.4 pu)
        std::array<speed_pu_t, 7> speed_axis = {speed_pu_t{0.0}, speed_pu_t{0.2}, speed_pu_t{0.4}, speed_pu_t{0.6},
            speed_pu_t{0.8}, speed_pu_t{1.0}, speed_pu_t{1.1}};
        std::array<torque_pu_t, 7> min_torque = {torque_pu_t{0.0}, torque_pu_t{0.0}, torque_pu_t{0.0}, torque_pu_t{0.0},
            torque_pu_t{0.0}, torque_pu_t{0.0}, torque_pu_t{0.0}};
        std::array<torque_pu_t, 7> max_torque = {torque_pu_t{1.2}, torque_pu_t{1.1}, torque_pu_t{0.9}, torque_pu_t{0.7},
            torque_pu_t{0.5}, torque_pu_t{0.4}, torque_pu_t{0.4}};

        auto& tq_params = this->system.torque_controller.get_parameters();
        tq_params.speed_torque_limit_speed_axis_pu.write(speed_axis);
        tq_params.speed_torque_limit_min_torque_pu.write(min_torque);
        tq_params.speed_torque_limit_max_torque_pu.write(max_torque);
        tq_params.enable_speed_based_torque_limits.write(true);

        // Phase 1: Accelerate to 0.5 pu speed with light load
        // At 0.5 pu speed, the LUT gives ~0.85 pu max torque
        // Total load at 0.5 pu: 0.1 + 0.3*0.25 = 0.175 pu (well within limit)
        this->set_speed_reference(speed_pu_t{0.5f});
        this->set_load_torque(torque_pu_t{0.1f});
        this->enable_speed_control(true);

        EXPECT_TRUE(this->sim.simulate_for(15.0));

        auto speed_mid = this->get_measured_speed();
        EXPECT_NEAR(speed_mid.value(), 0.5f, this->get_tolerance() + 0.03f) << "Machine should reach 0.5 pu speed";

        // Phase 2: Command higher speed (0.8 pu) where torque is limited to 0.5 pu
        // Total load at 0.8 pu: 0.25 + 0.3*0.64 = 0.442 pu (close to limit)
        this->set_speed_reference(speed_pu_t{0.8f});
        this->set_load_torque(torque_pu_t{0.25f});

        EXPECT_TRUE(this->sim.simulate_for(20.0));

        auto speed_high = this->get_measured_speed();
        EXPECT_NEAR(speed_high.value(), 0.8f, this->get_tolerance() + 0.05f)
            << "Machine should reach ~0.8 pu speed with reduced torque limit";

        // Verify torque limit is being applied
        const auto evaluated_max = this->system.torque_controller.get_outputs().speed_based_max_torque_limit.read();
        EXPECT_LT(evaluated_max.value(), 0.55f) << "Speed-based max torque should be limited at 0.8 pu speed";

        // Phase 3: Command very high speed (1.0 pu) where torque is limited to 0.4 pu
        // Total load at 1.0 pu: 0.25 + 0.3*1.0 = 0.55 pu, which exceeds the 0.4 pu limit
        // Machine cannot reach 1.0 pu with this load
        this->set_speed_reference(speed_pu_t{1.0f});

        EXPECT_TRUE(this->sim.simulate_for(30.0));

        auto speed_final = this->get_measured_speed();
        // Due to torque limit (0.4 pu) < load (0.55 pu), machine cannot reach 1.0 pu
        EXPECT_LT(speed_final.value(), 0.95f) << "Machine should not reach 1.0 pu due to insufficient torque limit";
        EXPECT_GT(speed_final.value(), 0.6f) << "Machine should stabilize above 0.6 pu";

        // Final validation: with very light load, machine should reach 1.0 pu
        // Total load at 1.0 pu: 0.05 + 0.3*1.0 = 0.35 pu (within 0.4 pu limit)
        this->set_load_torque(torque_pu_t{0.05f});
        EXPECT_TRUE(this->sim.simulate_for(20.0));

        auto speed_reduced_load = this->get_measured_speed();
        EXPECT_NEAR(speed_reduced_load.value(), 1.0f, this->get_tolerance() + 0.15f)
            << "With light load, machine should reach 1.0 pu despite torque limit";
    }
}
