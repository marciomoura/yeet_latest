/**
 * @file current_control_rectifier_test.cpp
 * @brief Tests for rectifier-only topology current control
 *
 * This test file validates the grid-side current controller operation when using
 * the rectifier topology (machine-side short-circuited with 0V back-EMF).
 *
 * Key validations:
 * - DC-link current regulation with reference steps
 * - Machine-side measurements are all zero (short-circuited)
 * - Grid-side current controller operation in isolation
 */

#include <gtest/gtest.h>

#include <cmath>
#include <string_view>
#include <variant>

#include "sfc_integration/plant_current_control_with_modulators_simulation_builder.hpp"
#include "sfc_integration/plant_sensorless_current_control_simulation_builder.hpp"
#include "sfc_integration/plant_simplified_current_control_simulation_builder.hpp"
#include "sfc_integration/sfc_plant_model_datamodel.hpp"
#include "sfc_loggers.hpp"
#include "yeet_control/units.hpp"
#include "yeet_testing/simulator_checker.hpp"
#include "yeet_testing/simulator_gtest.hpp"

using yeet::application::sfc::integration::plant_current_control_with_modulators_simulation_builder;
using yeet::application::sfc::integration::plant_sensorless_current_control_simulation_builder;
using yeet::application::sfc::integration::plant_simplified_current_control_simulation_builder;
using yeet::application::sfc::integration::sfc_plant_model_datamodel;

using namespace yeet;

// Variant type for rectifier-compatible builders
using rectifier_system_variant_t = std::variant<plant_simplified_current_control_simulation_builder*,
    plant_sensorless_current_control_simulation_builder*,
    plant_current_control_with_modulators_simulation_builder*>;

template <class... Ts>
struct overloaded : Ts... {
    using Ts::operator()...;
};
template <class... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

// ---------- Rectifier Current Control Test Fixture ----------
template <typename BuilderT>
class RectifierCurrentControlTest : public ::testing::Test {
public:
    using builder_type = BuilderT;

    void SetUp() override
    {
        sim.configure_sampler_sampling_rate(200e-6);
        system.register_applications(sim);

        // Configure rectifier topology
        system.plant.get_parameters().topology.write(sfc_plant_model_datamodel::topology_type::rectifier);

        system.initialize();
        register_common_loggers();
        initialize_simulation();
    }

    std::shared_ptr<checker_user_activation> create_current_error_checker(current_pu_t margin = current_pu_t{0.2f})
    {
        return sim.register_checker(yeet::make_range_checker<current_pu_t>(
            "current error checker", [&]() -> current_pu_t { return read_current_error(); }, -margin, margin,
            yeet::checker_activation_always{}));
    }

    std::shared_ptr<checker_user_activation> create_machine_voltage_zero_checker()
    {
        return sim.register_checker(yeet::make_range_checker<voltage_t>(
            "machine voltage zero checker", [&]() -> voltage_t { return read_machine_voltage_magnitude(); },
            voltage_t{-1.0f}, voltage_t{1.0f}, yeet::checker_activation_always{}));
    }

    void register_common_loggers()
    {
        std::visit(overloaded{[&](plant_simplified_current_control_simulation_builder* sys) {
                                  register_complete_logger(sim, sys->current_controller);
                                  register_complete_logger(sim, sys->dc_link_measurements);
                              },
                       [&](plant_sensorless_current_control_simulation_builder* sys) {
                           register_complete_logger(sim, sys->current_controller);
                           register_complete_logger(sim, sys->dc_link_measurements);
                           register_complete_logger(sim, sys->machine_side_observer);
                           register_complete_logger(sim, sys->grid_side_observer);
                       },
                       [&](plant_current_control_with_modulators_simulation_builder* sys) {
                           register_complete_logger(sim, sys->current_controller);
                           register_complete_logger(sim, sys->dc_link_measurements);
                       }},
            rectifier_system_variant_t{&system});
    }

    void initialize_simulation(double settling_time = 0.2)
    {
        // Disable grid-side pulses initially and freeze integrator
        std::visit(overloaded{[&](plant_simplified_current_control_simulation_builder* sys) {
                                  sys->current_controller_mock.freeze_integrator = true;
                                  sys->current_controller_mock.preset_request = false;
                                  sys->modulator_mock.release_pulses = false;
                                  sys->grid_side_modulator.get_parameters().enable.write(false);
                              },
                       [&](plant_sensorless_current_control_simulation_builder* sys) {
                           sys->current_controller_mock.freeze_integrator = true;
                           sys->current_controller_mock.preset_request = false;
                           sys->modulator_mock.release_pulses = false;
                           sys->grid_side_modulator.get_parameters().enable.write(false);
                       },
                       [&](plant_current_control_with_modulators_simulation_builder* sys) {
                           sys->current_controller_mock.freeze_integrator = true;
                           sys->current_controller_mock.preset_request = false;
                           sys->modulator_mock.release_pulses = false;
                           sys->grid_side_modulator.get_parameters().enable.write(false);
                       }},
            rectifier_system_variant_t{&system});

        sim.simulate_for(settling_time);

        // Enable grid-side pulses only (no machine-side pulses needed for rectifier) and unfreeze
        std::visit(overloaded{[&](plant_simplified_current_control_simulation_builder* sys) {
                                  sys->current_controller_mock.freeze_integrator = false;
                                  sys->modulator_mock.release_pulses = true;
                                  sys->grid_side_modulator.get_parameters().enable.write(true);
                              },
                       [&](plant_sensorless_current_control_simulation_builder* sys) {
                           sys->current_controller_mock.freeze_integrator = false;
                           sys->modulator_mock.release_pulses = true;
                           sys->grid_side_modulator.get_parameters().enable.write(true);
                       },
                       [&](plant_current_control_with_modulators_simulation_builder* sys) {
                           sys->current_controller_mock.freeze_integrator = false;
                           sys->modulator_mock.release_pulses = true;
                           sys->grid_side_modulator.get_parameters().enable.write(true);
                       }},
            rectifier_system_variant_t{&system});
    }

    void set_dc_link_current_reference(const current_pu_t& value) noexcept
    {
        std::visit(
            overloaded{
                [&](plant_simplified_current_control_simulation_builder* sys) { sys->current_reference = value; },
                [&](plant_sensorless_current_control_simulation_builder* sys) { sys->current_reference = value; },
                [&](plant_current_control_with_modulators_simulation_builder* sys) { sys->current_reference = value; }},
            rectifier_system_variant_t{&system});
    }

    current_pu_t read_current_error() const noexcept
    {
        return std::visit(
            overloaded{[&](plant_simplified_current_control_simulation_builder const* sys) -> current_pu_t {
                           return sys->current_controller.get_outputs().current_error.read();
                       },
                [&](plant_sensorless_current_control_simulation_builder const* sys) -> current_pu_t {
                    return sys->current_controller.get_outputs().current_error.read();
                },
                [&](plant_current_control_with_modulators_simulation_builder const* sys) -> current_pu_t {
                    return sys->current_controller.get_outputs().current_error.read();
                }},
            rectifier_system_variant_t{const_cast<builder_type*>(&system)});
    }

    voltage_t read_machine_voltage_magnitude() const noexcept
    {
        return std::visit(overloaded{[&](plant_simplified_current_control_simulation_builder const* sys) -> voltage_t {
                                         const auto& v_abc = sys->plant.get_outputs().machine_voltage_abc.read();
                                         return voltage_t{std::sqrt(v_abc.a().value() * v_abc.a().value() +
                                                                    v_abc.b().value() * v_abc.b().value() +
                                                                    v_abc.c().value() * v_abc.c().value())};
                                     },
                              [&](plant_sensorless_current_control_simulation_builder const* sys) -> voltage_t {
                                  const auto& v_abc = sys->plant.get_outputs().machine_voltage_abc.read();
                                  return voltage_t{std::sqrt(v_abc.a().value() * v_abc.a().value() +
                                                             v_abc.b().value() * v_abc.b().value() +
                                                             v_abc.c().value() * v_abc.c().value())};
                              },
                              [&](plant_current_control_with_modulators_simulation_builder const* sys) -> voltage_t {
                                  const auto& v_abc = sys->plant.get_outputs().machine_voltage_abc.read();
                                  return voltage_t{std::sqrt(v_abc.a().value() * v_abc.a().value() +
                                                             v_abc.b().value() * v_abc.b().value() +
                                                             v_abc.c().value() * v_abc.c().value())};
                              }},
            rectifier_system_variant_t{const_cast<builder_type*>(&system)});
    }

    abc<voltage_t> read_machine_voltage_abc() const noexcept
    {
        return std::visit(
            overloaded{[&](plant_simplified_current_control_simulation_builder const* sys) -> abc<voltage_t> {
                           return sys->plant.get_outputs().machine_voltage_abc.read();
                       },
                [&](plant_sensorless_current_control_simulation_builder const* sys) -> abc<voltage_t> {
                    return sys->plant.get_outputs().machine_voltage_abc.read();
                },
                [&](plant_current_control_with_modulators_simulation_builder const* sys) -> abc<voltage_t> {
                    return sys->plant.get_outputs().machine_voltage_abc.read();
                }},
            rectifier_system_variant_t{const_cast<builder_type*>(&system)});
    }

    current_t read_dc_link_current() const noexcept
    {
        return std::visit(overloaded{[&](plant_simplified_current_control_simulation_builder const* sys) -> current_t {
                                         return sys->plant.get_outputs().dc_link_current.read();
                                     },
                              [&](plant_sensorless_current_control_simulation_builder const* sys) -> current_t {
                                  return sys->plant.get_outputs().dc_link_current.read();
                              },
                              [&](plant_current_control_with_modulators_simulation_builder const* sys) -> current_t {
                                  return sys->plant.get_outputs().dc_link_current.read();
                              }},
            rectifier_system_variant_t{const_cast<builder_type*>(&system)});
    }

protected:
    builder_type system;
    yeet::simulator sim = yeet::make_gtest_simulator();
};

// ---- Name generator ----
struct RectifierTestNameGenerator {
    template <typename T>
    static std::string GetName(int)
    {
        return std::string{T::name.data()};
    }
};

// ---- Test configurations: builders compatible with rectifier topology ----
using RectifierTestedBuilders = ::testing::Types<plant_simplified_current_control_simulation_builder,
    plant_sensorless_current_control_simulation_builder,
    plant_current_control_with_modulators_simulation_builder>;

TYPED_TEST_SUITE(RectifierCurrentControlTest, RectifierTestedBuilders, RectifierTestNameGenerator);

// DC-link current regulation with two sequential reference steps (0.5 pu -> 1.0 pu)
TYPED_TEST(RectifierCurrentControlTest, CurrentRegulationNominalOperation)
{
    // Create checkers
    auto current_error_checker = this->create_current_error_checker(current_pu_t(0.2f));
    auto machine_voltage_checker = this->create_machine_voltage_zero_checker();

    // Machine voltage should always be zero
    machine_voltage_checker->activate();

    // ---- Phase 1: Step reference to 0.5 pu ----
    current_error_checker->deactivate();
    this->set_dc_link_current_reference(current_pu_t(0.5f));
    EXPECT_TRUE(this->sim.simulate_for(0.5));  // Transient window (unchecked)

    // Activate current error checker for steady-state
    current_error_checker->activate();
    EXPECT_TRUE(this->sim.simulate_for(1.5));  // Steady-state monitoring window

    // ---- Phase 2: Step reference to 1.0 pu ----
    current_error_checker->deactivate();
    this->set_dc_link_current_reference(current_pu_t(1.0f));
    EXPECT_TRUE(this->sim.simulate_for(1.0));  // Transient window (unchecked)

    // Re-activate checker for final steady-state
    current_error_checker->activate();
    EXPECT_TRUE(this->sim.simulate_for(1.5));  // Steady-state monitoring window

    // Verify machine-side voltages are zero (short-circuited)
    auto v_abc = this->read_machine_voltage_abc();
    EXPECT_NEAR(v_abc.a().value(), 0.0, 1.0) << "Machine voltage phase A should be zero";
    EXPECT_NEAR(v_abc.b().value(), 0.0, 1.0) << "Machine voltage phase B should be zero";
    EXPECT_NEAR(v_abc.c().value(), 0.0, 1.0) << "Machine voltage phase C should be zero";
}

// Test current regulation at lower reference (0.3 pu)
TYPED_TEST(RectifierCurrentControlTest, CurrentRegulationLowReference)
{
    auto current_error_checker = this->create_current_error_checker(current_pu_t(0.2f));
    auto machine_voltage_checker = this->create_machine_voltage_zero_checker();

    machine_voltage_checker->activate();

    // Step to low current reference
    current_error_checker->deactivate();
    this->set_dc_link_current_reference(current_pu_t(0.3f));
    EXPECT_TRUE(this->sim.simulate_for(0.5));  // Transient

    current_error_checker->activate();
    EXPECT_TRUE(this->sim.simulate_for(1.5));  // Steady-state

    // Verify DC link current is positive and regulated
    auto dc_current = this->read_dc_link_current();
    EXPECT_GT(dc_current.value(), 0.0) << "DC link current should be positive";

    // Verify machine-side voltages are zero (short-circuited)
    auto v_abc = this->read_machine_voltage_abc();
    EXPECT_NEAR(v_abc.a().value(), 0.0, 1.0) << "Machine voltage phase A should be zero";
    EXPECT_NEAR(v_abc.b().value(), 0.0, 1.0) << "Machine voltage phase B should be zero";
    EXPECT_NEAR(v_abc.c().value(), 0.0, 1.0) << "Machine voltage phase C should be zero";
}

// Test current step down (1.0 pu -> 0.5 pu)
TYPED_TEST(RectifierCurrentControlTest, CurrentRegulationStepDown)
{
    auto current_error_checker = this->create_current_error_checker(current_pu_t(0.2f));
    auto machine_voltage_checker = this->create_machine_voltage_zero_checker();

    machine_voltage_checker->activate();

    // ---- Phase 1: Step to 1.0 pu ----
    current_error_checker->deactivate();
    this->set_dc_link_current_reference(current_pu_t(1.0f));
    EXPECT_TRUE(this->sim.simulate_for(0.5));

    current_error_checker->activate();
    EXPECT_TRUE(this->sim.simulate_for(1.0));

    // ---- Phase 2: Step down to 0.5 pu ----
    current_error_checker->deactivate();
    this->set_dc_link_current_reference(current_pu_t(0.5f));
    EXPECT_TRUE(this->sim.simulate_for(0.5));  // Transient

    current_error_checker->activate();
    EXPECT_TRUE(this->sim.simulate_for(1.5));  // Steady-state at lower reference

    // Verify machine-side voltages are zero (short-circuited)
    auto v_abc = this->read_machine_voltage_abc();
    EXPECT_NEAR(v_abc.a().value(), 0.0, 1.0) << "Machine voltage phase A should be zero";
    EXPECT_NEAR(v_abc.b().value(), 0.0, 1.0) << "Machine voltage phase B should be zero";
    EXPECT_NEAR(v_abc.c().value(), 0.0, 1.0) << "Machine voltage phase C should be zero";
}
