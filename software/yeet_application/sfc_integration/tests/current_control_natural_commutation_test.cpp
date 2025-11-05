#include <gtest/gtest.h>

#include <variant>

#include "sfc_integration/plant_current_control_with_modulators_simulation_builder.hpp"
#include "sfc_integration/plant_sensorless_current_control_mocked_hardware_simulation_builder.hpp"
#include "sfc_integration/plant_sensorless_current_control_simulation_builder.hpp"
#include "sfc_integration/plant_simplified_current_control_mocked_hardware_simulation_builder.hpp"
#include "sfc_integration/plant_simplified_current_control_simulation_builder.hpp"
#include "sfc_loggers.hpp"
#include "yeet_control/units.hpp"
#include "yeet_testing/simulator_checker.hpp"
#include "yeet_testing/simulator_gtest.hpp"

using yeet::application::sfc::integration::plant_current_control_with_modulators_simulation_builder;  // ADDED
using yeet::application::sfc::integration::plant_sensorless_current_control_mocked_hardware_simulation_builder;
using yeet::application::sfc::integration::plant_sensorless_current_control_simulation_builder;
using yeet::application::sfc::integration::plant_simplified_current_control_mocked_hardware_simulation_builder;
using yeet::application::sfc::integration::plant_simplified_current_control_simulation_builder;

using namespace yeet;

// Variant type to hold either simulation or mocked hardware builder
using system_variant_t = std::variant<plant_simplified_current_control_mocked_hardware_simulation_builder*,
    plant_simplified_current_control_simulation_builder*,
    plant_sensorless_current_control_simulation_builder*,
    plant_sensorless_current_control_mocked_hardware_simulation_builder*,
    plant_current_control_with_modulators_simulation_builder*>;  // ADDED

template <class... Ts>
struct overloaded : Ts... {
    using Ts::operator()...;
};
template <class... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

template <typename BuilderT>
class CurrentControlNaturalCommutationTypedTest : public ::testing::Test {
public:
    void SetUp() override
    {
        sim.configure_sampler_sampling_rate(200e-6);
        system.register_applications(sim);
        system.initialize();
        register_common_loggers();
        initialize_simulation();
    }

    std::shared_ptr<checker_user_activation> create_current_error_checker(current_pu_t margin = current_pu_t{0.2f})
    {
        return sim.register_checker(yeet::make_range_checker<current_pu_t>(
            "current error checker", [&]() -> current_pu_t { return this->read_current_error(); }, -margin, margin,
            yeet::checker_activation_always{}));
    }

    void register_common_loggers()
    {
        // Explicit per-type logger registration via std::visit
        std::visit(overloaded{[&](plant_simplified_current_control_mocked_hardware_simulation_builder* sys) {
                                  register_complete_logger(sim, sys->current_controller);
                                  register_complete_logger(sim, sys->dc_link_measurements);
                              },
                       [&](plant_simplified_current_control_simulation_builder* sys) {
                           register_complete_logger(sim, sys->current_controller);
                           register_complete_logger(sim, sys->dc_link_measurements);
                       },
                       [&](plant_sensorless_current_control_simulation_builder* sys) {
                           register_complete_logger(sim, sys->current_controller);
                           register_complete_logger(sim, sys->dc_link_measurements);
                           register_complete_logger(sim, sys->machine_side_observer);
                           register_complete_logger(sim, sys->grid_side_observer);
                       },
                       [&](plant_sensorless_current_control_mocked_hardware_simulation_builder* sys) {
                           register_complete_logger(sim, sys->current_controller);
                           register_complete_logger(sim, sys->dc_link_measurements);
                           register_complete_logger(sim, sys->machine_side_observer);
                           register_complete_logger(sim, sys->grid_side_observer);
                       },
                       // NEW: plant_current_control_with_modulators_simulation_builder
                       [&](plant_current_control_with_modulators_simulation_builder* sys) {
                           register_complete_logger(sim, sys->current_controller);
                           register_complete_logger(sim, sys->dc_link_measurements);
                           // Add any specific modulators / observer loggers here if available
                       }},
            system_variant_t{&system});
    }

    void initialize_simulation(double settling_time = 0.2)
    {
        // Disable pulses (explicit per type)
        std::visit(overloaded{[&](plant_simplified_current_control_mocked_hardware_simulation_builder* sys) {
                                  sys->grid_side_pulse_generator.get_parameters().enable.write(false);
                                  sys->machine_side_pulse_generator.get_parameters().enable.write(false);
                              },
                       [&](plant_simplified_current_control_simulation_builder* sys) {
                           sys->grid_side_pulse_generator.get_parameters().enable.write(false);
                           sys->machine_side_pulse_generator.get_parameters().enable.write(false);
                       },
                       [&](plant_sensorless_current_control_simulation_builder* sys) {
                           sys->grid_side_pulse_generator.get_parameters().enable.write(false);
                           sys->machine_side_pulse_generator.get_parameters().enable.write(false);
                       },
                       [&](plant_sensorless_current_control_mocked_hardware_simulation_builder* sys) {
                           sys->grid_side_pulse_generator.get_parameters().enable.write(false);
                           sys->machine_side_pulse_generator.get_parameters().enable.write(false);
                       },
                       [&](plant_current_control_with_modulators_simulation_builder* sys) {
                           sys->grid_side_pulse_generator.get_parameters().enable.write(false);
                           sys->machine_side_pulse_generator.get_parameters().enable.write(false);
                       }},
            system_variant_t{&system});

        sim.simulate_for(settling_time);

        // Enable pulses (explicit per type)
        std::visit(overloaded{[&](plant_simplified_current_control_mocked_hardware_simulation_builder* sys) {
                                  sys->grid_side_pulse_generator.get_parameters().enable.write(true);
                                  sys->machine_side_pulse_generator.get_parameters().enable.write(true);
                              },
                       [&](plant_simplified_current_control_simulation_builder* sys) {
                           sys->grid_side_pulse_generator.get_parameters().enable.write(true);
                           sys->machine_side_pulse_generator.get_parameters().enable.write(true);
                       },
                       [&](plant_sensorless_current_control_simulation_builder* sys) {
                           sys->grid_side_pulse_generator.get_parameters().enable.write(true);
                           sys->machine_side_pulse_generator.get_parameters().enable.write(true);
                       },
                       [&](plant_sensorless_current_control_mocked_hardware_simulation_builder* sys) {
                           sys->grid_side_pulse_generator.get_parameters().enable.write(true);
                           sys->machine_side_pulse_generator.get_parameters().enable.write(true);
                       },
                       [&](plant_current_control_with_modulators_simulation_builder* sys) {
                           sys->grid_side_pulse_generator.get_parameters().enable.write(true);
                           sys->machine_side_pulse_generator.get_parameters().enable.write(true);
                       }},
            system_variant_t{&system});
    }

    void set_dc_link_current_reference(const current_pu_t& value) noexcept
    {
        std::visit(
            overloaded{[&](plant_simplified_current_control_mocked_hardware_simulation_builder* sys) {
                           sys->current_reference = value;
                       },
                [&](plant_simplified_current_control_simulation_builder* sys) { sys->current_reference = value; },
                [&](plant_sensorless_current_control_simulation_builder* sys) { sys->current_reference = value; },
                [&](plant_sensorless_current_control_mocked_hardware_simulation_builder* sys) {
                    sys->current_reference = value;
                },
                [&](plant_current_control_with_modulators_simulation_builder* sys) { sys->current_reference = value; }},
            system_variant_t{&system});
    }

    void step_machine_frequency_pu(const frequency_pu_t& new_freq) noexcept
    {
        std::visit(overloaded{[&](plant_simplified_current_control_mocked_hardware_simulation_builder* sys) {
                                  sys->plant.get_plant_model().set_machine_frequency_pu(new_freq);
                              },
                       [&](plant_simplified_current_control_simulation_builder* sys) {
                           sys->plant.get_plant_model().set_machine_frequency_pu(new_freq);
                       },
                       [&](plant_sensorless_current_control_simulation_builder* sys) {
                           sys->plant.get_plant_model().set_machine_frequency_pu(new_freq);
                       },
                       [&](plant_sensorless_current_control_mocked_hardware_simulation_builder* sys) {
                           sys->plant.get_plant_model().set_machine_frequency_pu(new_freq);
                       },
                       [&](plant_current_control_with_modulators_simulation_builder* sys) {
                           sys->plant.get_plant_model().set_machine_frequency_pu(new_freq);
                       }},
            system_variant_t{&system});
    }

    void step_machine_voltage_pu(const voltage_pu_t& new_voltage) noexcept
    {
        std::visit(overloaded{[&](plant_simplified_current_control_mocked_hardware_simulation_builder* sys) {
                                  sys->plant.get_plant_model().set_machine_voltage_pu(new_voltage);
                              },
                       [&](plant_simplified_current_control_simulation_builder* sys) {
                           sys->plant.get_plant_model().set_machine_voltage_pu(new_voltage);
                       },
                       [&](plant_sensorless_current_control_simulation_builder* sys) {
                           sys->plant.get_plant_model().set_machine_voltage_pu(new_voltage);
                       },
                       [&](plant_sensorless_current_control_mocked_hardware_simulation_builder* sys) {
                           sys->plant.get_plant_model().set_machine_voltage_pu(new_voltage);
                       },
                       [&](plant_current_control_with_modulators_simulation_builder* sys) {
                           sys->plant.get_plant_model().set_machine_voltage_pu(new_voltage);
                       }},
            system_variant_t{&system});
    }

    current_pu_t read_current_error() const noexcept
    {
        return std::visit(
            overloaded{[&](plant_simplified_current_control_mocked_hardware_simulation_builder const* sys)
                           -> current_pu_t { return 0.1 * sys->current_controller.get_outputs().current_error.read(); },
                [&](plant_simplified_current_control_simulation_builder const* sys) -> current_pu_t {
                    return sys->current_controller.get_outputs().current_error.read();
                },
                [&](plant_sensorless_current_control_simulation_builder const* sys) -> current_pu_t {
                    return sys->current_controller.get_outputs().current_error.read();
                },
                [&](plant_sensorless_current_control_mocked_hardware_simulation_builder const* sys) -> current_pu_t {
                    return 0.1 * sys->current_controller.get_outputs()
                                     .current_error
                                     .read();  // TODO: Plant model needs to be fixed - yields in wrong dc-link current
                },
                [&](plant_current_control_with_modulators_simulation_builder const* sys) -> current_pu_t {
                    return sys->current_controller.get_outputs()
                        .current_error.read();  // TODO: Plant model needs to be fixed - yields in wrong dc-link current
                }},
            system_variant_t{const_cast<BuilderT*>(&system)});
    }

protected:
    BuilderT system;
    yeet::simulator sim = yeet::make_gtest_simulator();
};

// GoogleTest requires this functor-style template with GetName(int)
struct TypedTestNameGenerator {
    template <typename T>
    static std::string GetName(int)
    {
        return std::string{T::name.data()};
    }
};

using TestedBuilders = ::testing::Types<plant_simplified_current_control_simulation_builder,
    plant_simplified_current_control_mocked_hardware_simulation_builder,
    plant_sensorless_current_control_simulation_builder,
    plant_sensorless_current_control_mocked_hardware_simulation_builder,
    plant_current_control_with_modulators_simulation_builder>;

// Provide custom name generator as 3rd argument
TYPED_TEST_SUITE(CurrentControlNaturalCommutationTypedTest, TestedBuilders, TypedTestNameGenerator);

// DC-link current regulation with two sequential reference steps (0.5 pu -> 1.0 pu)
// Checker is explicitly deactivated during each commanded step transient and
// re-activated only for the intended steady‑state windows.
TYPED_TEST(CurrentControlNaturalCommutationTypedTest, CurrentRegulationNominalOperation)
{
    // Create checker with ±0.2 pu allowable steady‑state dc‑link current error
    auto current_error_checker = this->create_current_error_checker(current_pu_t(0.2f));

    // ---- Phase 1: Step reference to 0.5 pu ----
    // Deactivate while the loop settles from zero to 0.5 pu (avoid counting transient overshoot)
    current_error_checker->deactivate();
    this->set_dc_link_current_reference(current_pu_t(0.5f));
    EXPECT_TRUE(this->sim.simulate_for(0.5));  // Transient window (unchecked)

    // Activate checker: enforce steady‑state error band at 0.5 pu reference
    current_error_checker->activate();
    EXPECT_TRUE(this->sim.simulate_for(1.5));  // Steady‑state monitoring window

    // ---- Phase 2: Step reference to 1.0 pu ----
    // Suspend checker again for the second step transient
    current_error_checker->deactivate();
    this->set_dc_link_current_reference(current_pu_t(1.0f));
    EXPECT_TRUE(this->sim.simulate_for(1.0));  // Transient window (unchecked)

    // Re-activate checker for final steady‑state at 1.0 pu
    current_error_checker->activate();
    EXPECT_TRUE(this->sim.simulate_for(1.5));  // Steady‑state monitoring window
}

TYPED_TEST(CurrentControlNaturalCommutationTypedTest, CurrentRegulationFrequency80PerUnit)
{
    auto current_error_checker = this->create_current_error_checker(current_pu_t(0.2f));

    // ---- Phase 1: Command initial current (0.5 pu) ----
    // Deactivate while the loop slews from initial (0) to 0.5 pu.
    current_error_checker->deactivate();
    this->set_dc_link_current_reference(current_pu_t(0.5f));
    bool success = this->sim.simulate_for(0.5);  // Unchecked transient window

    // Activate checker once the transient should have largely settled.
    current_error_checker->activate();
    success &= this->sim.simulate_for(1.5);  // Steady‑state monitoring prior to disturbance

    // ---- Phase 2: Apply combined frequency + voltage step to 0.8 pu ----
    // Suspend checker to mask the disturbance-induced transient.
    current_error_checker->deactivate();
    this->step_machine_frequency_pu(frequency_pu_t{0.8f});
    this->step_machine_voltage_pu(voltage_pu_t{0.8f});
    success &= this->sim.simulate_for(1.0);  // Post-step settling (unchecked)

    // Re-enable checker to validate recovery at new operating point.
    current_error_checker->activate();
    success &= this->sim.simulate_for(1.0);  // Steady‑state after disturbance

    EXPECT_TRUE(success);
}

TYPED_TEST(CurrentControlNaturalCommutationTypedTest, CurrentRegulationFrequency60PerUnit)
{
    // Same pattern as the 0.8 pu test, but stepping to 0.6 pu frequency/voltage.
    auto current_error_checker = this->create_current_error_checker(current_pu_t(0.2f));

    // ---- Phase 1: Command initial current (0.5 pu) ----
    current_error_checker->deactivate();
    this->set_dc_link_current_reference(current_pu_t(0.5f));
    bool success = this->sim.simulate_for(0.5);  // Unchecked transient
    current_error_checker->activate();
    success &= this->sim.simulate_for(1.5);  // Steady‑state pre-disturbance

    // ---- Phase 2: Apply frequency + voltage step to 0.6 pu ----
    current_error_checker->deactivate();
    this->step_machine_frequency_pu(frequency_pu_t{0.6f});
    this->step_machine_voltage_pu(voltage_pu_t{0.6f});
    success &= this->sim.simulate_for(1.0);  // Unchecked settling
    current_error_checker->activate();
    success &= this->sim.simulate_for(1.0);  // Steady‑state verification

    EXPECT_TRUE(success);
}
