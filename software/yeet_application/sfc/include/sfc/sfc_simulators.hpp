#pragma once

#include <cassert>

#include "sfc_6p_6p_plant_model.hpp"
#include "sfc_current_controller.hpp"
#include "sfc_dc_link_filtering.hpp"
#include "sfc_dc_link_measurements.hpp"
#include "sfc_grid_side_filtering.hpp"
#include "sfc_grid_side_measurements.hpp"
#include "sfc_grid_side_observer.hpp"
#include "sfc_machine_side_filtering.hpp"
#include "sfc_machine_side_lookup_table.hpp"
#include "sfc_machine_side_measurements.hpp"
#include "sfc_machine_side_observer.hpp"
#include "sfc_per_unit_system.hpp"
#include "sfc_plant_model_simulator.hpp"
#include "sfc_speed_controller.hpp"
#include "yeet_control/angle.hpp"
#include "yeet_control/reference_frame.hpp"
#include "yeet_control/six_pulse_generator.hpp"
#include "yeet_control/units.hpp"
#include "yeet_testing/simulator.hpp"

namespace yeet::application::sfc {

// A dedicated structure to hold control signals for the pulse generators.
// This object is owned by the test fixture to guarantee its lifetime.
struct sfc_pulse_generator_conduit {
    bool enable = true;
    angle<zero_to_two_pi> grid_firing_angle_rad = deg_to_rad(sfc_plant_model_simulator::kNominalGridFiringAngleDeg);
    angle<zero_to_two_pi> machine_firing_angle_rad =
        deg_to_rad(sfc_plant_model_simulator::kNominalMachineFiringAngleDeg);

    current_pu_t current_ref_pu{};
    current_pu_t measured_current_pu{};

    speed_pu_t speed_ref_pu{};
    speed_pu_t measured_speed_pu{};
};

class sfc_simulator_pulse_generators {
public:
    static void register_ideal_pulse_generator(yeet::testing::simulator& simulator,
        duration_t sampling_time,
        sfc_6p_6p_plant_model& model,
        sfc_simulator_pulse_generators& pulse_generator,
        const angle<::zero_to_two_pi>& grid_firing_angle,
        const angle<::zero_to_two_pi>& machine_firing_angle,
        const bool& enable)
    {
        simulator.register_lambda_runner(
            sampling_time, [&]() { pulse_generator.update(model, grid_firing_angle, machine_firing_angle, enable); });
    }

    static void register_pulse_generator(yeet::testing::simulator& simulator,
        duration_t sampling_time,
        sfc_6p_6p_plant_model& model,
        sfc_simulator_pulse_generators& pulse_generator,
        const angle<::zero_to_two_pi>& grid_reference_frame,
        const angle<::zero_to_two_pi>& grid_firing_angle,
        const angle<::zero_to_two_pi>& machine_reference_frame,
        const angle<::zero_to_two_pi>& machine_firing_angle,
        const bool& enable)
    {
        simulator.register_lambda_runner(sampling_time, [&]() {
            pulse_generator.update(
                model, grid_reference_frame, grid_firing_angle, machine_reference_frame, machine_firing_angle, enable);
        });
    }

    void update(sfc_6p_6p_plant_model& model,
        const angle<::zero_to_two_pi>& grid_firing_angle,
        const angle<::zero_to_two_pi>& machine_firing_angle,
        bool enable)
    {
        update(model, model.get_grid_voltage_source_angle(), grid_firing_angle,
            model.get_machine_voltage_source_angle(), machine_firing_angle, enable);
    }

    void update(sfc_6p_6p_plant_model& model,
        const angle<::zero_to_two_pi>& grid_reference_angle,
        const angle<::zero_to_two_pi>& grid_firing_angle,
        const angle<::zero_to_two_pi>& machine_reference_angle,
        const angle<::zero_to_two_pi>& machine_firing_angle,
        bool enable)
    {
        control::six_pulse_generator::pulses const grid_pulses =
            grid_side_pulse_generator.update(grid_reference_angle, grid_firing_angle, enable);
        model.set_grid_side_pulses(grid_pulses.to_array());

        control::six_pulse_generator::pulses const machine_pulses =
            machine_side_pulse_generator.update(machine_reference_angle, machine_firing_angle, enable);
        model.set_machine_side_pulses(machine_pulses.to_array());
    }

    control::six_pulse_generator grid_side_pulse_generator;
    control::six_pulse_generator machine_side_pulse_generator;
};

class sfc_simulator_measurement_and_filtering {
public:
    sfc_simulator_measurement_and_filtering(duration_t sampling_time)
        : sampling_time(sampling_time),
          grid_side_filtering(sampling_time),
          dc_link_filtering(sampling_time),
          machine_side_filtering(sampling_time)
    {
    }

    static void register_measurements_and_filtering(yeet::testing::simulator& simulator,
        duration_t sampling_time,
        sfc_per_unit_system& per_unit_system,
        sfc_simulator_measurement_and_filtering& measurements_and_filtering,
        sfc_6p_6p_plant_model& model)
    {
        simulator.register_lambda_runner(
            sampling_time, [&]() { measurements_and_filtering.update(model, per_unit_system); });
    }

    void update(sfc_6p_6p_plant_model& model, const sfc_per_unit_system& per_unit_system)
    {
        // Update measurements for all three sides
        grid_side_measurements.update(
            per_unit_system.get_grid_bases(), model.get_grid_side_voltages_abc(), model.get_grid_side_currents_abc());

        // For DC link, we need to estimate voltage since model doesn't provide it
        // Use a nominal DC voltage based on the per-unit system base
        voltage_t dc_voltage_estimate = per_unit_system.get_dc_link_bases().base_voltage.value();
        dc_link_measurements.update(
            per_unit_system.get_dc_link_bases(), dc_voltage_estimate, model.get_dc_link_current());

        machine_side_measurements.update(per_unit_system.get_machine_bases(), model.get_machine_side_voltages_abc(),
            model.get_machine_side_currents_abc());

        // Update filtering for all three sides
        grid_side_filtering.update(grid_side_measurements.get_voltage_pu(), grid_side_measurements.get_current_pu());

        dc_link_filtering.update(dc_link_measurements.get_voltage_pu(), dc_link_measurements.get_current_pu());

        machine_side_filtering.update(machine_side_measurements.get_voltage_pu(),
            machine_side_measurements.get_current_pu());  // TODO: Get actual machine frequency from model
    }

    // Accessors for measurements
    const sfc_grid_side_measurements& get_grid_side_measurements() const { return grid_side_measurements; }
    const sfc_dc_link_measurements& get_dc_link_measurements() const { return dc_link_measurements; }
    const sfc_machine_side_measurements& get_machine_side_measurements() const { return machine_side_measurements; }

    // Accessors for filtering
    const sfc_grid_side_filtering& get_grid_side_filtering() const { return grid_side_filtering; }
    const sfc_dc_link_filtering& get_dc_link_filtering() const { return dc_link_filtering; }
    const sfc_machine_side_filtering& get_machine_side_filtering() const { return machine_side_filtering; }

    const duration_t sampling_time;

    // Measurement components
    sfc_grid_side_measurements grid_side_measurements;
    sfc_dc_link_measurements dc_link_measurements;
    sfc_machine_side_measurements machine_side_measurements;

    // Filtering components
    sfc_grid_side_filtering grid_side_filtering;
    sfc_dc_link_filtering dc_link_filtering;
    sfc_machine_side_filtering machine_side_filtering;
};

class sfc_simulator_observers {
public:
    sfc_simulator_observers(duration_t sampling_time)
        : grid_side_observer(sampling_time), machine_side_observer(sampling_time), sampling_time(sampling_time)
    {
    }

    static void register_observers(yeet::testing::simulator& simulator,
        duration_t sampling_time,
        sfc_6p_6p_plant_model& model,
        sfc_simulator_observers& observers,
        sfc_simulator_pulse_generators& pulse_generator,
        const abc<voltage_pu_t>& grid_voltage_pu,
        const abc<voltage_pu_t>& machine_voltage_pu)
    {
        simulator.register_lambda_runner(sampling_time, [&]() {
            observers.update_grid(grid_voltage_pu, pulse_generator.grid_side_pulse_generator.get_pulses());

            observers.update_machine(machine_voltage_pu, pulse_generator.machine_side_pulse_generator.get_pulses());
        });
    }

    void update_grid(
        const abc<voltage_pu_t>& grid_voltage_pu, const control::six_pulse_generator::pulses& grid_side_pulses)
    {
        grid_side_observer.update(grid_voltage_pu, grid_side_pulses);
    }

    void update_machine(
        const abc<voltage_pu_t>& machine_voltage_pu, const control::six_pulse_generator::pulses& machine_side_pulses)
    {
        machine_side_observer.update(machine_voltage_pu, machine_side_pulses);
    }

    sfc_grid_side_observer grid_side_observer;
    sfc_machine_side_observer machine_side_observer;
    const duration_t sampling_time{};
};

class sfc_simulator_current_controller {
public:
    sfc_simulator_current_controller(duration_t sampling_time)
        : current_controller(sampling_time), sampling_time(sampling_time)
    {
    }

    static void register_current_controller(yeet::testing::simulator& simulator,
        duration_t sampling_time,
        sfc_simulator_current_controller& controller,
        sfc_pulse_generator_conduit& conduit,
        const sfc_simulator_measurement_and_filtering& measurements,
        const sfc_simulator_observers& observers,
        const sfc_per_unit_system& per_unit_system)
    {
        simulator.register_lambda_runner(sampling_time, [&]() {
            // One should "enable current control"
            if (observers.grid_side_observer.is_locked() == false) {
                // If grid observer is not locked, skip current controller update
                return;
            }

            // Get required measurements in per-unit
            const current_pu_t measured_current_pu = measurements.get_dc_link_filtering().get_filtered_current_pu();

            // Get ideal DC voltage from grid-side observer (already in grid-side pu)
            const voltage_pu_t ideal_dc_voltage_pu =
                observers.grid_side_observer.get_filtered_voltage_alphabeta().magnitude();

            // Get inverter back-EMF from machine-side observer (in machine-side pu)
            const voltage_pu_t inverter_back_emf_machine_pu =
                observers.machine_side_observer.get_filtered_voltage_alphabeta().magnitude();

            // Update the current controller using the overload that performs automatic per-unit reflection
            controller.current_controller.update(conduit.current_ref_pu, measured_current_pu, ideal_dc_voltage_pu,
                inverter_back_emf_machine_pu, per_unit_system.get_machine_bases().base_voltage_peak_phase.value(),
                per_unit_system.get_grid_bases().base_voltage_peak_phase);

            // Get the firing angle output and update the conduit
            const angle_t firing_angle_radians = controller.current_controller.get_firing_angle_reference_radians();
            conduit.grid_firing_angle_rad = angle<zero_to_two_pi>::from_radians(firing_angle_radians.value());

            // Update measured current in conduit for other controllers
            conduit.measured_current_pu = measured_current_pu;
        });
    }

    void configure_pi_gains(double proportional_gain, duration_t integral_time)
    {
        current_controller.configure_pi_gains(proportional_gain, integral_time);
    }

    void configure_current_limits_pu(current_pu_t min_current_pu, current_pu_t max_current_pu)
    {
        current_controller.configure_current_limits_pu(min_current_pu, max_current_pu);
    }

    void configure_current_reference_slew_rate(double slew_rate_pu_per_second)
    {
        current_controller.configure_current_reference_slew_rate(slew_rate_pu_per_second);
    }

    void configure_firing_angle_limits_radians(angle_t min_angle, angle_t max_angle)
    {
        current_controller.configure_firing_angle_limits_radians(min_angle, max_angle);
    }

    const sfc_current_controller& get_controller() const { return current_controller; }

    sfc_current_controller current_controller;
    const duration_t sampling_time{};
};

class sfc_simulator_machine_side_lookup_table {
public:
    sfc_simulator_machine_side_lookup_table(double nominal_gamma_deg) : lookup_table(nominal_gamma_deg) {}

    static void register_machine_side_lookup_table(yeet::testing::simulator& simulator,
        duration_t sampling_time,
        sfc_simulator_machine_side_lookup_table& lookup,
        sfc_pulse_generator_conduit& conduit)
    {
        simulator.register_lambda_runner(sampling_time, [&]() {
            angle<zero_to_two_pi> machine_firing_angle_rad = angle<zero_to_two_pi>::from_degrees(
                lookup.lookup_table.get_firing_angle_deg(conduit.measured_speed_pu, conduit.measured_current_pu));

            conduit.machine_firing_angle_rad = machine_firing_angle_rad;
        });
    }

    void configure_table(double gamma_setpoint_deg, double machine_inductance_pu)
    {
        lookup_table.recalculate_table(gamma_setpoint_deg, machine_inductance_pu);
    }

    const sfc_machine_side_firing_angle_lookup_table& get_lookup_table() const { return lookup_table; }

    sfc_machine_side_firing_angle_lookup_table lookup_table;
};

class sfc_simulator_speed_controller {
public:
    sfc_simulator_speed_controller(duration_t sampling_time)
        : speed_controller(sampling_time), sampling_time(sampling_time)
    {
    }

    static void register_speed_controller(yeet::testing::simulator& simulator,
        duration_t sampling_time,
        sfc_simulator_speed_controller& controller,
        sfc_pulse_generator_conduit& conduit,
        const sfc_simulator_observers& observers)
    {
        simulator.register_lambda_runner(sampling_time, [&]() {
            // One should "enable current control"
            if (observers.grid_side_observer.is_locked() == false) {
                // If grid observer is not locked, skip current controller update
                return;
            }

            controller.speed_controller.update(conduit.speed_ref_pu, conduit.measured_speed_pu);

            conduit.current_ref_pu = current_pu_t{controller.speed_controller.get_torque_ref_pu().value()};
        });
    }

    void configure_pi_gains(double kp, double ti) { speed_controller.configure_pi_gains(kp, duration_t{ti}); }

    void configure_torque_limits_pu(torque_pu_t min_pu, torque_pu_t max_pu)
    {
        speed_controller.configure_torque_limits_pu(min_pu, max_pu);
    }

    void configure_speed_reference_filter(double frequency_hz)
    {
        // speed_controller.c(frequency_hz);
    }

    const sfc_speed_controller& get_controller() const { return speed_controller; }

    sfc_speed_controller speed_controller;
    const duration_t sampling_time{};
};

// A component holder to be owned by the test fixture.
// This ensures all components have a lifetime that exceeds the simulation.
struct sfc_simulation_components {
    static constexpr duration_t kControlSamplingTime = duration_t{100e-6};  // Control logic at 100 µs

    sfc_simulation_components()
        : plant_model_simulator(),
          pulse_generators(),
          measurement_and_filtering(kControlSamplingTime),
          observers(kControlSamplingTime),
          current_controller(kControlSamplingTime),
          machine_side_lookup_table(15.0),  // Default nominal extinction angle of 15 degrees
          speed_controller(kControlSamplingTime)
    {
    }

    sfc_plant_model_simulator plant_model_simulator;
    sfc_simulator_pulse_generators pulse_generators;
    sfc_simulator_measurement_and_filtering measurement_and_filtering;
    sfc_simulator_observers observers;
    sfc_simulator_current_controller current_controller;
    sfc_simulator_machine_side_lookup_table machine_side_lookup_table;
    sfc_simulator_speed_controller speed_controller;
};

// Builder class to configure the simulator with different components.
class sfc_simulator_builder {
public:
    sfc_simulator_builder(sfc_simulation_components& components) : _components(components) {}

    // Add measurements and filtering to the simulation loop.
    sfc_simulator_builder& with_measurements_and_filtering()
    {
        _use_measurements = true;
        return *this;
    }

    // Add observers to the simulation loop.
    sfc_simulator_builder& with_observers()
    {
        _use_observers = true;
        _use_measurements = true;  // Observers implicitly depend on measurements
        return *this;
    }

    // Configure pulse generators to use the plant's internal "perfect" angle.
    sfc_simulator_builder& with_ideal_pulse_generation(const sfc_pulse_generator_conduit& conduit)
    {
        _use_ideal_pulses = true;
        _pulse_generator_conduit = &conduit;
        return *this;
    }

    // Configure pulse generators to use the observer's estimated angle.
    sfc_simulator_builder& with_observer_based_pulse_generation(const sfc_pulse_generator_conduit& conduit)
    {
        _use_ideal_pulses = false;
        _use_observers = true;  // This configuration depends on observers
        _pulse_generator_conduit = &conduit;
        return *this;
    }

    // Add current controller to the simulation loop.
    sfc_simulator_builder& with_current_controller(sfc_pulse_generator_conduit& conduit)
    {
        _use_current_controller = true;
        _pulse_generator_conduit = &conduit;
        _use_measurements = true;  // Current controller depends on measurements
        return *this;
    }

    // Add machine-side lookup table to the simulation loop.
    sfc_simulator_builder& with_machine_side_lookup_table(sfc_pulse_generator_conduit& conduit)
    {
        _use_machine_side_lookup = true;
        _pulse_generator_conduit = &conduit;
        _use_measurements = true;  // Machine-side lookup depends on measurements
        return *this;
    }

    // Add speed controller to the simulation loop.
    sfc_simulator_builder& with_speed_controller(sfc_pulse_generator_conduit& conduit)
    {
        _use_speed_controller = true;
        _pulse_generator_conduit = &conduit;
        _use_measurements = true;  // Speed controller depends on measurements
        return *this;
    }

    // Wires all the selected components together in the correct execution order.
    void configure(yeet::testing::simulator& simulator)
    {
        auto& plant_model = _components.plant_model_simulator.plant();
        auto& per_unit_system = const_cast<sfc_per_unit_system&>(_components.plant_model_simulator.per_unit_system());

        // The plant model runner is now configured by the builder
        _components.plant_model_simulator.setup_simulator(simulator);

        // The order of registration defines the execution order per simulation step.
        if (_use_measurements) {
            sfc_simulator_measurement_and_filtering::register_measurements_and_filtering(simulator,
                _components.kControlSamplingTime, per_unit_system, _components.measurement_and_filtering, plant_model);
        }

        if (_use_observers) {
            assert(_use_measurements && "Observers require measurements to be enabled.");
            sfc_simulator_observers::register_observers(simulator, _components.kControlSamplingTime, plant_model,
                _components.observers, _components.pulse_generators,
                _components.measurement_and_filtering.get_grid_side_measurements().get_voltage_pu(),
                _components.measurement_and_filtering.get_machine_side_measurements().get_voltage_pu());
        }

        if (_pulse_generator_conduit != nullptr) {
            if (_use_current_controller) {
                // Current controller manages the pulse generation
                assert(_use_measurements && "Current controller requires measurements to be enabled.");
                assert(_use_observers &&
                       "Current controller requires observers for ideal DC voltage and back-EMF estimation.");
                sfc_simulator_current_controller::register_current_controller(simulator,
                    _components.kControlSamplingTime, _components.current_controller,
                    const_cast<sfc_pulse_generator_conduit&>(*_pulse_generator_conduit),
                    _components.measurement_and_filtering, _components.observers, per_unit_system);
            }

            if (_use_machine_side_lookup) {
                // Machine-side lookup table manages machine firing angle
                assert(_use_measurements && "Machine-side lookup table requires measurements to be enabled.");
                sfc_simulator_machine_side_lookup_table::register_machine_side_lookup_table(simulator,
                    _components.kControlSamplingTime, _components.machine_side_lookup_table,
                    const_cast<sfc_pulse_generator_conduit&>(*_pulse_generator_conduit));
            }

            if (_use_speed_controller) {
                // Speed controller manages current reference
                assert(_use_measurements && "Speed controller requires measurements to be enabled.");
                sfc_simulator_speed_controller::register_speed_controller(simulator, _components.kControlSamplingTime,
                    _components.speed_controller, const_cast<sfc_pulse_generator_conduit&>(*_pulse_generator_conduit),
                    _components.observers);
            }

            if (_use_ideal_pulses) {
                sfc_simulator_pulse_generators::register_ideal_pulse_generator(simulator,
                    _components.kControlSamplingTime, plant_model, _components.pulse_generators,
                    _pulse_generator_conduit->grid_firing_angle_rad, _pulse_generator_conduit->machine_firing_angle_rad,
                    _pulse_generator_conduit->enable);
            }
            else {  // Observer-based pulses
                assert(_use_observers && "Observer-based pulse generation requires observers to be enabled.");
                sfc_simulator_pulse_generators::register_pulse_generator(simulator, _components.kControlSamplingTime,
                    plant_model, _components.pulse_generators,
                    _components.observers.grid_side_observer.get_estimated_angle(),
                    _pulse_generator_conduit->grid_firing_angle_rad,
                    _components.observers.machine_side_observer.get_estimated_angle(),
                    _pulse_generator_conduit->machine_firing_angle_rad, _pulse_generator_conduit->enable);
            }
        }
    }

private:
    sfc_simulation_components& _components;

    // Configuration flags
    bool _use_measurements = false;
    bool _use_observers = false;
    bool _use_ideal_pulses = true;
    bool _use_current_controller = false;
    bool _use_machine_side_lookup = false;
    bool _use_speed_controller = false;

    // Pointer to the specific conduit for pulse generation
    const sfc_pulse_generator_conduit* _pulse_generator_conduit = nullptr;
};

}  // namespace yeet::application::sfc