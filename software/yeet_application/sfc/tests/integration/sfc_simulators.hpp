#pragma once

#include "sfc/sfc_dc_link_filtering.hpp"
#include "sfc/sfc_dc_link_measurements.hpp"
#include "sfc/sfc_grid_side_filtering.hpp"
#include "sfc/sfc_grid_side_measurements.hpp"
#include "sfc/sfc_grid_side_observer.hpp"
#include "sfc/sfc_machine_side_filtering.hpp"
#include "sfc/sfc_machine_side_measurements.hpp"
#include "sfc/sfc_machine_side_observer.hpp"
#include "sfc/sfc_per_unit_system.hpp"
#include "sfc_6p_6p_plant_model.hpp"
#include "sfc_plant_model_simulator.hpp"
#include "yeet_control/angle.hpp"
#include "yeet_control/reference_frame.hpp"
#include "yeet_control/six_pulse_generator.hpp"
#include "yeet_control/units.hpp"
#include "yeet_testing/simulator.hpp"

namespace yeet::application::sfc {

class sfc_simulator_pulse_generators {
public:
    static void register_ideal_pulse_generator(yeet::testing::simulator& simulator, duration_t sampling_time,
                                               sfc_6p_6p_plant_model& model,
                                               sfc_simulator_pulse_generators& pulse_generator,
                                               const angle<::zero_to_two_pi>& grid_firing_angle,
                                               const angle<::zero_to_two_pi>& machine_firing_angle,
                                               const bool& enable) {
        simulator.register_lambda_runner(
            sampling_time, [&]() { pulse_generator.update(model, grid_firing_angle, machine_firing_angle, enable); });
    }

    static void register_pulse_generator(yeet::testing::simulator& simulator, duration_t sampling_time,
                                         sfc_6p_6p_plant_model& model, sfc_simulator_pulse_generators& pulse_generator,
                                         const angle<::zero_to_two_pi>& grid_reference_frame,
                                         const angle<::zero_to_two_pi>& grid_firing_angle,
                                         const angle<::zero_to_two_pi>& machine_reference_frame,
                                         const angle<::zero_to_two_pi>& machine_firing_angle, const bool& enable) {
        simulator.register_lambda_runner(sampling_time, [&]() {
            pulse_generator.update(model, grid_reference_frame, grid_firing_angle, machine_reference_frame,
                                   machine_firing_angle, enable);
        });
    }

    void update(sfc_6p_6p_plant_model& model, const angle<::zero_to_two_pi>& grid_firing_angle,
                const angle<::zero_to_two_pi>& machine_firing_angle, bool enable) {
        update(model, model.get_grid_voltage_source_angle(), grid_firing_angle,
               model.get_machine_voltage_source_angle(), machine_firing_angle, enable);
    }

    void update(sfc_6p_6p_plant_model& model, const angle<::zero_to_two_pi>& grid_reference_angle,
                const angle<::zero_to_two_pi>& grid_firing_angle,
                const angle<::zero_to_two_pi>& machine_reference_angle,
                const angle<::zero_to_two_pi>& machine_firing_angle, bool enable) {
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
          machine_side_filtering(sampling_time) {}

    static void register_measurements_and_filtering(yeet::testing::simulator& simulator,
                                                    sfc_per_unit_system& per_unit_system,
                                                    sfc_simulator_measurement_and_filtering& measurements_and_filtering,
                                                    sfc_6p_6p_plant_model& model) {
        simulator.register_lambda_runner(measurements_and_filtering.sampling_time,
                                         [&]() { measurements_and_filtering.update(model, per_unit_system); });
    }

    void update(sfc_6p_6p_plant_model& model, const sfc_per_unit_system& per_unit_system) {
        // Update measurements for all three sides
        grid_side_measurements.update(per_unit_system.get_grid_bases(), model.get_grid_side_voltages_abc(),
                                      model.get_grid_side_currents_abc());

        // For DC link, we need to estimate voltage since model doesn't provide it
        // Use a nominal DC voltage based on the per-unit system base
        voltage_t dc_voltage_estimate = per_unit_system.get_dc_link_bases().base_voltage.value();
        dc_link_measurements.update(per_unit_system.get_dc_link_bases(), dc_voltage_estimate,
                                    model.get_dc_link_current());

        machine_side_measurements.update(per_unit_system.get_machine_bases(), model.get_machine_side_voltages_abc(),
                                         model.get_machine_side_currents_abc());

        // Update filtering for all three sides
        grid_side_filtering.update(grid_side_measurements.get_voltage_pu(), grid_side_measurements.get_current_pu());

        dc_link_filtering.update(dc_link_measurements.get_voltage_pu(), dc_link_measurements.get_current_pu());

        machine_side_filtering.update(
            machine_side_measurements.get_voltage_pu(),
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
        : grid_side_observer(sampling_time), machine_side_observer(sampling_time), sampling_time(sampling_time) {}

    static void register_observers(yeet::testing::simulator& simulator, sfc_6p_6p_plant_model& model,
                                   sfc_simulator_observers& observers, sfc_simulator_pulse_generators& pulse_generator,
                                   const abc<voltage_pu_t>& grid_voltage_pu,
                                   const abc<voltage_pu_t>& machine_voltage_pu) {
        simulator.register_lambda_runner(observers.sampling_time, [&]() {
            observers.update_grid(grid_voltage_pu, pulse_generator.grid_side_pulse_generator.get_pulses());

            observers.update_machine(machine_voltage_pu, pulse_generator.machine_side_pulse_generator.get_pulses());
        });
    }

    void update_grid(const abc<voltage_pu_t>& grid_voltage_pu,
                     const control::six_pulse_generator::pulses& grid_side_pulses) {
        grid_side_observer.update(grid_voltage_pu, grid_side_pulses);
    }

    void update_machine(const abc<voltage_pu_t>& machine_voltage_pu,
                        const control::six_pulse_generator::pulses& machine_side_pulses) {
        machine_side_observer.update(machine_voltage_pu, machine_side_pulses);
    }

    sfc_grid_side_observer grid_side_observer;
    sfc_machine_side_observer machine_side_observer;
    const duration_t sampling_time{};
};

class sfc_simulator_minimal {
public:
    sfc_simulator_minimal(duration_t sampling_time) : sampling_time(sampling_time), pulse_generators() {}

    void configure_simulator(yeet::testing::simulator& simulator, sfc_6p_6p_plant_model& plant_model,
                             const angle<::zero_to_two_pi>& grid_firing_angle,
                             const angle<::zero_to_two_pi>& machine_firing_angle, const bool& enable) {
        // Register ideal pulse generators (uses plant model's internal angles)
        sfc_simulator_pulse_generators::register_ideal_pulse_generator(
            simulator, sampling_time, plant_model, pulse_generators, grid_firing_angle, machine_firing_angle, enable);
    }

    // Accessors
    const sfc_simulator_pulse_generators& get_pulse_generators() const { return pulse_generators; }
    const duration_t sampling_time;

private:
    sfc_simulator_pulse_generators pulse_generators;
};

class sfc_simulator_per_unit_with_observer {
public:
    sfc_simulator_per_unit_with_observer(duration_t sampling_time)
        : per_unit_system(
              sfc_plant_model_simulator::kNominalGridVoltageRMS, sfc_plant_model_simulator::kNominalGridFrequency,
              sfc_plant_model_simulator::kBaseDCLinkCurrent, sfc_plant_model_simulator::kMachineRatedPowerVA,
              sfc_plant_model_simulator::kNominalMachineVoltageRMS, sfc_plant_model_simulator::kNominalMachineFrequency,
              sfc_plant_model_simulator::kMachineNumPoles),
          pulse_generators(),
          measurement_and_filtering(sampling_time),
          observers(sampling_time) {}

    void configure_simulator(yeet::testing::simulator& simulator, sfc_6p_6p_plant_model& plant_model,
                             const angle<::zero_to_two_pi>& grid_firing_angle,
                             const angle<::zero_to_two_pi>& machine_firing_angle, const bool& enable) {
        // Register measurements and filtering
        sfc_simulator_measurement_and_filtering::register_measurements_and_filtering(
            simulator, per_unit_system, measurement_and_filtering, plant_model);

        // Register observers
        sfc_simulator_observers::register_observers(
            simulator, plant_model, observers, pulse_generators,
            measurement_and_filtering.grid_side_measurements.get_voltage_pu(),
            measurement_and_filtering.machine_side_measurements.get_voltage_pu());

        // Register pulse generators with explicit reference frames
        sfc_simulator_pulse_generators::register_pulse_generator(
            simulator, measurement_and_filtering.sampling_time, plant_model, pulse_generators,
            observers.grid_side_observer.get_estimated_angle(), grid_firing_angle,
            observers.machine_side_observer.get_estimated_angle(), machine_firing_angle, enable);
    }

    // Accessors
    const sfc_per_unit_system& get_per_unit_system() const { return per_unit_system; }
    const sfc_simulator_pulse_generators& get_pulse_generators() const { return pulse_generators; }
    const sfc_simulator_measurement_and_filtering& get_measurement_and_filtering() const {
        return measurement_and_filtering;
    }
    const sfc_simulator_observers& get_observers() const { return observers; }

private:
    sfc_per_unit_system per_unit_system;
    sfc_simulator_pulse_generators pulse_generators;
    sfc_simulator_measurement_and_filtering measurement_and_filtering;
    sfc_simulator_observers observers;
};

}  // namespace yeet::application::sfc