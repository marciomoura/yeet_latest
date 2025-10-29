#pragma once

#include "sfc_integration/sfc_machine_side_pulse_generator_datamodel.hpp"
#include "yeet/application_interface.hpp"
#include "yeet_control/six_pulse_generator.hpp"

namespace yeet::application::sfc::integration {

/**
 * @brief SFC Machine-Side Pulse Generator Runner
 *
 * This runner manages the machine-side 6-pulse generator for SFC applications.
 * It takes reference angle and firing angle commands and generates the appropriate
 * firing pulses for the machine-side thyristor bridge.
 */
class sfc_machine_side_pulse_generator_runner
    : public yeet::base_application<sfc_machine_side_pulse_generator_datamodel::inputs,
          sfc_machine_side_pulse_generator_datamodel::outputs,
          sfc_machine_side_pulse_generator_datamodel::parameters> {
public:
    using datamodel = sfc_machine_side_pulse_generator_datamodel;
    static constexpr duration_t sampling_time = us(10);

    /**
     * @brief Constructor
     */
    sfc_machine_side_pulse_generator_runner();

    void register_datamodel(yeet::model_registry& registry) override;
    void initialize() override;
    void run() override;
    void stop() override;

private:
    void set_parameters();
    void set_parameter_callback();

    control::six_pulse_generator _pulse_generator;
};

}  // namespace yeet::application::sfc::integration
