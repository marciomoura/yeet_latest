#pragma once

#include "sfc_integration/sfc_grid_side_pulse_generator_datamodel.hpp"
#include "yeet/application_interface.hpp"
#include "yeet_control/six_pulse_generator.hpp"

namespace yeet::application::sfc::integration {

/**
 * @brief SFC Grid-Side Pulse Generator Runner
 *
 * This runner manages the grid-side 6-pulse generator for SFC applications.
 * It takes reference angle and firing angle commands and generates the appropriate
 * firing pulses for the grid-side thyristor bridge.
 *
 * The runner supports three operating modes:
 * 1. **Normal mode**: Uses external reference angle from PLL and firing angle command
 * 2. **Virtual reference mode**: Uses internal angle integration based on configured frequency
 * 3. **Pulse masking**: Individual pulses can be enabled/disabled for commissioning
 *
 * All functionality is delegated to the six_pulse_generator class.
 */
class sfc_grid_side_pulse_generator_runner
    : public yeet::base_application<sfc_grid_side_pulse_generator_datamodel::inputs,
          sfc_grid_side_pulse_generator_datamodel::outputs,
          sfc_grid_side_pulse_generator_datamodel::parameters> {
public:
    using datamodel = sfc_grid_side_pulse_generator_datamodel;
    static constexpr duration_t sampling_time = us(200.0);

    /**
     * @brief Constructor
     */
    sfc_grid_side_pulse_generator_runner();

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
