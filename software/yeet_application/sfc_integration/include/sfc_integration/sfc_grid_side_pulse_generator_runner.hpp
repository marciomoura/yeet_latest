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
 */
class sfc_grid_side_pulse_generator_runner
    : public yeet::base_application<sfc_grid_side_pulse_generator_datamodel::inputs,
          sfc_grid_side_pulse_generator_datamodel::outputs,
          sfc_grid_side_pulse_generator_datamodel::parameters> {
public:
    using datamodel = sfc_grid_side_pulse_generator_datamodel;
    static constexpr double sampling_time = 100e-6;  // 100 µs sampling time (10 kHz)

    /**
     * @brief Constructor
     */
    sfc_grid_side_pulse_generator_runner();

    void initialize() override;
    void run() override;
    void stop() override;

private:
    void set_parameters();
    void set_parameter_callback();

    control::six_pulse_generator _pulse_generator;
};

}  // namespace yeet::application::sfc::integration
