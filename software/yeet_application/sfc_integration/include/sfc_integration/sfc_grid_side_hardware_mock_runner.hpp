#pragma once

#include "sfc/grid_side_hardware_mock.hpp"
#include "sfc_integration/sfc_grid_side_hardware_mock_datamodel.hpp"
#include "yeet/application_interface.hpp"

namespace yeet::application::sfc::integration {

/**
 * @brief Runner for the Grid-Side Hardware Mock.
 *
 * This class integrates the `grid_side_hardware_mock` with the application
 * framework. It reads raw analog signals, processes them through the mock
 * hardware simulation, and provides the quantized digital outputs.
 */
class sfc_grid_side_hardware_mock_runner : public yeet::base_application<sfc_grid_side_hardware_mock_datamodel::inputs,
                                               sfc_grid_side_hardware_mock_datamodel::outputs,
                                               sfc_grid_side_hardware_mock_datamodel::parameters> {
public:
    using datamodel = sfc_grid_side_hardware_mock_datamodel;

    /**
     * @brief Constructs the runner.
     * @param sampling_period The sampling period (Ts) for the ADC mock.
     */
    explicit sfc_grid_side_hardware_mock_runner(duration_t sampling_period);

    void initialize() override;
    void run() override;
    void stop() override;

private:
    void set_parameters();
    void set_parameter_callbacks();

    mock::grid_side_hardware_mock _hardware_mock;
};

}  // namespace yeet::application::sfc::integration