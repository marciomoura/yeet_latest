#pragma once

#include "sfc/sfc_grid_side_measurements.hpp"
#include "sfc_integration/sfc_grid_side_measurements_datamodel.hpp"
#include "yeet/application_interface.hpp"

namespace yeet::application::sfc::integration {

class sfc_grid_side_measurements_runner
    : public yeet::base_application<sfc_grid_side_measurements_datamodel::inputs,
                                    sfc_grid_side_measurements_datamodel::outputs,
                                    sfc_grid_side_measurements_datamodel::parameters> {
public:
    using datamodel = sfc_grid_side_measurements_datamodel;

    static constexpr double sampling_time = 100e-6;

    sfc_grid_side_measurements_runner()
        : base_application(sampling_time), _sfc_grid_side_measurements() {}

    void initialize() override;
    void run() override;
    void stop() override;

private:
    void set_parameters();
    void set_parameter_callback();

    sfc::sfc_grid_side_measurements _sfc_grid_side_measurements;
    sfc_per_unit_system::grid_side_bases _grid_side_bases;
};

}  // namespace yeet::application::sfc::integration
