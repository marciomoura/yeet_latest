#pragma once

#include "sfc/sfc_grid_side_modulator.hpp"
#include "sfc_integration/sfc_grid_side_modulator_datamodel.hpp"
#include "yeet/application_interface.hpp"

namespace yeet::application::sfc::integration {

class sfc_grid_side_modulator_runner : public yeet::base_application<sfc_grid_side_modulator_datamodel::inputs,
                                           sfc_grid_side_modulator_datamodel::outputs,
                                           sfc_grid_side_modulator_datamodel::parameters> {
public:
    using datamodel = sfc_grid_side_modulator_datamodel;

    static constexpr double sampling_time = 100e-6;

    sfc_grid_side_modulator_runner()
        : base_application(sampling_time), _sfc_grid_side_modulator(duration_t{sampling_time})
    {
    }

    void initialize() override;
    void run() override;
    void stop() override;

private:
    void set_parameters();
    void set_parameter_callback();

    sfc::sfc_grid_side_modulator _sfc_grid_side_modulator;
};

}  // namespace yeet::application::sfc::integration
