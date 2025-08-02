#pragma once

#include "sfc/sfc_speed_controller.hpp"
#include "sfc_integration/sfc_speed_controller_datamodel.hpp"
#include "yeet/application_interface.hpp"

namespace yeet::application::sfc::integration {

class sfc_speed_controller_runner
    : public yeet::base_application<sfc_speed_controller_datamodel::inputs, sfc_speed_controller_datamodel::outputs,
                                    sfc_speed_controller_datamodel::parameters> {
public:
    using datamodel = sfc_speed_controller_datamodel;

    static constexpr double sampling_time = 1e-3;

    sfc_speed_controller_runner() : base_application(sampling_time), _sfc_speed_controller(duration_t{sampling_time}) {}

    void initialize() override;
    void run() override;
    void stop() override;

private:
    void set_parameters();
    void set_parameter_callback();

    sfc::sfc_speed_controller _sfc_speed_controller;
};

}  // namespace yeet::application::sfc::integration
