#pragma once

#include "sfc/sfc_current_controller.hpp"
#include "sfc_integration/sfc_current_controller_datamodel.hpp"
#include "yeet/application_interface.hpp"

namespace yeet::application::sfc::integration {

class sfc_current_controller_runner : public yeet::base_application<sfc_current_controller_datamodel::inputs,
                                          sfc_current_controller_datamodel::outputs,
                                          sfc_current_controller_datamodel::parameters> {
public:
    using datamodel = sfc_current_controller_datamodel;

    static constexpr duration_t sampling_time = ms(1.0);

    sfc_current_controller_runner() : base_application(sampling_time), _sfc_current_controller(sampling_time) {}

    void initialize() override;
    void run() override;
    void stop() override;

private:
    void set_parameters();
    void set_parameter_callback();

    sfc::sfc_current_controller _sfc_current_controller;
};

}  // namespace yeet::application::sfc::integration
