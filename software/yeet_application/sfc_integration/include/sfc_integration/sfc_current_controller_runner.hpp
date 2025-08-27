#pragma once

#include "sfc/sfc_current_controller.hpp"
#include "sfc/sfc_per_unit_system.hpp"
#include "sfc_integration/sfc_current_controller_datamodel.hpp"
#include "yeet/application_interface.hpp"

namespace yeet::application::sfc::integration {

class sfc_current_controller_runner : public yeet::base_application<sfc_current_controller_datamodel::inputs,
                                          sfc_current_controller_datamodel::outputs,
                                          sfc_current_controller_datamodel::parameters> {
public:
    using datamodel = sfc_current_controller_datamodel;

    static constexpr duration_t sampling_time = ms(1.0);

    sfc_current_controller_runner(std::shared_ptr<sfc::sfc_per_unit_system> per_unit_system)
        : base_application(sampling_time), _sfc_current_controller(sampling_time), _per_unit_system(per_unit_system)
    {
    }

    void initialize() override;
    void run() override;
    void stop() override;

private:
    void set_controller_parameters();
    void set_limit_parameters();

    void set_parameter_callback();

    std::shared_ptr<sfc::sfc_per_unit_system> _per_unit_system;
    sfc::sfc_current_controller _sfc_current_controller;
};

}  // namespace yeet::application::sfc::integration
