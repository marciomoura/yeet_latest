#pragma once

#include "sfc/sfc_dc_link_measurements.hpp"
#include "sfc_integration/sfc_dc_link_measurements_datamodel.hpp"
#include "yeet/application_interface.hpp"

namespace yeet::application::sfc::integration {

class sfc_dc_link_measurements_runner
    : public yeet::base_application<sfc_dc_link_measurements_datamodel::inputs,
                                    sfc_dc_link_measurements_datamodel::outputs,
                                    sfc_dc_link_measurements_datamodel::parameters> {
public:
    using datamodel = sfc_dc_link_measurements_datamodel;

    static constexpr double sampling_time = 100e-6;

    sfc_dc_link_measurements_runner()
        : base_application(sampling_time), _sfc_dc_link_measurements() {}

    void initialize() override;
    void run() override;
    void stop() override;

private:
    void set_parameters();
    void set_parameter_callback();

    sfc::sfc_dc_link_measurements _sfc_dc_link_measurements;
    sfc_per_unit_system::dc_link_bases _dc_link_bases;
};

}  // namespace yeet::application::sfc::integration
