#pragma once

#include <memory>

#include "sfc/sfc_dc_link_filtering.hpp"
#include "sfc/sfc_dc_link_measurements.hpp"
#include "sfc/sfc_per_unit_system.hpp"
#include "sfc_integration/sfc_dc_link_measurements_datamodel.hpp"
#include "yeet/application_interface.hpp"

namespace yeet::application::sfc::integration {

class sfc_dc_link_measurements_runner : public yeet::base_application<sfc_dc_link_measurements_datamodel::inputs,
                                            sfc_dc_link_measurements_datamodel::outputs,
                                            sfc_dc_link_measurements_datamodel::parameters> {
public:
    using datamodel = sfc_dc_link_measurements_datamodel;

    static constexpr duration_t sampling_time = us(200);

    sfc_dc_link_measurements_runner(std::shared_ptr<sfc::sfc_per_unit_system> per_unit_system)
        : base_application(sampling_time), _per_unit_system(per_unit_system)
    {
    }

    void initialize() override;
    void run() override;
    void stop() override;

private:
    void set_parameters();
    void set_parameter_callback();

    sfc::sfc_dc_link_measurements _sfc_dc_link_measurements{};

    sfc::sfc_dc_link_filtering _sfc_dc_link_filtering{duration_t{sampling_time}};

    std::shared_ptr<sfc::sfc_per_unit_system> _per_unit_system;
};

}  // namespace yeet::application::sfc::integration
