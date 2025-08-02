#pragma once

#include <memory>

#include "sfc/sfc_machine_side_filtering.hpp"
#include "sfc/sfc_machine_side_measurements.hpp"
#include "sfc/sfc_per_unit_system.hpp"
#include "sfc_integration/sfc_machine_side_measurements_datamodel.hpp"
#include "yeet/application_interface.hpp"

namespace yeet::application::sfc::integration {

class sfc_machine_side_measurements_runner
    : public yeet::base_application<sfc_machine_side_measurements_datamodel::inputs,
                                    sfc_machine_side_measurements_datamodel::outputs,
                                    sfc_machine_side_measurements_datamodel::parameters> {
public:
    using datamodel = sfc_machine_side_measurements_datamodel;

    static constexpr double sampling_time = 100e-6;

    sfc_machine_side_measurements_runner(std::shared_ptr<sfc::sfc_per_unit_system> per_unit_system)
        : base_application(sampling_time),
          _sfc_machine_side_measurements(),
          _sfc_machine_side_filtering(duration_t{sampling_time}),
          _per_unit_system(per_unit_system) {}

    void initialize() override;
    void run() override;
    void stop() override;

private:
    void set_parameters();
    void set_parameter_callback();

    sfc::sfc_machine_side_measurements _sfc_machine_side_measurements;
    sfc::sfc_machine_side_filtering _sfc_machine_side_filtering;
    std::shared_ptr<sfc::sfc_per_unit_system> _per_unit_system;
};

}  // namespace yeet::application::sfc::integration
