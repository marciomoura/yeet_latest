#pragma once

#include <memory>

#include "sfc/sfc_commutation_mask.hpp"
#include "sfc/sfc_commutation_monitor.hpp"
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

    static constexpr duration_t sampling_time = us(200);

    sfc_machine_side_measurements_runner(std::shared_ptr<sfc::sfc_per_unit_system> per_unit_system)
        : base_application(sampling_time), _per_unit_system(per_unit_system)
    {
    }

    void initialize() override;
    void run() override;
    void stop() override;

private:
    void set_filter_parameters();
    void set_parameter_callback();
    void set_commutation_parameters();

    sfc_machine_side_measurements _measurements{};

    sfc_machine_side_filtering _measurement_filters{duration_t{sampling_time}};

    sfc_commutation_monitor _commutation_monitor{sfc_commutation_monitor::Config{
        .commutation_start_threshold = voltage_pu_t{0.1f},  // These values have to be tested...
        .commutation_end_threshold = voltage_pu_t{0.02f},   // These values have to be tested...
    }};

    sfc_commutation_mask _commutation_mask{duration_t{sampling_time}};

    std::shared_ptr<sfc::sfc_per_unit_system> _per_unit_system;
};

}  // namespace yeet::application::sfc::integration
