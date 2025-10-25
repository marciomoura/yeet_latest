#pragma once

#include <memory>

#include "sfc/sfc_commutation_mask.hpp"
#include "sfc/sfc_commutation_monitor.hpp"
#include "sfc/sfc_grid_side_filtering.hpp"
#include "sfc/sfc_grid_side_measurements.hpp"
#include "sfc/sfc_per_unit_system.hpp"
#include "sfc_integration/sfc_grid_side_measurements_datamodel.hpp"
#include "yeet/application_interface.hpp"

namespace yeet::application::sfc::integration {

class sfc_grid_side_measurements_ideal_runner
    : public yeet::base_application<sfc_grid_side_measurements_ideal_datamodel::inputs,
          sfc_grid_side_measurements_ideal_datamodel::outputs,
          sfc_grid_side_measurements_ideal_datamodel::parameters> {
public:
    using datamodel = sfc_grid_side_measurements_ideal_datamodel;

    static constexpr duration_t sampling_time = us(200);

    sfc_grid_side_measurements_ideal_runner(std::shared_ptr<sfc_per_unit_system> per_unit_system)
        : base_application(sampling_time), _per_unit_system(per_unit_system)
    {
    }

    void register_datamodel(yeet::model_registry& registry) override;
    void initialize() override;
    void run() override;
    void stop() override;

private:
    void set_filter_parameters();
    void set_commutation_parameters();
    void set_parameter_callback();

    sfc_grid_side_measurements_ideal _measurements;

    sfc_grid_side_filtering _measurement_filters{duration_t{sampling_time}};

    sfc_commutation_monitor _commutation_monitor{sfc_commutation_monitor::Config{
        .commutation_start_threshold = voltage_pu_t{0.1f},  // These values have to be tested...
        .commutation_end_threshold = voltage_pu_t{0.02f},   // These values have to be tested...
    }};

    sfc_commutation_mask _commutation_mask{duration_t{sampling_time}};

    std::shared_ptr<sfc_per_unit_system> _per_unit_system;
};

/**
 * @brief Runner that interfaces with ADC-based grid-side measurement data.
 *
 * This runner consumes ADC count signals (voltages & currents) provided by the
 * grid-side hardware mock, converts them to per-unit quantities using the
 * sfc_grid_side_measurements processing chain, applies commutation masking and
 * filtering, and exposes processed measurement signals through the datamodel
 * outputs.
 */
class sfc_grid_side_measurements_runner : public yeet::base_application<sfc_grid_side_measurements_datamodel::inputs,
                                              sfc_grid_side_measurements_datamodel::outputs,
                                              sfc_grid_side_measurements_datamodel::parameters> {
public:
    using datamodel = sfc_grid_side_measurements_datamodel;

    static constexpr duration_t sampling_time = us(200);

    explicit sfc_grid_side_measurements_runner(std::shared_ptr<sfc_per_unit_system> per_unit_system)
        : base_application(sampling_time), _per_unit_system(std::move(per_unit_system))
    {
    }

    void register_datamodel(yeet::model_registry& registry) override;
    void initialize() override;
    void run() override;
    void stop() override;

private:
    void set_filter_parameters();
    void set_commutation_parameters();
    void set_parameter_callback();
    void set_measurement_conversion_parameters();

    sfc_grid_side_measurements _measurements;  // ADC-based measurement processing

    sfc_grid_side_filtering _measurement_filters{duration_t{sampling_time}};

    sfc_commutation_monitor _commutation_monitor{sfc_commutation_monitor::Config{
        .commutation_start_threshold = voltage_pu_t{0.1f},
        .commutation_end_threshold = voltage_pu_t{0.02f},
    }};

    sfc_commutation_mask _commutation_mask{duration_t{sampling_time}};

    std::shared_ptr<sfc_per_unit_system> _per_unit_system;
};

}  // namespace yeet::application::sfc::integration
