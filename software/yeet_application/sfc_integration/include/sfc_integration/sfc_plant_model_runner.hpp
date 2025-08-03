#pragma once

#include <memory>

#include "sfc/sfc_6p_6p_plant_model.hpp"
#include "sfc/sfc_per_unit_system.hpp"
#include "sfc_integration/sfc_plant_model_datamodel.hpp"
#include "yeet/application_interface.hpp"
#include "yeet_control/mechanical_load.hpp"

namespace yeet::application::sfc::integration {

/**
 * @brief SFC Plant Model Runner
 *
 * This runner integrates the SFC 6-pulse plant model with the application framework.
 * It manages the plant model execution, mechanical load dynamics, and per-unit system
 * conversion for interfacing with other control components.
 *
 * Based on the sfc_plant_model_simulator but adapted for the integration framework.
 */
class sfc_plant_model_runner : public yeet::base_application<sfc_plant_model_datamodel::inputs,
                                   sfc_plant_model_datamodel::outputs,
                                   sfc_plant_model_datamodel::parameters> {
public:
    using datamodel = sfc_plant_model_datamodel;
    static constexpr double sampling_time = sfc::sfc_6p_6p_plant_model::kSamplingTime;

    /**
     * @brief Constructor
     * @param per_unit_system Shared pointer to the per-unit system for unit conversions
     */
    explicit sfc_plant_model_runner(std::shared_ptr<sfc::sfc_per_unit_system> per_unit_system);

    void initialize() override;
    void run() override;
    void stop() override;

private:
    void set_parameters();
    void set_parameter_callback();
    void update_mechanical_load();
    void convert_inputs();
    void convert_outputs();

    // Core components
    sfc::sfc_6p_6p_plant_model _plant;
    control::mechanical_load _mechanical_load;
    std::shared_ptr<sfc::sfc_per_unit_system> _per_unit_system;

    // Configuration
    datamodel::machine_operation_mode _mode = datamodel::machine_operation_mode::fixed_speed;
};

}  // namespace yeet::application::sfc::integration
