#pragma once

#include <memory>

#include "sfc/sfc_plant_model_6p_6p_machine.hpp"
#include "sfc_integration/sfc_plant_model_machine_datamodel.hpp"
#include "yeet/application_interface.hpp"

namespace yeet::application::sfc::integration {

/**
 * @brief SFC Plant Model Machine Runner
 *
 * This runner integrates the SFC 6-pulse plant model with complete synchronous machine
 * with the application framework. It provides simplified control with:
 * - Fixed or forced machine speed
 * - Configurable field excitation voltage
 * - Direct pulse control for grid and machine-side converters
 *
 * Based on sfc_plant_model_6p_6p_machine.
 */
class sfc_plant_model_machine_runner : public yeet::base_application<sfc_plant_model_machine_datamodel::inputs,
                                           sfc_plant_model_machine_datamodel::outputs,
                                           sfc_plant_model_machine_datamodel::parameters> {
public:
    using datamodel = sfc_plant_model_machine_datamodel;
    static constexpr double sampling_time = sfc::sfc_plant_model_6p_6p_machine::kSamplingTime;

    /**
     * @brief Constructor
     */
    sfc_plant_model_machine_runner();

    void initialize() override;
    void run() override;
    void stop() override;

    sfc::sfc_plant_model_6p_6p_machine& get_plant_model() { return _plant; }

private:
    void set_parameters();

    sfc::sfc_plant_model_6p_6p_machine _plant;
    std::array<real_t, 3> _load_coefficients{0.0, 0.0, 0.0};
};

}  // namespace yeet::application::sfc::integration
