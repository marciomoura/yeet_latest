#pragma once

#include <memory>

#include "sfc/sfc_per_unit_system.hpp"
#include "sfc/sfc_plant_model_voltage_behind_reactance.hpp"
#include "sfc_integration/sfc_plant_model_datamodel.hpp"
#include "yeet/application_interface.hpp"

namespace yeet::application::sfc::integration {

/**
 * @brief SFC Plant Model Runner
 *
 * This runner integrates the SFC 6-pulse plant model with the application framework.
 * It manages the plant model execution, mechanical load dynamics, and per-unit system
 * conversion for interfacing with other control components.
 *
 * Based on the sfc_plant_model_voltage_behind_reactance.
 */
class sfc_plant_model_runner : public yeet::base_application<sfc_plant_model_datamodel::inputs,
                                   sfc_plant_model_datamodel::outputs,
                                   sfc_plant_model_datamodel::parameters> {
public:
    using datamodel = sfc_plant_model_datamodel;
    static constexpr double sampling_time = sfc::sfc_plant_model_voltage_behind_reactance::kSamplingTime;

    /**
     * @brief Constructor
     * @param per_unit_system Shared pointer to the per-unit system for unit conversions
     */
    explicit sfc_plant_model_runner(std::shared_ptr<sfc::sfc_per_unit_system> per_unit_system);

    void register_datamodel(yeet::telemetry::parameter_handler& handler) override;
    void initialize() override;
    void run() override;
    void stop() override;

    sfc::sfc_plant_model_voltage_behind_reactance& get_plant_model() { return *_plant; }
    const std::shared_ptr<sfc::sfc_per_unit_system>& get_per_unit_system() const noexcept { return _per_unit_system; }

private:
    void set_parameters();
    void set_parameter_callback();
    void set_operation_mode();
    void set_voltage_control_mode();
    void set_mechanical_load();
    void set_grid_operating_point();
    void handle_topology_change();
    sfc::plant_topology_type convert_topology(datamodel::topology_type topology) const noexcept;
    void configure_plant_bases(speed_pu_t speed_pu);

    std::shared_ptr<sfc::sfc_per_unit_system> _per_unit_system;

    std::unique_ptr<sfc::sfc_plant_model_voltage_behind_reactance> _plant;
};

}  // namespace yeet::application::sfc::integration
