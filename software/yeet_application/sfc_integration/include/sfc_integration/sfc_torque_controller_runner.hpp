#pragma once

#include "sfc/sfc_torque_controller.hpp"
#include "sfc_integration/sfc_torque_controller_datamodel.hpp"
#include "yeet/application_interface.hpp"
#include "yeet_control/constants.hpp"

/**
 * @file sfc_torque_controller_runner.hpp
 * @brief Integration runner for SFC Torque Controller
 *
 * Responsible for executing the torque controller, managing parameter updates,
 * and coordinating saturation feedback from the current controller.
 */

namespace yeet::application::sfc::integration {

class sfc_torque_controller_runner : public yeet::base_application<sfc_torque_controller_datamodel::inputs,
                                         sfc_torque_controller_datamodel::outputs,
                                         sfc_torque_controller_datamodel::parameters> {
public:
    using datamodel = sfc_torque_controller_datamodel;

    static constexpr duration_t sampling_time = ms(10);

    sfc_torque_controller_runner() : base_application(sampling_time), _sfc_torque_controller(sampling_time) {}

    void register_datamodel(yeet::model_registry& registry) override;
    void initialize() override;
    void run() override;
    void stop() override;

private:
    void set_parameters();
    void set_parameter_callback();

    sfc::sfc_torque_controller _sfc_torque_controller;

    bool _was_shutdown_active{false};
    bool _was_operation_on{false};
};

}  // namespace yeet::application::sfc::integration
