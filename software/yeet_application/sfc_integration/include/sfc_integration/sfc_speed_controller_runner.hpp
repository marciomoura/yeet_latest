#pragma once

#include "sfc/sfc_speed_controller.hpp"
#include "sfc_integration/sfc_speed_controller_datamodel.hpp"
#include "yeet/application_interface.hpp"
#include "yeet/signal.hpp"
#include "yeet_control/constants.hpp"
#include "yeet_control/edge_detector.hpp"

namespace yeet::application::sfc::integration {

class sfc_speed_controller_runner : public yeet::base_application<sfc_speed_controller_datamodel::inputs,
                                        sfc_speed_controller_datamodel::outputs,
                                        sfc_speed_controller_datamodel::parameters> {
public:
    using datamodel = sfc_speed_controller_datamodel;

    static constexpr duration_t sampling_time = ms(10);

    sfc_speed_controller_runner() : base_application(sampling_time), _sfc_speed_controller(sampling_time) {}

    void register_datamodel(yeet::telemetry::parameter_handler& handler) override;
    void initialize() override;
    void run() override;
    void stop() override;

private:
    void set_parameters();
    void set_parameter_callback();

    sfc::sfc_speed_controller _sfc_speed_controller;

    yeet::control::edge_detector _release_edge_detector;
};

}  // namespace yeet::application::sfc::integration
