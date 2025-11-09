#pragma once

#include "sfc/sfc_excitation_controller.hpp"
#include "sfc_integration/sfc_excitation_controller_datamodel.hpp"
#include "yeet/application_interface.hpp"
#include "yeet_control/constants.hpp"

namespace yeet::application::sfc::integration {

class sfc_excitation_controller_runner : public yeet::base_application<sfc_excitation_controller_datamodel::inputs,
                                             sfc_excitation_controller_datamodel::outputs,
                                             sfc_excitation_controller_datamodel::parameters> {
public:
    using datamodel = sfc_excitation_controller_datamodel;

    static constexpr duration_t sampling_time = ms(1.0);

    sfc_excitation_controller_runner() : base_application(sampling_time), _sfc_excitation_controller(sampling_time) {}

    void register_datamodel(yeet::model_registry& registry) override;
    void initialize() override;
    void run() override;
    void stop() override;

private:
    void set_parameters();
    void set_parameter_callback();

    sfc::sfc_excitation_controller _sfc_excitation_controller;
};

}  // namespace yeet::application::sfc::integration
