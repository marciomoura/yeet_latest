#pragma once

#include "indirect_excitation_2mva_plant_model.h"
#include "synchronous_machine_infinite_bus_integration_model.hpp"
#include "yeet/application_interface.hpp"

namespace yeet::application::mpc {

class indirect_excitation_2mva_plant_model_integration_runner
    : public yeet::base_application<synchronous_machine_infinite_bus_integration_datamodel::inputs,
                                    synchronous_machine_infinite_bus_integration_datamodel::outputs,
                                    synchronous_machine_infinite_bus_integration_datamodel::parameters> {
public:
    using datamodel = synchronous_machine_infinite_bus_integration_datamodel;

    static constexpr double sampling_time = 0.0001;

    indirect_excitation_2mva_plant_model_integration_runner() : base_application(sampling_time) {}

    // datamodel get_datamodel() { return datamodel{&_inputs, &_outputs, &_parameters}; }

    // void register_datamodel(yeet::model_registry& registry) override { get_datamodel().register_data_model(registry);
    // }

    void initialize() override;
    void run() override;
    void stop() override;

private:
    void set_inputs();
    void set_outputs();

    plant::indirect_excitation_2mva _model;
};

}  // namespace yeet::application::mpc