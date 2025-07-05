#pragma once

#include "mpc/model.h"
#include "mpc/mpc.h"
#include "mpc_integration_model.hpp"
#include "yeet/application_interface.hpp"

namespace yeet::application::mpc {

class mpc_integration_runner
    : public yeet::base_application<mpc_integration_datamodel::inputs, mpc_integration_datamodel::outputs,
                                    mpc_integration_datamodel::parameters> {
public:
    using datamodel = mpc_integration_datamodel;

    static constexpr double sampling_time = 50e-3;

    mpc_integration_runner() : base_application(sampling_time) {}

    // void register_datamodel(yeet::model_registry& registry) override { get_datamodel().register_data_model(registry);
    // }

    void initialize() override;
    void run() override;
    void stop() override;

private:
    void set_parameters();
    void set_parameter_callback();
    operating_point_type get_operating_point();
    operating_point_type get_initial_operating_point();

    static constexpr std::size_t Np_Size = 10;
    excitation_mpc<Np_Size> _unconstrained_excitation_mpc;
};

}  // namespace yeet::application::mpc