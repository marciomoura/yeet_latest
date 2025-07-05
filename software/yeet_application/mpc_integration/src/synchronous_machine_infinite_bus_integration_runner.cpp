#include "mpc_integration/synchronous_machine_infinite_bus_integration_runner.hpp"

#include "plant_schematic.h"

namespace yeet::application::mpc {

void synchronous_machine_infinite_bus_integration_runner::initialize() {
    _model.initialize();

    run();

    set_outputs();
}

void synchronous_machine_infinite_bus_integration_runner::run() {
    set_inputs();

    _model.step();

    set_outputs();
}

void synchronous_machine_infinite_bus_integration_runner::stop() { _model.terminate(); }

void synchronous_machine_infinite_bus_integration_runner::set_inputs() {
    ::mpc::model::SynchronousMachineAttachedToInfiniteBusModel::ExtU_plant_schematic_T inputs{};
    inputs.field_voltage = _inputs.efd;
    inputs.mechanical_power = _parameters.pmech;

    _model.setExternalInputs(&inputs);
}

void synchronous_machine_infinite_bus_integration_runner::set_outputs() {
    auto outputs = _model.getExternalOutputs();

    _outputs.wm = outputs.wm;
    _outputs.ifd = outputs.ifd;
    _outputs.it = outputs.it;
    _outputs.id = outputs.id;
    _outputs.iq = outputs.iq;
    _outputs.vt = outputs.vt;
    _outputs.vd = outputs.vd;
    _outputs.vq = outputs.vq;
    _outputs.vg = 1.0f;
    _outputs.load_angle_deg = outputs.load_angle_deg;
    _outputs.wt = outputs.wt;

    double theta = outputs.wt;

    Eigen::Matrix2d inverse_park_matrix;
    inverse_park_matrix << std::cos(theta), -std::sin(theta), std::sin(theta), std::cos(theta);

    // Perform inverse Park transformation
    auto voltage_ab = inverse_park_matrix * Eigen::Vector2d{outputs.vd, outputs.vq};
    auto current_ab = inverse_park_matrix * Eigen::Vector2d{outputs.id, outputs.iq};

    _outputs.voltage_alpha = voltage_ab(0);
    _outputs.voltage_beta = voltage_ab(1);
    _outputs.current_alpha = current_ab(0);
    _outputs.current_beta = current_ab(1);
}

}  // namespace yeet::application::mpc