#pragma once

#include "sfc/sfc_types.hpp"
#include "yeet/signal.hpp"
#include "yeet/parameter.hpp"
#include "yeet_control/units.hpp"

namespace yeet::application::sfc {

// Grid-side observer datamodel
class sfc_grid_observer_datamodel {
public:
    struct inputs {
        yeet::signal_input<abc<voltage_pu_t>> grid_voltages;
        yeet::signal_input<bool> enable;
    };

    struct outputs {
        yeet::signal_output<angle_t> estimated_angle;
        yeet::signal_output<frequency_pu_t> estimated_frequency;
        yeet::signal_output<bool> is_locked;
    };

    struct parameters {
        yeet::parameter<frequency_t> nominal_frequency{frequency_t{50.0}};
        yeet::parameter<real_t> pll_bandwidth{10.0};
    };

    sfc_grid_observer_datamodel(inputs* model_inputs, outputs* model_outputs, parameters* model_parameters)
        : _model_inputs(model_inputs), _model_outputs(model_outputs), _model_parameters(model_parameters) {
        assert(_model_inputs != nullptr);
        assert(_model_outputs != nullptr);
        assert(_model_parameters != nullptr);
    }

    void register_data_model(yeet::model_registry& registry);

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }

private:
    inputs* _model_inputs{};
    outputs* _model_outputs{};
    parameters* _model_parameters{};
};

// Current controller datamodel
class sfc_current_controller_datamodel {
public:
    struct inputs {
        yeet::signal_input<current_pu_t> current_reference;
        yeet::signal_input<current_pu_t> measured_current;
        yeet::signal_input<voltage_pu_t> dc_voltage;
        yeet::signal_input<bool> enable;
        yeet::signal_input<sfc_operating_mode_t> operating_mode;
    };

    struct outputs {
        yeet::signal_output<angle_t> grid_firing_angle;
        yeet::signal_output<bool> is_saturated;
    };

    struct parameters {
        yeet::parameter<real_t> kp{0.2};
        yeet::parameter<duration_t> ti{duration_t{0.1}};
        yeet::parameter<current_pu_t> current_limit_min{current_pu_t{-1.5}};
        yeet::parameter<current_pu_t> current_limit_max{current_pu_t{1.5}};
    };

    sfc_current_controller_datamodel(inputs* model_inputs, outputs* model_outputs, parameters* model_parameters)
        : _model_inputs(model_inputs), _model_outputs(model_outputs), _model_parameters(model_parameters) {
        assert(_model_inputs != nullptr);
        assert(_model_outputs != nullptr);
        assert(_model_parameters != nullptr);
    }

    void register_data_model(yeet::model_registry& registry);

    inputs* get_inputs() { return _model_inputs; }
    outputs* get_outputs() { return _model_outputs; }
    parameters* get_parameters() { return _model_parameters; }

private:
    inputs* _model_inputs{};
    outputs* _model_outputs{};
    parameters* _model_parameters{};
};

// Speed controller datamodel
class sfc_speed_controller_datamodel {
public:
    struct inputs {
        yeet::signal_input<speed_pu_t> speed_reference;
        yeet::signal_input<speed_pu_t> measured_speed;
        yeet::signal_input<bool> enable;
    };

    struct outputs {
        yeet::signal_output<current_pu_t> current_reference;
        yeet::signal_output<bool> is_saturated;
    };

    struct parameters {
        yeet::parameter<real_t> kp{10.0};
        yeet::parameter<duration_t> ti{duration_t{0.5}};
        yeet::parameter<torque_pu_t> torque_limit_min{torque_pu_t{-1.5}};
        yeet::parameter<torque_pu_t> torque_limit_max{torque_pu_t{1.5}};
    };

    // ... similar structure as above
};

// Plant model datamodel
class sfc_plant_datamodel {
public:
    struct inputs {
        yeet::signal_input<angle_t> grid_firing_angle;
        yeet::signal_input<angle_t> machine_firing_angle;
        yeet::signal_input<torque_pu_t> load_torque;
    };

    struct outputs {
        yeet::signal_output<abc<voltage_pu_t>> grid_voltages;
        yeet::signal_output<abc<voltage_pu_t>> machine_voltages;
        yeet::signal_output<abc<current_t>> grid_currents;
        yeet::signal_output<abc<current_t>> machine_currents;
        yeet::signal_output<current_pu_t> dc_link_current;
        yeet::signal_output<voltage_pu_t> dc_link_voltage;
        yeet::signal_output<speed_pu_t> rotor_speed;
        yeet::signal_output<angle_t> rotor_position;
    };

    struct parameters {
        yeet::parameter<frequency_t> grid_frequency{frequency_t{50.0}};
        yeet::parameter<voltage_t> grid_voltage_rms{voltage_t{400.0}};
        yeet::parameter<frequency_t> machine_frequency{frequency_t{60.0}};
        yeet::parameter<voltage_t> machine_voltage_rms{voltage_t{400.0}};
    };

    // ... similar structure as above
};

}  // namespace yeet::application::sfc