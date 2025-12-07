#pragma once

#include "sfc_integration/sfc_grid_side_external_ios_datamodel.hpp"
#include "yeet/model_registry.hpp"
#include "yeet/parameter.hpp"
#include "yeet/utils/parameter_accessors.hpp"

namespace yeet::application::sfc::integration {

class sfc_io_mock_runner {
public:
    struct inputs {};

    struct outputs {};

    struct parameters {
        parameter<bool> auxiliaries_are_on{false};
        parameter<bool> mcb_is_closed{false};
        parameter<bool> start_command{false};
        parameter<bool> stop_command{false};
        parameter<bool> shutdown_request{false};
        parameter<bool> breaker_is_closed{false};
        parameter<bool> breaker_is_tripping{false};
        parameter<bool> emergency_off{false};
        parameter<bool> door_is_open{false};
    };

    explicit sfc_io_mock_runner(sfc_grid_side_external_ios_datamodel& external_ios);

    void register_datamodel(yeet::model_registry& registry);
    void initialize();
    void run();
    void stop();

    inputs& get_inputs() { return _inputs; }
    const inputs& get_inputs() const { return _inputs; }
    outputs& get_outputs() { return _outputs; }
    const outputs& get_outputs() const { return _outputs; }
    parameters& get_parameters() { return _parameters; }
    const parameters& get_parameters() const { return _parameters; }

private:
    sfc_grid_side_external_ios_datamodel& _external_ios;
    inputs _inputs;
    outputs _outputs;
    parameters _parameters;
};

}  // namespace yeet::application::sfc::integration
