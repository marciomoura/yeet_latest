#pragma once

#include "sfc/sfc_datamodels.hpp"
#include "sfc/sfc_grid_side_observer.hpp"
#include "yeet/application_interface.hpp"

namespace yeet::application::sfc {

class sfc_grid_observer_runner
    : public yeet::base_application<sfc_grid_observer_datamodel::inputs, sfc_grid_observer_datamodel::outputs,
                                    sfc_grid_observer_datamodel::parameters> {
public:
    using datamodel = sfc_grid_observer_datamodel;

    static constexpr double sampling_time = 100e-6;

    sfc_grid_observer_runner() : base_application(sampling_time), _observer(duration_t{sampling_time}) {}

    void initialize() override {
        // Configure observer with parameters
        _observer.configure_frequency_estimator_params(_parameters.nominal_frequency.get(),
                                                       _parameters.pll_bandwidth.get());
        _observer.reset();
    }

    void run() override {
        // Update observer with inputs
        _observer.update(_inputs.grid_voltages.get(), _inputs.enable.get());

        // Set outputs
        _outputs.estimated_angle = _observer.get_estimated_angle();
        _outputs.estimated_frequency = _observer.get_estimated_frequency_pu();
        _outputs.is_locked = _observer.is_locked();
    }

    void stop() override { _observer.reset(); }

private:
    sfc_grid_side_observer _observer;
};

}  // namespace yeet::application::sfc