#pragma once

#include "sfc/sfc_machine_side_observer.hpp"
#include "sfc_integration/sfc_machine_side_observer_datamodel.hpp"
#include "yeet/application_interface.hpp"

namespace yeet::application::sfc::integration {

class sfc_machine_side_observer_runner : public yeet::base_application<sfc_machine_side_observer_datamodel::inputs,
                                             sfc_machine_side_observer_datamodel::outputs,
                                             sfc_machine_side_observer_datamodel::parameters> {
public:
    using datamodel = sfc_machine_side_observer_datamodel;

    static constexpr duration_t sampling_time = us(200);

    sfc_machine_side_observer_runner() : base_application(sampling_time), _sfc_machine_side_observer(sampling_time)
    {
        initialize();
    }

    void register_datamodel(yeet::model_registry& registry) override;
    void initialize() override;
    void run() override;
    void stop() override;

private:
    sfc::sfc_machine_side_observer _sfc_machine_side_observer;
    bool _previous_preset_observer{false};
};

}  // namespace yeet::application::sfc::integration
