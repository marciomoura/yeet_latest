#pragma once

#include "sfc/sfc_machine_side_lookup_table.hpp"
#include "sfc_integration/sfc_machine_side_controller_datamodel.hpp"
#include "yeet/application_interface.hpp"
#include "yeet_control/first_order_low_pass_filter.hpp"

namespace yeet::application::sfc::integration {

class sfc_machine_side_controller_runner : public yeet::base_application<sfc_machine_side_controller_datamodel::inputs,
                                               sfc_machine_side_controller_datamodel::outputs,
                                               sfc_machine_side_controller_datamodel::parameters> {
public:
    using datamodel = sfc_machine_side_controller_datamodel;

    static constexpr duration_t sampling_time = ms(1.0);

    sfc_machine_side_controller_runner() : base_application(sampling_time), _firing_angle_filter(sampling_time) {}

    void register_datamodel(yeet::telemetry::parameter_handler& handler) override;
    void initialize() override;
    void run() override;
    void stop() override;

    /**
     * @brief Get access to the underlying lookup table for direct configuration
     * @return Reference to the lookup table
     */
    sfc::sfc_machine_side_firing_angle_lookup_table& get_lookup_table() { return _lookup_table; }

    /**
     * @brief Get the current lookup table (const version)
     * @return Const reference to the lookup table
     */
    const sfc::sfc_machine_side_firing_angle_lookup_table& get_lookup_table() const { return _lookup_table; }

private:
    void set_parameters();
    void set_parameter_callback();
    void recalculate_lookup_table();

    sfc::sfc_machine_side_firing_angle_lookup_table _lookup_table;

    control::first_order_low_pass_filter<real_t> _firing_angle_filter;
};

}  // namespace yeet::application::sfc::integration
