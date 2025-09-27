#pragma once

#include "sfc/sfc_types.hpp"
#include "yeet/parameter.hpp"
#include "yeet/signal.hpp"
#include "yeet_control/units.hpp"

namespace yeet::application::sfc::integration {

/**
 * @class sfc_operation_mode_monitor_datamodel
 * @brief Defines the data interface for the SFC Operation Mode Monitor.
 *
 * This datamodel aggregates all signals and parameters required for detecting
 * the primary LCI operating mode (pulse vs. load-commutated) and for monitoring
 * discontinuous conduction mode based on DC link current.
 */
class sfc_operation_mode_monitor_datamodel {
public:
    /**
     * @brief Input signals for the operation mode monitor.
     */
    struct inputs {
        /// @brief System enable signal. When false, detectors are reset to a safe state.
        signal_input<bool> enable;

        /// @brief Measured machine speed in per-unit for mode detection.
        signal_input<speed_pu_t> measured_speed_pu;

        /// @brief Measured DC link current in per-unit for discontinuous conduction detection.
        signal_input<current_pu_t> dc_link_current_pu;
    };

    /**
     * @brief Output signals from the operation mode monitor.
     */
    struct outputs {
        /// @brief The current operating mode (pulse_mode or load_commutated).
        signal_output<sfc_operating_mode_t> operating_mode;

        /// @brief True for one cycle immediately after an operating mode transition.
        signal_output<bool> has_mode_changed;

        /// @brief True if the DC link current is considered zero (with on-delay).
        signal_output<bool> is_current_zero;

        /// @brief True if the system is determined to be in discontinuous conduction mode.
        signal_output<bool> is_discontinuous_mode;
    };

    /**
     * @brief Configuration parameters for the operation mode monitor.
     */
    struct parameters {
        // Mode Detector Parameters
        /// @brief Base speed for switching between pulse and load-commutated modes.
        parameter<speed_pu_t> mode_handover_speed_pu{speed_pu_t{0.25f}};

        /// @brief Hysteresis factor for mode switching to prevent chattering.
        parameter<real_t> mode_hysteresis_factor{0.15f};

        // Discontinuous Conduction Detector Parameters
        /// @brief Current level below which DC link current is considered zero.
        parameter<current_pu_t> zero_current_threshold_pu{current_pu_t{0.02f}};

        /// @brief On-delay in seconds to confirm current is zero.
        parameter<duration_t> is_zero_on_delay_s{duration_t{(1.0 / 50.0 / 6.0)}};

        parameter<duration_t> discontinuous_on_delay_s{duration_t{0.002f}};
        parameter<duration_t> discontinuous_off_delay_s{duration_t{0.05f}};
    };

private:
    inputs* _model_inputs;
    outputs* _model_outputs;
    parameters* _model_parameters;
};

}  // namespace yeet::application::sfc::integration
