#pragma once

#include "sfc_integration/sfc_grid_side_hardware_mock_runner.hpp"
#include "sfc_integration/sfc_grid_side_measurements_runner.hpp"
#include "sfc_integration/sfc_machine_side_hardware_mock_runner.hpp"
#include "sfc_integration/sfc_machine_side_measurements_runner.hpp"
#include "yeet_control/units.hpp"

namespace yeet::application::sfc::integration::plant_hardware {

/**
 * @file plant_hardware_definition.hpp
 * @brief Defines hardware-specific constants and configuration functions.
 *
 * This file serves as a single source of truth for the physical hardware's
 * measurement chain, based on the provided signal definition table. It includes
 * definitions for transducers (ASI) and ADC cards (ICAM).
 *
 * The measurement chain is modeled as:
 * 1. Physical Signal (e.g., 200V) -> Transducer (e.g., ASI board)
 * 2. Transducer Output (e.g., 7V) -> ADC Card (e.g., ICAM board)
 * 3. ADC Card Output -> Digital Counts (e.g., 23000)
 *
 * The `adc_mock` simulates this by applying a `gain` (from the transducer)
 * and then quantizing the result based on the ADC card's `bits` and `full_scale_value`.
 */

//==============================================================================
// HARDWARE CONSTANTS
//==============================================================================

/**
 * @brief Defines the properties of the primary ICAM ADC card.
 *
 * Based on the table, this card has a bipolar input range and provides
 * 16-bit resolution.
 */
namespace icam_adc {
/// @brief Nominal ADC resolution in bits (signed bipolar). 10 V -> 32767 counts => 16-bit.
constexpr int bits = 16;
/// @brief Bipolar input full scale ( +/− full_scale_voltage ).
constexpr real_t full_scale_voltage = 10.0;  // Engineering full scale (physical card limit).
/// @brief Maximum positive code (2^(bits-1) - 1).
constexpr int max_code = (1 << (bits - 1)) - 1;  // 32767
/// @brief Minimum negative code (−2^(bits-1)).
constexpr int min_code = -(1 << (bits - 1));  // -32768
/// @brief LSB size in volts referred to card input.
constexpr real_t lsb_voltage = full_scale_voltage / static_cast<real_t>(max_code);
}  // namespace icam_adc

/**
 * @brief Defines the properties of the ASI signal conditioning board (transducers).
 *
 * This board converts physical voltage and current measurements into a
 * voltage signal compatible with the ICAM ADC card.
 */
namespace asi_transducer {
/// @brief Voltage transducer gain (board output voltage per primary line-to-line volts).
/// 100 V (primary) -> 3.5 V (secondary) => 0.035 V_out / V_primary.
constexpr real_t voltage_transducer_gain = real_t(3.5 / 100.0);
/// @brief Current transducer gain (board output volts per primary amps).
/// 5 A (primary) -> 5 V (secondary) => 1.0 V_out / A_primary.
constexpr real_t current_transducer_gain = real_t(1.0);
/// @brief Recommended anti-aliasing cutoff (from specification).
constexpr frequency_t default_cutoff_frequency = frequency_t{2500.0};
/// @brief Derived effective voltage channel full-scale at primary side (approx.) in Volts line-to-line.
constexpr real_t effective_primary_voltage_full_scale =
    icam_adc::full_scale_voltage / voltage_transducer_gain;  // ~285.714 V
/// @brief Derived effective current channel full-scale at primary side in Amps.
constexpr real_t effective_primary_current_full_scale = icam_adc::full_scale_voltage / current_transducer_gain;  // 10 A
}  // namespace asi_transducer

//==============================================================================
// DERIVED TRANSFORMER RATIOS FOR TARGET PER-UNIT MEASUREMENT SPANS
//==============================================================================
/**
 * We target observable measurement spans of:
 *  - Voltage: up to 1.3 pu of the selected base (line-to-line RMS base translated to peak phase internally).
 *  - Current: up to 2.0 pu of the selected base current.
 * The ADC secondary (post transducer) limits are fixed by ICAM (±10 V) and ASI gains.
 * We choose PT and CT such that: secondary_peak <= full_scale (avoid clipping at max expected pu).
 *
 * Let V_base_LL_rms be the grid/machine base voltage (provided when constructing the per-unit system).
 * Peak phase base used in per-unit system is: V_base_peak_phase = V_base_LL_rms * sqrt(2) / sqrt(3).
 * Max primary peak phase we want to capture: 1.3 * V_base_peak_phase.
 * Secondary peak (volts) = (Primary_peak_phase / PT_ratio) * voltage_transducer_gain.
 * We require: Secondary_peak <= icam_adc::full_scale_voltage.
 * => PT_ratio >= (Primary_peak_phase * voltage_transducer_gain) / full_scale_voltage.
 * In absence of runtime base values here, we provide helper functions below that, given runtime base values,
 * compute and stage transformer ratios into the hardware mock parameters (as direct ratios or by calling
 * configure_* after instantiation). For staging-only (static) we keep defaults = 1.0.
 */

// Forward helper to compute PT ratio at runtime (implemented inline for header-only staging utilities)
// Input: base_peak_phase_voltage (the per-unit base voltage already expressed as peak phase, consistent with PU system)
static inline real_t compute_pt_ratio_for_max_voltage(voltage_t base_peak_phase_voltage, real_t pu_span = real_t{1.3})
{
    // Desired maximum primary peak-line voltage span expressed in volts (quantity aware)
    const voltage_t max_primary_peak_line_voltage = sqrt_3 * base_peak_phase_voltage * pu_span;
    // PT_ratio >= (primary_peak_phase * transducer_gain) / adc_full_scale
    real_t required_ratio = (max_primary_peak_line_voltage.value() * asi_transducer::voltage_transducer_gain) /
                            icam_adc::full_scale_voltage;
    if (required_ratio < real_t{1.0}) required_ratio = real_t{1.0};
    return required_ratio;
}

static inline real_t compute_ct_ratio_for_max_current(current_t base_current, real_t pu_span = real_t{2.0})
{
    const current_t max_primary_current = pu_span * base_current;  // peak or RMS alignment assumed consistent
    // Secondary volts = (I_primary / CT_ratio) * current_transducer_gain <= full_scale
    // => CT_ratio >= (I_primary * gain) / full_scale
    real_t required_ratio =
        (max_primary_current * asi_transducer::current_transducer_gain) / icam_adc::full_scale_voltage;
    if (required_ratio < real_t{1.0}) required_ratio = real_t{1.0};
    return required_ratio;
}

//==============================================================================
// PARAMETER STAGING HELPERS (WRITE INTO RUNNER PARAMETER INTERFACES)
//==============================================================================

/**
 * @brief Configure parameters of the grid-side hardware mock runner.
 *
 * This does NOT touch the underlying mock directly (no get_hardware_mock()),
 * it only sets the exposed parameter interface so the runner's normal
 * initialize() / runtime callback logic applies the settings. In production
 * builds where the mock is not instantiated, these parameter writes are inert.
 */
static inline void stage_grid_side_hardware_mock_parameters(sfc_grid_side_hardware_mock_runner& mock_runner)
{
    auto& params = mock_runner.get_parameters();
    params.voltage_adc_gain.write(asi_transducer::voltage_transducer_gain);
    params.voltage_adc_cutoff_freq.write(asi_transducer::default_cutoff_frequency);
    params.voltage_adc_bits.write(icam_adc::bits);
    params.voltage_adc_full_scale.write(icam_adc::full_scale_voltage);

    params.current_adc_gain.write(asi_transducer::current_transducer_gain);
    params.current_adc_cutoff_freq.write(asi_transducer::default_cutoff_frequency);
    params.current_adc_bits.write(icam_adc::bits);
    params.current_adc_full_scale.write(icam_adc::full_scale_voltage);
    // Default transformer ratios (can be overwritten later via ratio staging helpers)
    params.pt_ratio.write(real_t{1.0});
    params.ct_ratio.write(real_t{1.0});
}

/**
 * @brief Configure parameters of the machine-side hardware mock runner.
 */
static inline void stage_machine_side_hardware_mock_parameters(sfc_machine_side_hardware_mock_runner& mock_runner)
{
    auto& params = mock_runner.get_parameters();
    params.voltage_adc_gain.write(asi_transducer::voltage_transducer_gain);
    params.voltage_adc_cutoff_freq.write(asi_transducer::default_cutoff_frequency);
    params.voltage_adc_bits.write(icam_adc::bits);
    params.voltage_adc_full_scale.write(icam_adc::full_scale_voltage);
    params.pt_ratio.write(real_t{1.0});
}

//==============================================================================
// RUNTIME STAGING OF TRANSFORMER RATIOS (MUST BE CALLED AFTER PER-UNIT SYSTEM EXISTS)
//==============================================================================
static inline void stage_transformer_ratios_for_grid_side(
    sfc_grid_side_hardware_mock_runner& mock_runner, const sfc::sfc_per_unit_system& pu)
{
    const real_t pt_ratio = compute_pt_ratio_for_max_voltage(pu.get_grid_bases().base_voltage_peak_phase.value());
    const real_t ct_ratio = compute_ct_ratio_for_max_current(pu.get_grid_bases().base_current_peak_phase.value());
    auto& params = mock_runner.get_parameters();
    params.pt_ratio.write(pt_ratio);
    params.ct_ratio.write(ct_ratio);
}

static inline void stage_transformer_ratios_for_machine_side(
    sfc_machine_side_hardware_mock_runner& mock_runner, const sfc::sfc_per_unit_system& pu)
{
    const real_t pt_ratio = compute_pt_ratio_for_max_voltage(pu.get_machine_bases().base_voltage_peak_phase.value());
    auto& params = mock_runner.get_parameters();
    params.pt_ratio.write(pt_ratio);
}

//==============================================================================
// MEASUREMENT CHAIN CONFIGURATION (ALGORITHMIC SIDE)
//==============================================================================

/**
 * @brief Configure grid-side measurement processing (counts → real → pu).
 *
 * Keeps separation from hardware mock; call this even when the hardware mock
 * is not present (e.g., HIL where real counts come from device drivers).
 */
static inline void stage_grid_side_measurement_parameters(sfc_grid_side_measurements_runner& measurement_runner)
{
    auto& params = measurement_runner.get_parameters();
    params.voltage_adc_bits.write(icam_adc::bits);
    params.voltage_adc_full_scale.write(icam_adc::full_scale_voltage);
    params.current_adc_bits.write(icam_adc::bits);
    params.current_adc_full_scale.write(icam_adc::full_scale_voltage);
    params.voltage_transducer_gain.write(asi_transducer::voltage_transducer_gain);
    params.current_transducer_gain.write(asi_transducer::current_transducer_gain);
}

/**
 * @brief Configure machine-side measurement processing (counts → real → pu).
 */
static inline void stage_machine_side_measurement_parameters(sfc_machine_side_measurements_runner& measurement_runner)
{
    auto& params = measurement_runner.get_parameters();
    params.voltage_adc_bits.write(icam_adc::bits);
    params.voltage_adc_full_scale.write(icam_adc::full_scale_voltage);
    params.voltage_transducer_gain.write(asi_transducer::voltage_transducer_gain);
}

//------------------------------------------------------------------------------
// RUNTIME STAGING OF TRANSFORMER RATIOS FOR MEASUREMENT RUNNERS
//------------------------------------------------------------------------------
static inline void stage_transformer_ratios_for_grid_side_measurements(
    sfc_grid_side_measurements_runner& measurement_runner, const sfc::sfc_per_unit_system& pu)
{
    const voltage_t base_peak_phase_voltage{pu.get_grid_bases().base_voltage_peak_phase.value()};
    const current_t base_peak_phase_current{pu.get_grid_bases().base_current_peak_phase.value()};
    const real_t pt_ratio = compute_pt_ratio_for_max_voltage(base_peak_phase_voltage);
    const real_t ct_ratio = compute_ct_ratio_for_max_current(base_peak_phase_current);
    auto& params = measurement_runner.get_parameters();
    params.pt_ratio.write(pt_ratio);
    params.ct_ratio.write(ct_ratio);
}

static inline void stage_transformer_ratios_for_machine_side_measurements(
    sfc_machine_side_measurements_runner& measurement_runner, const sfc::sfc_per_unit_system& pu)
{
    const voltage_t base_peak_phase_voltage{pu.get_machine_bases().base_voltage_peak_phase.value()};
    const real_t pt_ratio = compute_pt_ratio_for_max_voltage(base_peak_phase_voltage);
    auto& params = measurement_runner.get_parameters();
    params.pt_ratio.write(pt_ratio);
}

//==============================================================================
// CONFIGURATION FUNCTIONS
//==============================================================================

/**
 * @brief Configures the grid-side hardware mock and measurement runners.
 *
 * This function ensures that both the hardware simulation and the measurement
 * processing logic use the exact same scaling and conversion parameters,
 * preventing data misinterpretation.
 *
 * @param mock_runner The hardware mock runner to configure.
 * @param measurement_runner The measurement processing runner to configure.
 */
static inline void configure_grid_side_hardware_mock(
    sfc_grid_side_hardware_mock_runner& mock_runner, sfc_grid_side_measurements_runner& measurement_runner)
{
    stage_grid_side_hardware_mock_parameters(mock_runner);       // mock parameters
    stage_grid_side_measurement_parameters(measurement_runner);  // measurement runner parameters
}

/**
 * @brief Configures the machine-side hardware mock and measurement runners.
 *
 * Assumes the machine-side voltage measurements use the same hardware chain
 * as the grid-side voltage measurements.
 *
 * @param mock_runner The hardware mock runner to configure.
 * @param measurement_runner The measurement processing runner to configure.
 */
static inline void configure_machine_side_hardware_mock(
    sfc_machine_side_hardware_mock_runner& mock_runner, sfc_machine_side_measurements_runner& measurement_runner)
{
    stage_machine_side_hardware_mock_parameters(mock_runner);
    stage_machine_side_measurement_parameters(measurement_runner);
}

}  // namespace yeet::application::sfc::integration::plant_hardware