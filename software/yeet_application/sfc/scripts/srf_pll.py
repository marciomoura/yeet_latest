"""
SRF-PLL Control Loop Analysis (Modified for Damping Control)

This script performs comprehensive analysis of Synchronous Reference Frame Phase-Locked Loop
(SRF-PLL) systems. It uses modular components for system modeling, PI controller tuning,
and result visualization.

The key change in this version is the PI controller tuning method. Instead of relying on
open-loop phase margin, it directly targets closed-loop performance metrics:
- Natural Frequency (ωn): Determines the speed of the response.
- Damping Ratio (ζ): Controls overshoot and peaking. A value >= 0.707 prevents gain peaking.

The analysis includes:
- Multiple tuning scenarios with different natural frequencies and damping ratios.
- Open-loop and Closed-loop Bode plots.
- Step response comparison to visualize the effect of damping.
"""

import numpy as np
import matplotlib.pyplot as plt
import control as ct

# Assuming modules are in a 'modules' sub-folder or accessible in the path
from modules.pi_controller_tuning import (
    create_pi_controller,
    validate_tuning_margins
)
from modules.analysis_utilities import (
    configure_plot_style,
    setup_bode_plot_figure,
    setup_step_plot_figure,
    plot_bode_response,
    plot_step_response,
    finalize_bode_plot,
    finalize_step_plot,
    print_controller_summary
)

# Configure plotting style
configure_plot_style()

# =============================================================================
# SYSTEM PARAMETERS
# =============================================================================

# --- System Parameters ---
f_nominal = 50.0  # Nominal grid frequency [Hz]
w_nominal = 2 * np.pi * f_nominal  # Nominal grid frequency [rad/s]

# =============================================================================
# SRF-PLL TUNING FUNCTIONS
# =============================================================================

def compute_srf_pll_gains_damping(natural_freq_hz, damping_ratio, v_mag=1.0, w_nominal=w_nominal):
    """
    Compute PI controller gains for SRF-PLL based on closed-loop specifications.
    
    This method directly tunes for a desired natural frequency and damping ratio,
    allowing for precise control over the closed-loop response shape and preventing gain peaking.
    
    Args:
        natural_freq_hz: Desired closed-loop natural frequency (fn) in Hz.
        damping_ratio: Desired closed-loop damping ratio (ζ). ζ >= 0.707 avoids peaking.
        v_mag: Input voltage magnitude.
        w_nominal: Nominal angular frequency.
        
    Returns:
        (kp, ti): Tuple of proportional gain and integral time constant.
    """
    # Convert natural frequency from Hz to rad/s
    w_n = 2 * np.pi * natural_freq_hz
    
    # The closed-loop transfer function denominator for this system is:
    # s^2 + (V_mag * w_nominal * Kp) * s + (V_mag * w_nominal * Kp / Ti)
    # The standard second-order form is:
    # s^2 + (2 * ζ * w_n) * s + w_n^2
    
    # By comparing coefficients, we derive the gains:
    # 2*ζ*w_n = V_mag*w_nominal*Kp  => Kp = (2*ζ*w_n) / (V_mag*w_nominal)
    # w_n^2 = V_mag*w_nominal*Ki = V_mag*w_nominal*(Kp/Ti) => Ti = V_mag*w_nominal*Kp / w_n^2
    # Substituting Kp in the Ti equation gives: Ti = 2*ζ / w_n
    
    kp = (2 * damping_ratio * w_n) / (v_mag * w_nominal)
    ti = (2 * damping_ratio) / w_n
    
    return kp, ti

# =============================================================================
# SCENARIO DEFINITIONS
# =============================================================================

# Tuning scenarios based on Damping Ratio and Natural Frequency
# A damping ratio (ζ) of 0.707 is the critical value to eliminate gain peaking.
# A value of 1.0 provides critical damping for the fastest step response without overshoot.
pll_tuning_scenarios = [
    # Critically damped: flat magnitude and no step response overshoot
    {'natural_freq_hz': 50.0, 'damping_ratio': 1.0},
    # A faster, critically damped response
    {'natural_freq_hz': 100.0, 'damping_ratio': 1.0},
]

# Voltage magnitude scenarios
voltage_scenarios = [
    {'v_mag': 1.0},  # Nominal voltage
]

# =============================================================================
# FIGURE SETUP
# =============================================================================

fig_cl, axes_cl = setup_bode_plot_figure('SRF-PLL: Closed-Loop Bode Plot Comparison (Phase Tracking)')
fig_step, ax_step = setup_step_plot_figure('SRF-PLL: Closed-Loop Step Response Comparison')

# =============================================================================
# MAIN ANALYSIS LOOP
# =============================================================================

print("=" * 60)
print("SRF-PLL CONTROLLER TUNING ANALYSIS (DAMPING CONTROL)")
print("=" * 60)

s = ct.tf('s')
vco_tf = w_nominal / s

for i, voltage_params in enumerate(voltage_scenarios):
    v_mag_scenario = voltage_params['v_mag']
    
    for j, tune_params in enumerate(pll_tuning_scenarios):
        # Calculate controller parameters using the new damping-based method
        desired_natural_freq_hz = tune_params['natural_freq_hz']
        desired_damping_ratio = tune_params['damping_ratio']
        
        kp, ti_s = compute_srf_pll_gains_damping(
            desired_natural_freq_hz, 
            desired_damping_ratio, 
            v_mag=v_mag_scenario,
            w_nominal=w_nominal
        )
        
        # Create controller and system transfer functions
        controller_tf = create_pi_controller(kp, ti_s)
        open_loop_tf = v_mag_scenario * controller_tf * vco_tf
        closed_loop_tf = ct.feedback(open_loop_tf, 1)

        # Generate a descriptive label for this scenario
        scenario_label = (f"V_mag={v_mag_scenario:.1f}pu, "
                         f"f_n={desired_natural_freq_hz}Hz, ζ={desired_damping_ratio:.3f}")
        
        # Print controller summary
        print_controller_summary(
            kp, ti_s, 
            loop_type="SRF-PLL",
            scenario_label=scenario_label,
            units="[rad/s/rad]"
        )
        
        # Validate the resulting open-loop margins
        margins = validate_tuning_margins(controller_tf, v_mag_scenario * vco_tf)
        print(f"  Resulting PM = {margins['phase_margin_deg']:.1f}°, "
              f"Resulting GM = {margins['gain_margin_db']:.1f} dB")
        
        # Plot responses
        plot_bode_response(axes_cl, closed_loop_tf, scenario_label, frequency_range_hz=(1, 1000))
        plot_step_response(ax_step, closed_loop_tf, scenario_label, time_span_s=0.2)

# =============================================================================
# FINALIZE PLOTS
# =============================================================================

finalize_bode_plot(fig_cl, axes_cl, frequency_range_hz=(1, 1000))
finalize_step_plot(fig_step, ax_step)

print("\n" + "=" * 60)
print("ANALYSIS COMPLETE - Displaying Results")
print("=" * 60)

plt.show()
