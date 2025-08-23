"""
LCI Current Control Loop Analysis

This script performs comprehensive analysis of the inner current control loop
for Load Commutated Inverter (LCI) systems. It uses modular components for
system modeling, PI controller tuning, and result visualization.

The analysis includes:
- Plant model comparison for different measurement filter configurations
- Open-loop Bode plots for stability analysis
- Closed-loop Bode plots for tracking performance
- Disturbance rejection analysis
- Step response comparison
"""

import numpy as np
import matplotlib.pyplot as plt
import control as ct

# Import modular components
from modules.lci_system_model import CurrentLoopModel
from modules.pi_controller_tuning import (
    compute_pi_gains_crossover, 
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

# --- Electrical System Parameters ---
f_grid_hz = 50.0
v_ll_grid_pu = 1.0
l_dc_h = 8e-3    
r_dc_ohm = 0.05
current_control_task_cycle = 200e-6

# --- Rectifier Operating Point ---
v_do_v = 1.35 * v_ll_grid_pu
alpha_op_rad = np.deg2rad(90.0)  # Worst-case gain analysis point

# =============================================================================
# SCENARIO DEFINITIONS
# =============================================================================

# 1. Controller Tuning Scenarios (Crossover Frequency and Phase Margin)
tuning_scenarios = [
    {'cutoff_hz': 10.0, 'pm_deg': 60.0},
    {'cutoff_hz': 20.0, 'pm_deg': 60.0}
]

# 2. Measurement Filter Scenarios (Cutoff Frequency)
filter_scenarios = [
    {'f_meas_cutoff_hz': 100.0, 'damping': 0.707},
    {'f_meas_cutoff_hz': 200.0, 'damping': 0.707}
]

# =============================================================================
# FIGURE SETUP
# =============================================================================

# Create figures for comprehensive analysis
fig_plant, axes_plant = setup_bode_plot_figure('Plant Model Comparison (Actuator + DC Link + Sensor)')
fig_ol, axes_ol = setup_bode_plot_figure('Open-Loop Bode Plot Comparison')
fig_cl, axes_cl = setup_bode_plot_figure('Closed-Loop Bode Plot Comparison (Tracking)')
fig_dist, axes_dist = setup_bode_plot_figure('Disturbance Rejection Bode Plot Comparison')
fig_step, ax_step = setup_step_plot_figure('Closed-Loop Step Response Comparison')

# =============================================================================
# MAIN ANALYSIS LOOP
# =============================================================================

print("=" * 60)
print("LCI CURRENT CONTROLLER TUNING ANALYSIS")
print("=" * 60)

for i, filter_params in enumerate(filter_scenarios):
    # Create current loop model for this filter configuration
    current_model = CurrentLoopModel(
        task_sampling_rate=current_control_task_cycle,
        v_do_v=v_do_v,
        alpha_op_rad=alpha_op_rad,
        f_grid_hz=f_grid_hz,
        l_dc_h=l_dc_h,
        r_dc_ohm=r_dc_ohm,
        f_meas_cutoff_hz=filter_params['f_meas_cutoff_hz'],
        meas_damping=filter_params['damping']
    )
    
    # Plot plant model for this filter scenario
    plant_label = f"Filter: {filter_params['f_meas_cutoff_hz']} Hz"
    plot_bode_response(axes_plant, current_model.get_design_plant_tf(), plant_label)
    
    for j, tune_params in enumerate(tuning_scenarios):
        # Calculate controller parameters
        desired_cutoff_hz = tune_params['cutoff_hz']
        desired_pm_deg = tune_params['pm_deg']
        
        kp, ti_s = compute_pi_gains_crossover(
            desired_cutoff_hz, 
            desired_pm_deg, 
            current_model.get_design_plant_tf()
        )
        
        # Create controller and system transfer functions
        controller_tf = create_pi_controller(kp, ti_s)
        
        open_loop_tf = controller_tf * current_model.get_design_plant_tf()
        closed_loop_tf = current_model.get_closed_loop_tf(controller_tf)
        disturbance_tf = current_model.get_disturbance_rejection_tf(controller_tf)

        # Generate comprehensive label for this scenario
        scenario_label = (f"f_c={desired_cutoff_hz}Hz, PM={desired_pm_deg}°, "
                         f"f_filt={filter_params['f_meas_cutoff_hz']}Hz")
        
        # Print controller summary
        print_controller_summary(
            kp, ti_s, 
            loop_type="Current",
            scenario_label=scenario_label,
            units="[rad/A]"
        )
        
        # Validate actual margins achieved
        margins = validate_tuning_margins(controller_tf, current_model.get_design_plant_tf())
        print(f"  Actual PM = {margins['phase_margin_deg']:.1f}°, "
              f"GM = {margins['gain_margin_db']:.1f} dB")
        
        # Plot all responses
        plot_bode_response(axes_ol, open_loop_tf, scenario_label)
        plot_bode_response(axes_cl, closed_loop_tf, scenario_label)
        plot_bode_response(axes_dist, disturbance_tf, scenario_label)
        plot_step_response(ax_step, closed_loop_tf, scenario_label, time_span_s=1.0)

# =============================================================================
# FINALIZE PLOTS
# =============================================================================

# Apply standard formatting to all figures
finalize_bode_plot(fig_plant, axes_plant)
finalize_bode_plot(fig_ol, axes_ol)
finalize_bode_plot(fig_cl, axes_cl)
finalize_bode_plot(fig_dist, axes_dist)
finalize_step_plot(fig_step, ax_step)

print("\n" + "=" * 60)
print("ANALYSIS COMPLETE - Displaying Results")
print("=" * 60)

# Display all plots
plt.show()
