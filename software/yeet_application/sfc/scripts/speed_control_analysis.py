"""
LCI Speed Control Loop Analysis (Per-Unit)

This script performs analysis of the outer speed control loop for LCI systems,
using a per-unit system model. It builds upon a fixed, pre-tuned inner
current loop model.

The analysis includes:
- Open-loop Bode plots for stability analysis across different operating points
- Closed-loop step response comparison
"""

import numpy as np
import matplotlib.pyplot as plt
import control as ct

# Import modular components
from modules.lci_system_model import CurrentLoopModel, SpeedLoopPlantModel
from modules.pi_controller_tuning import (
    compute_pi_gains_crossover, 
    create_pi_controller,
    validate_tuning_margins
)
from modules.analysis_utilities import (
    configure_plot_style,
    setup_comprehensive_analysis_figure,
    plot_bode_subplot_pair,
    plot_step_subplot,
    finalize_comprehensive_analysis_figure,
    print_controller_summary
)

# Configure plotting style
configure_plot_style()

# =============================================================================
# PART 1: INNER CURRENT LOOP MODEL (FIXED AND PRE-TUNED)
# =============================================================================

# --- Electrical System Parameters ---
f_grid_hz = 50.0
v_ll_grid_pu = 1.0
l_dc_h = 8e-3
r_dc_ohm = 0.05
v_do_v = 1.35 * v_ll_grid_pu
alpha_op_rad = np.deg2rad(90.0)
dc_link_current_cutoff_hz = 100.0
dc_link_pi_cutoff_hz = 20.0
dc_link_pi_phase_margin_deg = 60.0
current_control_task_cycle = 200e-6

# --- Create a fixed current loop model ---
# Using a fast measurement filter for good performance
current_model = CurrentLoopModel(task_sampling_rate=current_control_task_cycle,
    v_do_v=v_do_v, alpha_op_rad=alpha_op_rad, f_grid_hz=f_grid_hz,
    l_dc_h=l_dc_h, r_dc_ohm=r_dc_ohm, f_meas_cutoff_hz=dc_link_current_cutoff_hz
)

# --- Tune a fixed, fast PI controller for the current loop ---
kp_i, ti_i = compute_pi_gains_crossover(
    f_cutoff_hz=dc_link_pi_cutoff_hz, pm_deg=dc_link_pi_phase_margin_deg, 
    plant_tf=current_model.get_design_plant_tf())
current_controller_tf = create_pi_controller(kp_i, ti_i)
g_cl_i = current_model.get_closed_loop_tf(current_controller_tf)

print("=" * 60)
print("FIXED INNER CURRENT LOOP MODEL INITIALIZED")
print_controller_summary(kp_i, ti_i, "Current", "Fixed for Speed Analysis", "[rad/A]")
print("=" * 60)


# =============================================================================
# PART 2: OUTER SPEED LOOP ANALYSIS
# =============================================================================

# --- Speed Loop System Parameters (Per-Unit) ---
#h_s = 4.0           # Total system inertia constant [s]
k_t_pu = 1.0        # Per-unit torque constant [pu_torque / pu_current]  
speed_control_task_cycle = 0.010 # 10 ms control task cycle

# --- Scenario Definitions ---
speed_tuning_scenarios = [
    {'cutoff_hz': 1.0, 'pm_deg': 60.0},
    # {'cutoff_hz': 2.0, 'pm_deg': 60.0},
]
observer_filter_scenarios = [
    {'f_obs_cutoff_hz': 10.0, 'damping': 0.707},
    # {'f_obs_cutoff_hz': 20.0, 'damping': 0.707},
]
pll_scenarios = [
    {'pll_cutoff_hz': 10.0, 'pll_damping': 1.0}, 
    {'pll_cutoff_hz': 50.0, 'pll_damping': 1.0}, 
    {'pll_cutoff_hz': 100.0, 'pll_damping': 1.0}, 
    # {'pll_cutoff_hz': 25.0, 'pll_damping': 1.0}, 
]
mech_scenarios = [
    # {'h_s': 4.0, 'd_eff_pu': 0.01}, # High inertia, low damping
    {'h_s': 0.5, 'd_eff_pu': 0.01}, # Low inertia, low damping
]

# --- Figure Setup ---
# Create comprehensive analysis figure with all subplots including plant comparison
fig, axes_dict = setup_comprehensive_analysis_figure(
    'Speed Loop Control Analysis Dashboard',
    include_plant=True,
    include_step=True,
    figsize=(18, 10)
)

# --- Main Analysis Loop ---
print("\nLCI SPEED CONTROLLER TUNING ANALYSIS (PER-UNIT)")
print("=" * 60)

for i, mech_params in enumerate(mech_scenarios):
    for j, obs_params in enumerate(observer_filter_scenarios):
        for k, pll_params in enumerate(pll_scenarios):
            # Create the speed loop plant model for this configuration
            speed_plant = SpeedLoopPlantModel(
                task_sampling_rate=speed_control_task_cycle,
                current_cl_tf=g_cl_i,
                k_t_pu=k_t_pu,
                h_s=mech_params['h_s'],
                d_eff_pu=mech_params['d_eff_pu'],
                observer_filter_cutoff_hz=obs_params['f_obs_cutoff_hz'],
                observer_filter_damping_ratio=obs_params['damping'],
                pll_closed_loop_cutoff_hz=pll_params['pll_cutoff_hz'],
                pll_closed_loop_damping_ratio=pll_params['pll_damping']
            )
            
            # Plot plant model for this configuration
            plant_label = (f"H={mech_params['h_s']}s, f_obs={obs_params['f_obs_cutoff_hz']}Hz, "
                           f"f_pll={pll_params['pll_cutoff_hz']}Hz")
            plot_bode_subplot_pair(
                axes_dict['plant_mag'], axes_dict['plant_phase'], 
                speed_plant.get_total_plant_tf(), plant_label
            )

            for l, tune_params in enumerate(speed_tuning_scenarios):
                # Calculate controller parameters
                desired_cutoff_hz = tune_params['cutoff_hz']
                desired_pm_deg = tune_params['pm_deg']
                
                kp_n, ti_n = compute_pi_gains_crossover(
                    f_cutoff_hz=desired_cutoff_hz,
                    pm_deg=desired_pm_deg,
                    plant_tf=speed_plant.get_total_plant_tf()
                )
                
                # Create controller and system transfer functions
                speed_controller_tf = create_pi_controller(kp_n, ti_n)
                
                open_loop_tf = speed_plant.get_open_loop_tf(speed_controller_tf)
                closed_loop_tf = speed_plant.get_closed_loop_tf(speed_controller_tf)
                disturbance_tf = speed_plant.get_disturbance_rejection_tf(speed_controller_tf)

                # Generate comprehensive label for this scenario
                scenario_label = (f"H={mech_params['h_s']}s, "
                                 f"f_obs={obs_params['f_obs_cutoff_hz']}Hz, f_pll={pll_params['pll_cutoff_hz']}Hz, "
                                 f"f_c={desired_cutoff_hz}Hz, PM={desired_pm_deg}°")
                
                # Print controller summary
                print_controller_summary(
                    kp_n, ti_n, 
                    loop_type="Speed",
                    scenario_label=scenario_label,
                    units="[pu_A/pu_w]"
                )
                
                # Validate actual margins achieved
                margins = validate_tuning_margins(speed_controller_tf, speed_plant.get_total_plant_tf())
                print(f"  Actual PM = {margins['phase_margin_deg']:.1f}°, "
                      f"GM = {margins['gain_margin_db']:.1f} dB")
                
                # Plot all responses using subplot pairs
                plot_bode_subplot_pair(
                    axes_dict['ol_mag'], axes_dict['ol_phase'], 
                    open_loop_tf, scenario_label
                )
                plot_bode_subplot_pair(
                    axes_dict['cl_mag'], axes_dict['cl_phase'], 
                    closed_loop_tf, scenario_label
                )
                plot_bode_subplot_pair(
                    axes_dict['dist_mag'], axes_dict['dist_phase'], 
                    disturbance_tf, scenario_label
                )
                plot_step_subplot(axes_dict['step'], closed_loop_tf, scenario_label, time_span_s=5.0)

# --- Finalize Plots ---
finalize_comprehensive_analysis_figure(fig, axes_dict)

print("\n" + "=" * 60)
print("ANALYSIS COMPLETE - Displaying Results")
print("=" * 60)

# Display all plots
plt.show()
