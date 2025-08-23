"""
PI Controller Tuning Algorithms

This module provides various PI controller tuning methods for LCI control systems.
All functions use lower-case naming conventions per the project dictionary.
"""

import numpy as np
import control as ct

def compute_pi_gains_crossover(f_cutoff_hz, pm_deg, plant_tf):
    """
    Compute PI controller gains using crossover frequency and phase margin method.
    
    Args:
        f_cutoff_hz: Desired open-loop gain crossover frequency [Hz]
        pm_deg: Desired phase margin [degrees]
        plant_tf: Plant transfer function for design
    Returns:
        (kp, ti): Tuple of proportional gain and integral time constant [s]
    """
    w_c_rad_s = 2 * np.pi * f_cutoff_hz
    pm_rad = np.deg2rad(pm_deg)
    
    # Evaluate plant response at crossover frequency
    plant_response = ct.frequency_response(plant_tf, w_c_rad_s)
    mag_plant = float(plant_response.magnitude)
    phase_plant_rad = float(plant_response.phase)
    
    # Calculate required controller phase contribution
    # Phase(Controller) + Phase(Plant) = -180° + Phase Margin
    phase_controller_rad = -np.pi + pm_rad - phase_plant_rad
    
    # Calculate ti from controller phase: Phase(C) = atan(ti*wc) - 90
    # Note: The phase of a PI is atan(kp/(ki/wc)) - 90, which is atan(ti*wc)-90
    ti_s = np.tan(phase_controller_rad + np.pi/2) / w_c_rad_s
    
    # Calculate kp from magnitude requirement: |Controller| * |Plant| = 1
    # |C(jw)| = Kp * sqrt(1 + 1/(w*Ti)^2)
    mag_controller_required = 1.0 / mag_plant
    # |C(s)| = Kp * |1 + 1/(s*Ti)| = Kp * sqrt(1 + 1/(w*Ti)^2)
    kp = mag_controller_required / np.sqrt(1 + 1/((w_c_rad_s * ti_s)**2))
    assert kp > 0, "Calculated Kp must be positive before sign adjustment."
    assert ti_s > 0, "Calculated Ti must be positive."

    return kp, ti_s


def compute_pi_gains_pole_zero_cancel(plant_tf, desired_bandwidth_hz, is_inverting=False):
    """
    Compute PI controller gains using pole-zero cancellation method.
    Suitable for first-order plants where the PI zero cancels the plant pole.
    
    Args:
        plant_tf: Plant transfer function (should be first-order for best results)
        desired_bandwidth_hz: Desired closed-loop bandwidth [Hz]
        is_inverting: True if plant has inherent sign inversion
        
    Returns:
        (kp, ti): Tuple of proportional gain and integral time constant [s]
    """
    # Extract plant parameters (assumes first-order: K/(τs + 1))
    plant_dc_gain = float(ct.dcgain(plant_tf))
    
    # For pole-zero cancellation, set Ti = plant time constant
    # This requires analyzing the plant's denominator
    plant_poles = ct.pole(plant_tf)
    if len(plant_poles) > 0 and np.real(plant_poles[0]) < 0:
        # For stable first-order pole at -1/τ
        tau_plant = -1.0 / np.real(plant_poles[0])
        ti_s = tau_plant
    else:
        # Fallback: use crossover method
        return compute_pi_gains_crossover(desired_bandwidth_hz, 60.0, plant_tf, is_inverting)
    
    # Set Kp for desired bandwidth
    w_desired = 2 * np.pi * desired_bandwidth_hz
    kp = w_desired / abs(plant_dc_gain)
    
    # Apply sign correction for inverting plants
    if is_inverting:
        kp = -kp
    
    return kp, ti_s


def validate_tuning_margins(controller_tf, plant_tf):
    """
    Calculate actual gain and phase margins for a controller-plant combination.
    
    Args:
        controller_tf: PI controller transfer function
        plant_tf: Plant transfer function
        
    Returns:
        dict: Contains 'gain_margin_db', 'phase_margin_deg', 'gain_crossover_hz', 'phase_crossover_hz'
    """
    open_loop_tf = controller_tf * plant_tf
    
    try:
        gm, pm, wg, wp = ct.margin(open_loop_tf)
        
        # Convert to standard units
        gain_margin_db = 20 * np.log10(gm) if gm > 0 else float('inf')
        phase_margin_deg = np.rad2deg(pm) if pm is not None else 0.0
        gain_crossover_hz = wg / (2 * np.pi) if wg is not None else 0.0
        phase_crossover_hz = wp / (2 * np.pi) if wp is not None else 0.0
        
        return {
            'gain_margin_db': gain_margin_db,
            'phase_margin_deg': phase_margin_deg,
            'gain_crossover_hz': gain_crossover_hz,
            'phase_crossover_hz': phase_crossover_hz
        }
    except:
        # Return default values if margin calculation fails
        return {
            'gain_margin_db': float('inf'),
            'phase_margin_deg': 0.0,
            'gain_crossover_hz': 0.0,
            'phase_crossover_hz': 0.0
        }


def create_pi_controller(kp, ti_s):
    """
    Create a PI controller transfer function.
    
    Args:
        kp: Proportional gain
        ti_s: Integral time constant [s]
        
    Returns:
        PI controller transfer function: Kp * (1 + 1/(Ti*s))
    """
    s = ct.tf('s')
    return kp * (1 + 1/(ti_s * s))
