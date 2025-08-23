"""
LCI System Model - Transfer Function Definitions

This module defines all transfer function models used in LCI current and speed control loops.
All functions use lower-case naming conventions per the project dictionary.
"""

import numpy as np
import control as ct


def create_rectifier_model(v_do_v, alpha_op_rad, f_grid_hz):
    """
    Creates the rectifier actuator transfer function model.
    
    Args:
        v_do_v: Ideal no-load DC voltage [V]
        alpha_op_rad: Operating point firing angle [rad] 
        f_grid_hz: Grid frequency [Hz]
        
    Returns:
        Rectifier transfer function (signed gain with transport delay)
    """
    # Small-signal gain (absolute): d(Vdc)/d(alpha) = abs(-V_do * sin(alpha))
    k_rectifier_signed = v_do_v * np.sin(alpha_op_rad)
    
    # Average transport delay for 6-pulse converter
    t_delay_s = 1 / (2 * 6 * f_grid_hz)
    
    # Padé approximation of transport delay
    pade_num, pade_den = ct.pade(t_delay_s, 1)
    delay_tf = ct.tf(pade_num, pade_den)
    
    return k_rectifier_signed * delay_tf


def create_dc_link_model(l_dc_h, r_dc_ohm):
    """
    Creates the DC link plant transfer function model.
    
    Args:
        l_dc_h: DC link inductance [H]
        r_dc_ohm: DC link resistance [Ohm]
        
    Returns:
        DC link transfer function (first-order RL circuit)
    """
    s = ct.tf('s')
    return 1 / (r_dc_ohm + s * l_dc_h)


def create_measurement_filter(f_cutoff_hz, damping_ratio=0.707):
    """
    Creates a second-order measurement filter transfer function.
    
    Args:
        f_cutoff_hz: Filter cutoff frequency [Hz]
        damping_ratio: Damping ratio (default: 0.707 for Butterworth)
        
    Returns:
        Second-order low-pass filter transfer function
    """
    s = ct.tf('s')
    w_cutoff_rad_s = 2 * np.pi * f_cutoff_hz
    
    return (w_cutoff_rad_s**2) / (
        s**2 + 2*damping_ratio*w_cutoff_rad_s*s + w_cutoff_rad_s**2
    )


def create_task_delay_model(task_delay_cycle):
    """
    Creates a control task execution delay model.
    
    Args:
        task_delay_cycle: Control task delay [s]
        
    Returns:
        Padé approximation of transport delay
    """
    pade_num, pade_den = ct.pade(task_delay_cycle, 1)
    return ct.tf(pade_num, pade_den)


def create_mechanical_model_pu(h_s, d_eff_pu):
    """
    Creates the per-unit mechanical system transfer function.
    
    Args:
        h_s: Inertia constant [seconds]
        d_eff_pu: Effective damping coefficient [pu]
        
    Returns:
        Per-unit mechanical transfer function
    """
    s = ct.tf('s')
    return 1 / (2 * h_s * s + d_eff_pu)


def create_speed_observer(observer_filter_cutoff_hz, observer_filter_damping_ratio, pll_closed_loop_cutoff_hz, pll_closed_loop_damping_ratio):
    """
    Creates a speed observer model by combining an observer filter and a PLL closed-loop filter.

    This function models the speed estimation process, which typically involves
    a phase-locked loop (PLL) to track the rotor position and a subsequent filter
    to smooth the calculated speed.

    Args:
        observer_filter_cutoff_hz: Cutoff frequency of the observer's speed filter [Hz].
        observer_filter_damping_ratio: Damping ratio of the observer's speed filter.
        pll_closed_loop_cutoff_hz: Cutoff frequency of the equivalent closed-loop PLL filter [Hz].
        pll_closed_loop_damping_ratio: Damping ratio of the PLL filter (default: 0.707).

    Returns:
        Combined transfer function of the speed observer.
    """
    pll_observer_filter = create_measurement_filter(observer_filter_cutoff_hz, observer_filter_damping_ratio)
    pll_closed_loop_filter = create_measurement_filter(pll_closed_loop_cutoff_hz, pll_closed_loop_damping_ratio)

    return pll_observer_filter * pll_closed_loop_filter


class CurrentLoopModel:
    """
    Encapsulates the complete current control loop model including
    actuator, plant, and sensor transfer functions.
    """
    
    def __init__(self, task_sampling_rate, v_do_v, alpha_op_rad, f_grid_hz, l_dc_h, r_dc_ohm, 
                 f_meas_cutoff_hz, meas_damping=0.707):
        """
        Initialize the current loop model.
        
        Args:
            task_sampling_rate: Control task delay [s]
            v_do_v: Ideal no-load DC voltage [V]
            alpha_op_rad: Operating point firing angle [rad]
            f_grid_hz: Grid frequency [Hz]
            l_dc_h: DC link inductance [H]
            r_dc_ohm: DC link resistance [Ohm]
            f_meas_cutoff_hz: Measurement filter cutoff frequency [Hz]
            meas_damping: Measurement filter damping ratio
        """
        self.f_grid_hz = f_grid_hz  # Store grid frequency
        self.actuator_tf = create_rectifier_model(v_do_v, alpha_op_rad, self.f_grid_hz)
        self.t_delay_tf = create_task_delay_model(task_sampling_rate)
        self.plant_tf = create_dc_link_model(l_dc_h, r_dc_ohm)
        self.sensor_tf = create_measurement_filter(f_meas_cutoff_hz, meas_damping)
        
        # Store the signed gain for reference
        self.k_rectifier_signed = v_do_v * np.sin(alpha_op_rad)

    def get_design_plant_tf(self):
        """Returns the plant transfer function for design purposes."""
        return self.actuator_tf * self.plant_tf * self.sensor_tf * self.t_delay_tf

    def get_open_loop_tf(self, controller_tf):
        """
        Returns the open-loop transfer function for a given controller.
        """
        return controller_tf * self.get_design_plant_tf()

    def get_closed_loop_tf(self, controller_tf):
        """
        Returns the closed-loop transfer function for a given controller.
        """
        open_loop_tf = self.get_open_loop_tf(controller_tf)
        return ct.feedback(open_loop_tf, 1)
    
    def get_disturbance_rejection_tf(self, controller_tf):
        """
        Returns the disturbance rejection transfer function.
        
        Args:
            controller_tf: PI controller transfer function
            
        Returns:
            Disturbance rejection transfer function
        """
        return ct.feedback(self.get_open_loop_tf(controller_tf))

class SpeedLoopPlantModel:
    """
    Encapsulates the complete speed control loop plant model including
    the closed-loop current controller, delays, mechanical system, and observer.
    """
    
    def __init__(self, task_sampling_rate, current_cl_tf, k_t_pu, h_s, d_eff_pu, 
                 observer_filter_cutoff_hz, observer_filter_damping_ratio, 
                 pll_closed_loop_cutoff_hz, pll_closed_loop_damping_ratio):
        """
        Initialize the speed loop plant model.
        
        Args:
            task_sampling_rate: Control task delay [s]
            current_cl_tf: Closed-loop current controller transfer function
            k_t_pu: Per-unit torque constant [pu]
            h_s: Inertia constant [s]
            d_eff_pu: Effective damping coefficient [pu]
            observer_filter_cutoff_hz: Cutoff frequency of the observer's speed filter [Hz].
            observer_filter_damping_ratio: Damping ratio of the observer's speed filter.
            pll_closed_loop_cutoff_hz: Cutoff frequency of the equivalent closed-loop PLL filter [Hz].
            pll_closed_loop_damping_ratio: Damping ratio of the PLL filter.
        """
        self.current_cl_tf = current_cl_tf
        self.t_delay_tf = create_task_delay_model(task_sampling_rate)
        self.mechanical_tf = create_mechanical_model_pu(h_s, d_eff_pu)
        self.observer_tf = create_speed_observer(observer_filter_cutoff_hz, observer_filter_damping_ratio, pll_closed_loop_cutoff_hz, pll_closed_loop_damping_ratio)
        
        # Torque constant (assumed to be a gain in the Laplace domain)
        self.k_t_pu = k_t_pu

    def get_total_plant_tf(self):
        """
        Returns the complete plant transfer function for speed loop design.
        This includes: closed-loop current controller -> task delay -> torque constant -> mechanical system -> observer
        """
        return self.current_cl_tf * self.t_delay_tf * self.k_t_pu * self.mechanical_tf * self.observer_tf
    
    def get_open_loop_tf(self, controller_tf):
        """
        Returns the open-loop transfer function for a given speed controller.
        """
        return controller_tf * self.get_total_plant_tf()
    
    def get_closed_loop_tf(self, controller_tf):
        """
        Returns the closed-loop transfer function for the speed control loop.
        
        Args:
            controller_tf: PI controller transfer function
            
        Returns:
            Closed-loop transfer function of the speed control loop
        """
        open_loop_tf = self.get_open_loop_tf(controller_tf)
        return ct.feedback(open_loop_tf, 1)
    
    def get_disturbance_rejection_tf(self, controller_tf):
        """
        Returns the disturbance rejection transfer function for the speed loop.
        
        Args:
            controller_tf: PI controller transfer function
            
        Returns:
            Disturbance rejection transfer function of the speed loop
        """
        return ct.feedback(self.get_total_plant_tf(), controller_tf)
