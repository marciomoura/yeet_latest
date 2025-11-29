"""
LCI Per-Unit System Definition

This module implements the Per-Unit (PU) system logic for the LCI drive,
strictly aligned with the C++ implementation in `sfc_per_unit_system.cpp`.
"""

import numpy as np

class SfcPerUnitSystem:
    """
    Per-Unit System for Load-Commutated Inverter (LCI) Drives.
    
    Mirrors the functionality of the C++ `sfc_per_unit_system` class.
    """
    
    def __init__(self, grid_voltage_ll_rms, grid_frequency, dc_link_current_rated,
                 machine_power, machine_voltage_ll_rms, machine_frequency, machine_num_poles):
        """
        Initialize the LCI Per-Unit System.
        
        Args:
            grid_voltage_ll_rms: Rated line-to-line RMS voltage on the grid-side (LV) [V]
            grid_frequency: Rated frequency on the grid-side [Hz]
            dc_link_current_rated: Rated nominal DC link current [A]
            machine_power: Rated 3-phase apparent power of the machine [VA]
            machine_voltage_ll_rms: Rated line-to-line RMS voltage on the machine-side [V]
            machine_frequency: Rated frequency on the machine-side [Hz]
            machine_num_poles: Number of poles of the synchronous machine
        """
        self.grid_voltage_ll_rms = float(grid_voltage_ll_rms)
        self.grid_frequency = float(grid_frequency)
        self.dc_link_current_rated = float(dc_link_current_rated)
        self.machine_power = float(machine_power)
        self.machine_voltage_ll_rms = float(machine_voltage_ll_rms)
        self.machine_frequency = float(machine_frequency)
        self.machine_num_poles = int(machine_num_poles)
        
        self.calculate_base_values()

    def calculate_base_values(self):
        """Calculate all derived base values."""
        self._calculate_grid_side_bases()
        self._calculate_dc_link_bases()
        self._calculate_machine_side_bases()

    def _calculate_grid_side_bases(self):
        """Calculate Grid Side (LV) bases."""
        self.grid_v_base_peak_phase = self.grid_voltage_ll_rms * np.sqrt(2) / np.sqrt(3)
        # I_base_peak = (2*sqrt(3)/pi) * I_dc_rated
        self.grid_i_base_peak_phase = (2.0 * np.sqrt(3) / np.pi) * self.dc_link_current_rated
        self.grid_z_base = self.grid_v_base_peak_phase / self.grid_i_base_peak_phase
        self.grid_w_base = 2.0 * np.pi * self.grid_frequency
        self.grid_l_base = self.grid_z_base / self.grid_w_base

    def _calculate_dc_link_bases(self):
        """Calculate DC Link bases."""
        # V_dc_base = (3*sqrt(2)/pi) * V_LL_RMS (Ideal no-load voltage of 6-pulse bridge)
        self.dc_v_base = (3.0 * np.sqrt(2) / np.pi) * self.grid_voltage_ll_rms
        self.dc_i_base = self.dc_link_current_rated
        self.dc_z_base = self.dc_v_base / self.dc_i_base
        self.dc_l_base = self.dc_z_base / (2.0 * np.pi * self.grid_frequency)

    def _calculate_machine_side_bases(self):
        """Calculate Machine Side bases."""
        self.machine_v_base_peak_phase = self.machine_voltage_ll_rms * np.sqrt(2) / np.sqrt(3)
        # I_base = (2/3) * S / V_base_peak
        self.machine_i_base_peak_phase = (2.0 / 3.0) * self.machine_power / self.machine_v_base_peak_phase
        self.machine_z_base = (3.0 / 2.0) * (self.machine_v_base_peak_phase**2) / self.machine_power
        self.machine_w_base_elec = 2.0 * np.pi * self.machine_frequency
        self.machine_l_base = self.machine_z_base / self.machine_w_base_elec
        
        # Torque and Flux
        pole_pairs = self.machine_num_poles / 2.0
        self.machine_w_base_mech = self.machine_w_base_elec / pole_pairs
        self.machine_torque_base = self.machine_power / self.machine_w_base_mech
        self.machine_flux_base = self.machine_v_base_peak_phase / self.machine_w_base_elec

    def to_pu_dc_impedance(self, r_ohm, l_h):
        """
        Convert physical DC link impedance to per-unit resistance and inductance factor.
        
        Args:
            r_ohm: Resistance in Ohms
            l_h: Inductance in Henrys
            
        Returns:
            r_pu: Resistance in Per-Unit
            l_factor_s: Inductance factor (L_phys / Z_base) in seconds
        """
        r_pu = r_ohm / self.dc_z_base
        # L term in TF is s*L. In PU: s * (L_phys / Z_base)
        l_factor_s = l_h / self.dc_z_base
        return r_pu, l_factor_s

    def get_rectifier_ideal_no_load_voltage_pu(self):
        """
        Returns the ideal no-load voltage of the rectifier in Per-Unit.
        Since the DC voltage base is defined as this value, it should be 1.0.
        """
        v_do_phys = (3.0 * np.sqrt(2) / np.pi) * self.grid_voltage_ll_rms
        return v_do_phys / self.dc_v_base
