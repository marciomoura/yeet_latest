# -*- coding: utf-8 -*-
import math
import sys

# Set UTF-8 encoding for console output (Windows compatibility)
if sys.platform == 'win32':
    import io
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

# ==============================================================================
#      LCI GAS TURBINE STARTER - COMPREHENSIVE SYSTEM ANALYZER
# ==============================================================================
# This script performs a complete analysis of the LCI system based on design
# data. It is structured as a computational notebook.
#
# It computes and displays:
#   1. Complete Per-Unit (PU) system bases (V, A, Omega, T, Flux).
#   2. Component parameters in both SI and PU units.
#   3. Steady-state values for the nominal operating point (V, A, P, Q).
#   4. Commutation notch analysis and other control-relevant parameters.
#   5. Field voltage requirements for synchronous machine excitation.
#   6. Field voltage profile for soft-starter operation.
# ==============================================================================


# ==============================================================================
# HELPER FUNCTIONS
# ==============================================================================

def calculate_field_voltage_and_current(machine_params, target_vt_ll_rms, frequency_hz):
    """
    Calculates the required field voltage and current for a synchronous machine.
    
    At no-load, the terminal voltage equals the internal generated voltage (Eq), 
    which is produced by the rotating magnetic field created by the field winding current.
    
    Physical relationships:
        - Eq (RMS, phase) = ω × Lmd × (Ns/Nfd) × If     [V]
        - Vf (DC) = If × Rf                              [V]
    
    Args:
        machine_params (dict): Machine electrical parameters
        target_vt_ll_rms (float): Desired line-to-line RMS terminal voltage [V]
        frequency_hz (float): Operating frequency [Hz]
    
    Returns:
        tuple: (field_voltage_V, field_current_A, internal_voltage_phase_rms_V)
    """
    Lmd = machine_params['Lmd']
    Rf = machine_params['Rf']
    Ns_Nfd = machine_params['Ns_Nfd']
    
    omega = 2 * math.pi * frequency_hz
    target_vt_phase_rms = target_vt_ll_rms / math.sqrt(3)
    Eq_rms = target_vt_phase_rms
    
    try:
        required_if = Eq_rms / (omega * Lmd * Ns_Nfd)
    except ZeroDivisionError:
        return None, None, None
        
    required_vf = required_if * Rf
    return required_vf, required_if, Eq_rms


def calculate_field_voltage_vs_speed(machine_params, base_voltage_ll_rms, base_frequency_hz, speeds_pu, control_strategy='constant_flux'):
    """
    Calculates field voltage required at different speeds for constant flux operation.
    
    Args:
        machine_params (dict): Machine electrical parameters
        base_voltage_ll_rms (float): Base line-to-line RMS voltage [V]
        base_frequency_hz (float): Base frequency [Hz]
        speeds_pu (list): List of per-unit speeds to evaluate
        control_strategy (str): 'constant_flux' or 'field_weakening'
    
    Returns:
        dict: Dictionary mapping speed [pu] to field voltage data
    """
    results = {}
    
    for speed_pu in speeds_pu:
        freq_hz = base_frequency_hz * speed_pu
        
        if control_strategy == 'constant_flux':
            target_voltage_ll_rms = base_voltage_ll_rms * speed_pu
            if speed_pu < 0.01:
                freq_hz = 0.01 * base_frequency_hz
                target_voltage_ll_rms = 0.01 * base_voltage_ll_rms
        elif control_strategy == 'field_weakening':
            if speed_pu <= 1.0:
                target_voltage_ll_rms = base_voltage_ll_rms * speed_pu
            else:
                target_voltage_ll_rms = base_voltage_ll_rms / speed_pu
            if speed_pu < 0.01:
                freq_hz = 0.01 * base_frequency_hz
                target_voltage_ll_rms = 0.01 * base_voltage_ll_rms
        
        vf, if_val, eq = calculate_field_voltage_and_current(machine_params, target_voltage_ll_rms, freq_hz)
        
        if vf is not None:
            omega = 2 * math.pi * freq_hz
            flux_pu = eq / (omega * (base_voltage_ll_rms / math.sqrt(3)) / (2 * math.pi * base_frequency_hz)) if freq_hz > 0.1 else 0.0
            
            results[speed_pu] = {
                'field_voltage_V': vf,
                'field_current_A': if_val,
                'terminal_voltage_ll_V': target_voltage_ll_rms,
                'terminal_voltage_phase_V': eq,
                'frequency_Hz': freq_hz,
                'flux_pu': flux_pu,
                'control_strategy': control_strategy
            }
    
    return results


# ------------------------------------------------------------------------------
# SECTION 1: USER INPUTS & DESIGN DATA
# ------------------------------------------------------------------------------
# All primary system data is defined here in SI units.

print("--- [SECTION 1] Loading System Design Data ---")

# --- Line ---
line_voltage_nominal = 10.5e3       # [V] Nominal line-to-line voltage
line_frequency = 50                   # [Hz] Line frequency
short_circuit_capacity_min = 300e6            # [VA] Minimum short-circuit power

# --- Transformer ---
transformer_rated_power = 2950e3               # [VA] Rated power
transformer_primary_voltage_noload = 10.5e3  # [V] Rated primary voltage
transformer_secondary_voltage_noload = 2090  # [V] Rated secondary no-load voltage (Uv0)

# --- DC Link ---
dc_link_inductance_H = 8e-3                # [H] DC reactor inductance
dc_link_current_rated = 1044            # [A] Rated DC current

# --- Machine (as Starter & Generator) ---
machine_rated_power = 2950e3           # [VA] Rated power of the machine
machine_service_voltage = 1980         # [V] Service voltage (starter mode)
machine_base_frequency = 50              # [Hz] Base frequency for starting
generator_rated_speed_rpm = 3000         # [rpm] Rated speed as generator
generator_rated_frequency = 50           # [Hz] Rated frequency as generator

# --- Converter Reactances (from design sheet) ---
cls_commutation_reactance_pu = 0.073      # [pu] Total CLS commutation reactance on converter base
cms_commutation_reactance_pu = 0.0593     # [pu] Total CMS commutation reactance on converter base

# --- Tunable Engineering Assumptions ---
inverter_extinction_angle_setpoint_deg = 18.0  # [degrees] Target extinction angle for inverter control.
dc_choke_quality_factor = 50     # [-] Quality factor of the DC choke at 50 Hz.
machine_number_of_poles = 2                      # [-] Number of poles of the synchronous machine (from 3000rpm @ 50Hz)
grid_x_over_r_ratio = 10.0                       # [-] Assumed X/R ratio for the grid impedance.
transformer_copper_loss_pu = 0.01                # [pu] Assumed transformer copper losses at rated load.
machine_stator_resistance_pu = 0.005             # [pu] Assumed machine stator resistance.

print("...Data loaded successfully.\n")


# ------------------------------------------------------------------------------
# SECTION 2: COMPLETE PER-UNIT SYSTEM BASES
# ------------------------------------------------------------------------------
print("--- [SECTION 2] Calculating Complete Per-Unit System Bases ---")

# --- DC Link ---
base_current_dc = dc_link_current_rated
base_voltage_dc_ideal = (3 * math.sqrt(2) / math.pi) * transformer_secondary_voltage_noload
base_impedance_dc = base_voltage_dc_ideal / base_current_dc

# --- LV Side (Converter Line Side) ---
base_voltage_peak_phase_lv = (transformer_secondary_voltage_noload / math.sqrt(3)) * math.sqrt(2)
base_current_peak_lv = (2 * math.sqrt(3) / math.pi) * base_current_dc
base_impedance_lv = base_voltage_peak_phase_lv / base_current_peak_lv
base_angular_frequency_lv = 2 * math.pi * line_frequency
base_inductance_lv = base_impedance_lv / base_angular_frequency_lv

# --- HV Side (Line) ---
base_power_system = transformer_rated_power # Use transformer rating for HV side PU
base_voltage_hv = transformer_primary_voltage_noload
base_current_hv = base_power_system / (math.sqrt(3) * base_voltage_hv)
base_impedance_hv = base_voltage_hv**2 / base_power_system

# --- Machine Side (Converter Machine Side) ---
base_power_cms = machine_rated_power
base_voltage_peak_phase_cms = (machine_service_voltage / math.sqrt(3)) * math.sqrt(2)
base_current_peak_cms = (2/3) * base_power_cms / base_voltage_peak_phase_cms
base_impedance_cms = (3/2) * (base_voltage_peak_phase_cms**2) / base_power_cms
base_angular_frequency_elec_cms = 2 * math.pi * machine_base_frequency
base_inductance_cms = base_impedance_cms / base_angular_frequency_elec_cms
base_angular_frequency_mech_cms = base_angular_frequency_elec_cms / (machine_number_of_poles / 2)
base_torque_cms = base_power_cms / base_angular_frequency_mech_cms
base_flux_linkage_cms = base_voltage_peak_phase_cms / base_angular_frequency_elec_cms

# --- PU System Summary Table ---
print("="*80)
print("                          PER-UNIT SYSTEM BASES SUMMARY")
print("="*80)
hdr_fmt = "{:<20} | {:<18} | {:<18} | {:<18}"
row_fmt = "{:<20} | {:<18.2f} | {:<18.2f} | {:<18.2f}"
unt_fmt = "{:<20} | {:<18} | {:<18} | {:<18}"
str_fmt = "{:<20} | {:<18} | {:<18} | {:<18}"

print(hdr_fmt.format("PARAMETER", "GRID-SIDE (LV)", "DC LINK", "MACHINE-SIDE (CMS)"))
print("-" * 80)
print(str_fmt.format("Base Power", "N/A", "N/A", f"{base_power_cms/1e3:.0f} kVA"))
print(row_fmt.format("Base Voltage", base_voltage_peak_phase_lv, base_voltage_dc_ideal, base_voltage_peak_phase_cms))
print(unt_fmt.format("  (Unit)", "V (pk-ph)", "V", "V (pk-ph)"))
print(row_fmt.format("Base Current", base_current_peak_lv, base_current_dc, base_current_peak_cms))
print(unt_fmt.format("  (Unit)", "A (peak)", "A", "A (peak)"))
print(row_fmt.format("Base Impedance", base_impedance_lv, base_impedance_dc, base_impedance_cms))
print(unt_fmt.format("  (Unit)", "Ohm", "Ohm", "Ohm"))
print(row_fmt.format("Base Frequency", line_frequency, line_frequency, machine_base_frequency))
print(unt_fmt.format("  (Unit)", "Hz", "Hz", "Hz"))
print(row_fmt.format("Base Inductance", base_inductance_lv * 1e3, (base_impedance_dc / (2*math.pi*line_frequency)) * 1e3, base_inductance_cms * 1e3))
print(unt_fmt.format("  (Unit)", "mH", "mH", "mH"))
print(str_fmt.format("Base Torque", "N/A", "N/A", f"{base_torque_cms/1e3:.2f} kNm"))
print(str_fmt.format("Base Flux Linkage", "N/A", "N/A", f"{base_flux_linkage_cms:.3f} Vs/rad"))
print("="*80)
print()


# ------------------------------------------------------------------------------
# SECTION 3: COMPONENT PARAMETERS (SI & PU)
# ------------------------------------------------------------------------------
print("--- [SECTION 3] Calculating Component Parameters (SI & PU) ---")

# --- Grid (Thevenin Equivalent) ---
grid_impedance_pu_hv = base_power_system / short_circuit_capacity_min
grid_impedance_ohm = grid_impedance_pu_hv * base_impedance_hv
grid_reactance_ohm = grid_impedance_ohm # Assuming grid is mostly reactive
grid_resistance_ohm = grid_reactance_ohm / grid_x_over_r_ratio
grid_inductance_H = grid_reactance_ohm / (2 * math.pi * line_frequency)

# --- Transformer ---
transformer_reactance_pu_own_base = 0.095 # From exT = 9.5%
transformer_base_impedance_lv = (transformer_secondary_voltage_noload**2) / transformer_rated_power
transformer_reactance_ohm = transformer_reactance_pu_own_base * transformer_base_impedance_lv
transformer_resistance_ohm = transformer_copper_loss_pu * transformer_base_impedance_lv
transformer_inductance_H = transformer_reactance_ohm / (2 * math.pi * line_frequency)

# --- Total Line-Side Commutation Impedance (Grid + Transformer) ---
transformer_turns_ratio_squared = (transformer_primary_voltage_noload / transformer_secondary_voltage_noload)**2
grid_reactance_ohm_referred_lv = grid_reactance_ohm / transformer_turns_ratio_squared
grid_resistance_ohm_referred_lv = grid_resistance_ohm / transformer_turns_ratio_squared
cls_commutation_reactance_ohm = grid_reactance_ohm_referred_lv + transformer_reactance_ohm
cls_commutation_resistance_ohm = grid_resistance_ohm_referred_lv + transformer_resistance_ohm
cls_commutation_inductance_H = cls_commutation_reactance_ohm / (2 * math.pi * line_frequency)
cls_commutation_reactance_pu_lv_base = cls_commutation_reactance_ohm / base_impedance_lv
cls_commutation_resistance_pu_lv_base = cls_commutation_resistance_ohm / base_impedance_lv

# --- DC Link Inductor ---
dc_link_resistance_ohm = (2 * math.pi * line_frequency * dc_link_inductance_H) / dc_choke_quality_factor
dc_link_inductance_pu = dc_link_inductance_H / (base_impedance_dc / (2 * math.pi * line_frequency))
dc_link_resistance_pu = dc_link_resistance_ohm / base_impedance_dc

# --- Machine (Voltage-Behind-Reactance Model) ---
cms_commutation_reactance_ohm = cms_commutation_reactance_pu * base_impedance_cms
cms_stator_resistance_ohm = machine_stator_resistance_pu * base_impedance_cms
cms_commutation_inductance_H = cms_commutation_reactance_ohm / base_angular_frequency_elec_cms

# --- Component Parameters Summary Table ---
print("="*80)
print("                          COMPONENT PARAMETERS SUMMARY")
print("="*80)
hdr_fmt = "{:<25} | {:<20} | {:<12} | {:<12}"
row_fmt = "{:<25} | {:<20} | {:<12.4f} | {:<12.4f}"
print(hdr_fmt.format("COMPONENT", "PARAMETER", "SI VALUE", "PU VALUE"))
print("-" * 80)
print(row_fmt.format("Grid (ref. LV)", "Reactance", grid_reactance_ohm_referred_lv, grid_reactance_ohm_referred_lv / base_impedance_lv))
print(row_fmt.format("", "Resistance", grid_resistance_ohm_referred_lv, grid_resistance_ohm_referred_lv / base_impedance_lv))
print(f"{'':<25} | {'Inductance (mH)':<20} | {grid_inductance_H / transformer_turns_ratio_squared * 1e3:<12.4f} | {'N/A':<12}")
print("-" * 80)
print(row_fmt.format("Transformer (ref. LV)", "Reactance", transformer_reactance_ohm, transformer_reactance_ohm / base_impedance_lv))
print(row_fmt.format("", "Resistance", transformer_resistance_ohm, transformer_resistance_ohm / base_impedance_lv))
print(f"{'':<25} | {'Inductance (mH)':<20} | {transformer_inductance_H * 1e3:<12.4f} | {'N/A':<12}")
print("-" * 80)
print(row_fmt.format("Total CLS Commutation", "Reactance", cls_commutation_reactance_ohm, cls_commutation_reactance_pu_lv_base))
print(row_fmt.format("", "Resistance", cls_commutation_resistance_ohm, cls_commutation_resistance_pu_lv_base))
print(f"{'':<25} | {'Inductance (mH)':<20} | {cls_commutation_inductance_H * 1e3:<12.4f} | {'N/A':<12}")
print("-" * 80)
print(row_fmt.format("DC Link Choke", "Inductance (mH)", dc_link_inductance_H * 1e3, dc_link_inductance_pu))
print(row_fmt.format("", "Resistance", dc_link_resistance_ohm, dc_link_resistance_pu))
print("-" * 80)
print(row_fmt.format("Machine Commutation", "Reactance", cms_commutation_reactance_ohm, cms_commutation_reactance_pu))
print(row_fmt.format("", "Resistance", cms_stator_resistance_ohm, machine_stator_resistance_pu))
print(f"{'':<25} | {'Inductance (mH)':<20} | {cms_commutation_inductance_H * 1e3:<12.4f} | {'N/A':<12}")
print("="*80)
print()


# ------------------------------------------------------------------------------
# SECTION 4: NOMINAL STEADY-STATE CALCULATION
# ------------------------------------------------------------------------------
print("--- [SECTION 4] Calculating Nominal Steady-State Operating Point (at 50 Hz) ---")

# --- 4a. Calculate Inverter DC Back-Voltage ---
# All calculations use the nominal machine frequency (50 Hz)
machine_ideal_no_load_dc_voltage = (3 * math.sqrt(2) / math.pi) * machine_service_voltage
inverter_commutation_voltage_drop = (3 * base_angular_frequency_elec_cms * cms_commutation_inductance_H / math.pi) * dc_link_current_rated

# The relationship between gamma, beta, and the commutation voltage drop (dVx) is:
# Vdo * cos(gamma) = Vdo * cos(beta) - 2 * dVx
# From this, we calculate the required advance angle (beta) to achieve the target extinction angle (gamma).
cos_beta = math.cos(math.radians(inverter_extinction_angle_setpoint_deg)) - (2 * inverter_commutation_voltage_drop / machine_ideal_no_load_dc_voltage)
beta_angle_deg = math.degrees(math.acos(cos_beta))

# The inverter's average DC back-voltage is determined by the extinction angle (gamma) and the ideal DC voltage.
# Vd_inv = Vdo * cos(gamma). This is the correct formula.
# The previous calculation was incorrectly subtracting dVx after already accounting for it in beta.
inverter_dc_voltage = machine_ideal_no_load_dc_voltage * math.cos(math.radians(inverter_extinction_angle_setpoint_deg))

# --- 4b. Calculate Required Rectifier DC Voltage ---
# The rectifier must overcome the inverter's back-voltage plus the resistive drop in the DC link.
rectifier_dc_voltage = inverter_dc_voltage + dc_link_current_rated * dc_link_resistance_ohm

# --- 4c. Calculate Grid-Side Angle (Alpha) ---
# The rectifier's firing angle (alpha) is calculated to produce the required DC voltage.
# Vd_rect = Vdo * cos(alpha) - dVx
line_ideal_no_load_dc_voltage = (3 * math.sqrt(2) / math.pi) * transformer_secondary_voltage_noload
rectifier_commutation_voltage_drop = (3 * base_angular_frequency_lv * cls_commutation_inductance_H / math.pi) * dc_link_current_rated
cos_alpha = (rectifier_dc_voltage + rectifier_commutation_voltage_drop) / line_ideal_no_load_dc_voltage
alpha_angle_deg = math.degrees(math.acos(cos_alpha))

print("...Calculations complete.\n")


# ------------------------------------------------------------------------------
# SECTION 5: NOMINAL OPERATING POINT CALCULATIONS (CONTINUED)
# ------------------------------------------------------------------------------
# Calculate final values for the summary table.

# --- Power ---
rectifier_dc_power = rectifier_dc_voltage * dc_link_current_rated
inverter_dc_power = inverter_dc_voltage * dc_link_current_rated
machine_power = inverter_dc_power
line_power = rectifier_dc_power

# --- Currents ---
machine_current_rms = dc_link_current_rated * math.sqrt(6) / math.pi
line_secondary_current_rms = dc_link_current_rated * math.sqrt(6) / math.pi

# --- Power Factors ---
machine_power_factor_calculated = machine_power / (math.sqrt(3) * machine_service_voltage * machine_current_rms)
line_displacement_power_factor = math.cos(math.radians(alpha_angle_deg))

# --- Reactive Power ---
machine_reactive_power = machine_power * math.tan(math.acos(machine_power_factor_calculated))
line_reactive_power = line_power * math.tan(math.radians(alpha_angle_deg))

# --- Overlap Angles ---
inverter_overlap_angle_deg = beta_angle_deg - inverter_extinction_angle_setpoint_deg
rectifier_overlap_angle_deg = math.degrees(math.acos(cos_alpha - 2 * rectifier_commutation_voltage_drop / line_ideal_no_load_dc_voltage)) - alpha_angle_deg

# --- True Power Factor ---
grid_power_factor = (math.cos(math.radians(alpha_angle_deg)) + math.cos(math.radians(alpha_angle_deg + rectifier_overlap_angle_deg))) / 2


# ------------------------------------------------------------------------------
# SECTION 6: FINAL SUMMARY TABLE
# ------------------------------------------------------------------------------
print("="*70)
print("            LCI SYSTEM - NOMINAL OPERATING POINT SUMMARY")
print("="*70)

# --- Table Formatting ---
hdr_fmt = "{:<25} | {:<20} | {:<20}"
row_fmt = "{:<25} | {:<20.2f} | {:<20.2f}"
unt_fmt = "{:<25} | {:<20} | {:<20}"

# --- Print Header ---
print(hdr_fmt.format("PARAMETER", "GRID-SIDE (CLS)", "MACHINE-SIDE (CMS)"))
print("-" * 70)

# --- Print Rows ---
print(unt_fmt.format("AC Voltage", f"{transformer_secondary_voltage_noload:.0f} V (L-L)", f"{machine_service_voltage:.0f} V (L-L)"))
print(row_fmt.format("AC Current", line_secondary_current_rms, machine_current_rms))
print(unt_fmt.format("  (Unit)", "A (rms)", "A (rms)"))
print(row_fmt.format("Frequency", line_frequency, machine_base_frequency))
print(unt_fmt.format("  (Unit)", "Hz", "Hz"))
print("-" * 70)
print(row_fmt.format("DC Voltage", rectifier_dc_voltage, inverter_dc_voltage))
print(unt_fmt.format("  (Unit)", "V", "V"))
print(row_fmt.format("DC Current", dc_link_current_rated, dc_link_current_rated))
print(unt_fmt.format("  (Unit)", "A", "A"))
print("-" * 70)
print(row_fmt.format("Active Power", line_power / 1e6, machine_power / 1e6))
print(unt_fmt.format("  (Unit)", "MW", "MW"))
print(row_fmt.format("Reactive Power", line_reactive_power / 1e6, machine_reactive_power / 1e6))
print(unt_fmt.format("  (Unit)", "MVAr", "MVAr"))
print(row_fmt.format("Displ. Power Factor", line_displacement_power_factor, machine_power_factor_calculated))
print(unt_fmt.format("  (Unit)", "", ""))
print(f"{'True Power Factor':<25} | {grid_power_factor:<20.3f} | {'N/A':<20}")
print("-" * 70)
print(row_fmt.format("Firing/Advance Angle", alpha_angle_deg, beta_angle_deg))
print(unt_fmt.format("  (Unit)", "deg (alpha)", "deg (beta)"))
print(row_fmt.format("Overlap Angle", rectifier_overlap_angle_deg, inverter_overlap_angle_deg))
print(unt_fmt.format("  (Unit)", "deg", "deg"))
print(f"{'Extinction Angle (gamma)':<25} | {'N/A':<20} | {inverter_extinction_angle_setpoint_deg:<20.2f}")
print(unt_fmt.format("  (Unit)", "", "deg"))
print("="*70)
print()


# ------------------------------------------------------------------------------
# SECTION 7: MACHINE PARAMETER VALIDATION & FIELD VOLTAGE ANALYSIS
# ------------------------------------------------------------------------------
print("--- [SECTION 7] Machine Parameter Validation & Field Voltage Analysis ---")

# --- Generator Rating for Inertia Analysis ---
GENERATOR_RATING = 52e6  # VA (full generator rating - this is the full GT-Gen rating)
SYNC_SPEED_RPM = 3000    # rpm (2-pole, 50 Hz)

# --- Machine Parameters for Field Voltage Calculation ---
# These parameters define the synchronous machine's electrical characteristics
# NOTE: Updated parametrization for realistic field voltage (~120V) and current (~150A)
# Previous parametrization had Ns_Nfd=0.049202, resulting in impractical 10,721A field current!
machine_electrical_params = {
    'Rs': 0.001187,      # Stator Resistance (Ohm)
    'Lls': 0.0003470,    # Stator Leakage Inductance (H)
    'Lmd': 0.006898,     # D-axis Magnetizing Inductance (H)
    'Lmq': 0.00651,      # Q-axis Magnetizing Inductance (H)
    'Rf': 0.800000,      # Field Winding Resistance (Ohm)
    'Llfd': 1.547952,    # Field Winding Leakage Inductance (H)
    'Ns_Nfd': 3.516740,  # Stator to Field turns ratio
    'Jm': 3688.09        # Moment of Inertia (kg*m^2) - H=4.5s for typical GT-Gen
}

# --- Calculate nominal field voltage at rated speed ---
vf_nominal, if_nominal, eq_nominal = calculate_field_voltage_and_current(
    machine_electrical_params, machine_service_voltage, machine_base_frequency)

# ==============================================================================
# 7.1 MOMENT OF INERTIA VALIDATION
# ==============================================================================
print("\n" + "="*80)
print("                    MOMENT OF INERTIA VALIDATION")
print("="*80)

# Calculate mechanical angular velocity
omega_mech = (2 * math.pi * SYNC_SPEED_RPM) / 60  # rad/s

# Calculate inertia constant H from Jm
Jm_current = machine_electrical_params['Jm']
H_current = (0.5 * Jm_current * omega_mech**2) / GENERATOR_RATING

print(f"\nGenerator Rating:                    {GENERATOR_RATING/1e6:.1f} MVA")
print(f"Synchronous Speed:                   {SYNC_SPEED_RPM} rpm")
print(f"Mechanical Angular Velocity:         {omega_mech:.2f} rad/s")
print(f"\nCurrent Jm:                          {Jm_current:.2f} kg*m^2")
print(f"Inertia Constant (H):                {H_current:.2f} seconds")

# Assessment
print("\nAssessment:")
if H_current > 6:
    print(f"  WARNING: H = {H_current:.2f}s is TOO HIGH for gas turbine generator!")
    print(f"  - Typical range: H = 3.5-5.5s for GT-Gen combined system")
    print(f"  - Current value suggests steam turbine or hydro (not GT)")
elif H_current < 3:
    print(f"  WARNING: H = {H_current:.2f}s is TOO LOW for GT-Gen system!")
    print(f"  - Typical range: H = 3.5-5.5s for GT-Gen combined system")
else:
    print(f"  OK: H = {H_current:.2f}s is within typical GT-Gen range (3.5-5.5s)")

# Provide reference values
print("\n" + "-"*80)
print("REFERENCE: Jm VALUES FOR DIFFERENT INERTIA CONSTANTS")
print("-"*80)
print(f"{'H [s]':<10} | {'Jm [kg*m^2]':<15} | {'Application Type':<30}")
print("-" * 60)

H_ref_values = [
    (3.5, "Light industrial GT-Gen"),
    (4.0, "Typical industrial GT-Gen"),
    (4.5, "Typical industrial GT-Gen (RECOMMENDED)"),
    (5.0, "Heavy industrial GT-Gen"),
    (5.5, "Heavy industrial GT-Gen"),
]

for H_val, app_type in H_ref_values:
    Jm_val = (2 * H_val * GENERATOR_RATING) / (omega_mech**2)
    marker = " <-- CURRENT" if abs(H_val - H_current) < 0.1 else ""
    print(f"{H_val:<10.1f} | {Jm_val:<15.2f} | {app_type:<30}{marker}")

# ==============================================================================
# 7.2 MACHINE ELECTRICAL PARAMETERS
# ==============================================================================
print("\n" + "="*80)
print("                    MACHINE ELECTRICAL PARAMETERS")
print("="*80)
print(f"\nStator Resistance (Rs):              {machine_electrical_params['Rs']*1000:.6f} mOhm")
print(f"Stator Leakage Inductance (Lls):     {machine_electrical_params['Lls']*1000:.6f} mH")
print(f"D-axis Magnetizing Inductance (Lmd): {machine_electrical_params['Lmd']*1000:.6f} mH")
print(f"Q-axis Magnetizing Inductance (Lmq): {machine_electrical_params['Lmq']*1000:.6f} mH")
print(f"Field Winding Resistance (Rf):       {machine_electrical_params['Rf']:.6f} Ohm")
print(f"Field Winding Leakage Inductance:    {machine_electrical_params['Llfd']:.6f} H")
print(f"Stator to Field Turns Ratio (Ns/Nfd):{machine_electrical_params['Ns_Nfd']:.6f}")

# Calculate field time constant
Lf = machine_electrical_params['Llfd'] + machine_electrical_params['Lmd'] * machine_electrical_params['Ns_Nfd']**2
tau_f = Lf / machine_electrical_params['Rf']
print(f"Field Time Constant (tau_f):         {tau_f:.3f} seconds")

# ==============================================================================
# 7.3 FIELD VOLTAGE PARAMETRIZATION VALIDATION
# ==============================================================================
print("\n" + "="*80)
print("                    FIELD VOLTAGE PARAMETRIZATION VALIDATION")
print("="*80)

# Original problematic parameters
original_Ns_Nfd = 0.049202
original_Rf = 0.000612
original_Llfd = 0.000303

# Calculate what the OLD parametrization would have given
omega_elec = 2 * math.pi * machine_base_frequency
Vt_phase_rms = machine_service_voltage / math.sqrt(3)
If_old = Vt_phase_rms / (omega_elec * machine_electrical_params['Lmd'] * original_Ns_Nfd)
Vf_old = If_old * original_Rf
Pf_old = Vf_old * If_old

# Calculate current parametrization
If_new = Vt_phase_rms / (omega_elec * machine_electrical_params['Lmd'] * machine_electrical_params['Ns_Nfd'])
Vf_new = If_new * machine_electrical_params['Rf']
Pf_new = Vf_new * If_new

print("\nComparison: Original vs. Corrected Parametrization")
print("-" * 80)
print(f"{'Parameter':<25} | {'Original (WRONG)':<20} | {'Corrected':<20}")
print("-" * 80)
print(f"{'Ns/Nfd turns ratio':<25} | {original_Ns_Nfd:<20.6f} | {machine_electrical_params['Ns_Nfd']:<20.6f}")
print(f"{'Field resistance [Ohm]':<25} | {original_Rf:<20.6f} | {machine_electrical_params['Rf']:<20.6f}")
print(f"{'Field inductance [H]':<25} | {original_Llfd:<20.6f} | {machine_electrical_params['Llfd']:<20.6f}")
print("-" * 80)
print(f"{'Field current [A]':<25} | {If_old:<20.1f} | {If_new:<20.1f}")
print(f"{'Field voltage [V]':<25} | {Vf_old:<20.2f} | {Vf_new:<20.2f}")
print(f"{'Field power [kW]':<25} | {Pf_old/1e3:<20.2f} | {Pf_new/1e3:<20.2f}")
print(f"{'Field power [% rated]':<25} | {100*Pf_old/machine_rated_power:<20.2f} | {100*Pf_new/machine_rated_power:<20.2f}")

print("\nValidation Status:")
if If_new < 500 and Vf_new > 50 and Vf_new < 200:
    print("  OK: Field current is realistic (< 500A)")
    print("  OK: Field voltage is practical (50-200V range)")
    print("  OK: Field power is typical (< 1% of rated)")
else:
    print("  WARNING: Field parameters may need adjustment!")

# ==============================================================================
# 7.4 FIELD VOLTAGE REQUIREMENTS
# ==============================================================================

print("\n" + "="*80)
print("                    FIELD VOLTAGE REQUIREMENTS")
print("="*80)
print("\n--- Nominal Operating Point (Synchronization @ 50 Hz) ---")
print(f"Terminal Voltage (L-L):          {machine_service_voltage:.2f} V")
print(f"Terminal Voltage (Phase RMS):    {machine_service_voltage/math.sqrt(3):.2f} V")
print(f"Operating Frequency:             {machine_base_frequency:.2f} Hz")
print(f"\nInternal Voltage (Eq):           {eq_nominal:.2f} V")
print(f"Required Field Current (If):     {if_nominal:.4f} A")
print(f"Required Field Voltage (Vf):     {vf_nominal:.6f} V")

# --- Field voltage vs. speed profile for soft-starter operation ---
print("\n" + "="*80)
print("              SOFT-STARTER FIELD VOLTAGE PROFILE")
print("="*80)
print("\nField voltage requirements at different operating speeds:")
print("(Based on V/Hz control for constant flux operation)\n")

# Define speed ranges based on typical gas turbine starting sequence
speed_profile_points = [
    (0.01, "Standstill / Initial Magnetization"),
    (0.05, "Creep Speed (inspection, oil circulation)"),
    (0.10, "Low-Speed Transition"),
    (0.20, "Purge Acceleration (chamber ventilation)"),
    (0.35, "Medium Speed"),
    (0.55, "Ignition Acceleration (fuel ignition)"),
    (0.70, "High Speed Acceleration"),
    (0.85, "Approach to Self-Sustaining"),
    (0.95, "Assist Acceleration (final phase)"),
    (1.00, "Synchronization Speed (rated)")
]

speeds_pu = [s[0] for s in speed_profile_points]
field_profile = calculate_field_voltage_vs_speed(
    machine_electrical_params, machine_service_voltage, machine_base_frequency, speeds_pu, control_strategy='constant_flux')

# Print field voltage profile table
print("\nControl Strategy: CONSTANT FLUX (V/Hz control)")
print("Note: For V/Hz control, field current and voltage remain CONSTANT.")
print("      The armature voltage and frequency vary proportionally with speed.\n")

hdr_fmt = "{:<12} | {:<38} | {:>10} | {:>12} | {:>12} | {:>10}"
row_fmt = "{:<12.2f} | {:<38} | {:>10.2f} | {:>12.1f} | {:>12.6f} | {:>10.4f}"
print(hdr_fmt.format("Speed [pu]", "Operating Phase", "Freq [Hz]", "V_term [V]", "Vf [V]", "If [A]"))
print("-" * 105)

for speed_pu, description in speed_profile_points:
    if speed_pu in field_profile:
        data = field_profile[speed_pu]
        print(row_fmt.format(
            speed_pu, 
            description,
            data['frequency_Hz'],
            data['terminal_voltage_ll_V'],
            data['field_voltage_V'],
            data['field_current_A']
        ))

# --- Field voltage control recommendations ---
print("\n" + "="*80)
print("           FIELD VOLTAGE CONTROL RECOMMENDATIONS")
print("="*80)

print("\nPHYSICAL UNDERSTANDING:")
print("-" * 80)
print("For CONSTANT FLUX (V/Hz) control used in soft-starters:")
print("  • Flux linkage Psi = V/omega is maintained constant")
print("  • Since Eq = omega × Lmd × (Ns/Nfd) × If, constant flux requires:")
print("  • Field current If = CONSTANT throughout acceleration")
print(f"  • Required field current: {if_nominal:.4f} A (constant)")
print(f"  • Required field voltage: {vf_nominal:.6f} V (DC steady-state)")
print("-" * 80)

print("\nOPERATIONAL STRATEGY:")
print("\nStandard V/Hz Control (Constant Flux):")
print(f"  • Field Current: {if_nominal:.4f} A (100% of rated, constant)")
print(f"  • Field Voltage: {vf_nominal:.6f} V (constant)")
print("  • Armature Voltage: 0 to {:.0f} V (proportional to speed)".format(machine_service_voltage))
print("  • Armature Frequency: 0 to {:.0f} Hz (proportional to speed)".format(machine_base_frequency))

print("\nALTERNATIVE: Reduced Field Starting (Industry Practice):")
print("  Some applications use reduced field at low speeds to:")
print("  • Reduce SFC current requirements")
print("  • Ease commutation at low back-EMF")
print("  • Reduce torque pulsations")

if 0.05 in field_profile and 0.20 in field_profile:
    print("\n  1. STARTUP PHASE (0-20% speed):")
    print(f"     - Industry practice: {0.3*if_nominal:.4f} - {0.4*if_nominal:.4f} A (30-40% of rated)")
    print(f"     - Corresponding Vf:  {0.3*vf_nominal:.6f} - {0.4*vf_nominal:.6f} V")

if 0.20 in field_profile and 0.70 in field_profile:
    print("\n  2. ACCELERATION PHASE (20-70% speed):")
    print(f"     - Industry practice: {0.6*if_nominal:.4f} - {0.8*if_nominal:.4f} A (60-80% of rated)")
    print(f"     - Corresponding Vf:  {0.6*vf_nominal:.6f} - {0.8*vf_nominal:.6f} V")

if 0.70 in field_profile and 1.00 in field_profile:
    print("\n  3. HIGH-SPEED PHASE (70-100% speed):")
    print(f"     - Industry practice: {0.8*if_nominal:.4f} - {if_nominal:.4f} A (80-100% of rated)")
    print(f"     - Corresponding Vf:  {0.8*vf_nominal:.6f} - {vf_nominal:.6f} V")

print("\n" + "="*80)
print("CRITICAL OPERATIONAL NOTES:")
print("="*80)
print("- NEVER step-change field current - always ramp smoothly")
print("- Typical ramp rates: 5-20% per second")
print("- Field changes MUST be coordinated with firing angle adjustments")
print("- Values shown are for V/Hz control (constant flux strategy)")
print("- Field winding time constant tau_f = L_f/R_f = {:.3f} s".format(tau_f))
print("- Allow 3-5 x tau_f for field current settling ({:.1f}-{:.1f} seconds)".format(3*tau_f, 5*tau_f))
print("="*80)


# ==============================================================================
# 7.5 PARAMETER UPDATE RECOMMENDATIONS
# ==============================================================================
print("\n" + "="*80)
print("                    PARAMETER UPDATE RECOMMENDATIONS")
print("="*80)

print("\nFOR TYPHOON HIL MODEL (.tse file):")
print("-" * 80)
print("Update Synchronous Machine component:")
print(f"  Jm = {machine_electrical_params['Jm']:.2f}  (moment of inertia, kg*m^2)")
print(f"  Rf = {machine_electrical_params['Rf']:.6f}  (field resistance, Ohm)")
print(f"  Llfd = {machine_electrical_params['Llfd']:.6f}  (field leakage inductance, H)")
print(f"  Ns/Nfd = {machine_electrical_params['Ns_Nfd']:.6f}  (turns ratio)")

print("\nFOR PYTHON ANALYSIS SCRIPTS:")
print("-" * 80)
print("machine_electrical_params = {")
print(f"    'Jm': {machine_electrical_params['Jm']:.2f},")
print(f"    'Rf': {machine_electrical_params['Rf']:.6f},")
print(f"    'Llfd': {machine_electrical_params['Llfd']:.6f},")
print(f"    'Ns_Nfd': {machine_electrical_params['Ns_Nfd']:.6f},")
print("}")
print("\n" + "="*80)


# ==============================================================================
# SECTION 8: GAS TURBINE LOAD CHARACTERISTIC & STARTUP CAPABILITY ANALYSIS
# ==============================================================================
print("\n" + "="*80)
print("        SECTION 8: GAS TURBINE LOAD CHARACTERISTIC & STARTUP CAPABILITY")
print("="*80)

def calculate_load_torque(speed_rpm, rated_power_VA=GENERATOR_RATING, rated_speed_rpm=SYNC_SPEED_RPM):
    """
    Calculate the resistive load torque for the gas turbine generator.
    
    Physical model:
        - Breakaway torque: Static friction that must be overcome to start rotation
        - Running friction: Coulomb friction (velocity-independent, ~80% of breakaway)
        - Windage losses: Aerodynamic drag proportional to speed squared
        - Self-sustaining: Speed at which GT can maintain itself without starter
    
    Args:
        speed_rpm: Current rotational speed in RPM
        rated_power_VA: Generator full rating [VA]
        rated_speed_rpm: Synchronous speed [RPM]
        
    Returns:
        Resistive torque in Nm
    """
    # Calculate rated torque
    omega_mech_rated = 2.0 * math.pi * rated_speed_rpm / 60.0  # rad/s
    rated_torque_Nm = rated_power_VA / omega_mech_rated
    
    # Breakaway torque (20% of rated for large GT)
    breakaway_torque = 0.20 * rated_torque_Nm
    
    # Windage coefficient (0.5% losses at rated speed)
    torque_at_rated_speed = 0.005 * rated_torque_Nm
    k_windage = torque_at_rated_speed / (rated_speed_rpm ** 2)
    
    # Starter disengagement speed (65% of rated - GT self-sustaining)
    starter_disengage_rpm = 0.65 * rated_speed_rpm
    
    # Calculate load torque based on speed
    if speed_rpm >= 0 and speed_rpm < 10:
        # Breakaway region - static friction dominates
        load_torque = breakaway_torque
        
    elif speed_rpm >= 10 and speed_rpm <= starter_disengage_rpm:
        # Running region - Coulomb friction + windage
        running_friction = 0.80 * breakaway_torque
        load_torque = running_friction + k_windage * (speed_rpm ** 2)
        
    else:
        # Self-sustaining region - GT provides its own power
        load_torque = 1e6  # Out-of-range indicator
        
    return load_torque


# ==============================================================================
# 8.1 LOAD TORQUE CHARACTERISTICS
# ==============================================================================
print("\n" + "="*80)
print("                    LOAD TORQUE CHARACTERISTICS")
print("="*80)

# System parameters for load analysis
omega_mech_rated = 2.0 * math.pi * SYNC_SPEED_RPM / 60.0
rated_torque_Nm = GENERATOR_RATING / omega_mech_rated

# LCI capability
lci_torque_continuous = machine_rated_power / omega_mech_rated
lci_torque_peak_typical = 3.0 * lci_torque_continuous  # Typical 3x overload capability

print(f"\nGENERATOR SPECIFICATIONS:")
print(f"  Full Generator Rating:          {GENERATOR_RATING/1e6:.1f} MVA")
print(f"  Synchronous Speed:              {SYNC_SPEED_RPM:.0f} RPM")
print(f"  Rated Torque:                   {rated_torque_Nm/1e3:.1f} kNm")

print(f"\nLCI SOFT-STARTER SPECIFICATIONS:")
print(f"  LCI Rating:                     {machine_rated_power/1e6:.2f} MVA")
print(f"  LCI Continuous Torque:          {lci_torque_continuous/1e3:.2f} kNm")
print(f"  LCI Peak Torque (3x rated):     {lci_torque_peak_typical/1e3:.2f} kNm (short duration)")

# Key operating points for gas turbine startup sequence
print("\n" + "-"*80)
print("GAS TURBINE STARTUP SEQUENCE - LOAD TORQUE REQUIREMENTS")
print("-"*80)

speeds_of_interest = [
    (0,    "Breakaway (Static Friction)"),
    (10,   "Initial Motion"),
    (150,  "Creep Speed (5% - Inspection)"),
    (600,  "Purge Acceleration (20% - Chamber Ventilation)"),
    (1050, "Medium Acceleration (35%)"),
    (1650, "Ignition Speed (55% - Fuel Ignition)"),
    (1950, "Self-Sustaining (65% - GT Independent)"),
]

hdr_fmt = "{:<42} | {:>12} | {:>15} | {:>18}"
row_fmt = "{:<42} | {:>8.0f} RPM | {:>11.1f} kNm | {:>18}"

print(hdr_fmt.format("Operating Phase", "Speed", "Load Torque", "LCI Margin"))
print("-" * 95)

for speed, description in speeds_of_interest:
    torque = calculate_load_torque(speed)
    
    if speed == 0:
        # Breakaway uses peak torque capability
        margin = torque / lci_torque_peak_typical
        margin_str = f"{margin:.2f}x peak"
    else:
        # Running uses continuous torque
        margin = torque / lci_torque_continuous
        margin_str = f"{margin:.2f}x cont."
    
    print(row_fmt.format(description, speed, torque/1e3, margin_str))


# ==============================================================================
# 8.2 MOTOR STARTUP CAPABILITY EVALUATION
# ==============================================================================
print("\n" + "="*80)
print("                    MOTOR STARTUP CAPABILITY EVALUATION")
print("="*80)

# Calculate critical torque values
breakaway_torque = calculate_load_torque(0)
running_torque_150rpm = calculate_load_torque(150)
running_torque_1000rpm = calculate_load_torque(1000)
torque_at_disengagement = calculate_load_torque(1950)

# Torque margins
breakaway_to_peak_ratio = breakaway_torque / lci_torque_peak_typical
breakaway_to_continuous_ratio = breakaway_torque / lci_torque_continuous
running_to_continuous_ratio_150 = running_torque_150rpm / lci_torque_continuous
running_to_continuous_ratio_1000 = running_torque_1000rpm / lci_torque_continuous

print("\nCRITICAL TORQUE ANALYSIS:")
print("-" * 80)
print(f"Breakaway Torque:                  {breakaway_torque/1e3:.1f} kNm ({100*breakaway_torque/rated_torque_Nm:.1f}% of rated)")
print(f"Running Torque @ 150 RPM:          {running_torque_150rpm/1e3:.1f} kNm")
print(f"Running Torque @ 1000 RPM:         {running_torque_1000rpm/1e3:.1f} kNm")
print(f"Torque @ Self-Sustaining (1950):   {torque_at_disengagement/1e3:.1f} kNm")

print("\nLCI CAPABILITY MARGINS:")
print("-" * 80)
print(f"Breakaway / LCI Peak:              {breakaway_to_peak_ratio:.3f}x")
print(f"Breakaway / LCI Continuous:        {breakaway_to_continuous_ratio:.2f}x")
print(f"Running (150 RPM) / LCI Cont.:     {running_to_continuous_ratio_150:.2f}x")
print(f"Running (1000 RPM) / LCI Cont.:    {running_to_continuous_ratio_1000:.2f}x")

# Startup capability assessment
print("\n" + "="*80)
print("                    STARTUP CAPABILITY ASSESSMENT")
print("="*80)

assessment_passed = True

# Check 1: Breakaway torque vs peak capability
print("\n[1] BREAKAWAY CAPABILITY:")
if breakaway_to_peak_ratio <= 1.0:
    print(f"    ✓ PASS: Breakaway torque ({breakaway_torque/1e3:.1f} kNm) is within LCI peak capability")
    print(f"            ({lci_torque_peak_typical/1e3:.1f} kNm, margin: {(1-breakaway_to_peak_ratio)*100:.1f}%)")
elif breakaway_to_peak_ratio <= 1.2:
    print(f"    ⚠ MARGINAL: Breakaway torque ({breakaway_torque/1e3:.1f} kNm) exceeds LCI peak by {(breakaway_to_peak_ratio-1)*100:.1f}%")
    print(f"                Recommendation: Use pulsed current strategy at low speeds")
    print(f"                - Apply current pulses with duty cycle control")
    print(f"                - Allow rotor to settle between pulses")
    print(f"                - Typical pulse duration: 0.5-2 seconds")
else:
    print(f"    ✗ FAIL: Breakaway torque ({breakaway_torque/1e3:.1f} kNm) significantly exceeds LCI peak")
    print(f"            ({lci_torque_peak_typical/1e3:.1f} kNm, {(breakaway_to_peak_ratio-1)*100:.1f}% overload)")
    print(f"            System may not be able to start reliably!")
    assessment_passed = False

# Check 2: Low-speed acceleration capability
print("\n[2] LOW-SPEED ACCELERATION (0-150 RPM):")
if running_to_continuous_ratio_150 <= 3.0:
    print(f"    ✓ PASS: Running torque ({running_torque_150rpm/1e3:.1f} kNm) can be handled with pulsed operation")
    print(f"            Margin to peak: {lci_torque_peak_typical/running_torque_150rpm:.2f}x")
    print(f"            Strategy: Slow acceleration with current limiting")
elif running_to_continuous_ratio_150 <= 4.0:
    print(f"    ⚠ MARGINAL: Running torque requires careful current management")
    print(f"                Recommendation: Very slow acceleration (< 10 RPM/s)")
else:
    print(f"    ✗ FAIL: Running torque exceeds practical LCI capability")
    assessment_passed = False

# Check 3: Medium-speed acceleration capability
print("\n[3] MEDIUM-SPEED ACCELERATION (150-1950 RPM):")
if running_to_continuous_ratio_1000 <= 3.0:
    print(f"    ✓ PASS: LCI can sustain acceleration through medium speeds")
    print(f"            Running torque @ 1000 RPM: {running_torque_1000rpm/1e3:.1f} kNm")
    print(f"            Margin to continuous: {lci_torque_continuous/running_torque_1000rpm:.2f}x")
else:
    print(f"    ⚠ MARGINAL: Sustained acceleration may be challenging")
    print(f"                Recommendation: Gradual acceleration profile")

# Check 4: Thermal considerations
total_startup_energy_estimate = breakaway_torque * (10 * 2*math.pi/60) * 5  # ~5 seconds breakaway
total_startup_energy_estimate += running_torque_1000rpm * (1000 * 2*math.pi/60) * 60  # ~60 seconds to 1000 RPM
thermal_load_MJ = total_startup_energy_estimate / 1e6

print("\n[4] THERMAL CONSIDERATIONS:")
print(f"    Estimated startup energy:       ~{thermal_load_MJ:.1f} MJ")
print(f"    Typical startup duration:       90-180 seconds (to self-sustaining)")
if thermal_load_MJ < 100:
    print(f"    ✓ PASS: Thermal load is manageable for LCI converter")
else:
    print(f"    ⚠ WARNING: High thermal load - verify LCI cooling capacity")

# Check 5: Commutation capability at low speeds
print("\n[5] COMMUTATION CAPABILITY:")
min_back_emf_speed = 150  # RPM - minimum for reliable natural commutation
print(f"    Minimum speed for natural commutation: ~{min_back_emf_speed} RPM")
print(f"    Below {min_back_emf_speed} RPM: Require forced commutation or pulsed operation")
print(f"    Above {min_back_emf_speed} RPM: Natural load commutation available")
print(f"    ✓ PASS: System designed for low-speed commutation")

# Check 6: Acceleration rate feasibility
Jm = machine_electrical_params['Jm']
max_accel_torque_avg = lci_torque_continuous - running_torque_1000rpm  # Net torque at 1000 RPM
max_acceleration_rad_s2 = max_accel_torque_avg / Jm
max_acceleration_rpm_s = max_acceleration_rad_s2 * 60 / (2*math.pi)

print("\n[6] ACCELERATION CAPABILITY:")
print(f"    Moment of Inertia:              {Jm:.2f} kg*m^2 (H = {H_current:.2f}s)")
print(f"    Net Torque @ 1000 RPM:          {max_accel_torque_avg/1e3:.1f} kNm (LCI - Load)")
if max_accel_torque_avg > 0:
    print(f"    Max Acceleration @ 1000 RPM:    {max_acceleration_rpm_s:.2f} RPM/s")
    time_to_1000rpm = 1000 / max_acceleration_rpm_s if max_acceleration_rpm_s > 0 else float('inf')
    print(f"    Est. Time to 1000 RPM:          {time_to_1000rpm:.1f} seconds (continuous acceleration)")
    print(f"    ✓ PASS: Adequate acceleration capability")
else:
    print(f"    ✗ FAIL: Insufficient torque margin for acceleration")
    assessment_passed = False

# Final verdict
print("\n" + "="*80)
print("                    FINAL STARTUP CAPABILITY VERDICT")
print("="*80)

if assessment_passed:
    print("\n✓ SYSTEM CAN START THE MOTOR")
    print("\nStartup Strategy Recommendations:")
    print("  1. BREAKAWAY PHASE (0-10 RPM):")
    print("     - Use pulsed current operation if needed")
    print(f"     - Target current: {1.05*breakaway_torque/lci_torque_continuous:.1f}x rated (~peak capability)")
    print("     - Pulse duration: 1-3 seconds, monitor temperature")
    print("     - Allow mechanical settling between pulses")
    
    print("\n  2. LOW-SPEED PHASE (10-150 RPM):")
    print("     - Slow, controlled acceleration")
    print("     - Target rate: 5-15 RPM/s")
    print("     - Current: 2.5-3.0x rated (within short-term capability)")
    print("     - Field current: 30-40% of rated (reduced flux for easier commutation)")
    
    print("\n  3. MEDIUM-SPEED PHASE (150-600 RPM):")
    print("     - Moderate acceleration")
    print("     - Target rate: 10-20 RPM/s")
    print("     - Current: 2.0-2.5x rated")
    print("     - Field current: 60-80% of rated (increase flux gradually)")
    
    print("\n  4. HIGH-SPEED PHASE (600-1950 RPM):")
    print("     - Normal acceleration")
    print("     - Target rate: 15-30 RPM/s")
    print("     - Current: 1.0-2.0x rated")
    print("     - Field current: 80-100% of rated (approach rated flux)")
    
    print("\n  5. TYPICAL TOTAL STARTUP TIME:")
    print("     - Breakaway to 150 RPM:  15-30 seconds")
    print("     - 150 to 600 RPM:        30-45 seconds")
    print("     - 600 to 1950 RPM:       45-90 seconds")
    print("     - TOTAL:                 90-165 seconds to self-sustaining")
    
    print("\n  CRITICAL SUCCESS FACTORS:")
    print("  • Coordinate field current ramping with speed")
    print("  • Monitor DC link current and limit to 3x rated maximum")
    print("  • Use extinction angle control to prevent commutation failures")
    print("  • Implement thermal monitoring for converter protection")
    print(f"  • Maintain field current: {if_nominal:.1f}A at rated speed")
    print(f"  • Maintain field voltage: {vf_nominal:.1f}V at rated speed")
    
else:
    print("\n✗ SYSTEM MAY NOT RELIABLY START THE MOTOR")
    print("\nIssues Identified:")
    if breakaway_to_peak_ratio > 1.2:
        print(f"  • Breakaway torque too high ({breakaway_to_peak_ratio:.2f}x peak capability)")
    if running_to_continuous_ratio_150 > 4.0:
        print(f"  • Low-speed running torque excessive ({running_to_continuous_ratio_150:.2f}x continuous)")
    if max_accel_torque_avg <= 0:
        print(f"  • Insufficient torque margin for acceleration")
    
    print("\nRecommendations:")
    print("  1. Verify load torque model parameters")
    print("  2. Consider larger LCI rating (e.g., 4-5 MVA)")
    print("  3. Investigate auxiliary starting methods")
    print("  4. Review mechanical system for friction reduction")

print("\n" + "="*80)


# ==============================================================================
# SECTION 9: PROTECTION PARAMETERS & OPERATIONAL LIMITS
# ==============================================================================
print("\n" + "="*80)
print("        SECTION 9: PROTECTION PARAMETERS & OPERATIONAL LIMITS")
print("="*80)

# ==============================================================================
# 9.1 TORQUE LIMITS DURING STARTUP
# ==============================================================================
print("\n" + "="*80)
print("                    TORQUE LIMITS DURING STARTUP")
print("="*80)

print("\nPHYSICAL BASIS:")
print("-" * 80)
print("Torque production in LCI-driven synchronous machine:")
print("  • Electromagnetic torque Te = (3/2) × (P/2) × Lmd × (Ns/Nfd) × If × Id")
print("  • Maximum torque limited by:")
print("    1. DC link current capability (thermal & thyristor ratings)")
print("    2. Machine synchronous reactance (pull-out torque)")
print("    3. Commutation requirements (extinction angle constraints)")
print("    4. Field current level (flux linkage)")

# Calculate torque limits for different operating regions
Lmd = machine_electrical_params['Lmd']
Ns_Nfd = machine_electrical_params['Ns_Nfd']
P = machine_number_of_poles
omega_base_elec = 2 * math.pi * machine_base_frequency

# Current limits
Idc_rated = dc_link_current_rated
Idc_continuous_max = 1.0 * Idc_rated  # 100% continuous
Idc_short_term_max = 2.0 * Idc_rated  # 200% for 60 seconds (typical)
Idc_peak_max = 3.0 * Idc_rated        # 300% for 5-10 seconds (typical)
Idc_absolute_max = 3.5 * Idc_rated    # 350% absolute trip limit

# Convert DC current to d-axis current (Id = Idc for current-source inverter)
# For 6-pulse CSI: Id_pk = (2/3) × Idc
Id_rated_peak = (2.0/3.0) * Idc_rated
Id_continuous_peak = (2.0/3.0) * Idc_continuous_max
Id_short_term_peak = (2.0/3.0) * Idc_short_term_max
Id_peak_max_peak = (2.0/3.0) * Idc_peak_max
Id_absolute_max_peak = (2.0/3.0) * Idc_absolute_max

print("\n" + "-"*80)
print("DC LINK CURRENT LIMITS")
print("-"*80)
print(f"{'Operating Mode':<30} | {'DC Current [A]':<15} | {'DC Current [pu]':<15} | {'Duration':<25}")
print("-" * 95)
print(f"{'Continuous (100%)':<30} | {Idc_continuous_max:>10.1f} A   | {1.00:>10.2f} pu  | Unlimited")
print(f"{'Short-term (200%)':<30} | {Idc_short_term_max:>10.1f} A   | {2.00:>10.2f} pu  | 60 seconds (typical)")
print(f"{'Peak (300%)':<30} | {Idc_peak_max:>10.1f} A   | {3.00:>10.2f} pu  | 5-10 seconds (pulsed)")
print(f"{'Absolute Trip (350%)':<30} | {Idc_absolute_max:>10.1f} A   | {3.50:>10.2f} pu  | Immediate trip")

# Calculate torque limits for different field current levels
# Te = (3/2) × (P/2) × Lmd × (Ns/Nfd) × If × Id
def calculate_torque_limit(If_value, Id_peak_value):
    """Calculate electromagnetic torque based on field and d-axis current."""
    return (3.0/2.0) * (P/2.0) * Lmd * Ns_Nfd * If_value * Id_peak_value

print("\n" + "-"*80)
print("TORQUE LIMITS vs FIELD CURRENT & DC LINK CURRENT")
print("-"*80)

# Define field current levels for different startup phases
field_current_levels = [
    (0.30 * if_nominal, "Startup Phase (30%)"),
    (0.40 * if_nominal, "Startup Phase (40%)"),
    (0.60 * if_nominal, "Accel Phase (60%)"),
    (0.80 * if_nominal, "Accel Phase (80%)"),
    (1.00 * if_nominal, "Rated (100%)"),
]

print(f"\n{'Field Current':<25} | {'@ 100% Idc':<15} | {'@ 200% Idc':<15} | {'@ 300% Idc':<15} | {'@ 350% Idc':<15}")
print("-" * 95)

torque_limit_matrix = {}
for If_pu, description in field_current_levels:
    T_100 = calculate_torque_limit(If_pu, Id_rated_peak)
    T_200 = calculate_torque_limit(If_pu, Id_short_term_peak)
    T_300 = calculate_torque_limit(If_pu, Id_peak_max_peak)
    T_350 = calculate_torque_limit(If_pu, Id_absolute_max_peak)
    
    torque_limit_matrix[description] = {
        '100%': T_100,
        '200%': T_200,
        '300%': T_300,
        '350%': T_350
    }
    
    print(f"{description:<25} | {T_100/1e3:>10.1f} kNm | {T_200/1e3:>10.1f} kNm | {T_300/1e3:>10.1f} kNm | {T_350/1e3:>10.1f} kNm")

# Recommended torque limits for control system
print("\n" + "-"*80)
print("RECOMMENDED TORQUE LIMITS FOR STARTUP CONTROL")
print("-"*80)

startup_torque_limits = [
    ("Breakaway Phase (0-10 RPM)", 0.40 * if_nominal, Id_peak_max_peak, 10),
    ("Low-Speed (10-150 RPM)", 0.40 * if_nominal, Id_peak_max_peak, 10),
    ("Medium-Speed (150-600 RPM)", 0.70 * if_nominal, Id_short_term_peak, 60),
    ("High-Speed (600-1950 RPM)", 1.00 * if_nominal, Id_continuous_peak, float('inf')),
    ("Synchronization (1950-3000 RPM)", 1.00 * if_nominal, Id_continuous_peak, float('inf')),
]

print(f"{'Operating Phase':<35} | {'Torque Limit [kNm]':<18} | {'Torque Limit [pu]':<18} | {'Max Duration':<15}")
print("-" * 100)

for phase, If_level, Id_level, duration in startup_torque_limits:
    T_limit = calculate_torque_limit(If_level, Id_level)
    T_limit_pu = T_limit / base_torque_cms
    duration_str = "Unlimited" if duration == float('inf') else f"{duration} seconds"
    print(f"{phase:<35} | {T_limit/1e3:>13.1f} kNm  | {T_limit_pu:>13.4f} pu  | {duration_str:<15}")


# ==============================================================================
# 9.2 STALL PROTECTION PARAMETERS
# ==============================================================================
print("\n" + "="*80)
print("                    STALL PROTECTION PARAMETERS")
print("="*80)

print("\nPHYSICAL BASIS:")
print("-" * 80)
print("Stall condition occurs when:")
print("  • Motor fails to accelerate despite applied torque")
print("  • Load torque exceeds available electromagnetic torque")
print("  • Mechanical jamming or excessive friction")
print("  • Detection: Speed derivative falls below threshold while current is high")

# Calculate stall detection parameters
min_acceleration_threshold_rpm_s = 1.0  # Minimum acceptable acceleration [RPM/s]
min_acceleration_threshold_rad_s2 = min_acceleration_threshold_rpm_s * (2*math.pi/60)

# Stall current threshold: When current exceeds limit without acceleration
stall_current_threshold_pu = 2.5  # 250% of rated
stall_current_threshold_A = stall_current_threshold_pu * Idc_rated

# Stall detection time windows for different speed ranges
stall_detection_windows = [
    ("Breakaway (0-10 RPM)", 0, 10, 30.0, 3.0 * Idc_rated),
    ("Low-Speed (10-150 RPM)", 10, 150, 20.0, 3.0 * Idc_rated),
    ("Medium-Speed (150-600 RPM)", 150, 600, 15.0, 2.5 * Idc_rated),
    ("High-Speed (600-1950 RPM)", 600, 1950, 10.0, 2.0 * Idc_rated),
]

print("\n" + "-"*80)
print("STALL DETECTION PARAMETERS")
print("-"*80)
print(f"{'Speed Range':<30} | {'Detection Time':<18} | {'Current Threshold':<20}")
print("-" * 80)

for description, speed_min, speed_max, timeout, current_threshold in stall_detection_windows:
    print(f"{description:<30} | {timeout:>12.1f} seconds | {current_threshold/Idc_rated:>8.1f}x rated ({current_threshold:>7.0f}A)")

print("\n" + "-"*80)
print("STALL PROTECTION LOGIC")
print("-"*80)
print("Condition 1: Speed derivative < 1.0 RPM/s for detection window")
print("Condition 2: DC link current > threshold for same window")
print("Condition 3: Both conditions must be met simultaneously")
print("\nAction on Stall Detection:")
print("  1. Reduce DC link current to 50% immediately")
print("  2. Hold for 5 seconds (allow mechanical settling)")
print("  3. Retry with pulsed current (50% duty cycle)")
print("  4. If 3 consecutive stalls → TRIP and alarm")
print("\nStall Recovery Strategy:")
print("  • Attempt 1: Reduce current, retry after 5s")
print("  • Attempt 2: Pulsed operation (0.5s ON, 1.0s OFF)")
print("  • Attempt 3: Reduced field current (reduce to 50%)")
print("  • After 3 attempts: FAULT - Manual intervention required")


# ==============================================================================
# 9.3 FLUX MONITORING & PROTECTION
# ==============================================================================
print("\n" + "="*80)
print("                    FLUX MONITORING & PROTECTION")
print("="*80)

print("\nPHYSICAL BASIS:")
print("-" * 80)
print("Flux linkage relationship:")
print("  • Flux (pu) = V / (ω × V_base/ω_base) = V / f (normalized)")
print("  • For constant V/Hz control: Flux should remain ≈ 1.0 pu")
print("  • Underflux (< 0.7 pu): Reduced torque capability, possible stall")
print("  • Overflux (> 1.15 pu): Core saturation, excessive magnetizing current")

# Calculate flux limits
flux_nominal_pu = 1.0
flux_underflux_warning_pu = 0.80
flux_underflux_trip_pu = 0.70
flux_overflux_warning_pu = 1.10
flux_overflux_trip_pu = 1.15

# Calculate corresponding voltage/frequency ratios
# V/f ratio in V/Hz (phase voltage)
vf_ratio_nominal = (machine_service_voltage / math.sqrt(3)) / machine_base_frequency
vf_ratio_underflux_warning = flux_underflux_warning_pu * vf_ratio_nominal
vf_ratio_underflux_trip = flux_underflux_trip_pu * vf_ratio_nominal
vf_ratio_overflux_warning = flux_overflux_warning_pu * vf_ratio_nominal
vf_ratio_overflux_trip = flux_overflux_trip_pu * vf_ratio_nominal

print("\n" + "-"*80)
print("FLUX PROTECTION THRESHOLDS")
print("-"*80)
print(f"{'Protection Level':<30} | {'Flux (pu)':<15} | {'V/f Ratio (V/Hz)':<20} | {'Action':<15}")
print("-" * 95)
print(f"{'Nominal Operating Point':<30} | {flux_nominal_pu:>10.2f} pu  | {vf_ratio_nominal:>15.2f} V/Hz | Normal")
print(f"{'Underflux Warning':<30} | {flux_underflux_warning_pu:>10.2f} pu  | {vf_ratio_underflux_warning:>15.2f} V/Hz | Alarm")
print(f"{'Underflux Trip':<30} | {flux_underflux_trip_pu:>10.2f} pu  | {vf_ratio_underflux_trip:>15.2f} V/Hz | Trip")
print(f"{'Overflux Warning':<30} | {flux_overflux_warning_pu:>10.2f} pu  | {vf_ratio_overflux_warning:>15.2f} V/Hz | Alarm")
print(f"{'Overflux Trip':<30} | {flux_overflux_trip_pu:>10.2f} pu  | {vf_ratio_overflux_trip:>15.2f} V/Hz | Trip")

# Flux monitoring implementation
print("\n" + "-"*80)
print("FLUX MONITORING IMPLEMENTATION")
print("-"*80)
print("\nMethod 1: V/Hz Monitoring (Primary)")
print("  • Measure terminal voltage Vt (L-L RMS)")
print("  • Measure frequency f from inverter firing")
print(f"  • Calculate: Flux_pu = (Vt/√3) / f / {vf_ratio_nominal:.2f}")
print("  • Sample rate: Every PWM cycle or 1 kHz minimum")
print("  • Filter: Moving average (10-50 samples)")

print("\nMethod 2: Field Current Monitoring (Secondary)")
print(f"  • Nominal field current: {if_nominal:.1f} A (1.00 pu)")
print(f"  • Warning if If < {0.5*if_nominal:.1f} A (0.50 pu, 50% of rated)")
print(f"  • Trip if If < {0.3*if_nominal:.1f} A (0.30 pu, 30% of rated)")
print("  • Indicates field winding fault or excitation failure")

print("\nMethod 3: Back-EMF Estimation (at speed > 10% rated)")
print("  • Estimate Eq = Vt + jXs×Is (voltage-behind-reactance)")
print("  • Calculate flux from Eq = ω × Ψ")
print("  • Cross-check with Method 1 for validation")

# Protection logic
print("\n" + "-"*80)
print("FLUX PROTECTION LOGIC & TIMING")
print("-"*80)
print(f"{'Condition':<35} | {'Time Delay':<15} | {'Action':<30}")
print("-" * 90)
print(f"{'Underflux < 0.80 pu':<35} | {'2.0 seconds':<15} | Warning alarm")
print(f"{'Underflux < 0.70 pu':<35} | {'1.0 second':<15} | Trip (loss of torque)")
print(f"{'Overflux > 1.10 pu':<35} | {'5.0 seconds':<15} | Warning alarm")
print(f"{'Overflux > 1.15 pu':<35} | {'2.0 seconds':<15} | Trip (saturation)")
print(f"{'Field current < 50%':<35} | {'1.0 second':<15} | Warning alarm")
print(f"{'Field current < 30%':<35} | {'0.5 second':<15} | Trip (field failure)")

print("\nFlux Control Recommendations:")
print("  • During startup (0-20% speed): Allow ±15% flux variation")
print("  • During acceleration (20-90% speed): Maintain ±10% flux tolerance")
print("  • During synchronization (90-100%): Tighten to ±5% flux tolerance")
print(f"  • V/Hz slope: {vf_ratio_nominal:.2f} V/Hz ± 2%")


# ==============================================================================
# 9.4 DC LINK PROTECTION PARAMETERS
# ==============================================================================
print("\n" + "="*80)
print("                    DC LINK PROTECTION PARAMETERS")
print("="*80)

print("\nPHYSICAL BASIS:")
print("-" * 80)
print("DC link current protection is critical for:")
print("  • Thyristor thermal protection (I²t capability)")
print("  • DC reactor current rating")
print("  • Commutation capability (di/dt limits)")
print("  • Short-circuit protection")

# DC link current limits (already defined above)
print("\n" + "-"*80)
print("DC LINK CURRENT PROTECTION LEVELS")
print("-"*80)
print(f"{'Protection Level':<35} | {'Current (A)':<15} | {'Current (pu)':<15} | {'Time Delay':<15} | {'Action':<15}")
print("-" * 105)
print(f"{'Rated Continuous':<35} | {Idc_rated:>10.1f} A   | {1.0:>10.2f} pu  | Unlimited       | Normal")
print(f"{'Overload Warning (150%)':<35} | {1.5*Idc_rated:>10.1f} A   | {1.5:>10.2f} pu  | 120 seconds     | Alarm")
print(f"{'Short-term Limit (200%)':<35} | {Idc_short_term_max:>10.1f} A   | {2.0:>10.2f} pu  | 60 seconds      | Reduce current")
print(f"{'Peak Limit (300%)':<35} | {Idc_peak_max:>10.1f} A   | {3.0:>10.2f} pu  | 10 seconds      | Force reduction")
print(f"{'Absolute Trip (350%)':<35} | {Idc_absolute_max:>10.1f} A   | {3.5:>10.2f} pu  | 0.1 second      | Immediate trip")
print(f"{'Short-circuit (400%)':<35} | {4.0*Idc_rated:>10.1f} A   | {4.0:>10.2f} pu  | Instantaneous   | Emergency trip")

# I²t thermal protection
print("\n" + "-"*80)
print("THERMAL PROTECTION (I²t INTEGRATION)")
print("-"*80)

# Calculate thermal capacity based on typical thyristor ratings
# Assume thyristor rated for continuous Idc_rated with thermal time constant τ_th ≈ 10 seconds
tau_thermal = 10.0  # seconds (typical for large thyristors)
I2t_continuous = Idc_rated**2 * tau_thermal  # Reference thermal capacity

print(f"Thermal Time Constant (τ_th):     {tau_thermal:.1f} seconds")
print(f"Continuous I²t capacity:           {I2t_continuous/1e6:.2f} MA²s")
print(f"Rated current:                     {Idc_rated:.1f} A")

print("\nThermal Protection Algorithm:")
print("  • Initialize: I²t_accumulated = 0")
print("  • Each sample: I²t_accumulated += (Idc² × Δt)")
print("  • Each sample: I²t_accumulated -= (I²t_accumulated × Δt / τ_th)  [cooling]")
print(f"  • Warning if I²t_accumulated > {0.8*I2t_continuous/1e6:.2f} MA²s (80% capacity)")
print(f"  • Trip if I²t_accumulated > {I2t_continuous/1e6:.2f} MA²s (100% capacity)")
print("  • Reset after 5 × τ_th with Idc < rated")

# Calculate allowable overload durations
print("\n" + "-"*80)
print("MAXIMUM OVERLOAD DURATIONS (based on I²t thermal limit)")
print("-"*80)
print(f"{'Current Level':<25} | {'Maximum Duration':<20} | {'I²t Capacity Used':<20}")
print("-" * 75)

overload_levels = [1.5, 2.0, 2.5, 3.0, 3.5, 4.0]
for overload_pu in overload_levels:
    # I²t_limit = (Idc_overload)² × t_max
    # For adiabatic limit (worst case, no cooling): t_max = I2t_continuous / Idc_overload²
    t_max_adiabatic = I2t_continuous / (overload_pu * Idc_rated)**2
    I2t_used_pu = 1.0  # At this duration, 100% capacity is used
    
    print(f"{overload_pu:>4.1f}x rated ({overload_pu*Idc_rated:>7.0f}A) | {t_max_adiabatic:>15.1f} seconds | {I2t_used_pu:>15.0%}")

print("\nNOTE: Actual allowable times may be longer if cooling is effective.")
print(f"      With cooling (τ_th = {tau_thermal}s), equilibrium is reached at:")
for overload_pu in [1.5, 2.0, 2.5, 3.0]:
    # At equilibrium: I² = I_rated² + (I²t_limit / τ_th)
    # Check if equilibrium is possible
    if overload_pu <= math.sqrt(2):  # Can reach thermal equilibrium
        print(f"        {overload_pu}x rated: Thermal equilibrium possible (infinite if cooling adequate)")
    else:
        t_eq = tau_thermal * math.log((overload_pu**2) / (overload_pu**2 - 1))
        print(f"        {overload_pu}x rated: Approaches trip after {t_eq:.1f} seconds")

# DC link voltage protection
print("\n" + "-"*80)
print("DC LINK VOLTAGE PROTECTION")
print("-"*80)

# Calculate nominal DC voltages
Vdc_rectifier_nominal = rectifier_dc_voltage
Vdc_inverter_nominal = inverter_dc_voltage
Vdc_nominal = Vdc_rectifier_nominal  # Use rectifier voltage as reference

# Voltage limits
Vdc_overvoltage_warning = 1.15 * Vdc_nominal
Vdc_overvoltage_trip = 1.25 * Vdc_nominal
Vdc_undervoltage_warning = 0.85 * Vdc_nominal
Vdc_undervoltage_trip = 0.75 * Vdc_nominal

print(f"{'Protection Level':<35} | {'Voltage (V)':<15} | {'Voltage (pu)':<15} | {'Time Delay':<15} | {'Action':<15}")
print("-" * 105)
print(f"{'Nominal Operating Point':<35} | {Vdc_nominal:>10.1f} V   | {1.0:>10.2f} pu  | N/A             | Normal")
print(f"{'Overvoltage Warning':<35} | {Vdc_overvoltage_warning:>10.1f} V   | {1.15:>10.2f} pu  | 2.0 seconds     | Alarm")
print(f"{'Overvoltage Trip':<35} | {Vdc_overvoltage_trip:>10.1f} V   | {1.25:>10.2f} pu  | 0.5 second      | Trip")
print(f"{'Undervoltage Warning':<35} | {Vdc_undervoltage_warning:>10.1f} V   | {0.85:>10.2f} pu  | 2.0 seconds     | Alarm")
print(f"{'Undervoltage Trip':<35} | {Vdc_undervoltage_trip:>10.1f} V   | {0.75:>10.2f} pu  | 1.0 second      | Trip")

print("\nDC Voltage Protection Considerations:")
print("  • Overvoltage: Grid voltage swell, regenerative braking, loss of load")
print("  • Undervoltage: Grid voltage sag, excessive load, rectifier fault")
print("  • Fast voltage transients (< 100ms) should be filtered to avoid nuisance trips")
print("  • Use moving average filter (50-100ms window) for voltage measurement")


# ==============================================================================
# 9.5 COMMUTATION FAILURE PROTECTION
# ==============================================================================
print("\n" + "="*80)
print("                    COMMUTATION FAILURE PROTECTION")
print("="*80)

print("\nPHYSICAL BASIS:")
print("-" * 80)
print("Commutation failure occurs when:")
print("  • Insufficient back-EMF to reverse-bias outgoing thyristor")
print("  • Extinction angle γ < minimum safe value (typically 15-18°)")
print("  • Line voltage disturbance during commutation interval")
print("  • Excessive DC link current (long overlap, insufficient turn-off time)")

# Calculate minimum extinction angle requirements
gamma_min_safe = 15.0  # degrees (absolute minimum for reliable commutation)
gamma_nominal_setpoint = inverter_extinction_angle_setpoint_deg  # 18° from earlier
gamma_warning_threshold = 16.0  # degrees (warning if approaching minimum)

# Calculate corresponding beta angles
print("\n" + "-"*80)
print("EXTINCTION ANGLE MONITORING & PROTECTION")
print("-"*80)
print(f"{'Parameter':<40} | {'Value':<15} | {'Status':<20}")
print("-" * 85)
print(f"{'Nominal extinction angle (γ) setpoint':<40} | {gamma_nominal_setpoint:>10.1f}°    | Normal operation")
print(f"{'Warning threshold':<40} | {gamma_warning_threshold:>10.1f}°    | Reduce current")
print(f"{'Minimum safe extinction angle':<40} | {gamma_min_safe:>10.1f}°    | Critical limit")
print(f"{'Trip threshold':<40} | {gamma_min_safe:>10.1f}°    | Immediate trip")

# Calculate back-EMF requirements for different speeds
print("\n" + "-"*80)
print("BACK-EMF REQUIREMENTS FOR COMMUTATION")
print("-"*80)
print("\nMinimum back-EMF for natural commutation:")
print(f"  • At rated speed ({SYNC_SPEED_RPM} RPM): {machine_service_voltage:.0f} V (L-L RMS)")
print(f"  • At 50% speed ({0.5*SYNC_SPEED_RPM:.0f} RPM): {0.5*machine_service_voltage:.0f} V (L-L RMS)")
print(f"  • At 20% speed ({0.2*SYNC_SPEED_RPM:.0f} RPM): {0.2*machine_service_voltage:.0f} V (L-L RMS)")
print(f"  • Below 10% speed: Natural commutation unreliable → Use pulsed mode")

# Overlap angle limits
overlap_angle_max_safe = 60.0  # degrees (excessive overlap indicates problems)
overlap_angle_warning = 45.0   # degrees

print("\n" + "-"*80)
print("COMMUTATION OVERLAP ANGLE MONITORING")
print("-"*80)
print(f"{'Parameter':<40} | {'Angle':<15} | {'Indication':<25}")
print("-" * 90)
print(f"{'Nominal overlap (μ) @ rated current':<40} | {inverter_overlap_angle_deg:>10.1f}°    | Normal")
print(f"{'Warning threshold':<40} | {overlap_angle_warning:>10.1f}°    | High current/low voltage")
print(f"{'Maximum safe overlap':<40} | {overlap_angle_max_safe:>10.1f}°    | Approaching commutation limit")

# Commutation failure detection
print("\n" + "-"*80)
print("COMMUTATION FAILURE DETECTION & RECOVERY")
print("-"*80)
print("\nDetection Methods:")
print("  1. Extinction Angle Measurement")
print(f"     • Monitor γ via thyristor gate/anode voltage sensing")
print(f"     • Trigger warning if γ < {gamma_warning_threshold}° for > 5 cycles")
print(f"     • Trigger trip if γ < {gamma_min_safe}° for > 2 cycles")
print("\n  2. DC Current Ripple Analysis")
print("     • Normal operation: ~5-10% ripple at 6× line frequency")
print("     • Commutation failure: Sudden increase to > 20% ripple")
print("     • Missing current notches indicate failed commutation")
print("\n  3. Overlap Angle Monitoring")
print(f"     • Warning if μ > {overlap_angle_warning}° (indicates marginal commutation)")
print(f"     • Critical if μ > {overlap_angle_max_safe}° (commutation failure imminent)")

print("\nRecovery Actions:")
print("  1. Immediate: Reduce DC link current by 25%")
print("  2. After 100ms: Increase extinction angle setpoint by 2-3°")
print("  3. After 500ms: If persistent, reduce to 50% current (limp mode)")
print("  4. After 3 consecutive failures: TRIP and fault alarm")

print("\nPrevention Strategy:")
print("  • Maintain γ ≥ 18° during normal operation")
print("  • At low speeds (< 20%): Use extended extinction angles (γ = 20-25°)")
print("  • During grid disturbances: Temporarily increase γ by 3-5°")
print("  • Coordinate with field current: Higher If → Better commutation margin")


# ==============================================================================
# 9.6 COMPREHENSIVE PROTECTION SUMMARY
# ==============================================================================
print("\n" + "="*80)
print("                    COMPREHENSIVE PROTECTION SUMMARY")
print("="*80)

print("\n" + "-"*80)
print("CRITICAL PROTECTION PARAMETERS - READY FOR IMPLEMENTATION")
print("-"*80)

print("\n[1] DC LINK CURRENT PROTECTION")
print(f"    • I_rated_continuous    = {Idc_rated:.1f} A  (1.00 pu)")
print(f"    • I_warning_150pct      = {1.5*Idc_rated:.1f} A  (1.50 pu, 120s max)")
print(f"    • I_shortterm_200pct    = {Idc_short_term_max:.1f} A  (2.00 pu, 60s max)")
print(f"    • I_peak_300pct         = {Idc_peak_max:.1f} A  (3.00 pu, 10s max)")
print(f"    • I_trip_350pct         = {Idc_absolute_max:.1f} A  (3.50 pu, 0.1s trip)")
print(f"    • I_emergency_400pct    = {4.0*Idc_rated:.1f} A  (4.00 pu, instantaneous trip)")

print("\n[2] DC LINK VOLTAGE PROTECTION")
print(f"    • V_nominal             = {Vdc_nominal:.1f} V  (1.00 pu)")
print(f"    • V_overvolt_warning    = {Vdc_overvoltage_warning:.1f} V  (1.15 pu, 2s delay)")
print(f"    • V_overvolt_trip       = {Vdc_overvoltage_trip:.1f} V  (1.25 pu, 0.5s delay)")
print(f"    • V_undervolt_warning   = {Vdc_undervoltage_warning:.1f} V  (0.85 pu, 2s delay)")
print(f"    • V_undervolt_trip      = {Vdc_undervoltage_trip:.1f} V  (0.75 pu, 1s delay)")

print("\n[3] THERMAL PROTECTION (I²t)")
print(f"    • Thermal_time_constant = {tau_thermal:.1f} seconds")
print(f"    • I2t_warning_80pct     = {0.8*I2t_continuous/1e6:.2f} MA²s  (0.80 pu)")
print(f"    • I2t_trip_100pct       = {I2t_continuous/1e6:.2f} MA²s  (1.00 pu)")
print(f"    • Cooling_reset_time    = {5*tau_thermal:.0f} seconds (5× τ_th)")

print("\n[4] FLUX PROTECTION (V/Hz)")
print(f"    • Vf_ratio_nominal      = {vf_ratio_nominal:.2f} V/Hz  (1.00 pu)")
print(f"    • Flux_underflux_warn   = {flux_underflux_warning_pu:.2f} pu  ({vf_ratio_underflux_warning:.2f} V/Hz, 2s delay)")
print(f"    • Flux_underflux_trip   = {flux_underflux_trip_pu:.2f} pu  ({vf_ratio_underflux_trip:.2f} V/Hz, 1s delay)")
print(f"    • Flux_overflux_warn    = {flux_overflux_warning_pu:.2f} pu  ({vf_ratio_overflux_warning:.2f} V/Hz, 5s delay)")
print(f"    • Flux_overflux_trip    = {flux_overflux_trip_pu:.2f} pu  ({vf_ratio_overflux_trip:.2f} V/Hz, 2s delay)")

print("\n[5] FIELD CURRENT PROTECTION")
print(f"    • If_nominal            = {if_nominal:.1f} A  (1.00 pu)")
print(f"    • If_warning_50pct      = {0.5*if_nominal:.1f} A  (0.50 pu, 1s delay)")
print(f"    • If_trip_30pct         = {0.3*if_nominal:.1f} A  (0.30 pu, 0.5s delay)")
print(f"    • Vf_nominal            = {vf_nominal:.1f} V  (1.00 pu)")
print(f"    • Vf_overvolt_trip      = {1.5*vf_nominal:.1f} V  (1.50 pu, field voltage limit)")

print("\n[6] COMMUTATION PROTECTION")
print(f"    • Gamma_nominal         = {gamma_nominal_setpoint:.1f}° (extinction angle setpoint)")
print(f"    • Gamma_warning         = {gamma_warning_threshold:.1f}° (reduce current)")
print(f"    • Gamma_trip            = {gamma_min_safe:.1f}° (minimum safe)")
print(f"    • Overlap_warning       = {overlap_angle_warning:.1f}° (μ angle)")
print(f"    • Overlap_max           = {overlap_angle_max_safe:.1f}° (critical)")

print("\n[7] STALL PROTECTION")
print(f"    • Min_acceleration      = {min_acceleration_threshold_rpm_s:.1f} RPM/s")
print(f"    • Stall_current_thresh  = {stall_current_threshold_pu:.1f}× rated ({stall_current_threshold_A:.0f} A, {stall_current_threshold_pu:.2f} pu)")
print(f"    • Stall_timeout_0-10    = 30 seconds")
print(f"    • Stall_timeout_10-150  = 20 seconds")
print(f"    • Stall_timeout_150-600 = 15 seconds")
print(f"    • Stall_retry_attempts  = 3 (then trip)")

print("\n[8] TORQUE LIMITS BY PHASE")
T_breakaway = calculate_torque_limit(0.40*if_nominal, Id_peak_max_peak)
T_lowspeed = calculate_torque_limit(0.40*if_nominal, Id_peak_max_peak)
T_medspeed = calculate_torque_limit(0.70*if_nominal, Id_short_term_peak)
T_highspeed = calculate_torque_limit(1.00*if_nominal, Id_continuous_peak)
print(f"    • Breakaway (0-10 RPM)       : {T_breakaway/1e3:.1f} kNm ({T_breakaway/base_torque_cms:.4f} pu, 10s)")
print(f"    • Low-speed (10-150 RPM)     : {T_lowspeed/1e3:.1f} kNm ({T_lowspeed/base_torque_cms:.4f} pu, 10s)")
print(f"    • Medium-speed (150-600 RPM) : {T_medspeed/1e3:.1f} kNm ({T_medspeed/base_torque_cms:.4f} pu, 60s)")
print(f"    • High-speed (600-1950 RPM)  : {T_highspeed/1e3:.1f} kNm ({T_highspeed/base_torque_cms:.4f} pu, continuous)")

print("\n" + "="*80)
print("END OF COMPREHENSIVE SYSTEM ANALYSIS")
print("="*80)
