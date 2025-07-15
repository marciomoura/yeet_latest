import math

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
# ==============================================================================


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