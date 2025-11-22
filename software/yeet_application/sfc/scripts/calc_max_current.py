
import math

# --- Parameters from steady_state_analysis.py output ---
# Voltages
V_grid_AC_LL = 2090.0
V_mach_AC_LL = 1980.0

# Inductances (H)
L_c_grid = 0.4941e-3
L_c_mach = 0.2508e-3

# Resistances (Ohm)
R_phys_grid = 0.0163
R_phys_mach = 0.0066
R_dc_link = 0.0503

# Frequency
f = 50.0
omega = 2 * math.pi * f

# Ideal DC Voltages (Vd0)
Vd0_grid = (3 * math.sqrt(2) / math.pi) * V_grid_AC_LL
Vd0_mach = (3 * math.sqrt(2) / math.pi) * V_mach_AC_LL

# Commutation Resistances (Rc = 3*w*L/pi = 6*f*L)
Rc_grid = 6 * f * L_c_grid
Rc_mach = 6 * f * L_c_mach

# Control Limits
alpha_min = 0.0 # degrees
gamma_min = 18.0 # degrees

# --- Calculation ---
# Max Voltage Difference available
# V_rect_max = Vd0_grid * cos(0)
# V_inv_back_emf_min = Vd0_mach * cos(18)
V_rect_max_ideal = Vd0_grid * math.cos(math.radians(alpha_min))
V_inv_back_emf_min = Vd0_mach * math.cos(math.radians(gamma_min))

numerator = V_rect_max_ideal - V_inv_back_emf_min

# Effective Resistance
# Loop: V_rect_ideal - Rc_grid*I - Rphys_grid*I - Rdc*I - Rphys_mach*I = V_inv_ideal - Rc_mach*I
# Note: Inverter commutation drop REDUCES the effective back-EMF seen by the DC link.
# So: V_rect_ideal - V_inv_ideal = I * (Rc_grid + Rphys_grid + Rdc + Rphys_mach - Rc_mach)

R_eff_physical = Rc_grid + R_phys_grid + R_dc_link + R_phys_mach - Rc_mach
R_eff_script_model = Rc_grid + R_dc_link - Rc_mach # Script neglects AC physical resistances

I_max_physical = numerator / R_eff_physical
I_max_script = numerator / R_eff_script_model

print(f"Vd0_grid: {Vd0_grid:.2f} V")
print(f"Vd0_mach: {Vd0_mach:.2f} V")
print(f"Numerator (Delta V): {numerator:.2f} V")
print("-" * 30)
print(f"Rc_grid: {Rc_grid:.5f} Ohm")
print(f"Rc_mach: {Rc_mach:.5f} Ohm")
print(f"R_phys_grid: {R_phys_grid:.5f} Ohm")
print(f"R_phys_mach: {R_phys_mach:.5f} Ohm")
print(f"R_dc_link: {R_dc_link:.5f} Ohm")
print("-" * 30)
print(f"R_eff_physical: {R_eff_physical:.5f} Ohm")
print(f"R_eff_script_model: {R_eff_script_model:.5f} Ohm")
print("-" * 30)
print(f"I_max (Physical): {I_max_physical:.2f} A")
print(f"I_max (Script Model): {I_max_script:.2f} A")
