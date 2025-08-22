import numpy as np
import matplotlib.pyplot as plt
import control as ct
from matplotlib.ticker import MultipleLocator

# --- System Parameters ---
f_nominal = 50.0  # Nominal grid frequency [Hz]
w_nominal = 2 * np.pi * f_nominal  # Nominal grid frequency [rad/s]
v_mag = 1.0  # Magnitude of the input voltage vector [p.u.]

# --- Function to compute PI gains for a desired crossover and phase margin ---
def compute_pi_gains(f_cutoff_hz, pm_deg, v_mag=1.0, w_nominal=w_nominal):
    """
    Compute PI controller gains (kp, ti) for a desired open-loop crossover
    frequency and phase margin.
    
    Args:
        f_cutoff_hz: Desired open-loop gain crossover frequency in Hz.
        pm_deg: Desired phase margin in degrees.
        v_mag: Input voltage magnitude.
        w_nominal: Nominal angular frequency.
        
    Returns:
        (kp, ti): Tuple of proportional gain and integral time constant.
    """
    # Convert inputs to standard units (rad/s and radians)
    w_c = 2 * np.pi * f_cutoff_hz
    pm_rad = np.deg2rad(pm_deg)
    
    # From the phase margin equation: Phase(G_ol(j*w_c)) = atan(ti*w_c) - 180° = -180° + PM
    # This gives: ti = tan(PM) / w_c
    ti = np.tan(pm_rad) / w_c
    
    # From the magnitude equation: |G_ol(j*w_c)| = 1
    # This gives: kp = (w_c * sin(PM)) / (V_mag * w_nominal)
    kp = (w_c * np.sin(pm_rad)) / (v_mag * w_nominal)
    
    return kp, ti

# --- Tuning Goals ---
desired_cutoff_hz = 30.0  # Desired open-loop crossover frequency in Hz
desired_phase_margin_deg = 60.0  # Desired phase margin for stability

# --- Calculate Controller Parameters ---
kp, ti = compute_pi_gains(desired_cutoff_hz, desired_phase_margin_deg)
ki = kp / ti

print("--- SRF-PLL Tuning for Crossover Frequency & Phase Margin ---")
print(f"Desired Crossover Frequency: {desired_cutoff_hz:.1f} Hz")
print(f"Desired Phase Margin: {desired_phase_margin_deg:.1f} deg")
print("\nCalculated PI Controller Gains:")
print(f"  Proportional Gain (Kp): {kp:.4f}")
print(f"  Integral Time (Ti): {ti*1000:.2f} ms")
print(f"  Integral Gain (Ki): {ki:.2f}")

# --- Define Final Transfer Functions ---
s = ct.tf('s')
controller_tf = kp * (1 + 1/(ti*s))
vco_tf = w_nominal / s

# --- System Analysis (No LPF) ---
open_loop_tf = v_mag * controller_tf * vco_tf
closed_loop_tf = ct.feedback(open_loop_tf, 1)
disturbance_rejection_tf = ct.feedback(1, open_loop_tf)

# --- Verification ---
gm, pm, wcg, wcp = ct.margin(open_loop_tf)
print("\n--- Verification ---")
print(f"Actual Gain Margin: {20*np.log10(gm):.2f} dB @ {wcp/(2*np.pi):.2f} Hz")
print(f"Actual Phase Margin: {pm:.2f} deg @ {wcg/(2*np.pi):.2f} Hz")

# --- Plotting ---
plt.style.use('seaborn-v0_8-whitegrid')

# 1. Open-Loop Bode Plot
plt.figure(1, figsize=(10, 8))
ct.bode_plot(open_loop_tf, omega_limits=(1, 1e5), plot=True, dB=True, Hz=True)
plt.suptitle('Open-Loop Transfer Function', fontsize=14)
ax_mag_ol, ax_phase_ol = plt.gcf().get_axes()

# Highlight 0dB crossover (Gain Crossover Frequency)
if np.isfinite(wcg):
    wcg_hz = wcg / (2 * np.pi)
    ax_mag_ol.axvline(x=wcg_hz, color='purple', linestyle=':')
    ax_phase_ol.axvline(x=wcg_hz, color='purple', linestyle=':')
    ax_mag_ol.text(wcg_hz * 1.1, -5, f'Gain Crossover\n{wcg_hz:.1f} Hz',
                   color='purple', verticalalignment='center', backgroundcolor='w')


# 2. Closed-Loop Bode Plot
plt.figure(2, figsize=(10, 8))
ct.bode_plot(closed_loop_tf, omega_limits=(1, 1e5), plot=True, dB=True, Hz=True)
plt.suptitle('Closed-Loop Transfer Function (Phase Tracking)', fontsize=14)
ax_mag_cl, ax_phase_cl = plt.gcf().get_axes()

# --- Add vertical line for 0dB crossover ---
# Generate frequency response data to find the 0dB crossover point
omega_vec = np.logspace(0, 5, 2000)  # rad/s
resp = ct.frequency_response(closed_loop_tf, omega_vec)
mag_db = 20 * np.log10(resp.magnitude)

# Find the first frequency where gain crosses below 0 dB
crossover_indices = np.where(np.diff(np.sign(mag_db)) < 0)[0]

if len(crossover_indices) > 0:
    zero_db_idx = crossover_indices[0]
    zero_db_freq_rads = omega_vec[zero_db_idx]
    zero_db_freq_hz = zero_db_freq_rads / (2 * np.pi)

    # Draw the vertical line and add text
    ax_mag_cl.axvline(x=zero_db_freq_hz, color='green', linestyle=':')
    ax_phase_cl.axvline(x=zero_db_freq_hz, color='green', linestyle=':')
    ax_mag_cl.text(zero_db_freq_hz * 1.1, -5, f' 0 dB Crossover\n {zero_db_freq_hz:.1f} Hz',
                 color='green', verticalalignment='center', backgroundcolor='w')


# 3. Disturbance Rejection Bode Plot
plt.figure(3, figsize=(10, 8))
ct.bode_plot(disturbance_rejection_tf, omega_limits=(1, 1e5), plot=True, dB=True, Hz=True)
plt.suptitle('Disturbance Rejection Transfer Function', fontsize=14)

# 4. Closed-Loop Step Response
plt.figure(4, figsize=(10, 8))
T = np.linspace(0, 0.5, 1500)  # Simulator for N seconds
_, yout = ct.step_response(closed_loop_tf, T=T)
plt.plot(T, yout)
plt.title('Closed-Loop Step Response (Phase Tracking)')
plt.xlabel('Time (s)')
plt.ylabel('Amplitude')
plt.grid(True)

plt.show()
