"""
CT Commissioning Analysis Utility for LCI Systems.

This script calculates the error and recommended correction factors for the 
current sensing system by comparing measured primary current (multimeter) 
with telemetry values reported in per-unit.

Reference Base Current: 500 A
"""

import numpy as np

# ==============================================================================
# COMMISSIONING DATA - EDIT THESE LISTS
# Format: [Measured Amps, Telemetry PU, Theoretical PU (Optional/Calculated)]
# ==============================================================================
NOMINAL_CURRENT = 500.0  # Amps

# Data points: [Degrees, Measured Amps, Telemetry PU]
DATA_POINTS = [
    [85, 200, 0.425],
    [83, 290, 0.65],
    [82, 330, 0.775],
    [81, 380, 0.89],
    [80, 430, 1.00],
    [79, 470, 1.10]
]
# ==============================================================================

def analyze_calibration():
    data = np.array(DATA_POINTS)
    degrees = data[:, 0]
    measured_amps = data[:, 1]
    telemetry_pu = data[:, 2]
    
    # Calculate theoretical PU if not provided
    theoretical_pu = measured_amps / NOMINAL_CURRENT
    
    # 1. Calculate Relative Error (%)
    # Error = (Measured_by_System - Actual) / Actual
    relative_errors = (telemetry_pu - theoretical_pu) / theoretical_pu * 100
    
    # 2. Calculate Correction Factor
    # New_Gain = Old_Gain * (Actual / Measured_by_System)
    correction_factors = theoretical_pu / telemetry_pu
    
    # Calculate averages
    avg_error = np.mean(relative_errors)
    avg_correction = np.mean(correction_factors)
    
    # Weighted average (giving more weight to higher current readings as they are usually more accurate)
    # This prevents low-current noise/offset from skewing the gain calibration
    weighted_correction = np.sum(correction_factors * theoretical_pu) / np.sum(theoretical_pu)

    print("\n" + "="*70)
    print(" LCI CT COMMISSIONING ANALYSIS")
    print("="*70)
    print(f"Nominal Current Base: {NOMINAL_CURRENT} A")
    print("-" * 70)
    print(f"{'Angle (°)':<10} | {'Meas (A)':<10} | {'Telem (pu)':<12} | {'Theo (pu)':<12} | {'Error (%)':<10}")
    print("-" * 70)
    for i in range(len(measured_amps)):
        print(f"{degrees[i]:<10.1f} | {measured_amps[i]:<10.1f} | {telemetry_pu[i]:<12.3f} | {theoretical_pu[i]:<12.3f} | {relative_errors[i]:>9.2f}%")
    
    print("-" * 70)
    print(f"Arithmetic Mean Error : {avg_error:.2f}%")
    print(f"Mean Correction Factor: {avg_correction:.4f}")
    print(f"Weighted Correction   : {weighted_correction:.4f} (Recommended)")
    print("-" * 70)
    
    print("\nAPPLICATION:")
    print("Update 'current_transducer_gain' or 'ct_ratio' in the control configuration.")
    print(f"New Value = Old Value * {weighted_correction:.4f}")
    print("="*70 + "\n")

if __name__ == "__main__":
    analyze_calibration()
