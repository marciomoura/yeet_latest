"""
Hardware Gain Calibration Utility for LCI Systems.

This script helps commissioning engineers calculate and verify transducer gains 
by comparing physical multimeter measurements with software-reported values.
"""

import numpy as np

# ==============================================================================
# CALIBRATION CONSTANTS - EDIT THESE VALUES DIRECTLY
# ==============================================================================
ADC_BITS = 16
ADC_RANGE = 20.0  # Full scale range (e.g. 20 for +/- 10V)

# --- VOLTAGE CALIBRATION ---
ENABLE_VOLTAGE = True
PT_RATIO = 25.0
V_MEASURED_RMS = [100.0, 100.0]  # Primary Side RMS (Multimeter)
V_COUNTS_PEAK = [18500, 18500]       # Software Peak Counts
V_IS_PRIMARY = False

# --- CURRENT CALIBRATION ---
ENABLE_CURRENT = True
CT_RATIO = 1000.0
I_MEASURED_RMS = [1.0, 1.0]    # Primary Side RMS (Multimeter)
I_COUNTS_PEAK = [15000, 15000]       # Software Peak Counts
I_IS_PRIMARY = False
# ==============================================================================

class HardwareCalibrator:
    def __init__(self, adc_bits, adc_range_v):
        self.adc_bits = adc_bits
        self.adc_range_v = adc_range_v
        self.counts_to_volts = adc_range_v / (2**adc_bits)

    def calculate_transducer_gain(self, multimeter_val, raw_counts, ratio, is_primary):
        # Convert peak counts to RMS volts at the card terminal
        v_card_peak = raw_counts * self.counts_to_volts
        v_card_rms = v_card_peak / np.sqrt(2)
        
        # If multimeter is primary, convert to secondary equivalent
        target_secondary_rms = multimeter_val / ratio if is_primary else multimeter_val
        
        if target_secondary_rms == 0:
            return 0.0
            
        # Transducer Gain [V/V or V/A] = V_card / V_secondary
        return v_card_rms / target_secondary_rms

def run_calibration(cal, name, meas_rms, counts_peak, ratio, is_primary, unit):
    if len(meas_rms) != len(counts_peak):
        print(f"\nError [{name}]: Number of measurements and counts must match.")
        return

    gains = []
    for m, c in zip(meas_rms, counts_peak):
        g = cal.calculate_transducer_gain(m, c, ratio, is_primary)
        gains.append(g)
        
    avg_gain = np.mean(gains)
    ref_side = "Primary" if is_primary else "Secondary"
    
    print(f"\n[{name}] Results (Reference: {ref_side}):")
    print("-" * 35)
    for i, (m, c, g) in enumerate(zip(meas_rms, counts_peak, gains)):
        print(f" Phase {chr(65+i)} | Meas: {m:7.2f} | Counts: {int(c):6} | Gain: {g:8.6f}")
    
    print("-" * 35)
    print(f"Average Transducer Gain: {avg_gain:8.6f} V/{'V' if unit == 'V' else 'A'}")
    
    # Check resolution
    phys_lsb = (avg_gain * ratio) * (cal.counts_to_volts / np.sqrt(2))
    print(f"Resolution   : {phys_lsb:8.4f} {unit}/LSB")

def main():
    cal = HardwareCalibrator(ADC_BITS, ADC_RANGE)
    lsb = cal.counts_to_volts
    
    print("\n" + "="*50)
    print(" LCI HARDWARE CALIBRATION TOOL")
    print("="*50)
    print(f"ADC Resolution : {ADC_BITS}-bit")
    print(f"ADC FS Range   : \u00B1{ADC_RANGE/2} V")
    print(f"LSB            : {lsb*1000:8f} mV")
    
    if ENABLE_VOLTAGE:
        run_calibration(cal, "VOLTAGE", V_MEASURED_RMS, V_COUNTS_PEAK, PT_RATIO, V_IS_PRIMARY, "V")

    if ENABLE_CURRENT:
        run_calibration(cal, "CURRENT", I_MEASURED_RMS, I_COUNTS_PEAK, CT_RATIO, I_IS_PRIMARY, "A")

    print("\n" + "="*50 + "\n")

if __name__ == "__main__":
    main()
