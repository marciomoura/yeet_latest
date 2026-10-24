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
V_COUNTS_PEAK = [18673, 18443]   # Software Peak Counts
V_IS_PRIMARY = False

# --- CURRENT CALIBRATION ---
ENABLE_CURRENT = True
CT_RATIO = 1000.0
I_MEASURED_RMS = [1.0, 1.0]    # Primary Side RMS (Multimeter)
I_COUNTS_PEAK = [11750, 11775] # Software Peak Counts
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
    secondary_vals = []
    card_vals_rms = []

    for m, c in zip(meas_rms, counts_peak):
        g = cal.calculate_transducer_gain(m, c, ratio, is_primary)
        gains.append(g)
        
        target_secondary_rms = m / ratio if is_primary else m
        secondary_vals.append(target_secondary_rms)
        card_vals_rms.append(c * cal.counts_to_volts / np.sqrt(2))
        
    avg_gain = np.mean(gains)
    
    # Perform linear regression to find gain and offset
    if len(gains) > 1:
        # y = m*x + b -> Card_RMS = Gain * Secondary_RMS + Offset
        slope, intercept = np.polyfit(secondary_vals, card_vals_rms, 1)
        r_squared = np.corrcoef(secondary_vals, card_vals_rms)[0, 1]**2
    else:
        slope, intercept, r_squared = avg_gain, 0.0, 1.0

    ref_side = "Primary" if is_primary else "Secondary"
    
    print(f"\n[{name}] Results (Reference: {ref_side}):")
    print("-" * 65)
    print(f"{'Point':<8} | {'Meas ('+unit+')':<10} | {'Counts':<8} | {'Gain (V/'+unit+')':<12} | {'Error %':<8}")
    print("-" * 65)
    for i, (m, c, g) in enumerate(zip(meas_rms, counts_peak, gains)):
        error = (g - avg_gain) / avg_gain * 100
        print(f" {i+1:<7} | {m:10.2f} | {int(c):8} | {g:12.6f} | {error:+8.2f}%")
    
    print("-" * 65)
    print(f"Arithmetic Average Gain : {avg_gain:12.6f} V/{'V' if unit == 'V' else 'A'}")
    print(f"Regression Gain (Slope) : {slope:12.6f} V/{'V' if unit == 'V' else 'A'}")
    print(f"Regression Offset       : {intercept:12.6f} V (Referred to Card)")
    print(f"R-Squared (Linearity)   : {r_squared:12.6f}")
    
    # Check resolution
    phys_lsb = (avg_gain * ratio) * (cal.counts_to_volts / np.sqrt(2))
    print(f"Effective Resolution    : {phys_lsb:12.4f} {unit}/LSB")

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
