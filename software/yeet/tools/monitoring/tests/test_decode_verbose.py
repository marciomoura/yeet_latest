#!/usr/bin/env python3
"""Test script to verify telemetry decoding with verbose logging.

This script creates a test telemetry packet exactly matching what the
C++ simulator sends, then decodes it with verbose logging to verify
the protocol implementation is correct.

Usage:
    python test_decode_verbose.py
"""

import struct
from telemetry_protocol import TelemetryDecoder, PacketType, TelemetryFlags


def create_test_packet():
    """Create a test packet matching C++ simulator output."""
    
    # Header (16 bytes)
    packet_type = int(PacketType.GRID_SIDE_TELEMETRY)
    protocol_version = 2
    signal_count = 14  # 5 + 2 + 3 + 3 + 1
    timestamp_ms = 1000
    sequence_number = 100
    reserved = 0
    
    header = struct.pack('<BBHIII',
                        packet_type,
                        protocol_version,
                        signal_count,
                        timestamp_ms,
                        sequence_number,
                        reserved)
    
    # Measurements (20 bytes)
    measurements = struct.pack('<fffff',
                              1.0,  # grid_voltage_magnitude_pu
                              1.0,  # grid_voltage_a_pu
                              1.0,  # grid_voltage_b_pu
                              1.0,  # grid_voltage_c_pu
                              0.0)  # dc_link_current_pu
    
    # Observer (8 bytes)
    observer = struct.pack('<ff',
                          1.0,  # estimated_frequency_pu
                          0.0)  # estimated_angle_rad
    
    # Modulator (12 bytes)
    modulator = struct.pack('<ffBBBB',
                           0.0,  # firing_angle_reference_rad
                           0.0,  # firing_angle_actual_rad
                           0,    # force_commutation_ack
                           0,    # reserved_1
                           0,    # reserved_2
                           0)    # reserved_3
    
    # Controller (12 bytes)
    controller = struct.pack('<ffBBBB',
                            0.0,  # current_error_pu
                            0.0,  # controller_output_pu
                            0,    # operating_mode
                            0,    # controller_saturated
                            0,    # freeze_integrator
                            0)    # reserved
    
    # State machine (4 bytes)
    state_machine = struct.pack('<BBBB',
                               0,  # current_state
                               0,  # pulse_releases
                               0,  # reserved_1
                               0)  # reserved_2
    
    return header + measurements + observer + modulator + controller + state_machine


def main():
    """Test telemetry decoding with verbose output."""
    
    print("="*70)
    print("Telemetry Protocol Decode Test (Verbose Mode)")
    print("="*70)
    
    # Create test packet
    test_packet = create_test_packet()
    
    print(f"\nTest packet created: {len(test_packet)} bytes")
    print(f"Hex dump (first 32 bytes): {test_packet[:32].hex()}")
    print(f"Hex dump (full): {test_packet.hex()}")
    
    # Decode with verbose logging
    print("\n" + "="*70)
    print("Starting decode with verbose logging:")
    print("="*70 + "\n")
    
    decoder = TelemetryDecoder()
    packet = decoder.decode(test_packet, verbose=True)
    
    print("\n" + "="*70)
    if packet:
        print("✅ DECODE SUCCESS!")
        print("="*70)
        print("\nDecoded packet contents:")
        print(f"  Sequence: {packet.header.sequence_number}")
        print(f"  Timestamp: {packet.header.timestamp_ms} ms")
        
        if packet.measurements:
            print(f"\n  Measurements:")
            print(f"    Grid Voltage Magnitude: {packet.measurements.grid_voltage_magnitude_pu:.3f} pu")
            print(f"    Grid Voltage A: {packet.measurements.grid_voltage_a_pu:.3f} pu")
            print(f"    Grid Voltage B: {packet.measurements.grid_voltage_b_pu:.3f} pu")
            print(f"    Grid Voltage C: {packet.measurements.grid_voltage_c_pu:.3f} pu")
            print(f"    DC Link Current: {packet.measurements.dc_link_current_pu:.3f} pu")
        
        if packet.observer:
            print(f"\n  Observer:")
            print(f"    Frequency: {packet.observer.estimated_frequency_pu:.3f} pu")
            print(f"    Angle: {packet.observer.estimated_angle_rad:.3f} rad")
        
        if packet.controller:
            print(f"\n  Controller:")
            print(f"    Error: {packet.controller.current_error_pu:.3f} pu")
            print(f"    Output: {packet.controller.controller_output_pu:.3f} pu")
            print(f"    Mode: {packet.controller.operating_mode}")
        
        if packet.state_machine:
            print(f"\n  State Machine:")
            print(f"    State: {packet.state_machine.current_state}")
            print(f"    Pulses: {packet.state_machine.pulse_releases}")
    else:
        print("❌ DECODE FAILED!")
        print("="*70)
        print("\nCheck the verbose output above for details on why decode failed.")
    
    print("\n" + "="*70)


if __name__ == '__main__':
    main()
