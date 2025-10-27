#!/usr/bin/env python3
"""Test the structured properties of GridSideTelemetryPacket."""

import sys
import struct
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from monitor_client.core.telemetry_protocol import (
    TelemetryDecoder, GridSideTelemetryPacket, SignalIndex, PacketType
)


def test_structured_properties():
    """Test that structured properties work correctly."""
    print("\n" + "="*70)
    print("TEST: Structured Properties Access")
    print("="*70)
    
    decoder = TelemetryDecoder()
    
    # Create a full 216-byte packet with 50 signals
    packet_type = PacketType.GRID_SIDE_TELEMETRY
    version = 1
    signal_count = 50
    timestamp = 12345
    sequence = 1
    reserved = 0
    
    header_bytes = struct.pack('<BBHIII', packet_type, version, signal_count, timestamp, sequence, reserved)
    
    # Create 50 signal values with identifiable patterns
    signals = [float(i) * 0.1 for i in range(50)]
    signal_bytes = b''.join(struct.pack('<f', s) for s in signals)
    
    full_data = header_bytes + signal_bytes
    
    print(f"Created packet with {signal_count} signals")
    
    result = decoder.decode(full_data, verbose=False)
    
    if not result:
        print("[FAIL] Decoder failed!")
        return False
    
    packet, _ = result
    
    print(f"\n[OK] Packet decoded successfully")
    print(f"\nTesting structured properties:")
    
    # Test measurements property
    print(f"\n1. Measurements:")
    try:
        m = packet.measurements
        print(f"   [OK] measurements property accessible")
        print(f"   - grid_voltage_magnitude_pu: {m.grid_voltage_magnitude_pu}")
        print(f"   - dc_link_current_pu: {m.dc_link_current_pu}")
        print(f"   - is_commuting: {m.is_commuting}")
    except Exception as e:
        print(f"   [FAIL] Error accessing measurements: {e}")
        return False
    
    # Test observer property
    print(f"\n2. Observer:")
    try:
        o = packet.observer
        print(f"   [OK] observer property accessible")
        print(f"   - estimated_voltage_magnitude_pu: {o.estimated_voltage_magnitude_pu}")
        print(f"   - estimated_frequency_pu: {o.estimated_frequency_pu}")
    except Exception as e:
        print(f"   [FAIL] Error accessing observer: {e}")
        return False
    
    # Test modulator property
    print(f"\n3. Modulator:")
    try:
        mod = packet.modulator
        print(f"   [OK] modulator property accessible")
        print(f"   - firing_angle_input_pu: {mod.firing_angle_input_pu}")
        print(f"   - is_enabled: {mod.is_enabled}")
    except Exception as e:
        print(f"   [FAIL] Error accessing modulator: {e}")
        return False
    
    # Test controller property
    print(f"\n4. Controller:")
    try:
        c = packet.controller
        print(f"   [OK] controller property accessible")
        print(f"   - dc_link_current_reference: {c.dc_link_current_reference}")
        print(f"   - current_error_pu: {c.current_error_pu}")
        print(f"   - controller_saturated: {c.controller_saturated}")
    except Exception as e:
        print(f"   [FAIL] Error accessing controller: {e}")
        return False
    
    # Test state_machine property
    print(f"\n5. State Machine:")
    try:
        sm = packet.state_machine
        print(f"   [OK] state_machine property accessible")
        print(f"   - current_state: {sm.current_state}")
        print(f"   - auxiliaries_are_on: {sm.auxiliaries_are_on}")
        print(f"   - is_shutdown_requested: {sm.is_shutdown_requested}")
    except Exception as e:
        print(f"   [FAIL] Error accessing state_machine: {e}")
        return False
    
    # Test operation_mode_monitor property
    print(f"\n6. Operation Mode Monitor:")
    try:
        omm = packet.operation_mode_monitor
        print(f"   [OK] operation_mode_monitor property accessible")
        print(f"   - mode: {omm.mode}")
        print(f"   - discontinuous_mode: {omm.discontinuous_mode}")
    except Exception as e:
        print(f"   [FAIL] Error accessing operation_mode_monitor: {e}")
        return False
    
    # Test pulse_generator property
    print(f"\n7. Pulse Generator:")
    try:
        pg = packet.pulse_generator
        print(f"   [OK] pulse_generator property accessible")
        print(f"   - firing_pulse_1: {pg.firing_pulse_1}")
        print(f"   - is_active: {pg.is_active}")
    except Exception as e:
        print(f"   [FAIL] Error accessing pulse_generator: {e}")
        return False
    
    print(f"\n[OK] All structured properties work correctly!")
    return True


if __name__ == '__main__':
    print("\n" + "="*70)
    print("TELEMETRY PACKET STRUCTURED PROPERTIES TEST")
    print("="*70)
    
    try:
        passed = test_structured_properties()
    except Exception as e:
        print(f"\n[FATAL] Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
    
    print("\n" + "="*70)
    if passed:
        print("[OK] TEST PASSED! Structured properties are working.")
        print("="*70)
        sys.exit(0)
    else:
        print("[FAIL] TEST FAILED!")
        print("="*70)
        sys.exit(1)
