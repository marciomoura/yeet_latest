#!/usr/bin/env python3
"""Test state machine signal extraction."""

import sys
import struct
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent / "monitor_client"))

from core.telemetry_protocol import (
    TelemetryDecoder, SignalIndex, PacketType
)


def test_state_machine_signals():
    """Test that state machine signals are extracted correctly."""
    print("\n" + "="*70)
    print("TEST: State Machine Signal Extraction")
    print("="*70)
    
    decoder = TelemetryDecoder()
    
    # Create a packet with test values
    packet_type = PacketType.GRID_SIDE_TELEMETRY
    version = 1
    signal_count = 50
    timestamp = 12345
    sequence = 1
    reserved = 0
    
    header_bytes = struct.pack('<BBHIII', packet_type, version, signal_count, timestamp, sequence, reserved)
    
    # Create 50 signals with specific test values for state machine
    signals = [0.0] * 50
    
    # Set state machine signals (indices 31-38)
    signals[SignalIndex.STATE_MACHINE_STATE] = 3.0  # State = OPERATION_ON
    signals[SignalIndex.PULSE_RELEASES] = 1.0  # True
    signals[SignalIndex.AUXILIARIES_ON] = 1.0  # True
    signals[SignalIndex.BREAKERS_ON] = 0.0  # False
    signals[SignalIndex.START_COMMAND] = 0.0  # False
    signals[SignalIndex.STOP_COMMAND] = 0.0  # False
    signals[SignalIndex.CURRENT_EXTINGUISHED] = 1.0  # True
    signals[SignalIndex.SHUTDOWN_REQUESTED] = 0.0  # False
    
    signal_bytes = b''.join(struct.pack('<f', s) for s in signals)
    full_data = header_bytes + signal_bytes
    
    print(f"\nCreated test packet with state machine signals")
    print(f"Test values:")
    print(f"  STATE_MACHINE_STATE: 3.0 → current_state = 3")
    print(f"  PULSE_RELEASES: 1.0 → True")
    print(f"  AUXILIARIES_ON: 1.0 → True")
    print(f"  BREAKERS_ON: 0.0 → False")
    print(f"  START_COMMAND: 0.0 → False")
    print(f"  STOP_COMMAND: 0.0 → False")
    print(f"  CURRENT_EXTINGUISHED: 1.0 → True")
    print(f"  SHUTDOWN_REQUESTED: 0.0 → False")
    
    # Decode packet
    result = decoder.decode(full_data, verbose=False)
    
    if not result:
        print("\n[FAIL] Decoder failed!")
        return False
    
    packet, _ = result
    
    print(f"\n[OK] Packet decoded successfully")
    
    # Extract state machine data
    sm = packet.state_machine
    
    print(f"\n[*] Extracted state machine data:")
    print(f"  current_state: {sm.current_state} (expected: 3)")
    print(f"  pulse_releases: {sm.pulse_releases} (expected: True)")
    print(f"  auxiliaries_are_on: {sm.auxiliaries_are_on} (expected: True)")
    print(f"  breakers_are_on: {sm.breakers_are_on} (expected: False)")
    print(f"  start_command: {sm.start_command} (expected: False)")
    print(f"  stop_command: {sm.stop_command} (expected: False)")
    print(f"  is_current_extinguished: {sm.is_current_extinguished} (expected: True)")
    print(f"  is_shutdown_requested: {sm.is_shutdown_requested} (expected: False)")
    
    # Verify values
    print(f"\n[*] Verifying extracted values:")
    
    checks = [
        ("current_state", sm.current_state, 3, "int"),
        ("pulse_releases", sm.pulse_releases, True, "bool"),
        ("auxiliaries_are_on", sm.auxiliaries_are_on, True, "bool"),
        ("breakers_are_on", sm.breakers_are_on, False, "bool"),
        ("start_command", sm.start_command, False, "bool"),
        ("stop_command", sm.stop_command, False, "bool"),
        ("is_current_extinguished", sm.is_current_extinguished, True, "bool"),
        ("is_shutdown_requested", sm.is_shutdown_requested, False, "bool"),
    ]
    
    all_pass = True
    for field_name, actual, expected, field_type in checks:
        if actual == expected:
            print(f"  [OK] {field_name}: {actual} == {expected}")
        else:
            print(f"  [FAIL] {field_name}: {actual} != {expected}")
            all_pass = False
    
    if all_pass:
        print(f"\n[OK] All state machine signals extracted correctly!")
        return True
    else:
        print(f"\n[FAIL] Some signals don't match!")
        return False


if __name__ == '__main__':
    print("\n" + "="*70)
    print("STATE MACHINE SIGNAL EXTRACTION TEST")
    print("="*70)
    
    try:
        passed = test_state_machine_signals()
    except Exception as e:
        print(f"\n[FATAL] Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
    
    print("\n" + "="*70)
    if passed:
        print("[OK] TEST PASSED! State machine signals working correctly.")
        print("="*70)
        sys.exit(0)
    else:
        print("[FAIL] TEST FAILED!")
        print("="*70)
        sys.exit(1)
