#!/usr/bin/env python3
"""Validate that main_window can import and use the telemetry packet."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent / "monitor_client"))

print("[*] Testing main_window imports...")

try:
    from core.telemetry_protocol import GridSideTelemetryPacket, TelemetryPacketHeader
    print("[OK] Imported GridSideTelemetryPacket")
    
    # Create a mock packet with 50 signals
    header = TelemetryPacketHeader(
        packet_type=0x01,
        protocol_version=1,
        signal_count=50,
        timestamp_ms=0,
        sequence_number=0,
        reserved=0
    )
    
    signals = [float(i) * 0.1 for i in range(50)]
    packet = GridSideTelemetryPacket(header=header, signals=signals)
    
    print("[OK] Created GridSideTelemetryPacket instance")
    
    # Verify all properties that main_window uses
    print("\n[*] Testing properties that main_window accesses:")
    
    properties_to_test = [
        'measurements',
        'observer',
        'controller',
        'modulator',
        'state_machine',
        'operation_mode_monitor',
        'pulse_generator'
    ]
    
    for prop_name in properties_to_test:
        try:
            prop = getattr(packet, prop_name)
            print(f"  [OK] packet.{prop_name}")
            
            # List the fields
            if hasattr(prop, '__dataclass_fields__'):
                fields = list(prop.__dataclass_fields__.keys())
                print(f"        Fields: {', '.join(fields[:3])}...")
        except AttributeError as e:
            print(f"  [FAIL] packet.{prop_name}: {e}")
            sys.exit(1)
    
    print("\n[OK] All properties accessible!")
    print("[OK] main_window should now work without AttributeError")
    
except Exception as e:
    print(f"\n[FAIL] Error: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)
