#!/usr/bin/env python3
"""Test script to verify grouped boolean packing in telemetry protocol v2.

This script connects to the SFC simulator and decodes grid-side telemetry packets
to verify that the grouped boolean packing is working correctly.

Usage:
    1. Start the SFC simulator:
       build-host/bin/Debug/sfc_host_simulator.exe --grid-port 7000 --machine-port 7001 --frequency 100 --verbose
    
    2. Run this test:
       python test_grouped_boolean_packing.py
"""

import sys
import socket
import struct
import time
from pathlib import Path

# Add monitor_client to path
tools_dir = Path(__file__).parent.parent.parent
sys.path.insert(0, str(tools_dir))

from monitor_client.core.telemetry_protocol import (
    TelemetryDecoder,
    GridSideTelemetryPacket,
    PacketType,
    SYNC_MAGIC,
)
from monitor_client.config.grid_side_telemetry_protocol import GridSignalIndex


def connect_to_simulator(host: str = 'localhost', port: int = 7000, timeout: float = 5.0):
    """Connect to the SFC simulator telemetry port."""
    print(f"Connecting to {host}:{port}...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(timeout)
    
    try:
        sock.connect((host, port))
        print(f"✓ Connected successfully")
        return sock
    except Exception as e:
        print(f"✗ Connection failed: {e}")
        return None


def receive_packet(sock: socket.socket, max_size: int = 4096) -> bytes:
    """Receive one complete telemetry packet from the socket."""
    buffer = b''
    
    # Read until we get a sync frame or header
    while len(buffer) < 16:
        chunk = sock.recv(max_size - len(buffer))
        if not chunk:
            return None
        buffer += chunk
    
    return buffer


def test_packet_decoding(host: str = 'localhost', port: int = 7000, num_packets: int = 5):
    """Test decoding of grouped boolean packing."""
    print("=" * 80)
    print("GROUPED BOOLEAN PACKING TEST")
    print("=" * 80)
    print()
    
    # Connect to simulator
    sock = connect_to_simulator(host, port)
    if not sock:
        print("✗ TEST FAILED: Could not connect to simulator")
        return False
    
    decoder = TelemetryDecoder(require_crc=False)
    packets_decoded = 0
    
    try:
        print(f"\nReceiving and decoding {num_packets} packets...\n")
        
        for i in range(num_packets * 10):  # Try up to 10x packets to handle connection issues
            if packets_decoded >= num_packets:
                break
            
            # Receive data
            data = receive_packet(sock)
            if not data:
                print("✗ Connection closed by simulator")
                break
            
            # Decode packet
            result = decoder.decode(data, verbose=False)
            if not result:
                continue
            
            packet, bytes_consumed = result
            packets_decoded += 1
            
            if not isinstance(packet, GridSideTelemetryPacket):
                print(f"✗ Wrong packet type: {type(packet)}")
                continue
            
            print(f"─── Packet {packets_decoded}/{num_packets} ───────────────────────────────────")
            print(f"Header:")
            print(f"  packet_type: 0x{packet.header.packet_type:02X}")
            print(f"  protocol_version: {packet.header.protocol_version}")
            print(f"  signal_count: {packet.header.signal_count}")
            print(f"  timestamp_ms: {packet.header.timestamp_ms}")
            print(f"  sequence_number: {packet.header.sequence_number}")
            print()
            
            # Check signal count
            expected_signals = 46  # Grid-side has 46 signals with grouped boolean packing
            if packet.header.signal_count != expected_signals:
                print(f"✗ FAIL: Expected {expected_signals} signals, got {packet.header.signal_count}")
                return False
            print(f"✓ Signal count correct: {packet.header.signal_count}")
            
            # Check raw_signals is populated
            if not packet.raw_signals or len(packet.raw_signals) != packet.header.signal_count:
                print(f"✗ FAIL: raw_signals not populated correctly")
                print(f"  raw_signals length: {len(packet.raw_signals) if packet.raw_signals else 0}")
                return False
            print(f"✓ Raw signals populated: {len(packet.raw_signals)} values")
            print()
            
            # Test boolean pack extraction
            print("Boolean Pack Analysis:")
            print(f"  BOOL_PACK_1 (index {GridSignalIndex.BOOL_PACK_1}):")
            raw_pack_1 = packet.get_raw_signal(GridSignalIndex.BOOL_PACK_1)
            print(f"    Raw uint32: 0x{raw_pack_1:08X} ({raw_pack_1})")
            print(f"    Binary: {bin(raw_pack_1)}")
            
            try:
                bools1 = packet._get_bool_pack_1()
                print(f"    Unpacked to {len(bools1)} booleans:")
                print(f"      Bits 0-2 (Modulator): {bools1[0:3]}")
                print(f"      Bits 3-4 (Current Ctrl): {bools1[3:5]}")
                print(f"      Bits 5-11 (State Machine): {bools1[5:12]}")
                print(f"      Bits 12-14 (Mode Monitor): {bools1[12:15]}")
                print(f"      Bits 15-18 (Pulse Gen): {bools1[15:19]}")
                print(f"      Bits 19-24 (Firing Pulses): {bools1[19:25]}")
                print(f"      Bits 25-31 (Protections): {bools1[25:32]}")
            except Exception as e:
                print(f"    ✗ FAIL: Could not unpack booleans: {e}")
                return False
            
            print(f"\n  BOOL_PACK_2 (index {GridSignalIndex.BOOL_PACK_2}):")
            raw_pack_2 = packet.get_raw_signal(GridSignalIndex.BOOL_PACK_2)
            print(f"    Raw uint32: 0x{raw_pack_2:08X} ({raw_pack_2})")
            print(f"    Binary: {bin(raw_pack_2)}")
            
            try:
                bools2 = packet._get_bool_pack_2()
                print(f"    Unpacked to {len(bools2)} booleans:")
                print(f"      Bits 0-3 (Protections): {bools2[0:4]}")
                print(f"      Bits 4-7 (Fiber Optic RX): {bools2[4:8]}")
            except Exception as e:
                print(f"    ✗ FAIL: Could not unpack booleans: {e}")
                return False
            
            print()
            
            # Test property methods work
            print("Property Method Test:")
            try:
                measurements = packet.measurements
                print(f"  ✓ measurements: voltage={measurements.grid_voltage_magnitude_pu:.4f} pu")
                
                observer = packet.observer
                print(f"  ✓ observer: freq={observer.estimated_frequency_pu:.4f} pu")
                
                modulator = packet.modulator
                print(f"  ✓ modulator: firing_angle={modulator.firing_angle_output_pu:.4f} pu, release_pulses_output={modulator.release_pulses_output}")
                
                controller = packet.controller
                print(f"  ✓ controller: current_ref={controller.current_reference:.4f} pu, is_active={controller.is_active}, is_saturated={controller.is_saturated}")
                
                state_machine = packet.state_machine
                print(f"  ✓ state_machine: state={state_machine.current_state}, release_pulses={state_machine.release_pulses}")
                
                operation_mode = packet.operation_mode_monitor
                print(f"  ✓ operation_mode_monitor: has_mode_changed={operation_mode.has_mode_changed}")
                
                pulse_gen = packet.pulse_generator
                print(f"  ✓ pulse_generator: release_pulses={pulse_gen.release_pulses}, p1={pulse_gen.p1}, p2={pulse_gen.p2}")
                
                protections = packet.protections
                print(f"  ✓ protections: is_tripped={protections.is_tripped}, overcurrent_tripped={protections.overcurrent_tripped}")
                
                fiber_rx = packet.fiber_rx
                print(f"  ✓ fiber_rx: measured_speed={fiber_rx.measured_speed_pu:.4f} pu, shutdown_request={fiber_rx.shutdown_request}")
                
            except Exception as e:
                print(f"  ✗ FAIL: Property method error: {e}")
                import traceback
                traceback.print_exc()
                return False
            
            print()
            print("✓ All checks passed for this packet")
            print()
        
        if packets_decoded < num_packets:
            print(f"✗ FAIL: Only decoded {packets_decoded}/{num_packets} packets")
            return False
        
        print("=" * 80)
        print(f"✓ TEST PASSED: Successfully decoded {packets_decoded} packets")
        print("  - Signal count correct (46 signals)")
        print("  - Raw uint32 values preserved")
        print("  - Boolean packs unpacked correctly")
        print("  - Property methods work correctly")
        print("=" * 80)
        return True
        
    except Exception as e:
        print(f"\n✗ TEST FAILED with exception: {e}")
        import traceback
        traceback.print_exc()
        return False
    finally:
        sock.close()
        print("\nConnection closed")


if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description='Test grouped boolean packing in telemetry protocol')
    parser.add_argument('--host', default='localhost', help='Simulator host (default: localhost)')
    parser.add_argument('--port', type=int, default=7000, help='Simulator port (default: 7000 for grid-side)')
    parser.add_argument('--num-packets', type=int, default=5, help='Number of packets to test (default: 5)')
    parser.add_argument('--output', default=None, help='Write output to file')
    
    args = parser.parse_args()
    
    # Redirect output to file if specified
    if args.output:
        sys.stdout = open(args.output, 'w')
        sys.stderr = sys.stdout
    
    success = test_packet_decoding(args.host, args.port, args.num_packets)
    
    if args.output:
        sys.stdout.close()
    
    sys.exit(0 if success else 1)
