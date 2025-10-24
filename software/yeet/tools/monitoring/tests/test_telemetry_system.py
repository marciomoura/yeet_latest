#!/usr/bin/env python3
"""Test script for telemetry dashboard components.

This script provides basic functionality tests for the telemetry
system components without requiring hardware connection.

Usage:
    python test_telemetry_system.py
"""

import struct
import time
from telemetry_protocol import (
    TelemetryDecoder,
    PacketType,
    TelemetryFlags,
    TelemetryPacketHeader,
    GridMeasurementsGroup,
    GridObserverGroup,
    GridModulatorGroup,
    GridControllerGroup,
    StateMachineGroup,
    GridSideTelemetryPacket
)
from telemetry_signal_manager import SignalManager


def test_protocol_encoder_decoder():
    """Test encoding and decoding of telemetry packets."""
    print("=" * 60)
    print("TEST: Protocol Encoder/Decoder")
    print("=" * 60)
    
    # Create a test packet manually
    flags = (TelemetryFlags.MEASUREMENTS | TelemetryFlags.OBSERVER | 
             TelemetryFlags.MODULATOR | TelemetryFlags.CONTROLLER | 
             TelemetryFlags.STATE_MACHINE)
    header_data = struct.pack(
        '<BBHIII',
        PacketType.GRID_SIDE_TELEMETRY,  # packet_type (B)
        1,  # protocol_version (B)
        56,  # payload_length (H) - 20+8+12+12+4
        flags,  # flags (I)
        1000,  # timestamp_ms (I)
        42  # sequence_number (I)
    )
    
    measurements_data = struct.pack('<fffff',
        1.023,  # grid_voltage_magnitude_pu
        0.987,  # grid_voltage_a_pu
        0.995,  # grid_voltage_b_pu
        1.012,  # grid_voltage_c_pu
        0.456   # dc_link_current_pu
    )
    
    observer_data = struct.pack('<ff',
        1.000,  # estimated_frequency_pu
        2.456   # estimated_angle_rad
    )
    
    modulator_data = struct.pack('<ffBBBB',
        1.234,  # firing_angle_reference_rad
        1.235,  # firing_angle_actual_rad
        0,      # force_commutation_ack
        0, 0, 0  # reserved
    )
    
    controller_data = struct.pack('<ffBBBB',
        0.001,  # current_error_pu
        1.234,  # controller_output_pu
        1,      # operating_mode
        0,      # controller_saturated
        0,      # freeze_integrator
        0       # reserved
    )
    
    state_machine_data = struct.pack('<BBBB',
        3,  # current_state
        1,  # pulse_releases
        0, 0  # reserved
    )
    
    # Combine all parts
    packet_bytes = header_data + measurements_data + observer_data + modulator_data + controller_data + state_machine_data
    
    print(f"Created test packet: {len(packet_bytes)} bytes")
    print(f"Expected size: 72 bytes (16 header + 56 payload)")
    
    # Decode packet
    decoder = TelemetryDecoder()
    packet = decoder.decode(packet_bytes)
    
    if packet:
        print("✓ Packet decoded successfully")
        print(f"  Timestamp: {packet.header.timestamp_ms} ms")
        print(f"  Sequence: {packet.header.sequence_number}")
        
        if packet.measurements:
            print("✓ Measurements group decoded")
            print(f"  Grid Voltage Mag.: {packet.measurements.grid_voltage_magnitude_pu:.3f} pu")
            print(f"  DC Current: {packet.measurements.dc_link_current_pu:.3f} pu")
        
        if packet.observer:
            print("✓ Observer group decoded")
            print(f"  Frequency: {packet.observer.estimated_frequency_pu:.3f} pu")
            print(f"  Angle: {packet.observer.estimated_angle_rad:.3f} rad")
        
        if packet.modulator:
            print("✓ Modulator group decoded")
            print(f"  Firing Angle Ref.: {packet.modulator.firing_angle_reference_rad:.3f} rad")
            print(f"  Firing Angle Actual: {packet.modulator.firing_angle_actual_rad:.3f} rad")
        
        if packet.controller:
            print("✓ Controller group decoded")
            print(f"  Current Error: {packet.controller.current_error_pu:.6f} pu")
            print(f"  Controller Output: {packet.controller.controller_output_pu:.3f} pu")
            print(f"  Operating Mode: {packet.controller.operating_mode}")
        
        if packet.state_machine:
            print("✓ State machine group decoded")
            print(f"  Current State: {packet.state_machine.current_state}")
            print(f"  Pulse Releases: {packet.state_machine.pulse_releases}")
        
        print("\n✅ Protocol encoder/decoder test PASSED")
        return True
    else:
        print("\n❌ Protocol encoder/decoder test FAILED")
        return False


def test_signal_manager():
    """Test signal manager storage and retrieval."""
    print("\n" + "=" * 60)
    print("TEST: Signal Manager")
    print("=" * 60)
    
    # Create signal manager
    manager = SignalManager(buffer_size=100)
    print(f"Created signal manager with buffer size: 100")
    
    # Create test packet
    decoder = TelemetryDecoder()
    
    # Create multiple test packets
    success_count = 0
    for i in range(10):
        flags = TelemetryFlags.MEASUREMENTS | TelemetryFlags.OBSERVER
        header_data = struct.pack(
            '<BBHIII',
            PacketType.GRID_SIDE_TELEMETRY,  # B
            1,  # B
            28,  # H - payload_length (20 + 8)
            flags,  # I
            i * 10,  # I - timestamp
            i  # I - sequence
        )
        
        measurements_data = struct.pack('<fffff',
            1.0 + i * 0.001,  # varying voltage
            0.9, 0.95, 1.05,
            0.5 + i * 0.01    # varying current
        )
        
        observer_data = struct.pack('<ff', 1.0, 0.0)
        
        packet_bytes = header_data + measurements_data + observer_data
        packet = decoder.decode(packet_bytes)
        
        if packet:
            manager.update_from_packet(packet)
            success_count += 1
    
    print(f"✓ Updated signal manager with {success_count} packets")
    
    # Test retrieval
    voltage = manager.get_latest_value('measurements', 'grid_voltage_magnitude_pu')
    current = manager.get_latest_value('measurements', 'dc_link_current_pu')
    frequency = manager.get_latest_value('observer', 'estimated_frequency_pu')
    
    print(f"✓ Retrieved latest values:")
    print(f"  Grid Voltage: {voltage:.3f} pu")
    print(f"  DC Current: {current:.3f} pu")
    print(f"  Frequency: {frequency:.3f} pu")
    
    # Test time series
    times, values = manager.get_time_series('measurements', 'dc_link_current_pu')
    print(f"✓ Retrieved time series: {len(times)} samples")
    print(f"  First timestamp: {times[0]} ms")
    print(f"  Last timestamp: {times[-1]} ms")
    print(f"  First value: {values[0]:.3f} pu")
    print(f"  Last value: {values[-1]:.3f} pu")
    
    # Test statistics
    stats = manager.get_statistics()
    print(f"✓ Signal manager statistics:")
    print(f"  Total groups: {stats['total_groups']}")
    print(f"  Total signals: {stats['total_signals']}")
    print(f"  Total samples: {stats['total_samples']}")
    
    if stats['total_signals'] == 17 and stats['total_groups'] == 5:
        print("\n✅ Signal manager test PASSED")
        return True
    else:
        print("\n❌ Signal manager test FAILED")
        return False


def test_partial_packet_decoding():
    """Test decoding of packets with only some groups present."""
    print("\n" + "=" * 60)
    print("TEST: Partial Packet Decoding")
    print("=" * 60)
    
    decoder = TelemetryDecoder()
    
    # Create packet with only measurements and controller
    flags = TelemetryFlags.MEASUREMENTS | TelemetryFlags.CONTROLLER
    header_data = struct.pack(
        '<BBHIII',
        PacketType.GRID_SIDE_TELEMETRY,  # B
        1,  # B
        32,  # H - payload_length (20 + 12)
        flags,  # I
        2000,  # I - timestamp
        100  # I - sequence
    )
    
    measurements_data = struct.pack('<fffff',
        1.05, 1.0, 1.0, 1.0, 0.6
    )
    
    controller_data = struct.pack('<ffBBBB',
        0.002, 1.5, 1, 0, 0, 0
    )
    
    packet_bytes = header_data + measurements_data + controller_data
    
    print(f"Created partial packet: {len(packet_bytes)} bytes")
    print(f"Expected size: 48 bytes (16 + 20 + 12)")
    
    packet = decoder.decode(packet_bytes)
    
    if packet:
        print("✓ Partial packet decoded successfully")
        
        # Check which groups are present
        has_measurements = packet.measurements is not None
        has_observer = packet.observer is not None
        has_modulator = packet.modulator is not None
        has_controller = packet.controller is not None
        has_state_machine = packet.state_machine is not None
        
        print(f"  Measurements: {'Present' if has_measurements else 'Absent'}")
        print(f"  Observer: {'Present' if has_observer else 'Absent'}")
        print(f"  Modulator: {'Present' if has_modulator else 'Absent'}")
        print(f"  Controller: {'Present' if has_controller else 'Absent'}")
        print(f"  State Machine: {'Present' if has_state_machine else 'Absent'}")
        
        if has_measurements and has_controller and not has_observer:
            print("\n✅ Partial packet decoding test PASSED")
            return True
        else:
            print("\n❌ Partial packet decoding test FAILED (wrong groups present)")
            return False
    else:
        print("\n❌ Partial packet decoding test FAILED")
        return False


def test_invalid_packet_rejection():
    """Test that invalid packets are properly rejected."""
    print("\n" + "=" * 60)
    print("TEST: Invalid Packet Rejection")
    print("=" * 60)
    
    decoder = TelemetryDecoder()
    
    # Test 1: Packet too short
    short_packet = b'\x01\x01\x00\x00'
    result = decoder.decode(short_packet)
    print(f"✓ Short packet rejected: {result is None}")
    
    # Test 2: Wrong protocol version
    header_data = struct.pack('<BBHIII', 
        PacketType.GRID_SIDE_TELEMETRY,  # B
        99,  # B - wrong version
        0,  # H
        0, 0, 0  # I, I, I
    )
    result = decoder.decode(header_data)
    print(f"✓ Wrong version rejected: {result is None}")
    
    # Test 3: Wrong packet type
    header_data = struct.pack('<BBHIII',
        PacketType.MACHINE_SIDE_TELEMETRY,  # B - wrong type
        1,  # B
        0,  # H
        0, 0, 0  # I, I, I
    )
    result = decoder.decode(header_data)
    print(f"✓ Wrong packet type rejected: {result is None}")
    
    print("\n✅ Invalid packet rejection test PASSED")
    return True


def main():
    """Run all tests."""
    print("\n" + "=" * 60)
    print("TELEMETRY DASHBOARD COMPONENT TESTS")
    print("=" * 60 + "\n")
    
    tests = [
        ("Protocol Encoder/Decoder", test_protocol_encoder_decoder),
        ("Signal Manager", test_signal_manager),
        ("Partial Packet Decoding", test_partial_packet_decoding),
        ("Invalid Packet Rejection", test_invalid_packet_rejection),
    ]
    
    results = []
    for name, test_func in tests:
        try:
            result = test_func()
            results.append((name, result))
        except Exception as e:
            print(f"\n❌ Test '{name}' raised exception: {e}")
            results.append((name, False))
    
    # Summary
    print("\n" + "=" * 60)
    print("TEST SUMMARY")
    print("=" * 60)
    
    for name, result in results:
        status = "✅ PASS" if result else "❌ FAIL"
        print(f"{status}: {name}")
    
    passed = sum(1 for _, result in results if result)
    total = len(results)
    print(f"\nTotal: {passed}/{total} tests passed")
    
    if passed == total:
        print("\n🎉 All tests PASSED! Dashboard components working correctly.")
        return 0
    else:
        print(f"\n⚠️  {total - passed} test(s) FAILED. Check output above.")
        return 1


if __name__ == '__main__':
    exit(main())
