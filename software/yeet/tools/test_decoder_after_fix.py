#!/usr/bin/env python3
"""Test the fixed telemetry decoder with real C++ output."""

import sys
import struct
import os
from pathlib import Path

# Force UTF-8 output on Windows
if sys.platform == 'win32':
    sys.stdout.reconfigure(encoding='utf-8', errors='ignore')

# Add the monitor_client to path
sys.path.insert(0, str(Path(__file__).parent))

from monitor_client.core.telemetry_protocol import (
    TelemetryDecoder, GridSideTelemetryPacket, SignalIndex, PacketType
)


def test_decoder_with_cpp_header():
    """Test with the actual header bytes from C++ debug output."""
    print("\n" + "="*70)
    print("TEST 1: Decoder with C++ header + mock signal data")
    print("="*70)
    
    # From C++ debug output: [SEND] First 32 bytes:
    # 01 01 32 00 00 00 00 00 00 00 00 00 00 00 00 00
    # a5 bf b4 3f 86 5b 9c bf 00 00 00 00 86 5b 9c 3f
    
    # This is: packet_type=0x01, version=0x01, signal_count=0x0032 (50), rest zeros for timestamp/seq/res
    # Then 50 float signals (4 bytes each) = 200 bytes
    
    cpp_debug_hex = "01 01 32 00 00 00 00 00 00 00 00 00 00 00 00 00 a5 bf b4 3f 86 5b 9c bf 00 00 00 00 86 5b 9c 3f"
    cpp_debug_bytes = bytes.fromhex(cpp_debug_hex.replace(" ", ""))
    
    print(f"Header bytes (16 bytes): {cpp_debug_bytes[:16].hex()}")
    print(f"  Packet type: 0x{cpp_debug_bytes[0]:02x} (expected 0x01)")
    print(f"  Protocol version: 0x{cpp_debug_bytes[1]:02x} (expected 0x01)")
    print(f"  Signal count bytes: {cpp_debug_bytes[2:4].hex()} -> {struct.unpack('<H', cpp_debug_bytes[2:4])[0]} (expected 50)")
    
    # Create full 216-byte packet (16 header + 50*4 signals)
    # The hex string cpp_debug_bytes is 32 bytes total: 16 header + 16 (4 signals)
    # We need to add 46 more signals (46*4 = 184 bytes) to get to 216 total
    full_packet = cpp_debug_bytes[:32]  # Start with what we have
    
    # Add remaining signal data (46 more signals to make 50 total)
    # Fill with plausible float values
    for i in range(46):
        full_packet += struct.pack('<f', 0.5 + i * 0.01)
    
    print(f"\nFull packet size: {len(full_packet)} bytes (expected 216)")
    print(f"  Have: 16 (header) + 4 (initial signals) + {46*4} (added signals) = {len(full_packet)} bytes")
    
    # Test decoder
    decoder = TelemetryDecoder()
    result = decoder.decode(full_packet, verbose=True)
    
    if result:
        packet, bytes_consumed = result
        print(f"\n[OK] Decode SUCCESS!")
        print(f"  Packet type: 0x{packet.header.packet_type:02x}")
        print(f"  Protocol version: {packet.header.protocol_version}")
        print(f"  Signal count: {packet.header.signal_count}")
        print(f"  Timestamp: {packet.header.timestamp_ms} ms")
        print(f"  Sequence: {packet.header.sequence_number}")
        print(f"  Signals decoded: {len(packet.signals)}")
        print(f"  Bytes consumed: {bytes_consumed}")
        print(f"  First 5 signals: {packet.signals[:5]}")
        # Check that we decoded the right number of signals
        if len(packet.signals) == 50 and len(packet.signals) == packet.header.signal_count:
            print(f"  [OK] Signal count correct!")
            return True
        else:
            print(f"  [FAIL] Signal count mismatch!")
            return False
    else:
        print(f"\n[FAIL] Decode FAILED!")
        return False


def test_decoder_valid_packets():
    """Test decoder with several valid packets."""
    print("\n" + "="*70)
    print("TEST 2: Create and decode valid telemetry packets")
    print("="*70)
    
    decoder = TelemetryDecoder()
    
    # Create a minimal valid packet (header + 3 signals for quick test)
    packet_type = PacketType.GRID_SIDE_TELEMETRY
    version = 1
    signal_count = 3
    timestamp = 12345
    sequence = 1
    reserved = 0
    
    # Pack header
    header_bytes = struct.pack('<BBHIII', packet_type, version, signal_count, timestamp, sequence, reserved)
    
    # Add 3 signal values
    signals = [1.5, -2.3, 0.0]
    signal_bytes = b''.join(struct.pack('<f', s) for s in signals)
    
    full_data = header_bytes + signal_bytes
    
    print(f"Created packet with {signal_count} signals")
    print(f"  Header bytes: {header_bytes.hex()}")
    print(f"  Signal bytes: {signal_bytes.hex()}")
    print(f"  Total size: {len(full_data)} bytes")
    
    result = decoder.decode(full_data, verbose=True)
    
    if result:
        packet, bytes_consumed = result
        print(f"\n[OK] Decode SUCCESS!")
        print(f"  Decoded signals: {packet.signals}")
        print(f"  Expected: {signals}")
        # Use approximate comparison for floating point values
        import math
        all_match = all(
            abs(decoded - expected) < 1e-5 or 
            (decoded == 0 and expected == 0)
            for decoded, expected in zip(packet.signals, signals)
        )
        if all_match:
            print(f"  [OK] Signal values match (within FP tolerance)!")
            return True
        else:
            print(f"  [FAIL] Signal values DO NOT match!")
            return False
    else:
        print(f"\n[FAIL] Decode FAILED!")
        return False


def test_decoder_50_signals():
    """Test decoder with full 50-signal packet."""
    print("\n" + "="*70)
    print("TEST 3: Full 50-signal telemetry packet")
    print("="*70)
    
    decoder = TelemetryDecoder()
    
    # Create packet with all 50 signals
    packet_type = PacketType.GRID_SIDE_TELEMETRY
    version = 1
    signal_count = 50
    timestamp = 67890
    sequence = 5
    reserved = 0
    
    header_bytes = struct.pack('<BBHIII', packet_type, version, signal_count, timestamp, sequence, reserved)
    
    # Create 50 plausible signal values
    signals = [float(i) * 0.1 for i in range(50)]
    signal_bytes = b''.join(struct.pack('<f', s) for s in signals)
    
    full_data = header_bytes + signal_bytes
    
    print(f"Created packet with {signal_count} signals")
    print(f"  Total size: {len(full_data)} bytes (expected 216)")
    print(f"  First 5 signal values: {signals[:5]}")
    print(f"  Last 5 signal values: {signals[-5:]}")
    
    result = decoder.decode(full_data, verbose=False)
    
    if result:
        packet, bytes_consumed = result
        print(f"\n[OK] Decode SUCCESS!")
        print(f"  Signals decoded: {len(packet.signals)}")
        print(f"  Bytes consumed: {bytes_consumed}")
        
        # Verify first and last signals using approximate comparison
        import math
        first_match = abs(packet.signals[0] - signals[0]) < 1e-5
        last_match = abs(packet.signals[-1] - signals[-1]) < 1e-5
        
        if first_match and last_match and len(packet.signals) == 50:
            print(f"  [OK] First and last signals match (within FP tolerance)!")
            return True
        else:
            print(f"  [FAIL] Signal mismatch!")
            print(f"    First: {packet.signals[0]} vs {signals[0]}")
            print(f"    Last: {packet.signals[-1]} vs {signals[-1]}")
            return False
    else:
        print(f"\n[FAIL] Decode FAILED!")
        return False


def test_error_cases():
    """Test error handling."""
    print("\n" + "="*70)
    print("TEST 4: Error cases and edge conditions")
    print("="*70)
    
    decoder = TelemetryDecoder()
    all_pass = True
    
    # Test 1: Empty data
    print("\n[TEST 4.1] Empty data")
    result = decoder.decode(b'', verbose=False)
    if result is None:
        print("  [OK] Correctly rejected empty data")
    else:
        print("  [FAIL] Should have rejected empty data")
        all_pass = False
    
    # Test 2: Incomplete header
    print("\n[TEST 4.2] Incomplete header")
    result = decoder.decode(b'\x01\x01\x32', verbose=False)
    if result is None:
        print("  [OK] Correctly rejected incomplete header")
    else:
        print("  [FAIL] Should have rejected incomplete header")
        all_pass = False
    
    # Test 3: Wrong packet type
    print("\n[TEST 4.3] Wrong packet type")
    bad_packet = struct.pack('<BBHIII', 0x99, 1, 50, 0, 0, 0)
    bad_packet += b'\x00' * 200
    result = decoder.decode(bad_packet, verbose=False)
    if result is None:
        print("  [OK] Correctly rejected wrong packet type")
    else:
        print("  [FAIL] Should have rejected wrong packet type")
        all_pass = False
    
    # Test 4: Insufficient signal data
    print("\n[TEST 4.4] Insufficient signal data")
    bad_packet = struct.pack('<BBHIII', PacketType.GRID_SIDE_TELEMETRY, 1, 50, 0, 0, 0)
    bad_packet += b'\x00' * 100  # Only 100 bytes instead of 200
    result = decoder.decode(bad_packet, verbose=False)
    if result is None:
        print("  [OK] Correctly rejected insufficient data")
    else:
        print("  [FAIL] Should have rejected insufficient data")
        all_pass = False
    
    return all_pass


if __name__ == '__main__':
    print("\n" + "="*70)
    print("TELEMETRY DECODER TEST SUITE - AFTER FIX")
    print("="*70)
    
    results = []
    
    try:
        results.append(("C++ Header Test", test_decoder_with_cpp_header()))
        results.append(("Valid Packets", test_decoder_valid_packets()))
        results.append(("50-Signal Packet", test_decoder_50_signals()))
        results.append(("Error Cases", test_error_cases()))
    except Exception as e:
        print(f"\n[FATAL] Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
    
    # Summary
    print("\n" + "="*70)
    print("TEST SUMMARY")
    print("="*70)
    for test_name, passed in results:
        status = "[OK]" if passed else "[FAIL]"
        print(f"{status}: {test_name}")
    
    all_passed = all(passed for _, passed in results)
    print("\n" + "="*70)
    if all_passed:
        print("[OK] ALL TESTS PASSED! Decoder is working correctly.")
        print("="*70)
        sys.exit(0)
    else:
        print("[FAIL] SOME TESTS FAILED!")
        print("="*70)
        sys.exit(1)
