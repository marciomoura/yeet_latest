#!/usr/bin/env python3
"""Direct test of telemetry decoder with verbose output."""

from monitor_client.core.telemetry_protocol import TelemetryDecoder, PacketType

# Create decoder
decoder = TelemetryDecoder()

# Test 1: Valid packet header + 50 zero signals (216 bytes total)
print("=" * 80)
print("TEST 1: Valid telemetry packet (50 signals, all zeros)")
print("=" * 80)

# Build valid packet: header (16 bytes) + 50 signals * 4 bytes
header = bytes([
    0x01,        # packet_type = 0x01
    0x01,        # protocol_version = 1
    0x32, 0x00,  # signal_count = 50 (little-endian)
    0x00, 0x00, 0x00, 0x00,  # timestamp_ms = 0
    0x00, 0x00, 0x00, 0x00,  # sequence_number = 0
    0x00, 0x00, 0x00, 0x00,  # reserved = 0
])
signals = bytes([0x00] * 200)  # 50 signals * 4 bytes
valid_packet = header + signals

print(f"Packet length: {len(valid_packet)} bytes")
print(f"Header (hex): {header.hex()}")

result = decoder.decode(valid_packet, verbose=True)
if result:
    packet, consumed = result
    print(f"✓ Successfully decoded!")
    print(f"  - Signals: {len(packet.signals)}")
    print(f"  - Bytes consumed: {consumed}")
    print(f"  - Packet type: 0x{packet.header.packet_type:02x}")
    print(f"  - Signal count: {packet.header.signal_count}")
else:
    print("✗ Decode FAILED")

# Test 2: Actual packet from simulator output
print("\n" + "=" * 80)
print("TEST 2: Real packet from simulator")
print("=" * 80)

# From simulator output: [SEND] Sending 216 bytes. First 32: 01 01 32 00 00 00 00 00 00 00 00 00 00 00 00 00 a5 bf b4 3f 86 5b 9c bf 00 00 00 00 86 5b 9c 3f
real_packet_hex = (
    "01 01 32 00 00 00 00 00 00 00 00 00 00 00 00 00 "  # Header
    "a5 bf b4 3f 86 5b 9c bf 00 00 00 00 86 5b 9c 3f "  # First few signals
    + "00 " * (200 - 32)  # Pad with zeros for rest of signals
)
real_packet = bytes.fromhex(real_packet_hex.replace(" ", ""))

print(f"Packet length: {len(real_packet)} bytes")
print(f"First 32 bytes (hex): {real_packet[:32].hex()}")

result = decoder.decode(real_packet, verbose=True)
if result:
    packet, consumed = result
    print(f"✓ Successfully decoded!")
    print(f"  - Signals: {len(packet.signals)}")
    print(f"  - Bytes consumed: {consumed}")
    print(f"  - First signal: {packet.signals[0]:.6f}")
    print(f"  - Second signal: {packet.signals[1]:.6f}")
else:
    print("✗ Decode FAILED")

# Test 3: Incomplete packet (should fail gracefully)
print("\n" + "=" * 80)
print("TEST 3: Incomplete packet (only header, no signals)")
print("=" * 80)

incomplete_packet = header  # Just the header, no signals
print(f"Packet length: {len(incomplete_packet)} bytes")

result = decoder.decode(incomplete_packet, verbose=True)
if result:
    print(f"✗ Unexpectedly decoded an incomplete packet!")
else:
    print(f"✓ Correctly rejected incomplete packet")

# Test 4: Corrupted packet_type
print("\n" +"=" * 80)
print("TEST 4: Invalid packet_type (0x86)")
print("=" * 80)

bad_packet_type = bytes([
    0x86,        # packet_type = 0x86 (INVALID)
    0x01,        # protocol_version = 1
    0x32, 0x00,  # signal_count = 50
    0x00, 0x00, 0x00, 0x00,  # timestamp_ms = 0
    0x00, 0x00, 0x00, 0x00,  # sequence_number = 0
    0x00, 0x00, 0x00, 0x00,  # reserved = 0
]) + bytes([0x00] * 200)

print(f"Packet length: {len(bad_packet_type)} bytes")
print(f"First byte (packet_type): 0x{bad_packet_type[0]:02x}")

result = decoder.decode(bad_packet_type, verbose=True)
if result:
    print(f"✗ Unexpectedly decoded packet with invalid type!")
else:
    print(f"✓ Correctly rejected packet with invalid type")

print("\n" + "=" * 80)
print("DIAGNOSTIC COMPLETE")
print("=" * 80)
