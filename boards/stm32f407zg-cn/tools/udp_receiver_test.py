#!/usr/bin/env python3
"""
Simple UDP receiver to test W5200 + TLV protocol integration
Receives UDP packets on port 7000 and decodes TLV data
"""

import socket
import struct
import sys
from datetime import datetime

# TLV Tag definitions (matching the embedded code)
TLV_TAG_SYSTEM_TIME = 0x00
TLV_TAG_MEASUREMENT_0 = 0x10

def decode_tlv_packet(data):
    """
    Decode TLV packet and return list of (tag, value) tuples
    """
    entries = []
    offset = 0
    
    while offset < len(data):
        if offset + 3 > len(data):
            print(f"Warning: Incomplete TLV header at offset {offset}")
            break
            
        # Parse TLV header: [Tag (1 byte)] [Length (2 bytes, big-endian)]
        tag = data[offset]
        length = struct.unpack('>H', data[offset+1:offset+3])[0]
        offset += 3
        
        if offset + length > len(data):
            print(f"Warning: Incomplete TLV value at offset {offset}, expected {length} bytes")
            break
        
        # Extract value bytes
        value_bytes = data[offset:offset+length]
        offset += length
        
        # Decode based on tag
        if tag == TLV_TAG_SYSTEM_TIME:
            value = struct.unpack('>I', value_bytes)[0]  # uint32_t
            entries.append((tag, 'SYSTEM_TIME', value, f'{value} ms'))
        elif tag == TLV_TAG_MEASUREMENT_0:
            value = struct.unpack('>I', value_bytes)[0]  # uint32_t
            entries.append((tag, 'MEASUREMENT_0', value, f'0x{value:08X}'))
        else:
            # Unknown tag - display as hex
            entries.append((tag, f'UNKNOWN_0x{tag:02X}', value_bytes, value_bytes.hex()))
    
    return entries

def main():
    # Configuration
    UDP_IP = "0.0.0.0"  # Listen on all interfaces
    UDP_PORT = 7000
    
    print("=" * 70)
    print("W5200 UDP TLV Packet Receiver Test")
    print("=" * 70)
    print(f"Listening on {UDP_IP}:{UDP_PORT}")
    print(f"Expected STM32 IP: 192.168.10.31")
    print(f"Make sure to configure your PC's IP to 192.168.10.100")
    print("Press Ctrl+C to stop")
    print("=" * 70)
    print()
    
    # Create UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((UDP_IP, UDP_PORT))
    
    packet_count = 0
    
    try:
        while True:
            # Receive UDP packet
            data, addr = sock.recvfrom(2048)
            packet_count += 1
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            
            print(f"[{timestamp}] Packet #{packet_count} from {addr[0]}:{addr[1]} ({len(data)} bytes)")
            
            # Decode TLV entries
            entries = decode_tlv_packet(data)
            
            if not entries:
                print("  ⚠️  No valid TLV entries found")
                print(f"  Raw data: {data.hex()}")
            else:
                for tag, name, value, formatted in entries:
                    print(f"  Tag 0x{tag:02X} ({name}): {formatted}")
                
                # Check for expected test value
                for tag, name, value, formatted in entries:
                    if tag == TLV_TAG_MEASUREMENT_0:
                        if value == 0xDEADBEEF:
                            print("  ✅ Test value 0xDEADBEEF received correctly!")
                        else:
                            print(f"  ⚠️  Expected 0xDEADBEEF but got 0x{value:08X}")
            
            print()
            
    except KeyboardInterrupt:
        print("\n" + "=" * 70)
        print(f"Stopped. Received {packet_count} packets total.")
        print("=" * 70)
        sock.close()
        sys.exit(0)

if __name__ == "__main__":
    main()
