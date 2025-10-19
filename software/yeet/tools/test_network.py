#!/usr/bin/env python3
"""
Quick Test Script for MCU Network Communication
Tests connection, sends commands, and prints received data to console.
Use this for quick validation before running the full dashboard.
"""

import socket
import struct
import time
import sys


def test_connection(host='192.168.2.100', port=7000):
    """Test basic TCP connection to MCU"""
    print(f"Testing connection to {host}:{port}...")
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5.0)
        sock.connect((host, port))
        print("✓ Connection successful!")
        return sock
    except Exception as e:
        print(f"✗ Connection failed: {e}")
        return None


def send_command(sock, command):
    """Send text command to MCU"""
    try:
        if not command.endswith('\n'):
            command += '\n'
        sock.sendall(command.encode('ascii'))
        print(f"→ Sent: {command.strip()}")
        return True
    except Exception as e:
        print(f"✗ Send failed: {e}")
        return False


def receive_packets(sock, duration=5.0):
    """Receive and parse packets for specified duration"""
    print(f"\nReceiving data for {duration} seconds...")
    sock.settimeout(0.1)
    
    packet_buffer = b''
    packets_received = 0
    start_time = time.time()
    
    while time.time() - start_time < duration:
        try:
            chunk = sock.recv(4096)
            if not chunk:
                print("✗ Connection closed by MCU")
                break
            
            packet_buffer += chunk
            
            # Process complete packets
            while len(packet_buffer) >= 24:
                # Find header
                header_idx = packet_buffer.find(b'\xDE\xAD\xBE\xEF')
                if header_idx == -1:
                    packet_buffer = b''
                    break
                
                if header_idx > 0:
                    packet_buffer = packet_buffer[header_idx:]
                
                if len(packet_buffer) < 24:
                    break
                
                # Extract and parse packet
                packet = packet_buffer[:24]
                packet_buffer = packet_buffer[24:]
                
                # Parse fields
                packet_count = struct.unpack('>I', packet[4:8])[0]
                timestamp_ms = struct.unpack('>I', packet[8:12])[0]
                sample_count = struct.unpack('>I', packet[12:16])[0]
                signal_value = struct.unpack('>f', packet[16:20])[0]
                checksum_rx = struct.unpack('>H', packet[20:22])[0]
                
                # Verify checksum
                checksum_calc = sum(packet[:20]) & 0xFFFF
                status = "✓" if checksum_calc == checksum_rx else "✗"
                
                # Print packet info
                print(f"{status} Packet #{packet_count:5d} | "
                      f"Sample #{sample_count:6d} | "
                      f"Value: {signal_value:8.4f} | "
                      f"Time: {timestamp_ms:6d} ms")
                
                packets_received += 1
                
        except socket.timeout:
            continue
        except Exception as e:
            print(f"✗ Receive error: {e}")
            break
    
    print(f"\n✓ Received {packets_received} packets")
    return packets_received


def main():
    """Main test sequence"""
    print("=" * 70)
    print("MCU Network Communication Test")
    print("=" * 70)
    
    # Parse command line arguments
    host = sys.argv[1] if len(sys.argv) > 1 else '192.168.2.100'
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 7000
    
    # Test connection
    sock = test_connection(host, port)
    if not sock:
        print("\n✗ Test failed: Cannot connect to MCU")
        return 1
    
    print("\n" + "─" * 70)
    print("Test 1: Start streaming and receive data")
    print("─" * 70)
    
    if not send_command(sock, "START"):
        sock.close()
        return 1
    
    time.sleep(0.5)  # Wait for MCU to process command
    packets = receive_packets(sock, duration=3.0)
    
    if packets == 0:
        print("✗ No packets received!")
        sock.close()
        return 1
    
    print("\n" + "─" * 70)
    print("Test 2: Change amplitude parameter")
    print("─" * 70)
    
    if not send_command(sock, "AMP:2.5"):
        sock.close()
        return 1
    
    time.sleep(0.5)
    packets = receive_packets(sock, duration=2.0)
    
    print("\n" + "─" * 70)
    print("Test 3: Change noise parameter")
    print("─" * 70)
    
    if not send_command(sock, "NOISE:0.05"):
        sock.close()
        return 1
    
    time.sleep(0.5)
    packets = receive_packets(sock, duration=2.0)
    
    print("\n" + "─" * 70)
    print("Test 4: Stop streaming")
    print("─" * 70)
    
    if not send_command(sock, "STOP"):
        sock.close()
        return 1
    
    time.sleep(0.5)
    packets = receive_packets(sock, duration=2.0)
    
    if packets > 0:
        print("⚠ Warning: Still receiving packets after STOP command")
    else:
        print("✓ Streaming stopped successfully")
    
    # Clean up
    sock.close()
    
    print("\n" + "=" * 70)
    print("✓ All tests completed successfully!")
    print("=" * 70)
    
    return 0


if __name__ == '__main__':
    try:
        exit(main())
    except KeyboardInterrupt:
        print("\n\n✓ Test interrupted by user")
        exit(0)
