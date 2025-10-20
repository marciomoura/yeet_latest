#!/usr/bin/env python3
"""Simple diagnostic tool to test telemetry connection.

This script connects to the embedded controller and tests:
1. TCP connection
2. START command transmission
3. Packet reception
4. Packet decoding

Usage:
    python telemetry_diagnostic.py --host 192.168.2.100 --port 7000
"""

import argparse
import socket
import struct
import time
from telemetry_protocol import TelemetryDecoder


def test_connection(host: str, port: int) -> bool:
    """Test TCP connection to controller.
    
    Args:
        host: Controller IP address
        port: Controller TCP port
        
    Returns:
        True if connection successful
    """
    print(f"Testing connection to {host}:{port}...")
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5.0)
        sock.connect((host, port))
        print("✅ Connection successful!")
        sock.close()
        return True
    except Exception as e:
        print(f"❌ Connection failed: {e}")
        return False


def test_start_command(host: str, port: int) -> bool:
    """Send START command and check response.
    
    Args:
        host: Controller IP address
        port: Controller TCP port
        
    Returns:
        True if command sent successfully
    """
    print(f"\nSending START command...")
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5.0)
        sock.connect((host, port))
        
        # Send START command
        command = b"START\n"
        sock.sendall(command)
        print(f"✅ Sent command: {command.decode().strip()}")
        
        sock.close()
        return True
    except Exception as e:
        print(f"❌ Failed to send command: {e}")
        return False


def test_packet_reception(host: str, port: int, duration_sec: float = 5.0) -> None:
    """Receive and analyze telemetry packets.
    
    Args:
        host: Controller IP address
        port: Controller TCP port
        duration_sec: How long to receive packets
    """
    print(f"\nReceiving telemetry packets for {duration_sec} seconds...")
    
    decoder = TelemetryDecoder()
    
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(2.0)
        sock.connect((host, port))
        
        # Send START command first
        sock.sendall(b"START\n")
        print("Sent START command")
        time.sleep(0.5)  # Wait for controller to start streaming
        
        # Statistics
        packets_received = 0
        packets_decoded = 0
        total_bytes = 0
        decode_errors = 0
        
        receive_buffer = bytearray()
        start_time = time.time()
        
        while time.time() - start_time < duration_sec:
            try:
                # Receive data
                data = sock.recv(4096)
                if not data:
                    print("Connection closed by remote")
                    break
                
                receive_buffer.extend(data)
                total_bytes += len(data)
                
                # Try to decode packets from buffer
                while len(receive_buffer) >= decoder.HEADER_SIZE:
                    # Peek at header to get packet size
                    header_bytes = bytes(receive_buffer[:decoder.HEADER_SIZE])
                    
                    try:
                        # Extract flags to calculate expected size
                        flags = int.from_bytes(header_bytes[8:12], byteorder='little')
                        expected_size = decoder.calculate_expected_size(flags)
                        
                        # Check if we have complete packet
                        if len(receive_buffer) < expected_size:
                            break  # Wait for more data
                        
                        # Extract packet
                        packet_bytes = bytes(receive_buffer[:expected_size])
                        receive_buffer = receive_buffer[expected_size:]
                        packets_received += 1
                        
                        # Try to decode
                        packet = decoder.decode(packet_bytes)
                        
                        if packet:
                            packets_decoded += 1
                            
                            # Print first packet details
                            if packets_decoded == 1:
                                print(f"\n✅ First packet decoded successfully!")
                                print(f"   Timestamp: {packet.header.timestamp_ms} ms")
                                print(f"   Sequence: {packet.header.sequence_number}")
                                print(f"   Packet size: {expected_size} bytes")
                                
                                if packet.measurements:
                                    print(f"   Grid Voltage: {packet.measurements.grid_voltage_magnitude_pu:.3f} pu")
                                    print(f"   DC Current: {packet.measurements.dc_link_current_pu:.3f} pu")
                                if packet.observer:
                                    print(f"   Frequency: {packet.observer.estimated_frequency_pu:.3f} pu")
                        else:
                            decode_errors += 1
                            print(f"❌ Failed to decode packet {packets_received}")
                            # Print first 32 bytes of failed packet for debugging
                            print(f"   Header bytes: {packet_bytes[:min(32, len(packet_bytes))].hex()}")
                    
                    except Exception as e:
                        print(f"❌ Error processing packet: {e}")
                        # Skip one byte and try again
                        receive_buffer = receive_buffer[1:]
                        decode_errors += 1
                
            except socket.timeout:
                continue
        
        # Send STOP command
        sock.sendall(b"STOP\n")
        sock.close()
        
        # Print statistics
        print(f"\n{'='*60}")
        print(f"RECEPTION STATISTICS")
        print(f"{'='*60}")
        print(f"Duration: {duration_sec:.1f} seconds")
        print(f"Total bytes received: {total_bytes}")
        print(f"Packets received: {packets_received}")
        print(f"Packets decoded: {packets_decoded}")
        print(f"Decode errors: {decode_errors}")
        
        if packets_received > 0:
            decode_rate = (packets_decoded / packets_received) * 100
            print(f"Decode success rate: {decode_rate:.1f}%")
            
            if decode_rate < 50:
                print(f"\n⚠️  WARNING: High decode failure rate!")
                print(f"   Possible issues:")
                print(f"   - Protocol version mismatch")
                print(f"   - Incorrect struct packing")
                print(f"   - Endianness issue")
                print(f"   - Network corruption")
        else:
            print(f"\n❌ NO PACKETS RECEIVED!")
            print(f"   Possible issues:")
            print(f"   - Controller not streaming telemetry")
            print(f"   - START command not processed")
            print(f"   - Network connectivity issue")
            print(f"   - Firewall blocking traffic")
        
    except Exception as e:
        print(f"❌ Error: {e}")


def main():
    """Main diagnostic program."""
    parser = argparse.ArgumentParser(description="Telemetry connection diagnostic tool")
    parser.add_argument('--host', default='192.168.2.100', help='Controller IP address')
    parser.add_argument('--port', type=int, default=7000, help='Controller TCP port')
    parser.add_argument('--duration', type=float, default=5.0, help='Reception test duration (seconds)')
    args = parser.parse_args()
    
    print("="*60)
    print("TELEMETRY CONNECTION DIAGNOSTIC")
    print("="*60)
    
    # Test 1: Connection
    if not test_connection(args.host, args.port):
        print("\n❌ Cannot connect to controller. Check:")
        print("   - Controller is powered on")
        print("   - IP address is correct")
        print("   - Network cable connected")
        print("   - Firewall settings")
        return 1
    
    # Test 2: START command
    if not test_start_command(args.host, args.port):
        print("\n❌ Cannot send START command")
        return 1
    
    # Test 3: Packet reception
    test_packet_reception(args.host, args.port, args.duration)
    
    print(f"\n{'='*60}")
    print("Diagnostic complete!")
    print(f"{'='*60}")
    
    return 0


if __name__ == '__main__':
    exit(main())
