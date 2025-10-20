#!/usr/bin/env python3
"""Raw packet capture tool for debugging telemetry protocol issues.

This tool captures raw bytes from the TCP connection and displays them
in hex format for manual analysis.

Usage:
    python capture_raw_packets.py --host 192.168.2.100 --port 7000 --duration 5
"""

import argparse
import socket
import time


def capture_packets(host: str, port: int, duration: float):
    """Capture and display raw packets.
    
    Args:
        host: Controller IP address
        port: Controller TCP port
        duration: Capture duration in seconds
    """
    print(f"Connecting to {host}:{port}...")
    
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(2.0)
        sock.connect((host, port))
        print(f"✅ Connected!")
        
        # Send START command
        sock.sendall(b"START\n")
        print(f"✅ Sent START command\n")
        time.sleep(0.5)
        
        total_bytes = 0
        packet_count = 0
        start_time = time.time()
        
        print("Capturing packets...")
        print("="*80)
        
        while time.time() - start_time < duration:
            try:
                data = sock.recv(4096)
                if not data:
                    print("Connection closed")
                    break
                
                total_bytes += len(data)
                packet_count += 1
                
                # Display received data
                print(f"\nPacket {packet_count}: {len(data)} bytes received")
                print("-"*80)
                
                # Display hex dump
                for i in range(0, len(data), 16):
                    hex_part = ' '.join(f'{b:02x}' for b in data[i:i+16])
                    ascii_part = ''.join(chr(b) if 32 <= b < 127 else '.' for b in data[i:i+16])
                    print(f"{i:04x}:  {hex_part:<48}  {ascii_part}")
                
                # Try to parse header if we have at least 16 bytes
                if len(data) >= 16:
                    print("\nHeader Analysis:")
                    print(f"  packet_type:      0x{data[0]:02x} ({data[0]})")
                    print(f"  protocol_version: 0x{data[1]:02x} ({data[1]})")
                    payload_len = int.from_bytes(data[2:4], byteorder='little')
                    print(f"  payload_length:   {payload_len} bytes")
                    flags = int.from_bytes(data[8:12], byteorder='little')
                    print(f"  flags:            0x{flags:08x}")
                    timestamp = int.from_bytes(data[12:16], byteorder='little')
                    print(f"  timestamp_ms:     {timestamp}")
                    
                    # Decode flags
                    flag_names = []
                    if flags & 0x01:
                        flag_names.append("MEASUREMENTS")
                    if flags & 0x02:
                        flag_names.append("OBSERVER")
                    if flags & 0x04:
                        flag_names.append("MODULATOR")
                    if flags & 0x08:
                        flag_names.append("CONTROLLER")
                    if flags & 0x10:
                        flag_names.append("STATE_MACHINE")
                    
                    if flag_names:
                        print(f"  Active groups:    {', '.join(flag_names)}")
                
            except socket.timeout:
                continue
            except Exception as e:
                print(f"Error: {e}")
                break
        
        # Send STOP
        sock.sendall(b"STOP\n")
        sock.close()
        
        print("\n" + "="*80)
        print(f"Capture complete!")
        print(f"Total bytes: {total_bytes}")
        print(f"Packets: {packet_count}")
        print(f"Duration: {duration:.1f} seconds")
        
        if total_bytes == 0:
            print("\n❌ NO DATA RECEIVED!")
            print("   - Check that telemetry is being transmitted")
            print("   - Verify START command was processed")
        
    except Exception as e:
        print(f"❌ Error: {e}")


def main():
    parser = argparse.ArgumentParser(description="Raw packet capture tool")
    parser.add_argument('--host', default='192.168.2.100', help='Controller IP address')
    parser.add_argument('--port', type=int, default=7000, help='Controller TCP port')
    parser.add_argument('--duration', type=float, default=5.0, help='Capture duration (seconds)')
    args = parser.parse_args()
    
    capture_packets(args.host, args.port, args.duration)


if __name__ == '__main__':
    main()
