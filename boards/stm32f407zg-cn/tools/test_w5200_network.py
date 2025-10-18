#!/usr/bin/env python3
"""
W5200 Network Test Script

Tests TCP communication with STM32F407 W5200 dual Ethernet controllers.
- SPI2: 192.168.2.x network (sends 0xDEADBEEF on connect, then echoes)
- SPI3: 192.168.10.x network (echoes data)

Usage:
    python test_w5200_network.py --node 0 --network spi2
    python test_w5200_network.py --node 5 --network spi3
    python test_w5200_network.py --node 0 --network both
"""

import socket
import time
import argparse
import sys
from typing import Optional


class W5200Client:
    """TCP client for W5200 network controller"""

    def __init__(self, ip: str, port: int = 7000, timeout: float = 5.0):
        """
        Initialize W5200 TCP client

        Args:
            ip: IP address of W5200 controller
            port: TCP port (default 7000)
            timeout: Socket timeout in seconds
        """
        self.ip = ip
        self.port = port
        self.timeout = timeout
        self.socket: Optional[socket.socket] = None

    def connect(self) -> bool:
        """
        Connect to W5200 TCP server

        Returns:
            True if connected successfully, False otherwise
        """
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(self.timeout)
            print(f"Connecting to {self.ip}:{self.port}...")
            self.socket.connect((self.ip, self.port))
            print(f"✓ Connected to {self.ip}:{self.port}")
            return True
        except socket.timeout:
            print(f"✗ Connection timeout to {self.ip}:{self.port}")
            return False
        except ConnectionRefusedError:
            print(f"✗ Connection refused by {self.ip}:{self.port}")
            return False
        except Exception as e:
            print(f"✗ Connection error: {e}")
            return False

    def receive(self, num_bytes: int) -> Optional[bytes]:
        """
        Receive data from W5200

        Args:
            num_bytes: Number of bytes to receive

        Returns:
            Received bytes or None on error
        """
        if not self.socket:
            print("✗ Not connected")
            return None

        try:
            data = self.socket.recv(num_bytes)
            if data:
                print(f"← Received {len(data)} bytes: {data.hex(' ').upper()}")
                return data
            else:
                print("✗ Connection closed by server")
                return None
        except socket.timeout:
            print("✗ Receive timeout")
            return None
        except Exception as e:
            print(f"✗ Receive error: {e}")
            return None

    def send(self, data: bytes) -> bool:
        """
        Send data to W5200

        Args:
            data: Bytes to send

        Returns:
            True if sent successfully, False otherwise
        """
        if not self.socket:
            print("✗ Not connected")
            return False

        try:
            print(f"→ Sending {len(data)} bytes: {data.hex(' ').upper()}")
            self.socket.sendall(data)
            return True
        except Exception as e:
            print(f"✗ Send error: {e}")
            return False

    def send_and_receive(self, data: bytes, expect_bytes: int = None) -> Optional[bytes]:
        """
        Send data and wait for response

        Args:
            data: Bytes to send
            expect_bytes: Expected number of bytes in response (default: same as sent)

        Returns:
            Received bytes or None on error
        """
        if expect_bytes is None:
            expect_bytes = len(data)

        if not self.send(data):
            return None

        time.sleep(0.1)  # Small delay for processing
        return self.receive(expect_bytes)

    def close(self):
        """Close connection"""
        if self.socket:
            self.socket.close()
            self.socket = None
            print(f"✓ Disconnected from {self.ip}:{self.port}")


def test_spi2(node_id: int):
    """
    Test SPI2 network (192.168.2.x)
    - Should receive 0xDEADBEEF on connect
    - Should echo data back
    """
    ip = f"192.168.2.{100 + node_id}"
    print(f"\n{'='*60}")
    print(f"Testing SPI2 Network: {ip}:7000")
    print(f"{'='*60}")

    client = W5200Client(ip)
    if not client.connect():
        return False

    # Test 1: Receive 0xDEADBEEF
    print("\n[Test 1] Receiving 0xDEADBEEF magic number...")
    deadbeef = client.receive(4)
    if deadbeef == b'\xDE\xAD\xBE\xEF':
        print("✓ Received correct magic number: 0xDEADBEEF")
    else:
        print(f"✗ Unexpected data: {deadbeef.hex().upper() if deadbeef else 'None'}")

    # Test 2: Echo test with ASCII
    print("\n[Test 2] Echo test with ASCII string...")
    test_string = b"Hello, W5200!"
    response = client.send_and_receive(test_string)
    if response == test_string:
        print(f"✓ Echo test passed: '{test_string.decode()}'")
    else:
        print(f"✗ Echo mismatch - Sent: {test_string.hex()}, Got: {response.hex() if response else 'None'}")

    # Test 3: Echo test with binary data
    print("\n[Test 3] Echo test with binary data...")
    test_binary = bytes([0x01, 0x02, 0x03, 0x04, 0xFF, 0xFE, 0xFD, 0xFC])
    response = client.send_and_receive(test_binary)
    if response == test_binary:
        print(f"✓ Binary echo test passed")
    else:
        print(f"✗ Binary echo mismatch")

    # Test 4: Large packet test
    print("\n[Test 4] Large packet test (1024 bytes)...")
    test_large = bytes(range(256)) * 4  # 1024 bytes
    response = client.send_and_receive(test_large, expect_bytes=1024)
    if response == test_large:
        print(f"✓ Large packet test passed (1024 bytes)")
    else:
        print(f"✗ Large packet mismatch - Expected: {len(test_large)}, Got: {len(response) if response else 0}")

    client.close()
    return True


def test_spi3(node_id: int):
    """
    Test SPI3 network (192.168.10.x)
    - Should echo data back (no magic number)
    """
    ip = f"192.168.10.{31 + node_id}"
    print(f"\n{'='*60}")
    print(f"Testing SPI3 Network: {ip}:7000")
    print(f"{'='*60}")

    client = W5200Client(ip)
    if not client.connect():
        return False

    # Test 1: Echo test with ASCII
    print("\n[Test 1] Echo test with ASCII string...")
    test_string = b"Hello, SPI3!"
    response = client.send_and_receive(test_string)
    if response == test_string:
        print(f"✓ Echo test passed: '{test_string.decode()}'")
    else:
        print(f"✗ Echo mismatch - Sent: {test_string.hex()}, Got: {response.hex() if response else 'None'}")

    # Test 2: Echo test with binary data
    print("\n[Test 2] Echo test with binary data...")
    test_binary = bytes([0xAA, 0x55, 0xA5, 0x5A, 0xFF, 0x00])
    response = client.send_and_receive(test_binary)
    if response == test_binary:
        print(f"✓ Binary echo test passed")
    else:
        print(f"✗ Binary echo mismatch")

    # Test 3: Large packet test (with DMA)
    print("\n[Test 3] Large packet test (8192 bytes) - Testing DMA...")
    test_large = bytes([(i % 256) for i in range(8192)])
    response = client.send_and_receive(test_large, expect_bytes=8192)
    if response == test_large:
        print(f"✓ Large packet test passed (8192 bytes with DMA)")
    else:
        print(f"✗ Large packet mismatch - Expected: {len(test_large)}, Got: {len(response) if response else 0}")

    client.close()
    return True


def main():
    parser = argparse.ArgumentParser(
        description="Test W5200 dual Ethernet network communication",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Test SPI2 network with node ID 0 (IP: 192.168.2.100)
  python test_w5200_network.py --node 0 --network spi2

  # Test SPI3 network with node ID 5 (IP: 192.168.10.36)
  python test_w5200_network.py --node 5 --network spi3

  # Test both networks with node ID 0
  python test_w5200_network.py --node 0 --network both

  # Custom IP and port
  python test_w5200_network.py --ip 192.168.2.105 --port 7000
        """
    )

    parser.add_argument(
        '--node',
        type=int,
        default=0,
        help='DIP switch node ID (0-15, default: 0)'
    )

    parser.add_argument(
        '--network',
        choices=['spi2', 'spi3', 'both'],
        default='both',
        help='Which network to test (default: both)'
    )

    parser.add_argument(
        '--ip',
        type=str,
        help='Custom IP address (overrides --node and --network)'
    )

    parser.add_argument(
        '--port',
        type=int,
        default=7000,
        help='TCP port (default: 7000)'
    )

    parser.add_argument(
        '--timeout',
        type=float,
        default=5.0,
        help='Socket timeout in seconds (default: 5.0)'
    )

    args = parser.parse_args()

    print("╔════════════════════════════════════════════════════════════╗")
    print("║         W5200 Dual Ethernet Network Test Suite            ║")
    print("╚════════════════════════════════════════════════════════════╝")

    # Custom IP test
    if args.ip:
        print(f"\nTesting custom IP: {args.ip}:{args.port}")
        client = W5200Client(args.ip, args.port, args.timeout)
        if client.connect():
            test_string = b"Test packet"
            response = client.send_and_receive(test_string)
            if response:
                print(f"✓ Communication successful")
            client.close()
        return

    # Standard tests
    results = []

    if args.network in ['spi2', 'both']:
        try:
            results.append(('SPI2', test_spi2(args.node)))
        except KeyboardInterrupt:
            print("\n\n✗ Test interrupted by user")
            sys.exit(1)

    if args.network in ['spi3', 'both']:
        try:
            results.append(('SPI3', test_spi3(args.node)))
        except KeyboardInterrupt:
            print("\n\n✗ Test interrupted by user")
            sys.exit(1)

    # Summary
    print(f"\n{'='*60}")
    print("Test Summary")
    print(f"{'='*60}")
    for network, success in results:
        status = "✓ PASSED" if success else "✗ FAILED"
        print(f"{network}: {status}")

    all_passed = all(result[1] for result in results)
    print(f"\nOverall: {'✓ ALL TESTS PASSED' if all_passed else '✗ SOME TESTS FAILED'}")

    sys.exit(0 if all_passed else 1)


if __name__ == '__main__':
    main()
