#!/usr/bin/env python3
"""Network client for telemetry reception.

This module provides the TelemetryNetworkClient class which handles
TCP connection, telemetry reception, and protocol decoding.

Architecture:
    - Manages TCP socket connection to embedded controller
    - Runs receive loop in background thread
    - Decodes binary telemetry packets
    - Provides callback interface for packet processing

Example Usage:
    def on_packet_received(packet):
        print(f"Voltage: {packet.measurements.grid_voltage_magnitude_pu}")
    
    client = TelemetryNetworkClient('192.168.2.100', 7000)
    client.set_packet_callback(on_packet_received)
    client.connect()
    client.send_command('START')
"""

import logging
import socket
import struct
import threading
import time
from typing import Callable, Optional

from .telemetry_protocol import GridSideTelemetryPacket, TelemetryDecoder


logger = logging.getLogger(__name__)


class TelemetryNetworkClient:
    """TCP client for telemetry reception from embedded controller.
    
    Handles connection management, command transmission, and continuous
    reception of binary telemetry packets in background thread.
    
    Attributes:
        host: Controller IP address
        port: Controller TCP port
        connected: Connection status
        
    Example:
        client = TelemetryNetworkClient('192.168.2.100', 7000)
        client.set_packet_callback(handle_packet)
        if client.connect():
            client.send_command('START')
            # Packets received in background...
            time.sleep(10)
            client.send_command('STOP')
            client.disconnect()
    """
    
    def __init__(self, host: str = '192.168.2.100', port: int = 7000, verbose: bool = False):
        """Initialize network client.
        
        Args:
            host: Controller IP address
            port: Controller TCP port
            verbose: Enable verbose logging for debugging decode issues
        """
        self.host = host
        self.port = port
        self.connected = False
        self.verbose = verbose
        
        self._socket: Optional[socket.socket] = None
        self._receive_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._pause_event = threading.Event()
        self._pause_confirm_event = threading.Event()
        self._decoder = TelemetryDecoder()
        self._packet_callback: Optional[Callable[[GridSideTelemetryPacket], None]] = None
        
        # Statistics
        self._packets_received = 0
        self._packets_lost = 0
        self._last_sequence = -1
        self._bytes_received = 0
        self._decode_errors = 0
        self._stats_lock = threading.Lock()
    
    def set_packet_callback(self, callback: Callable[[GridSideTelemetryPacket], None]) -> None:
        """Set callback function for received packets.
        
        The callback is invoked from the receive thread whenever a valid
        telemetry packet is decoded.
        
        Args:
            callback: Function taking GridSideTelemetryPacket as argument
            
        Example:
            def on_packet(packet):
                if packet.measurements:
                    print(f"DC Current: {packet.measurements.dc_link_current_pu:.3f} pu")
            
            client.set_packet_callback(on_packet)
        """
        self._packet_callback = callback
    
    def get_socket(self) -> Optional[socket.socket]:
        """Get the underlying TCP socket for shared use.
        
        This allows other clients (e.g., ParameterClient) to share the
        same TCP connection, which is necessary when the server only
        supports one client connection at a time.
        
        Returns:
            The active socket, or None if not connected
            
        Warning:
            The caller should NOT close this socket - it's managed by
            TelemetryNetworkClient and will be closed on disconnect().
        """
        return self._socket if self.connected else None
    
    def connect(self) -> bool:
        """Connect to embedded controller.
        
        Returns:
            True if connection successful, False otherwise
        """
        if self.connected:
            logger.warning("Already connected")
            return True
        
        try:
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._socket.settimeout(5.0)
            self._socket.connect((self.host, self.port))
            self._socket.settimeout(0.1)  # Short timeout allows pause handling
            
            self.connected = True
            logger.info(f"Connected to {self.host}:{self.port}")
            
            # Start receive thread
            self._stop_event.clear()
            self._receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
            self._receive_thread.start()
            
            return True
            
        except (socket.error, OSError) as e:
            logger.error(f"Connection failed: {e}")
            self.connected = False
            if self._socket:
                self._socket.close()
                self._socket = None
            return False
    
    def disconnect(self) -> None:
        """Disconnect from controller and stop receive thread."""
        if not self.connected:
            return
        
        self.connected = False
        self._stop_event.set()
        self._pause_event.clear()
        self._pause_confirm_event.clear()
        
        # Close socket (will unblock receive)
        if self._socket:
            try:
                self._socket.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self._socket.close()
            self._socket = None
        
        # Wait for receive thread to finish
        if self._receive_thread and self._receive_thread.is_alive():
            self._receive_thread.join(timeout=2.0)
        
        logger.info("Disconnected")
    
    def send_command(self, command: str) -> bool:
        """Send text command to controller.
        
        Args:
            command: Command string (e.g., 'START', 'STOP')
            
        Returns:
            True if sent successfully, False otherwise
            
        Example:
            client.send_command('START')  # Start telemetry streaming
            client.send_command('STOP')   # Stop telemetry streaming
        """
        if not self.connected or not self._socket:
            logger.warning("Not connected, cannot send command")
            return False
        
        try:
            message = f"{command}\n".encode('utf-8')
            self._socket.sendall(message)
            logger.info(f"Sent command: {command}")
            return True
        except (socket.error, OSError) as e:
            logger.error(f"Failed to send command: {e}")
            return False

    def pause_receiving(self) -> bool:
        """Pause the receive loop without dropping the socket.

        Returns:
            True if the receive loop acknowledged the pause, False otherwise.
        """
        if not self.connected or not self._receive_thread:
            return False

        self._pause_event.set()
        # Wait briefly for receive loop to acknowledge pause
        return self._pause_confirm_event.wait(timeout=0.5)

    def resume_receiving(self) -> None:
        """Resume the receive loop after a pause."""
        self._pause_event.clear()
        self._pause_confirm_event.clear()
    
    def _receive_loop(self) -> None:
        """Background thread for continuous packet reception.
        
        Receives binary data, decodes telemetry packets, and invokes
        callback for each valid packet.
        """
        logger.info("Receive loop started")
        
        # Buffer for partial packets
        receive_buffer = bytearray()
        
        while not self._stop_event.is_set() and self.connected:
            if self._pause_event.is_set():
                self._pause_confirm_event.set()
                time.sleep(0.01)
                continue
            else:
                if self._pause_confirm_event.is_set():
                    self._pause_confirm_event.clear()
            try:
                # Receive data from socket
                data = self._socket.recv(4096)
                if not data:
                    logger.warning("Socket closed by remote")
                    break
                
                receive_buffer.extend(data)
                self._bytes_received += len(data)
                
                # Try to decode complete packets from buffer
                while len(receive_buffer) >= self._decoder.HEADER_SIZE:
                    header_slice = bytes(receive_buffer[:self._decoder.HEADER_SIZE])

                    try:
                        header_tuple = struct.unpack(self._decoder.HEADER_FORMAT, header_slice)
                    except struct.error as exc:
                        logger.error(f"Failed to unpack telemetry header: {exc}")
                        self._decode_errors += 1
                        receive_buffer = receive_buffer[self._decoder.HEADER_SIZE:]
                        continue

                    payload_length = header_tuple[2]
                    flags = header_tuple[3]
                    expected_size = self._decoder.HEADER_SIZE + payload_length

                    if expected_size < self._decoder.HEADER_SIZE:
                        logger.error(
                            "Invalid payload length reported in header: %d (payload=%d)",
                            expected_size,
                            payload_length,
                        )
                        self._decode_errors += 1
                        receive_buffer = receive_buffer[self._decoder.HEADER_SIZE:]
                        continue

                    if len(receive_buffer) < expected_size:
                        break

                    packet_bytes = bytes(receive_buffer[:expected_size])
                    receive_buffer = receive_buffer[expected_size:]
                    
                    if self.verbose:
                        print(f"\n[NETWORK] Attempting to decode packet ({expected_size} bytes)")
                        print(f"[NETWORK] First 32 bytes (hex): {packet_bytes[:32].hex()}")
                        expected_from_flags = self._decoder.calculate_expected_size(flags)
                        if expected_from_flags != expected_size:
                            print(
                                "[NETWORK WARNING] Header payload length (%d) differs from flag-derived size (%d)",
                                expected_size,
                                expected_from_flags,
                            )
                    
                    packet = self._decoder.decode(packet_bytes, verbose=self.verbose)
                    
                    if packet:
                        self._update_statistics(packet)
                        
                        # Invoke callback
                        if self._packet_callback:
                            try:
                                self._packet_callback(packet)
                            except Exception as e:
                                logger.error(f"Callback error: {e}")
                    else:
                        with self._stats_lock:
                            self._decode_errors += 1
                        if self.verbose:
                            print(f"[NETWORK ERROR] Failed to decode packet")
                        logger.warning("Failed to decode packet")
                
            except socket.timeout:
                continue
            except (socket.error, OSError) as e:
                if self.connected:  # Only log if unexpected
                    logger.error(f"Receive error: {e}")
                break
        
        logger.info("Receive loop stopped")
    
    def _update_statistics(self, packet: GridSideTelemetryPacket) -> None:
        """Update reception statistics from packet.
        
        Args:
            packet: Decoded telemetry packet
        """
        with self._stats_lock:
            self._packets_received += 1
            
            # Check for lost packets (sequence number gaps)
            if self._last_sequence >= 0:
                expected_seq = (self._last_sequence + 1) & 0xFFFFFFFF
                if packet.header.sequence_number != expected_seq:
                    # Detect stuck sequence number (likely firmware bug)
                    if packet.header.sequence_number == self._last_sequence:
                        logger.warning(f"Sequence number stuck at {packet.header.sequence_number} (firmware may not be incrementing)")
                        # Don't count as lost packets - this is a different issue
                    # Calculate number of lost packets only if sequence advanced
                    elif packet.header.sequence_number > expected_seq:
                        lost = packet.header.sequence_number - expected_seq
                        self._packets_lost += lost
                        logger.warning(f"Lost {lost} packets (seq {expected_seq} -> {packet.header.sequence_number})")
                    else:
                        # Handle sequence number wraparound (0xFFFFFFFF -> 0)
                        # Only count as wraparound if the jump is very large (> 0x80000000)
                        gap = expected_seq - packet.header.sequence_number
                        if gap > 0x80000000:  # Likely wraparound
                            lost = (0xFFFFFFFF - expected_seq) + packet.header.sequence_number + 1
                            self._packets_lost += lost
                            logger.warning(f"Lost {lost} packets on wraparound (seq {expected_seq} -> {packet.header.sequence_number})")
                        else:
                            # Sequence went backwards significantly - likely a reset or major issue
                            logger.error(f"Sequence number went backwards unexpectedly: {expected_seq} -> {packet.header.sequence_number}")
            
            self._last_sequence = packet.header.sequence_number
    
    def get_statistics(self) -> dict:
        """Get reception statistics.
        
        Returns:
            Dictionary with statistics (packets_received, packets_lost, etc.)
            
        Example:
            stats = client.get_statistics()
            print(f"Received: {stats['packets_received']}")
            print(f"Lost: {stats['packets_lost']}")
            print(f"Loss rate: {stats['loss_rate_percent']:.2f}%")
        """
        with self._stats_lock:
            total = self._packets_received + self._packets_lost
            loss_rate = (self._packets_lost / total * 100.0) if total > 0 else 0.0
            
            return {
                'packets_received': self._packets_received,
                'packets_lost': self._packets_lost,
                'bytes_received': self._bytes_received,
                'decode_errors': self._decode_errors,
                'loss_rate_percent': loss_rate
            }
    
    def reset_statistics(self) -> None:
        """Reset all statistics counters."""
        with self._stats_lock:
            self._packets_received = 0
            self._packets_lost = 0
            self._last_sequence = -1
            self._bytes_received = 0
            self._decode_errors = 0
