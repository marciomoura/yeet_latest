#!/usr/bin/env python3
"""Unified single-threaded network client for telemetry and parameters.

This module provides a UnifiedNetworkClient that handles both telemetry
streaming and parameter configuration without threading races.

Architecture:
    - Single TCP socket connection
    - Qt timer-based polling (no threads)
    - Stateless packet processing (displays whatever arrives from server)
    - Automatic packet type detection and routing
    - Server manages telemetry START/STOP state

Usage:
    client = UnifiedNetworkClient('192.168.2.100', 7000)
    client.telemetry_packet_received.connect(handle_telemetry)
    client.parameter_response_received.connect(handle_param)
    
    client.connect()
    client.start_telemetry()
    # ... later ...
    client.send_parameter_request(...)
"""

import logging
import socket
import struct
from enum import Enum
from typing import Callable, Optional, List

from PyQt5 import QtCore

from .telemetry_protocol import GridSideTelemetryPacket, MachineSideTelemetryPacket, TelemetryDecoder, PROTOCOL_VERSION
from .parameter_protocol import (
    ParameterMetadata,
    ParameterValue,
    ParameterProtocolEncoder,
    ParameterProtocolDecoder,
    ParameterSetStatus
)


logger = logging.getLogger(__name__)


class NetworkState(Enum):
    """Network client connection state (NOT telemetry state - server manages that)."""
    DISCONNECTED = 0
    CONNECTED = 1  # Connected, receiving whatever server sends


class UnifiedNetworkClient(QtCore.QObject):
    """Single-threaded stateless network client for telemetry and parameters.
    
    Uses Qt signals/slots and timers to avoid threading issues.
    Displays whatever packets arrive from server (server manages telemetry state).
    
    Signals:
        telemetry_packet_received(GridSideTelemetryPacket): Emitted when telemetry packet decoded
        parameter_response_received(dict): Emitted when parameter response received
        connected_changed(bool): Emitted when connection status changes
        error_occurred(str): Emitted on network or protocol errors
    """
    
    # Qt Signals
    telemetry_packet_received = QtCore.pyqtSignal(object)  # GridSideTelemetryPacket
    parameter_response_received = QtCore.pyqtSignal(dict)  # Response data
    connected_changed = QtCore.pyqtSignal(bool)
    error_occurred = QtCore.pyqtSignal(str)
    
    def __init__(self, host: str = '192.168.2.100', port: int = 7000, verbose: bool = False, debug_log: Optional[str] = None, parent=None):
        """Initialize unified network client.
        
        Args:
            host: Controller IP address
            port: Controller TCP port
            verbose: Enable verbose logging
            debug_log: Path to telemetry debug log file (enables detailed packet decoding logs)
            parent: Qt parent object
        """
        super().__init__(parent)
        
        self.host = host
        self.port = port
        self.verbose = verbose
        
        self._socket: Optional[socket.socket] = None
        self._state = NetworkState.DISCONNECTED
        
        # Protocol handlers
        # Enable CRC checking for telemetry - server now appends CRC32 to all packets
        self._telemetry_decoder = TelemetryDecoder(require_crc=True, debug_log=debug_log)  # Enforce CRC validation
        self._param_encoder = ParameterProtocolEncoder()
        self._param_decoder = ParameterProtocolDecoder()
        
        # Statistics
        self._packets_received = 0
        self._packets_lost = 0
        self._last_sequence = -1
        self._decode_errors = 0
        
        # Receive buffer
        self._recv_buffer = bytearray()
        
        # Heartbeat counter for debugging
        self._poll_count = 0
        
        # Track consecutive empty polls to detect stalled connections
        self._empty_poll_count = 0
        self._max_empty_polls = 10000  # If 10000 polls (10s at 1ms rate) get no data while streaming, socket may be stalled
        
        # Resilience tracking
        self._consecutive_errors = 0
        self._max_consecutive_errors = 10  # Disconnect after 10 consecutive errors
        self._last_packet_time = 0
        self._packet_timeout_ms = 10000  # 10 seconds without packets = timeout
        self._packet_timeout_enabled = True  # Can be disabled during parameter loading
        self._telemetry_streaming = False  # Track if telemetry should be streaming
        self._recv_buffer_max_size = 5 * 1024 * 1024  # 5MB max buffer (handle high-rate bursts)
        
        # Qt timer for polling socket
        self._poll_timer = QtCore.QTimer(self)
        self._poll_timer.timeout.connect(self._poll_socket)
        self._poll_timer.setInterval(1)  # 1ms polling = 1000Hz max
        
        # Qt timer for connection health check
        self._health_timer = QtCore.QTimer(self)
        self._health_timer.timeout.connect(self._check_connection_health)
        self._health_timer.setInterval(500)  # Check every 500ms
        
        # Qt timer for deferred packet processing (separate from polling)
        self._process_timer = QtCore.QTimer(self)
        self._process_timer.timeout.connect(self._process_receive_buffer_deferred)
        self._process_timer.setInterval(0)  # 0ms = process as fast as possible (Qt event loop limited)
        
        # Flag to control processing
        self._processing_enabled = True
        
        logger.info(f"UnifiedNetworkClient initialized for {host}:{port}")
    
    @property
    def connected(self) -> bool:
        """Check if connected to controller."""
        return self._state == NetworkState.CONNECTED
    
    @property
    def is_streaming(self) -> bool:
        """Check if actively streaming telemetry.
        
        Note: This is not tracked client-side anymore. Server manages state.
        Returns True if connected (client doesn't know server's telemetry state).
        """
        return self.connected
    
    def connect(self) -> bool:
        """Connect to controller.
        
        Returns:
            True if connection successful, False otherwise
        """
        if self.connected:
            logger.warning("Already connected")
            return True
        
        try:
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            
            # Set socket options for better reliability
            self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
            self._socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            
            self._socket.settimeout(5.0)
            self._socket.connect((self.host, self.port))
            self._socket.setblocking(False)  # Non-blocking for polling
            
            self._state = NetworkState.CONNECTED
            self._recv_buffer.clear()
            self._consecutive_errors = 0
            self._last_packet_time = QtCore.QDateTime.currentMSecsSinceEpoch()
            
            logger.info(f"Connected to {self.host}:{self.port}")
            self.connected_changed.emit(True)
            
            # Start polling timer
            self._poll_timer.start()
            
            # Start health check timer
            self._health_timer.start()
            
            # Start deferred processing timer
            self._process_timer.start()
            
            return True
            
        except Exception as e:
            error_msg = f"Connection failed: {e}"
            logger.error(error_msg)
            self.error_occurred.emit(error_msg)
            self._cleanup_socket()
            return False
    
    def disconnect(self) -> None:
        """Disconnect from controller."""
        if not self.connected:
            return
        
        # Send STOP command to be polite (but don't wait for response)
        try:
            self._send_command('STOP')
        except:
            pass  # Ignore errors during shutdown
        
        # Stop timers
        self._poll_timer.stop()
        self._health_timer.stop()
        self._process_timer.stop()
        
        # Close socket
        self._cleanup_socket()
        
        # Reset state
        self._state = NetworkState.DISCONNECTED
        self._telemetry_streaming = False  # Reset streaming flag
        logger.info("Disconnected")
        self.connected_changed.emit(False)
    
    def _cleanup_socket(self) -> None:
        """Clean up socket resources."""
        if self._socket:
            try:
                self._socket.close()
            except:
                pass
            self._socket = None
    
    def start_telemetry(self) -> bool:
        """Request server to start telemetry streaming.
        
        Note: Server manages actual telemetry state. This just sends the START command.
        
        Returns:
            True if command sent successfully, False otherwise
        """
        if not self.connected:
            logger.error("Not connected - cannot start telemetry")
            return False
        
        # Send START command (server will manage state)
        if self._send_command('START'):
            self._packets_received = 0
            self._last_sequence = -1
            self._telemetry_streaming = True  # Expect telemetry packets
            self._empty_poll_count = 0  # Reset stall detector
            logger.info("Sent START telemetry command to server")
            return True
        
        return False
    
    def stop_telemetry(self) -> bool:
        """Request server to stop telemetry streaming.
        
        Note: Server manages actual telemetry state. This just sends the STOP command.
        
        Returns:
            True if command sent successfully, False otherwise
        """
        if not self.connected:
            logger.warning("Not connected - cannot stop telemetry")
            return False
        
        # Send STOP command (server will manage state)
        if self._send_command('STOP'):
            self._telemetry_streaming = False  # No longer expecting telemetry packets
            self._empty_poll_count = 0  # Reset stall detector
            logger.info("Sent STOP telemetry command to server")
            return True
        
        return False
    
    def _send_command(self, command: str) -> bool:
        """Send text command to controller.
        
        Args:
            command: Command string (e.g., 'START', 'STOP')
            
        Returns:
            True if sent successfully, False otherwise
        """
        if not self._socket:
            return False
        
        try:
            data = command.encode('ascii')
            self._socket.sendall(data)
            logger.debug(f"Sent command: {command}")
            return True
        except Exception as e:
            logger.error(f"Failed to send command '{command}': {e}")
            return False
    
    def send_parameter_count_request(self) -> bool:
        """Request total parameter count from controller.
        
        Returns:
            True if request sent successfully, False otherwise
        """
        if not self.connected:
            logger.error("Not connected - cannot send parameter request")
            return False
        
        try:
            # Encode COUNT request
            request_bytes = self._param_encoder.encode_count_request()
            
            # Send request (server will respond while telemetry continues)
            self._socket.sendall(request_bytes)
            
            logger.debug("Sent parameter COUNT request")
            return True
            
        except Exception as e:
            error_msg = f"Failed to send parameter COUNT request: {e}"
            logger.error(error_msg)
            self.error_occurred.emit(error_msg)
            return False
    
    def send_parameter_get_by_index_request(self, index: int) -> bool:
        """Request parameter metadata by index.
        
        Args:
            index: Zero-based parameter index
            
        Returns:
            True if request sent successfully, False otherwise
        """
        if not self.connected:
            logger.error("Not connected - cannot send parameter request")
            return False
        
        try:
            # Encode GET_BY_INDEX request
            request_bytes = self._param_encoder.encode_get_by_index_request(index)
            
            # Send request (server will respond while telemetry continues)
            self._socket.sendall(request_bytes)
            
            logger.debug(f"Sent parameter GET_BY_INDEX request for index {index}")
            return True
            
        except Exception as e:
            error_msg = f"Failed to send parameter GET_BY_INDEX request: {e}"
            logger.error(error_msg)
            self.error_occurred.emit(error_msg)
            return False
    
    def send_parameter_set_request(self, component: str, name: str, value: float) -> bool:
        """Set parameter value on controller.
        
        Args:
            component: Component name
            name: Parameter name
            value: New parameter value
            
        Returns:
            True if request sent successfully, False otherwise
        """
        if not self.connected:
            logger.error(f"Not connected - cannot send parameter request")
            return False
        
        try:
            # Encode SET request
            request_bytes = self._param_encoder.encode_set_request(component, name, value)
            
            # Send request (server will respond while telemetry continues)
            self._socket.sendall(request_bytes)
            
            logger.debug(f"Sent parameter SET request for {component}.{name} = {value}")
            return True
            
        except Exception as e:
            error_msg = f"Failed to send parameter SET request: {e}"
            logger.error(error_msg)
            self.error_occurred.emit(error_msg)
            return False
    
    def _poll_socket(self) -> None:
        """Poll socket for incoming data (called by Qt timer)."""
        if not self._socket:
            return
        
        # Heartbeat logging every 1000 polls (~5 seconds at 5ms interval)
        self._poll_count += 1
        if self.verbose and self._poll_count % 1000 == 0:
            logger.debug(f"Network polling active: {self._poll_count} polls, {self._packets_received} packets received")
        
        try:
            # Read available data (non-blocking) in a loop to drain OS buffer
            # Use larger chunk size for high-speed telemetry
            while True:
                try:
                    data = self._socket.recv(65536)
                    
                    if not data:
                        # Connection closed by remote
                        if self._empty_poll_count == 0: # Only log once
                            logger.warning("Connection closed by controller")
                            self._handle_connection_error("Connection closed by remote")
                        return
                    
                    # Successfully received data - reset empty poll counter
                    self._empty_poll_count = 0
                    
                    # Log data reception
                    if self.verbose:
                        logger.debug(f"Received {len(data)} bytes from socket (buffer now {len(self._recv_buffer) + len(data)} bytes)")
                    
                    # Check buffer overflow
                    if len(self._recv_buffer) + len(data) > self._recv_buffer_max_size:
                        logger.error(f"Receive buffer overflow ({len(self._recv_buffer)} + {len(data)} > {self._recv_buffer_max_size}), clearing buffer")
                        self._recv_buffer.clear()
                        self._consecutive_errors += 1
                        return
                    
                    # Append to receive buffer
                    self._recv_buffer.extend(data)
                    
                    # Update last packet time
                    self._last_packet_time = QtCore.QDateTime.currentMSecsSinceEpoch()
                    
                    # Reset error counter on successful receive
                    self._consecutive_errors = 0
                    
                except BlockingIOError:
                    # No more data available for now
                    break
                except (ConnectionResetError, ConnectionAbortedError) as e:
                    logger.error(f"Connection error: {e}")
                    self._handle_connection_error(str(e))
                    return
            
            # If telemetry is streaming and we get too many empty polls in a row,
            # the socket may be stalled and need reconnection
            if self._telemetry_streaming and self._empty_poll_count > self._max_empty_polls:
                self._empty_poll_count += 1
                if self._empty_poll_count > self._max_empty_polls:
                    logger.error(f"Socket appears stalled: {self._empty_poll_count} consecutive empty polls while telemetry streaming")
                    self._handle_connection_error("Socket stalled - no data")
            elif self._telemetry_streaming:
                self._empty_poll_count += 1
            
        except Exception as e:
            error_msg = f"Socket error during receive: {e}"
            logger.error(error_msg, exc_info=True)
            self._consecutive_errors += 1
            if self._consecutive_errors >= self._max_consecutive_errors:
                self._handle_connection_error(f"Too many consecutive errors ({self._consecutive_errors})")
            # Socket error (could be EWOULDBLOCK, EAGAIN, or actual error)
            if e.errno not in [10035, 10060]:  # Windows: WSAEWOULDBLOCK, WSAETIMEDOUT
                logger.error(f"Socket OS error: {e}")
                self._consecutive_errors += 1
                if self._consecutive_errors >= self._max_consecutive_errors:
                    self._handle_connection_error(f"Too many consecutive errors ({self._consecutive_errors})")
        except Exception as e:
            error_msg = f"Socket error during receive: {e}"
            logger.error(error_msg, exc_info=True)
            self._consecutive_errors += 1
            if self._consecutive_errors >= self._max_consecutive_errors:
                self._handle_connection_error(f"Too many consecutive errors ({self._consecutive_errors})")
    
    def _process_receive_buffer_deferred(self) -> None:
        """Process packets from buffer on a separate timer (not during socket polling).
        
        This deferred approach prevents heavy packet processing from blocking
        socket reads, which is critical for high-speed data streams.
        """
        if not self.connected or not self._processing_enabled:
            return
        
        try:
            # Process more packets per timer tick to handle high data rates
            # With large window sizes (1800+ samples), UI updates are heavy,
            # so we need aggressive packet processing to prevent buffer overflow
            packets_processed = 0
            max_packets_per_tick = 1000  # Aggressively process packets to prevent buffer overflow
            
            while (len(self._recv_buffer) >= 4 and  # Minimum size for sync frame or packet header
                   packets_processed < max_packets_per_tick and
                   self._processing_enabled):
                
                # Check for sync frame first (0xDEADBEEF = 0xEF 0xBE 0xAD 0xDE in little-endian)
                if len(self._recv_buffer) >= 4 and self._telemetry_decoder.is_sync_frame(bytes(self._recv_buffer[:4])):
                    if self.verbose:
                        logger.info("Received sync frame (0xDEADBEEF) - stream synchronized")
                    # Remove sync frame from buffer and continue processing
                    del self._recv_buffer[:4]
                    packets_processed += 1
                    continue
                
                # Need at least 16 bytes for actual packets
                if len(self._recv_buffer) < 16:
                    break
                
                # Peek at packet type
                packet_type = self._recv_buffer[0]
                
                # Try to process one packet
                consumed = False
                
                if packet_type in [0x01, 0x02]:  # Telemetry packets
                    consumed = self._process_telemetry_packet()
                elif packet_type in [0x41, 0x47, 0x49]:  # Parameter response packets
                    consumed = self._process_parameter_response(packet_type)
                else:
                    # Unknown packet type - try recovery
                    consumed = self._attempt_packet_resync(packet_type)
                
                if consumed:
                    packets_processed += 1
                    continue
                else:
                    # Need more data for this packet
                    break
        
        except Exception as e:
            logger.error(f"Error in deferred buffer processing: {e}", exc_info=True)
            self._consecutive_errors += 1
    
    def _attempt_packet_resync(self, packet_type: int) -> bool:
        """Attempt to recover from unknown packet type by resyncing.
        
        Args:
            packet_type: The unknown packet type byte
            
        Returns:
            True if handled (packet discarded or found), False if need more data
        """
        # Truly unknown - search for next valid sync point
        # First, search for the 0xDEADBEEF sync frame (most reliable)
        sync_magic = b'\xEF\xBE\xAD\xDE' # 0xDEADBEEF in little-endian
        magic_pos = self._recv_buffer.find(sync_magic)
        
        if magic_pos >= 0:
            logger.info(f"Found sync magic (0xDEADBEEF) at offset {magic_pos}, discarding {magic_pos} bytes")
            del self._recv_buffer[:magic_pos]
            return True
            
        # If no magic found, try decoding as telemetry anyway (might be a valid packet with weird type)
        consumed = self._process_telemetry_packet()
        if consumed:
            if self.verbose:
                logger.debug(f"Packet type 0x{packet_type:02x} was actually valid telemetry")
            return True
        
        # Try as parameter packet
        if packet_type <= 0xFF:
            consumed = self._process_parameter_response(packet_type)
            if consumed:
                if self.verbose:
                    logger.debug(f"Packet type 0x{packet_type:02x} was actually valid parameter response")
                return True
        
        # Search for next valid packet header (limited search)
        logger.warning(f"Unknown packet type 0x{packet_type:02x} at buffer start, attempting header search")
        
        found_sync = False
        # Scan up to 2KB for a valid header
        for i in range(1, min(2048, len(self._recv_buffer))):
            test_byte = self._recv_buffer[i]
            if test_byte in [0x01, 0x02, 0x41, 0x47, 0x49]:
                # Basic validation: check protocol version for telemetry
                if test_byte in [0x01, 0x02]:
                    if i + 1 < len(self._recv_buffer) and self._recv_buffer[i+1] == PROTOCOL_VERSION:
                        logger.info(f"Found potential telemetry sync at offset {i}, discarding {i} bytes")
                        del self._recv_buffer[:i]
                        found_sync = True
                        break
                else:
                    # Parameter packets don't have a version byte at offset 1 in the same way, 
                    # but we can still try to sync to them
                    logger.info(f"Found potential parameter sync at offset {i}, discarding {i} bytes")
                    del self._recv_buffer[:i]
                    found_sync = True
                    break
        
        if not found_sync:
            # If we have a lot of data and still no sync, clear some of it to avoid memory bloat
            if len(self._recv_buffer) > 4096:
                logger.error(f"Could not resync in first 2KB, discarding 4KB of buffer ({len(self._recv_buffer)} bytes total)")
                del self._recv_buffer[:4096]
                self._consecutive_errors += 1
            else:
                # Need more data to find a sync point
                return False
        
        return True  # Handled (cleared or resynced)
    
    def _check_connection_health(self) -> None:
        """Check connection health periodically."""
        if not self.connected:
            return
        
        # Skip timeout check if disabled (e.g., during parameter loading)
        if not self._packet_timeout_enabled:
            return
        
        # Only apply timeout when telemetry is actively streaming
        # When stopped, connection stays open but no packets are expected
        if not self._telemetry_streaming:
            return
        
        # Check if we've received data recently
        current_time = QtCore.QDateTime.currentMSecsSinceEpoch()
        time_since_last_packet = current_time - self._last_packet_time
        
        if time_since_last_packet > self._packet_timeout_ms:
            logger.warning(f"No packets received for {time_since_last_packet}ms (timeout: {self._packet_timeout_ms}ms)")
            logger.info("Attempting automatic reconnection...")
            
            # Attempt automatic reconnection
            self.disconnect()
            
            # Give socket time to clean up
            QtCore.QTimer.singleShot(500, self._attempt_reconnect)
    
    def _attempt_reconnect(self) -> None:
        """Attempt to reconnect to controller after timeout."""
        logger.info(f"Reconnecting to {self.host}:{self.port}...")
        if self.connect():
            logger.info("Reconnection successful")
            # Restart telemetry on successful reconnect
            if not self.start_telemetry():
                logger.error("Failed to restart telemetry after reconnect")
        else:
            logger.error("Reconnection failed, will retry on next timeout")
    
    def set_packet_timeout_enabled(self, enabled: bool) -> None:
        """Enable or disable packet timeout check.
        
        Useful for disabling timeout during long operations like parameter loading.
        
        Args:
            enabled: True to enable timeout check, False to disable
        """
        self._packet_timeout_enabled = enabled
        if enabled:
            # Reset timer when re-enabling
            self._last_packet_time = QtCore.QDateTime.currentMSecsSinceEpoch()
            logger.debug("Packet timeout check enabled")
        else:
            logger.debug("Packet timeout check disabled")
    
    def _handle_connection_error(self, reason: str) -> None:
        """Handle connection error and disconnect.
        
        Args:
            reason: Error reason for logging
        """
        error_msg = f"Connection error: {reason}"
        logger.error(error_msg)
        self.error_occurred.emit(error_msg)
        self.disconnect()
    
    def _process_telemetry_packet(self) -> bool:
        """Process telemetry packet from buffer.
        
        Returns:
            True if packet was consumed (or should be removed), False if need more data
        """
        try:
            # Try to decode telemetry packet
            result = self._telemetry_decoder.decode(bytes(self._recv_buffer), verbose=self.verbose)
            
            if result:
                packet, bytes_consumed = result
                
                if self.verbose:
                    logger.debug(f"Successfully decoded packet: consumed {bytes_consumed} bytes, packet type 0x{packet.header.packet_type:02x}, seq={packet.header.sequence_number}")
                
                # Successfully decoded - remove consumed bytes from buffer
                del self._recv_buffer[:bytes_consumed]
                
                # Update statistics
                self._packets_received += 1
                
                if self.verbose and self._packets_received % 100 == 0:
                    logger.debug(f"Received {self._packets_received} telemetry packets (seq={packet.header.sequence_number})")
                
                # Check for sequence gaps (but don't report as "lost" since server
                # intentionally skips telemetry cycles when sending parameter responses)
                if self._last_sequence >= 0:
                    expected_seq = (self._last_sequence + 1) & 0xFFFFFFFF
                    if packet.header.sequence_number != expected_seq:
                        gap = (packet.header.sequence_number - expected_seq) & 0xFFFFFFFF
                        # Update loss counter but don't log warning
                        # These are intentional skips, not actual packet loss
                        self._packets_lost += gap
                        if self.verbose:
                            logger.debug(f"Sequence gap of {gap} (expected {expected_seq}, got {packet.header.sequence_number}) - likely parameter response")
                
                self._last_sequence = packet.header.sequence_number
                
                # Reset error counter on successful decode
                self._consecutive_errors = 0
                
                # Emit signal
                self.telemetry_packet_received.emit(packet)
                
                return True
            else:
                # Need more data
                if self.verbose:
                    logger.debug(f"Decoder needs more data (buffer has {len(self._recv_buffer)} bytes)")
                return False
        
        except Exception as e:
            self._decode_errors += 1
            self._consecutive_errors += 1
            logger.error(f"Telemetry decode error ({self._decode_errors} total): {e}", exc_info=True)
            
            # Try to recover by discarding one byte and continuing
            if len(self._recv_buffer) > 0:
                del self._recv_buffer[0]
            
            return True  # Consumed (discarded) data
    
    def _process_parameter_response(self, packet_type: int) -> bool:
        """Process parameter response from buffer.
        
        Args:
            packet_type: Packet type byte from header
            
        Returns:
            True if packet was consumed (or should be removed), False if need more data
        """
        try:
            # Need at least 16 bytes for header to validate
            if len(self._recv_buffer) < 16:
                return False  # Need more data
            
            # Validate protocol version (byte 1) before attempting decode
            protocol_version = self._recv_buffer[1]
            if protocol_version != PROTOCOL_VERSION:
                # This is likely telemetry data being misidentified as a parameter packet
                # Don't spam warnings - this happens with batched telemetry packets
                if self.verbose:
                    logger.debug(f"Invalid protocol version 0x{protocol_version:02x} for parameter packet type 0x{packet_type:02x}, likely telemetry data")
                
                # Search for next sync frame or valid packet within reasonable distance
                found_valid = False
                for i in range(1, min(64, len(self._recv_buffer))):
                    # Check for sync frame
                    if i + 3 < len(self._recv_buffer):
                        if self._telemetry_decoder.is_sync_frame(bytes(self._recv_buffer[i:i+4])):
                            del self._recv_buffer[:i]  # Discard up to sync frame
                            found_valid = True
                            break
                    # Check for valid telemetry packet type
                    if self._recv_buffer[i] in [0x01, 0x02] and i + 1 < len(self._recv_buffer) and self._recv_buffer[i+1] == PROTOCOL_VERSION:
                        del self._recv_buffer[:i]  # Discard up to telemetry packet
                        found_valid = True
                        break
                    # Check for valid parameter packet type
                    elif self._recv_buffer[i] in [0x41, 0x47, 0x49] and i + 1 < len(self._recv_buffer) and self._recv_buffer[i+1] == PROTOCOL_VERSION:
                        del self._recv_buffer[:i]  # Discard up to parameter packet
                        found_valid = True
                        break
                
                if not found_valid:
                    # No valid marker found in search window - discard search window
                    discard_count = min(64, len(self._recv_buffer))
                    del self._recv_buffer[:discard_count]
                
                return True
            
            response_data = None
            bytes_consumed = 0
            
            if packet_type == 0x47:  # COUNT_RESPONSE
                result = self._param_decoder.decode_count_response(bytes(self._recv_buffer))
                if result:
                    total_params, bytes_consumed = result
                    response_data = {
                        'type': 'count_response',
                        'total_parameters': total_params
                    }
                    del self._recv_buffer[:bytes_consumed]
            
            elif packet_type == 0x49:  # GET_BY_INDEX_RESPONSE
                result = self._param_decoder.decode_get_by_index_response(bytes(self._recv_buffer))
                if result:
                    metadata, bytes_consumed = result
                    
                    # Validate metadata for garbage data
                    if not self._validate_parameter_metadata(metadata):
                        logger.warning(f"Received invalid parameter metadata (garbage data), discarding packet")
                        del self._recv_buffer[:bytes_consumed]
                        return True  # Consumed but don't emit
                    
                    response_data = {
                        'type': 'get_by_index_response',
                        'metadata': metadata
                    }
                    del self._recv_buffer[:bytes_consumed]
            
            elif packet_type == 0x41:  # SET_RESPONSE
                result = self._param_decoder.decode_set_response(bytes(self._recv_buffer))
                if result:
                    response_dict, bytes_consumed = result
                    response_data = {
                        'type': 'set_response',
                        'component': response_dict['component'],
                        'name': response_dict['name'],
                        'status': response_dict['status'],
                        'old_value': response_dict['old_value'],
                        'new_value': response_dict['new_value'],
                        'error_message': response_dict['error_message']
                    }
                    del self._recv_buffer[:bytes_consumed]
            
            if response_data:
                # Emit signal to UI
                self.parameter_response_received.emit(response_data)
                self._consecutive_errors = 0
                return True
            else:
                # Not enough data yet
                if len(self._recv_buffer) > 1024:
                    # Too much buffered data - something is wrong
                    logger.error(f"Parameter response decode failed with large buffer, clearing (packet_type=0x{packet_type:02x}, buffer_size={len(self._recv_buffer)})")
                    self._recv_buffer.clear()
                    self._consecutive_errors += 1
                    return True
                return False  # Need more data
        
        except Exception as e:
            logger.error(f"Error processing parameter response: {e}", exc_info=True)
            self._consecutive_errors += 1
            # Discard one byte and try to recover
            if len(self._recv_buffer) > 0:
                del self._recv_buffer[0]
            return True
    
    def _validate_parameter_metadata(self, metadata) -> bool:
        """Validate parameter metadata to detect garbage data.
        
        Args:
            metadata: ParameterMetadata object to validate
            
        Returns:
            True if metadata appears valid, False if likely garbage
        """
        try:
            # Check for reasonable index bounds
            if metadata.parameter_index < 0 or metadata.parameter_index > 10000:
                logger.debug(f"Invalid parameter_index: {metadata.parameter_index}")
                return False
            
            if metadata.total_parameters < 0 or metadata.total_parameters > 10000:
                logger.debug(f"Invalid total_parameters: {metadata.total_parameters}")
                return False
            
            # Check that index is within total
            if metadata.parameter_index >= metadata.total_parameters:
                logger.debug(f"Index {metadata.parameter_index} >= total {metadata.total_parameters}")
                return False
            
            # Check for printable ASCII strings (no null bytes in middle, reasonable length)
            if not metadata.component or len(metadata.component) > 63:
                logger.debug(f"Invalid component: '{metadata.component}'")
                return False
            
            if not metadata.name or len(metadata.name) > 63:
                logger.debug(f"Invalid name: '{metadata.name}'")
                return False
            
            # Check for non-printable characters in critical fields
            if not all(32 <= ord(c) <= 126 or c == '\x00' for c in metadata.component):
                logger.debug(f"Non-printable chars in component")
                return False
            
            if not all(32 <= ord(c) <= 126 or c == '\x00' for c in metadata.name):
                logger.debug(f"Non-printable chars in name")
                return False
            
            # Check for reasonable numeric values (not NaN or Inf)
            import math
            if math.isnan(metadata.current_value) or math.isinf(metadata.current_value):
                logger.debug(f"Invalid current_value: {metadata.current_value}")
                return False
            
            if math.isnan(metadata.min_value) or math.isinf(metadata.min_value):
                logger.debug(f"Invalid min_value: {metadata.min_value}")
                return False
            
            if math.isnan(metadata.max_value) or math.isinf(metadata.max_value):
                logger.debug(f"Invalid max_value: {metadata.max_value}")
                return False
            
            return True
            
        except Exception as e:
            logger.debug(f"Exception during metadata validation: {e}")
            return False
    
    def get_statistics(self) -> dict:
        """Get reception statistics.
        
        Returns:
            Dictionary with statistics
        """
        total = self._packets_received + self._packets_lost
        loss_rate = (self._packets_lost / total * 100.0) if total > 0 else 0.0
        
        return {
            'packets_received': self._packets_received,
            'packets_lost': self._packets_lost,
            'loss_rate_percent': loss_rate,
            'decode_errors': self._decode_errors
        }
