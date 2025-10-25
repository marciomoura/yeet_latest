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

from .telemetry_protocol import GridSideTelemetryPacket, TelemetryDecoder
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
    
    def __init__(self, host: str = '192.168.2.100', port: int = 7000, verbose: bool = False, parent=None):
        """Initialize unified network client.
        
        Args:
            host: Controller IP address
            port: Controller TCP port
            verbose: Enable verbose logging
            parent: Qt parent object
        """
        super().__init__(parent)
        
        self.host = host
        self.port = port
        self.verbose = verbose
        
        self._socket: Optional[socket.socket] = None
        self._state = NetworkState.DISCONNECTED
        
        # Protocol handlers
        self._telemetry_decoder = TelemetryDecoder()
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
        
        # Resilience tracking
        self._consecutive_errors = 0
        self._max_consecutive_errors = 10  # Disconnect after 10 consecutive errors
        self._last_packet_time = 0
        self._packet_timeout_ms = 5000  # 5 seconds without packets = timeout
        self._recv_buffer_max_size = 65536  # 64KB max buffer to prevent memory issues
        
        # Qt timer for polling socket
        self._poll_timer = QtCore.QTimer(self)
        self._poll_timer.timeout.connect(self._poll_socket)
        self._poll_timer.setInterval(5)  # 5ms polling = 200Hz max
        
        # Qt timer for connection health check
        self._health_timer = QtCore.QTimer(self)
        self._health_timer.timeout.connect(self._check_connection_health)
        self._health_timer.setInterval(1000)  # Check every second
        
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
        
        # Close socket
        self._cleanup_socket()
        
        self._state = NetworkState.DISCONNECTED
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
            # Read available data (non-blocking)
            data = self._socket.recv(4096)
            
            if not data:
                # Connection closed by remote
                logger.warning("Connection closed by controller")
                self._handle_connection_error("Connection closed by remote")
                return
            
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
            
            # Process packets in buffer
            self._process_receive_buffer()
            
        except BlockingIOError:
            # No data available - normal for non-blocking socket
            pass
        except ConnectionResetError as e:
            logger.error(f"Connection reset by peer: {e}")
            self._handle_connection_error("Connection reset")
        except ConnectionAbortedError as e:
            logger.error(f"Connection aborted: {e}")
            self._handle_connection_error("Connection aborted")
        except OSError as e:
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
    
    def _check_connection_health(self) -> None:
        """Check connection health periodically."""
        if not self.connected:
            return
        
        # Check if we've received data recently
        current_time = QtCore.QDateTime.currentMSecsSinceEpoch()
        time_since_last_packet = current_time - self._last_packet_time
        
        if time_since_last_packet > self._packet_timeout_ms:
            logger.warning(f"No packets received for {time_since_last_packet}ms (timeout: {self._packet_timeout_ms}ms)")
            self._handle_connection_error("Packet timeout")
    
    def _handle_connection_error(self, reason: str) -> None:
        """Handle connection error and disconnect.
        
        Args:
            reason: Error reason for logging
        """
        error_msg = f"Connection error: {reason}"
        logger.error(error_msg)
        self.error_occurred.emit(error_msg)
        self.disconnect()
    
    def _process_receive_buffer(self) -> None:
        """Process packets in receive buffer.
        
        Stateless processing - displays whatever arrives from server.
        Server manages telemetry state and packet sequencing.
        """
        # Limit iterations to prevent infinite loop on malformed data
        max_iterations = 100
        iteration = 0
        
        while len(self._recv_buffer) >= 16 and iteration < max_iterations:  # Minimum header size
            iteration += 1
            
            # Peek at packet type from header
            packet_type = self._recv_buffer[0]
            
            # Route based on packet type
            if packet_type in [0x01, 0x02]:  # Telemetry packets
                consumed = self._process_telemetry_packet()
                if not consumed:
                    # Not enough data for complete packet
                    break
            elif packet_type in [0x41, 0x47, 0x49]:  # Parameter response packets
                consumed = self._process_parameter_response(packet_type)
                if not consumed:
                    # Not enough data for complete packet
                    break
            else:
                # Unknown packet type - try to resync
                logger.warning(f"Unknown packet type 0x{packet_type:02x} at buffer start, attempting resync")
                
                # Look for valid packet type in next bytes (limited search)
                found_sync = False
                for i in range(1, min(100, len(self._recv_buffer))):
                    if self._recv_buffer[i] in [0x01, 0x02, 0x41, 0x47, 0x49]:
                        logger.info(f"Found potential sync at offset {i}, discarding {i} bytes")
                        del self._recv_buffer[:i]
                        found_sync = True
                        break
                
                if not found_sync:
                    logger.error("Could not resync, clearing entire buffer")
                    self._recv_buffer.clear()
                    self._consecutive_errors += 1
                break
        
        if iteration >= max_iterations:
            logger.warning(f"Buffer processing hit max iterations ({max_iterations}), possible packet storm")
            # Don't clear buffer, but stop processing for this poll cycle
    
    def _process_telemetry_packet(self) -> bool:
        """Process telemetry packet from buffer.
        
        Returns:
            True if packet was consumed (or should be removed), False if need more data
        """
        try:
            # Try to decode telemetry packet
            result = self._telemetry_decoder.decode(bytes(self._recv_buffer))
            
            if result:
                packet, bytes_consumed = result
                
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
