#!/usr/bin/env python3
"""Network client for parameter configuration.

This module provides the ParameterClient class which handles TCP communication
for parameter configuration (setting and querying control parameters).

Architecture:
    - Separate from telemetry client (can use same or different port)
    - Request-response pattern (not streaming like telemetry)
    - Thread-safe parameter cache
    - Automatic parameter discovery from controller

Example Usage:
    client = ParameterClient('192.168.2.100', 7000)
    if client.connect():
        # Discover all parameters
        params = client.get_all_parameters()
        # print(f"Found {len(params)} parameters")
        
        # Set a parameter
        success = client.set_parameter('grid_observer', 'kp', 0.5)
        
        # Query current value
        value = client.get_parameter('grid_observer', 'kp')
        # print(f"Current Kp: {value.value}")
"""

import errno
import logging
import socket
import threading
import time
from typing import Dict, List, Optional, Tuple

from .parameter_protocol import (
    ParameterMetadata,
    ParameterValue,
    ParameterProtocolEncoder,
    ParameterProtocolDecoder,
    ParameterSetStatus
)


logger = logging.getLogger(__name__)


class ParameterClient:
    """TCP client for parameter configuration.
    
    Handles connection management, parameter discovery, and parameter
    get/set operations with the embedded controller.
    
    Attributes:
        host: Controller IP address
        port: Controller TCP port
        connected: Connection status
        
    Example:
        client = ParameterClient('192.168.2.100', 7000)
        client.connect()
        
        # Discover parameters
        params = client.get_all_parameters()
        
        # Set parameter
        client.set_parameter('grid_observer', 'kp', 0.5)
        
        client.disconnect()
    """
    
    def __init__(self, host: str = '192.168.2.100', port: int = 7000, timeout: float = 5.0, verbose: bool = False):
        """Initialize parameter client.
        
        Args:
            host: Controller IP address
            port: Controller TCP port (same as telemetry or separate)
            timeout: Socket timeout for request-response operations
            verbose: Enable verbose debug logging
        """
        self.host = host
        self.port = port
        self.timeout = timeout
        self.verbose = verbose
        self.connected = False
        
        self._socket: Optional[socket.socket] = None
        self._external_socket: Optional[socket.socket] = None  # Socket provided externally
        self._encoder = ParameterProtocolEncoder()
        self._decoder = ParameterProtocolDecoder()
        self._previous_socket_timeout: Optional[float] = None
        
        # Parameter cache: {(component, name): ParameterMetadata}
        self._parameter_cache: Dict[Tuple[str, str], ParameterMetadata] = {}
        self._cache_lock = threading.Lock()
    
    def set_external_socket(self, sock: socket.socket) -> None:
        """Use an external socket connection instead of creating own.
        
        This allows sharing a single TCP connection for both telemetry
        and parameter operations, which is required when the server only
        supports one client connection at a time.
        
        Args:
            sock: Active socket.socket connected to the controller
        """
        self._external_socket = sock
        self.connected = True
        logger.info(f"Using external socket for parameter operations")
    
    def connect(self) -> bool:
        """Connect to embedded controller.
        
        Returns:
            True if connection successful, False otherwise
        """
        if self._external_socket:
            logger.warning("Using external socket - connect() not needed")
            return True
            
        if self.connected:
            logger.warning("Already connected")
            return True
        
        try:
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._socket.settimeout(self.timeout)
            self._socket.connect((self.host, self.port))
            
            self.connected = True
            logger.info(f"Connected to {self.host}:{self.port} for parameter configuration")
            return True
            
        except (socket.error, OSError) as e:
            logger.error(f"Connection failed: {e}")
            self.connected = False
            if self._socket:
                self._socket.close()
                self._socket = None
            return False
    
    def disconnect(self) -> None:
        """Disconnect from controller."""
        if self._external_socket:
            # Don't close external socket - managed elsewhere
            logger.info("Releasing external socket (not closing)")
            self._external_socket = None
            self.connected = False
            return
            
        if not self.connected:
            return
        
        self.connected = False
        
        if self._socket:
            try:
                self._socket.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self._socket.close()
            self._socket = None
        
        logger.info("Disconnected")
    
    def _get_active_socket(self) -> Optional[socket.socket]:
        """Get the active socket (external or internal).
        
        Returns:
            Active socket, or None if not connected
        """
        return self._external_socket if self._external_socket else self._socket

    def _flush_pending_data(self, sock: socket.socket) -> None:
        """Drain any pending data from the shared socket before a transaction."""
        # Temporarily switch to non-blocking to flush without waiting
        sock.setblocking(False)
        total_flushed = 0
        try:
            while True:
                try:
                    data = sock.recv(4096)
                    if not data:
                        break
                    total_flushed += len(data)
                except BlockingIOError:
                    break
                except socket.error as err:
                    if err.errno in (errno.EWOULDBLOCK, errno.EAGAIN):
                        break
                    raise
        finally:
            sock.setblocking(True)
            if total_flushed > 0:
                logger.info(f"Flushed {total_flushed} bytes of stale data from socket before parameter transaction")

    def _prepare_socket_for_transaction(self) -> socket.socket:
        """Prepare socket for a request/response transaction."""
        active_socket = self._get_active_socket()
        if not self.connected or not active_socket:
            raise RuntimeError("Not connected to controller")

        self._previous_socket_timeout = active_socket.gettimeout()
        if self._external_socket:
            # Flush any telemetry bytes left in the buffer before parameter exchange
            self._flush_pending_data(active_socket)

        active_socket.settimeout(self.timeout)
        return active_socket

    def _restore_socket_after_transaction(self, sock: socket.socket) -> None:
        """Restore socket timeout/settings after transaction."""
        if self._previous_socket_timeout is None:
            sock.settimeout(None)
        else:
            sock.settimeout(self._previous_socket_timeout)
        self._previous_socket_timeout = None
    
    def get_all_parameters(self, force_refresh: bool = False, max_retries: int = 3) -> List[ParameterMetadata]:
        """Request full parameter catalog from controller.
        
        Uses robust two-phase protocol:
        1. Get parameter count
        2. Request each parameter individually by index with retry on failure
        
        Args:
            force_refresh: If True, clear cache and re-fetch from controller
            max_retries: Maximum retry attempts per parameter (default: 3)
        
        Returns:
            List of ParameterMetadata objects
            
        Raises:
            RuntimeError: If not connected or communication fails
        """
        if not force_refresh and self._parameter_cache:
            with self._cache_lock:
                return list(self._parameter_cache.values())

        active_socket = self._prepare_socket_for_transaction()
        try:
            logger.info("Requesting parameter catalog from controller...")
            
            # Phase 1: Get total parameter count
            total_params = self._get_parameter_count_with_retry(active_socket, max_retries)
            if total_params is None:
                logger.error("Failed to get parameter count after retries")
                return []
            
            logger.info(f"Controller reports {total_params} parameters available")
            
            # Phase 2: Request each parameter by index
            parameters = []
            failed_indices = []
            
            for index in range(total_params):
                param = self._get_parameter_by_index_with_retry(active_socket, index, max_retries)
                if param:
                    parameters.append(param)
                    logger.debug(f"Received parameter {index}: {param.component}.{param.name}")
                    
                    # Store in cache
                    with self._cache_lock:
                        self._parameter_cache[(param.component, param.name)] = param
                else:
                    logger.warning(f"Failed to get parameter at index {index} after {max_retries} retries")
                    failed_indices.append(index)
            
            if failed_indices:
                logger.warning(f"Failed to retrieve {len(failed_indices)} parameters: {failed_indices}")
            
            logger.info(f"Parameter catalog: {len(parameters)}/{total_params} parameters loaded")
            return parameters
        finally:
            self._restore_socket_after_transaction(active_socket)
    
    def _get_parameter_count_with_retry(self, sock: socket.socket, max_retries: int) -> Optional[int]:
        """Get parameter count with retry logic.
        
        Args:
            sock: Active socket already prepared for the transaction
            max_retries: Maximum number of retry attempts
        
        Returns:
            Parameter count, or None if all retries failed
        """
        for attempt in range(max_retries):
            try:
                # Send COUNT request
                request = self._encoder.encode_count_request()
                
                if self.verbose:
                    logger.info(f"[COUNT] Sending request (attempt {attempt + 1}/{max_retries}): {len(request)} bytes")
                    logger.info(f"[COUNT] Request hex: {request.hex()}")
                
                sock.sendall(request)
                
                if self.verbose:
                    logger.info(f"[COUNT] Waiting for response (timeout={self.timeout}s)...")
                
                # Receive response with timeout
                response = sock.recv(1024)
                
                if self.verbose:
                    logger.info(f"[COUNT] Received {len(response)} bytes")
                    if response:
                        logger.info(f"[COUNT] Response hex: {response.hex()}")
                
                if not response:
                    logger.warning(f"Empty response on COUNT attempt {attempt + 1}/{max_retries}")
                    # Don't break - retry with delay
                    time.sleep(0.1)
                    continue
                
                result = self._decoder.decode_count_response(response)
                if result is not None:
                    count, _ = result
                    if self.verbose:
                        logger.info(f"[COUNT] Decoded count: {count}")
                    return count
                else:
                    logger.warning(f"Failed to decode COUNT response on attempt {attempt + 1}/{max_retries}")
                    if self.verbose and response:
                        logger.info(f"[COUNT] Raw response (first 32 bytes): {response[:32].hex()}")
                    else:
                        # Log even in non-verbose mode when decode fails
                        logger.info(f"[COUNT] Decode failed. Response length: {len(response)}, first 32 bytes: {response[:32].hex()}")
                    # Retry with delay
                    time.sleep(0.1)
            
            except socket.timeout:
                logger.warning(f"Timeout on COUNT attempt {attempt + 1}/{max_retries}")
                # Continue retrying on timeout
                time.sleep(0.1)
            except (socket.error, OSError) as e:
                logger.error(f"Socket error on COUNT attempt {attempt + 1}/{max_retries}: {e}")
                # Still retry - socket might recover
                time.sleep(0.2)
                if attempt == max_retries - 1:
                    # Last attempt - give up
                    return None
        
        return None
    
    def _get_parameter_by_index_with_retry(self, sock: socket.socket, index: int, max_retries: int) -> Optional[ParameterMetadata]:
        """Get parameter by index with retry logic.
        
        Args:
            sock: Active socket already prepared for the transaction
            index: Zero-based parameter index
            max_retries: Maximum number of retry attempts
        
        Returns:
            ParameterMetadata, or None if all retries failed
        """
        for attempt in range(max_retries):
            try:
                # Send GET_BY_INDEX request
                request = self._encoder.encode_get_by_index_request(index)
                sock.sendall(request)
                
                # Receive response with timeout
                # Expected size: 16 (header) + 380 (payload) = 396 bytes
                response = sock.recv(512)
                if not response:
                    logger.warning(f"Empty response for index {index} attempt {attempt + 1}/{max_retries}")
                    time.sleep(0.05)
                    continue
                
                result = self._decoder.decode_get_by_index_response(response)
                if result:
                    param, _ = result
                    return param
                else:
                    logger.warning(f"Failed to decode index {index} response on attempt {attempt + 1}/{max_retries}")
                    time.sleep(0.05)
            
            except socket.timeout:
                logger.warning(f"Timeout for index {index} attempt {attempt + 1}/{max_retries}")
                time.sleep(0.05)
            except (socket.error, OSError) as e:
                logger.error(f"Socket error for index {index} attempt {attempt + 1}/{max_retries}: {e}")
                # Continue retrying even on socket errors - might be transient
                time.sleep(0.1)
                if attempt == max_retries - 1:
                    # Last attempt - give up on this parameter
                    return None
        
        return None
    
    def get_all_parameters_legacy(self, force_refresh: bool = False) -> List[ParameterMetadata]:
        """Request full parameter catalog using legacy LIST protocol.
        
        DEPRECATED: Use get_all_parameters() which uses the robust COUNT + GET_BY_INDEX protocol.
        
        Sends PARAM_LIST_REQUEST and receives multiple PARAM_LIST_RESPONSE
        packets (one per parameter). Results are cached for future queries.
        
        Args:
            force_refresh: If True, clear cache and re-fetch from controller
        
        Returns:
            List of ParameterMetadata objects
            
        Raises:
            RuntimeError: If not connected or communication fails
        """
        if not force_refresh and self._parameter_cache:
            with self._cache_lock:
                return list(self._parameter_cache.values())

        active_socket = self._prepare_socket_for_transaction()
        try:
            logger.info("Requesting parameter catalog from controller...")
            
            # Send LIST request
            request = self._encoder.encode_list_request()
            active_socket.sendall(request)
            
            parameters = []
            total_expected = None
            
            # Receive responses (one per parameter)
            while True:
                try:
                    response = active_socket.recv(2048)
                    if not response:
                        logger.error("Socket closed during parameter list reception")
                        break
                    
                    param = self._decoder.decode_list_response(response)
                    if param:
                        parameters.append(param)
                        logger.debug(f"Received parameter {param.parameter_index + 1}: {param.component}.{param.name}")
                        
                        # Store in cache
                        with self._cache_lock:
                            self._parameter_cache[(param.component, param.name)] = param
                        
                        # Check if this is the last parameter
                        if total_expected is None:
                            total_expected = param.total_parameters
                        
                        if param.parameter_index + 1 >= total_expected:
                            logger.info(f"Received all {total_expected} parameters")
                            break
                    else:
                        logger.warning("Failed to decode parameter response")
                
                except socket.timeout:
                    logger.error("Timeout waiting for parameter response")
                    break
                except (socket.error, OSError) as e:
                    logger.error(f"Socket error during parameter list: {e}")
                    break
            
            logger.info(f"Parameter catalog: {len(parameters)} parameters loaded")
            return parameters
        finally:
            self._restore_socket_after_transaction(active_socket)
    
    def get_parameter(self, component: str, name: str) -> Optional[ParameterValue]:
        """Query current value of a parameter.
        
        Args:
            component: Component name (e.g., "grid_observer")
            name: Parameter name (e.g., "kp")
        
        Returns:
            ParameterValue with current value, or None if failed
            
        Raises:
            RuntimeError: If not connected
        """
        logger.debug(f"Querying parameter {component}.{name}")

        active_socket = self._prepare_socket_for_transaction()
        try:
            # Send GET request
            request = self._encoder.encode_get_request(component, name)
            active_socket.sendall(request)
            
            # Receive response
            response = active_socket.recv(1024)
            param_value = self._decoder.decode_get_response(response)
            
            if param_value:
                logger.debug(f"Got value: {param_value.value}")
                
                # Attach metadata from cache if available
                with self._cache_lock:
                    if (component, name) in self._parameter_cache:
                        param_value.metadata = self._parameter_cache[(component, name)]
                
                return param_value
            logger.error(f"Failed to decode GET response for {component}.{name}")
            return None
        except socket.timeout:
            logger.error(f"Timeout waiting for GET response: {component}.{name}")
            return None
        except (socket.error, OSError) as e:
            logger.error(f"Socket error during GET: {e}")
            return None
        finally:
            self._restore_socket_after_transaction(active_socket)
    
    def set_parameter(self, component: str, name: str, value: float) -> bool:
        """Set parameter value on controller.
        
        Args:
            component: Component name (e.g., "grid_observer")
            name: Parameter name (e.g., "kp")
            value: New value to set
        
        Returns:
            True if set successful, False otherwise
            
        Raises:
            RuntimeError: If not connected
        """
        # Validate against cached metadata if available
        with self._cache_lock:
            if (component, name) in self._parameter_cache:
                meta = self._parameter_cache[(component, name)]
                if not (meta.min_value <= value <= meta.max_value):
                    logger.error(
                        f"Value {value} out of range [{meta.min_value}, {meta.max_value}] "
                        f"for {component}.{name}"
                    )
                    return False
        
        logger.info(f"Setting parameter {component}.{name} = {value}")

        active_socket = self._prepare_socket_for_transaction()
        try:
            # Send SET request
            request = self._encoder.encode_set_request(component, name, value)
            active_socket.sendall(request)
            
            # Receive response
            response = active_socket.recv(1024)
            decode_result = self._decoder.decode_set_response(response)
            
            if decode_result:
                result, _ = decode_result
                status = result['status']
                
                if status == ParameterSetStatus.SUCCESS:
                    logger.info(
                        f"✓ Set {component}.{name} = {result['new_value']} "
                        f"(was {result['old_value']})"
                    )
                    
                    # Update cache
                    with self._cache_lock:
                        if (component, name) in self._parameter_cache:
                            self._parameter_cache[(component, name)].current_value = result['new_value']
                    
                    return True
                logger.error(
                    f"✗ Failed to set {component}.{name}: "
                    f"{status.name} - {result['error_message']}"
                )
                return False
            logger.error(f"Failed to decode SET response for {component}.{name}")
            return False
        except socket.timeout:
            logger.error(f"Timeout waiting for SET response: {component}.{name}")
            return False
        except (socket.error, OSError) as e:
            logger.error(f"Socket error during SET: {e}")
            return False
        finally:
            self._restore_socket_after_transaction(active_socket)
    
    def get_component_parameters(self, component: str) -> List[ParameterMetadata]:
        """Get all parameters for a specific component.
        
        Args:
            component: Component name (e.g., "grid_observer")
        
        Returns:
            List of ParameterMetadata for the component
        """
        with self._cache_lock:
            return [
                meta for (comp, name), meta in self._parameter_cache.items()
                if comp == component
            ]
    
    def get_all_components(self) -> List[str]:
        """Get list of all component names.
        
        Returns:
            List of unique component names
        """
        with self._cache_lock:
            return sorted(set(comp for comp, name in self._parameter_cache.keys()))
    
    def clear_cache(self) -> None:
        """Clear parameter cache (force refresh on next get_all_parameters)."""
        with self._cache_lock:
            self._parameter_cache.clear()
        logger.info("Parameter cache cleared")
