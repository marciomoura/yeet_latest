#!/usr/bin/env python3
"""Parameter configuration protocol for SFC control system.

This module defines the parameter configuration protocol structures and provides
encoding/decoding functionality for setting and querying control parameters
on the embedded controller.

Protocol Structure:
    - Request packets: Host → Controller
    - Response packets: Controller → Host
    - Uses same header format as telemetry protocol

Command Types:
    - PARAM_LIST_REQUEST: Get catalog of all available parameters
    - PARAM_LIST_RESPONSE: Parameter metadata (name, type, range, unit)
    - PARAM_GET_REQUEST: Query current value of a parameter
    - PARAM_GET_RESPONSE: Current parameter value
    - PARAM_SET_REQUEST: Set new parameter value
    - PARAM_SET_RESPONSE: Confirmation or error

Example Usage:
    encoder = ParameterProtocolEncoder()
    decoder = ParameterProtocolDecoder()
    
    # Request parameter list
    request = encoder.encode_list_request()
    sock.send(request)
    
    # Decode response
    response = sock.recv(1024)
    metadata = decoder.decode_list_response(response)
    # print(f"Parameter: {metadata.name}, Range: [{metadata.min_value}, {metadata.max_value}]")
"""

import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional, List


# Protocol constants (match C++ telemetry.hpp)
PROTOCOL_VERSION = 2

# Packet types (extending telemetry protocol)
class PacketType(IntEnum):
    """Parameter protocol packet type identifiers."""
    PARAM_SET_REQUEST = 0x40       # Host → Controller: Set parameter
    PARAM_SET_RESPONSE = 0x41      # Controller → Host: Confirm/reject
    PARAM_GET_REQUEST = 0x42       # Host → Controller: Query parameter value
    PARAM_GET_RESPONSE = 0x43      # Controller → Host: Parameter value
    PARAM_LIST_REQUEST = 0x44      # Host → Controller: Get all parameters (DEPRECATED)
    PARAM_LIST_RESPONSE = 0x45     # Controller → Host: Parameter metadata (DEPRECATED)
    PARAM_COUNT_REQUEST = 0x46     # Host → Controller: Get parameter count
    PARAM_COUNT_RESPONSE = 0x47    # Controller → Host: Total parameter count
    PARAM_GET_BY_INDEX_REQUEST = 0x48  # Host → Controller: Get parameter by index
    PARAM_GET_BY_INDEX_RESPONSE = 0x49 # Controller → Host: Parameter metadata by index


# Parameter value types (match C++ enum)
class ParameterValueType(IntEnum):
    """Parameter data type identifiers."""
    FLOAT32 = 0
    INT32 = 1
    BOOLEAN = 2
    ENUM_VALUE = 3


# Parameter SET response status codes
class ParameterSetStatus(IntEnum):
    """Status codes for parameter SET response."""
    SUCCESS = 0
    NOT_FOUND = 1
    OUT_OF_RANGE = 2
    READ_ONLY = 3
    INVALID_TYPE = 4


@dataclass
class ParameterMetadata:
    """Parameter metadata structure.
    
    Describes a single parameter including its name, type, constraints,
    and current value.
    
    Attributes:
        component: Component name (e.g., "grid_observer")
        name: Parameter name within component (e.g., "kp")
        display_name: Human-readable name (e.g., "PLL Proportional Gain")
        unit: Unit string (e.g., "Hz", "pu", "rad")
        type: Parameter value type (float, int, bool, enum)
        min_value: Minimum allowed value
        max_value: Maximum allowed value
        default_value: Default/initial value
        current_value: Current runtime value
        description: Detailed parameter description
        parameter_index: Index in full parameter list
        total_parameters: Total parameter count
    """
    component: str
    name: str
    display_name: str
    unit: str
    type: ParameterValueType
    min_value: float
    max_value: float
    default_value: float
    current_value: float
    description: str
    parameter_index: int = 0
    total_parameters: int = 0
    
    @property
    def full_name(self) -> str:
        """Get fully qualified parameter name (component.name)."""
        return f"{self.component}.{self.name}"


@dataclass
class ParameterValue:
    """Parameter value with optional metadata.
    
    Attributes:
        component: Component name
        name: Parameter name
        value: Current value
        metadata: Optional full metadata
    """
    component: str
    name: str
    value: float
    metadata: Optional[ParameterMetadata] = None
    
    @property
    def full_name(self) -> str:
        """Get fully qualified parameter name (component.name)."""
        return f"{self.component}.{self.name}"


class ParameterProtocolEncoder:
    """Encoder for parameter protocol requests (Host → Controller)."""
    
    # Header format: packet_type(1) + version(1) + payload_length(2) + flags(4) + timestamp(4) + sequence(4)
    HEADER_FORMAT = '<BBHIII'
    HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
    
    def __init__(self):
        """Initialize encoder."""
        self._sequence = 0
    
    def encode_list_request(self) -> bytes:
        """Create binary PARAM_LIST_REQUEST packet.
        
        Requests full parameter catalog from controller. Controller
        will respond with multiple PARAM_LIST_RESPONSE packets (one per parameter).
        
        Returns:
            Binary packet ready to send over TCP
        """
        # Simple header-only packet
        header = struct.pack(
            self.HEADER_FORMAT,
            PacketType.PARAM_LIST_REQUEST,
            PROTOCOL_VERSION,
            0,  # payload_length
            0,  # flags (unused)
            0,  # timestamp (unused)
            self._sequence
        )
        self._sequence += 1
        return header
    
    def encode_count_request(self) -> bytes:
        """Create binary PARAM_COUNT_REQUEST packet.
        
        Requests total number of parameters from controller.
        
        Returns:
            Binary packet ready to send over TCP
        """
        header = struct.pack(
            self.HEADER_FORMAT,
            PacketType.PARAM_COUNT_REQUEST,
            PROTOCOL_VERSION,
            0,  # payload_length
            0,  # flags (unused)
            0,  # timestamp (unused)
            self._sequence
        )
        self._sequence += 1
        return header
    
    def encode_get_by_index_request(self, index: int) -> bytes:
        """Create binary PARAM_GET_BY_INDEX_REQUEST packet.
        
        Requests parameter metadata by zero-based index.
        
        Args:
            index: Zero-based parameter index
        
        Returns:
            Binary packet ready to send over TCP
        """
        payload = struct.pack('<HH', index, 0)  # index + padding
        
        header = struct.pack(
            self.HEADER_FORMAT,
            PacketType.PARAM_GET_BY_INDEX_REQUEST,
            PROTOCOL_VERSION,
            len(payload),
            0,  # flags
            0,  # timestamp
            self._sequence
        )
        self._sequence += 1
        
        return header + payload
    
    def encode_list_request(self) -> bytes:
        """Create binary PARAM_LIST_REQUEST packet.
        
        Requests full parameter catalog from controller. Controller
        will respond with multiple PARAM_LIST_RESPONSE packets (one per parameter).
        
        DEPRECATED: Use encode_count_request() and encode_get_by_index_request() instead.
        
        Returns:
            Binary packet ready to send over TCP
        """
        # Simple header-only packet
        header = struct.pack(
            self.HEADER_FORMAT,
            PacketType.PARAM_LIST_REQUEST,
            PROTOCOL_VERSION,
            0,  # payload_length
            0,  # flags (unused)
            0,  # timestamp (unused)
            self._sequence
        )
        self._sequence += 1
        return header
    
    def encode_get_request(self, component: str, param_name: str) -> bytes:
        """Create binary PARAM_GET_REQUEST packet.
        
        Queries current value of a specific parameter.
        
        Args:
            component: Component name (e.g., "grid_observer")
            param_name: Parameter name (e.g., "kp")
        
        Returns:
            Binary packet ready to send over TCP
        """
        # Encode component and parameter name as null-terminated strings
        component_bytes = component.encode('utf-8')[:63] + b'\x00'
        param_bytes = param_name.encode('utf-8')[:63] + b'\x00'
        
        # Pad to 64 bytes each
        component_bytes = component_bytes.ljust(64, b'\x00')
        param_bytes = param_bytes.ljust(64, b'\x00')
        
        payload = component_bytes + param_bytes
        
        header = struct.pack(
            self.HEADER_FORMAT,
            PacketType.PARAM_GET_REQUEST,
            PROTOCOL_VERSION,
            len(payload),
            0,  # flags
            0,  # timestamp
            self._sequence
        )
        self._sequence += 1
        
        return header + payload
    
    def encode_set_request(self, component: str, param_name: str, value: float) -> bytes:
        """Create binary PARAM_SET_REQUEST packet.
        
        Sets new value for a specific parameter.
        
        Args:
            component: Component name (e.g., "grid_observer")
            param_name: Parameter name (e.g., "kp")
            value: New value to set
        
        Returns:
            Binary packet ready to send over TCP
        """
        # Encode strings
        component_bytes = component.encode('utf-8')[:63] + b'\x00'
        param_bytes = param_name.encode('utf-8')[:63] + b'\x00'
        
        # Pad to 64 bytes each
        component_bytes = component_bytes.ljust(64, b'\x00')
        param_bytes = param_bytes.ljust(64, b'\x00')
        
        # Pack value as float32
        value_bytes = struct.pack('<f', value)
        
        payload = component_bytes + param_bytes + value_bytes
        
        header = struct.pack(
            self.HEADER_FORMAT,
            PacketType.PARAM_SET_REQUEST,
            PROTOCOL_VERSION,
            len(payload),
            0,  # flags
            0,  # timestamp
            self._sequence
        )
        self._sequence += 1
        
        return header + payload


class ParameterProtocolDecoder:
    """Decoder for parameter protocol responses (Controller → Host)."""
    
    HEADER_FORMAT = '<BBHIII'
    HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
    
    def decode_count_response(self, data: bytes) -> Optional[tuple[int, int]]:
        """Decode PARAM_COUNT_RESPONSE packet.
        
        Returns total number of parameters available on controller.
        
        Args:
            data: Binary response data
        
        Returns:
            Tuple of (total_parameter_count, bytes_consumed), or None if decode failed
        """
        expected_size = self.HEADER_SIZE + 4
        if len(data) < expected_size:
            return None
        
        # Decode header
        packet_type, version, payload_len, flags, timestamp, sequence = struct.unpack(
            self.HEADER_FORMAT, data[:self.HEADER_SIZE]
        )
        
        if packet_type != PacketType.PARAM_COUNT_RESPONSE:
            return None
        
        # Decode payload: total_parameters (uint16) + padding (uint16)
        total_params, _ = struct.unpack('<HH', data[self.HEADER_SIZE:self.HEADER_SIZE+4])
        
        return (total_params, expected_size)
    
    def decode_get_by_index_response(self, data: bytes) -> Optional[tuple[ParameterMetadata, int]]:
        """Decode PARAM_GET_BY_INDEX_RESPONSE packet.
        
        Returns metadata for a single parameter at the requested index.
        
        Args:
            data: Binary response data
        
        Returns:
            Tuple of (ParameterMetadata, bytes_consumed), or None if decode failed
        """
        # Calculate expected size: header + payload (64+64+64+16+4+16+128+4+4 = 364 bytes)
        expected_size = self.HEADER_SIZE + 364
        if len(data) < expected_size:
            return None
        
        # Decode header
        packet_type, version, payload_len, flags, timestamp, sequence = struct.unpack(
            self.HEADER_FORMAT, data[:self.HEADER_SIZE]
        )
        
        if packet_type != PacketType.PARAM_GET_BY_INDEX_RESPONSE:
            return None
        
        offset = self.HEADER_SIZE
        
        # Decode payload (same structure as LIST_RESPONSE but with status field)
        component = self._decode_string(data[offset:offset+64])
        offset += 64
        
        name = self._decode_string(data[offset:offset+64])
        offset += 64
        
        display_name = self._decode_string(data[offset:offset+64])
        offset += 64
        
        unit = self._decode_string(data[offset:offset+16])
        offset += 16
        
        param_type, = struct.unpack('<B', data[offset:offset+1])
        offset += 4  # 1 byte + 3 padding
        
        min_val, max_val, default_val, current_val = struct.unpack('<ffff', data[offset:offset+16])
        offset += 16
        
        description = self._decode_string(data[offset:offset+128])
        offset += 128
        
        param_idx, total_params = struct.unpack('<HH', data[offset:offset+4])
        offset += 4
        
        status, = struct.unpack('<B', data[offset:offset+1])
        
        # Check status
        if status != 0:  # 0 = SUCCESS
            return None  # Index out of range or other error
        
        metadata = ParameterMetadata(
            component=component,
            name=name,
            display_name=display_name,
            unit=unit,
            type=ParameterValueType(param_type),
            min_value=min_val,
            max_value=max_val,
            default_value=default_val,
            current_value=current_val,
            description=description,
            parameter_index=param_idx,
            total_parameters=total_params
        )
        
        return (metadata, expected_size)
    
    def decode_list_response(self, data: bytes) -> Optional[ParameterMetadata]:
        """Decode PARAM_LIST_RESPONSE packet.
        
        Each response contains metadata for one parameter. Controller sends
        multiple responses (one per parameter) in response to LIST_REQUEST.
        
        DEPRECATED: Use decode_count_response() and decode_get_by_index_response() instead.
        
        Args:
            data: Binary response data
        
        Returns:
            ParameterMetadata object, or None if decode failed
        """
        if len(data) < self.HEADER_SIZE:
            return None
        
        # Decode header
        packet_type, version, payload_len, flags, timestamp, sequence = struct.unpack(
            self.HEADER_FORMAT, data[:self.HEADER_SIZE]
        )
        
        if packet_type != PacketType.PARAM_LIST_RESPONSE:
            return None
        
        offset = self.HEADER_SIZE
        
        # Decode payload:
        # - component name (64 bytes, null-terminated)
        # - param name (64 bytes, null-terminated)
        # - display_name (64 bytes, null-terminated)
        # - unit (16 bytes, null-terminated)
        # - type (1 byte)
        # - padding (3 bytes)
        # - min_value (4 bytes float)
        # - max_value (4 bytes float)
        # - default_value (4 bytes float)
        # - current_value (4 bytes float)
        # - description (128 bytes, null-terminated)
        # - parameter_index (2 bytes uint16)
        # - total_parameters (2 bytes uint16)
        
        component = self._decode_string(data[offset:offset+64])
        offset += 64
        
        name = self._decode_string(data[offset:offset+64])
        offset += 64
        
        display_name = self._decode_string(data[offset:offset+64])
        offset += 64
        
        unit = self._decode_string(data[offset:offset+16])
        offset += 16
        
        param_type, = struct.unpack('<B', data[offset:offset+1])
        offset += 4  # 1 byte + 3 padding
        
        min_val, max_val, default_val, current_val = struct.unpack('<ffff', data[offset:offset+16])
        offset += 16
        
        description = self._decode_string(data[offset:offset+128])
        offset += 128
        
        param_idx, total_params = struct.unpack('<HH', data[offset:offset+4])
        
        return ParameterMetadata(
            component=component,
            name=name,
            display_name=display_name,
            unit=unit,
            type=ParameterValueType(param_type),
            min_value=min_val,
            max_value=max_val,
            default_value=default_val,
            current_value=current_val,
            description=description,
            parameter_index=param_idx,
            total_parameters=total_params
        )
    
    def decode_get_response(self, data: bytes) -> Optional[ParameterValue]:
        """Decode PARAM_GET_RESPONSE packet.
        
        Args:
            data: Binary response data
        
        Returns:
            ParameterValue object, or None if decode failed
        """
        if len(data) < self.HEADER_SIZE:
            return None
        
        # Decode header
        packet_type, version, payload_len, flags, timestamp, sequence = struct.unpack(
            self.HEADER_FORMAT, data[:self.HEADER_SIZE]
        )
        
        if packet_type != PacketType.PARAM_GET_RESPONSE:
            return None
        
        offset = self.HEADER_SIZE
        
        # Payload: component(64) + name(64) + value(4 float)
        component = self._decode_string(data[offset:offset+64])
        offset += 64
        
        name = self._decode_string(data[offset:offset+64])
        offset += 64
        
        value, = struct.unpack('<f', data[offset:offset+4])
        
        return ParameterValue(
            component=component,
            name=name,
            value=value
        )
    
    def decode_set_response(self, data: bytes) -> Optional[tuple[dict, int]]:
        """Decode PARAM_SET_RESPONSE packet.
        
        Args:
            data: Binary response data
        
        Returns:
            Tuple of (response_dict, bytes_consumed) where response_dict contains
            status, component, name, old_value, new_value, error_message
        """
        # Calculate expected size: header + payload (64+64+8+4+128 = 268 bytes)
        expected_size = self.HEADER_SIZE + 268
        if len(data) < expected_size:
            return None
        
        # Decode header
        packet_type, version, payload_len, flags, timestamp, sequence = struct.unpack(
            self.HEADER_FORMAT, data[:self.HEADER_SIZE]
        )
        
        if packet_type != PacketType.PARAM_SET_RESPONSE:
            return None
        
        offset = self.HEADER_SIZE
        
        # Payload: component(64) + name(64) + old_value(4) + new_value(4) + status(1) + padding(3) + error_msg(128)
        component = self._decode_string(data[offset:offset+64])
        offset += 64
        
        name = self._decode_string(data[offset:offset+64])
        offset += 64
        
        old_val, new_val = struct.unpack('<ff', data[offset:offset+8])
        offset += 8
        
        status, = struct.unpack('<B', data[offset:offset+1])
        offset += 4  # 1 byte + 3 padding
        
        error_msg = self._decode_string(data[offset:offset+128])
        
        response_dict = {
            'status': ParameterSetStatus(status),
            'component': component,
            'name': name,
            'old_value': old_val,
            'new_value': new_val,
            'error_message': error_msg
        }
        
        return (response_dict, expected_size)
    
    @staticmethod
    def _decode_string(data: bytes) -> str:
        """Decode null-terminated string from byte array."""
        # Find null terminator
        try:
            null_idx = data.index(b'\x00')
            return data[:null_idx].decode('utf-8', errors='ignore')
        except ValueError:
            # No null terminator, decode entire buffer
            return data.decode('utf-8', errors='ignore').rstrip('\x00')
