#!/usr/bin/env python3
"""Telemetry protocol decoder for SFC control system.

This module defines the binary telemetry protocol structures and provides
decoding functionality for real-time control data transmitted from the
embedded grid-side and machine-side controllers.

Protocol Structure:
    - Header (16 bytes): type, version, flags, timestamp, sequence
    - Payload (variable): signal groups indicated by flags

Signal Groups:
    - Measurements: Grid voltage, DC current
    - Observer: Frequency and angle estimates
    - Modulator: Firing angles and commutation status
    - Controller: PI controller state
    - State Machine: Current operating state

Example Usage:
    decoder = TelemetryDecoder()
    packet = decoder.decode(raw_bytes)
    if packet:
        print(f"Grid voltage: {packet.measurements.grid_voltage_magnitude_pu:.3f} pu")
        print(f"DC current: {packet.measurements.dc_link_current_pu:.3f} pu")
"""

import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional


# Protocol constants
PROTOCOL_VERSION = 1
MAX_PACKET_SIZE = 1024

# Packet types
class PacketType(IntEnum):
    """Telemetry packet type identifiers."""
    GRID_SIDE_TELEMETRY = 0x01
    MACHINE_SIDE_TELEMETRY = 0x02
    SYSTEM_STATUS = 0x10
    CONFIGURATION = 0x20
    COMMAND = 0x30


# Signal group flags (bit masks)
class TelemetryFlags(IntEnum):
    """Bit flags indicating which signal groups are present in packet."""
    MEASUREMENTS = (1 << 0)      # Voltage/current measurements
    OBSERVER = (1 << 1)          # Observer estimates (freq, angle)
    MODULATOR = (1 << 2)         # Modulator outputs (firing angle)
    CONTROLLER = (1 << 3)        # Controller state
    STATE_MACHINE = (1 << 4)     # State machine status
    PULSE_GENERATOR = (1 << 5)   # Pulse generator outputs
    EXCITATION = (1 << 6)        # Excitation controller
    SPEED_CONTROL = (1 << 7)     # Speed controller
    DIAGNOSTICS = (1 << 8)       # Diagnostic counters


@dataclass
class TelemetryPacketHeader:
    """Common header for all telemetry packets (16 bytes).
    
    Attributes:
        packet_type: Packet type identifier (PacketType enum)
        protocol_version: Protocol version for compatibility checking
        payload_length: Total payload size in bytes (after header)
        flags: Bit flags indicating which signal groups are present
        timestamp_ms: Controller timestamp in milliseconds
        sequence_number: Incrementing sequence number (detect drops)
    """
    packet_type: int
    protocol_version: int
    payload_length: int
    flags: int
    timestamp_ms: int
    sequence_number: int


@dataclass
class GridMeasurementsGroup:
    """Grid and DC link measurement signals (per-unit).
    
    Size: 40 bytes (9 floats + 1 bool with padding)
    Flag: MEASUREMENTS
    
    Attributes:
        grid_voltage_magnitude_pu: Grid voltage magnitude [pu]
        grid_voltage_a_pu: Phase A voltage [pu]
        grid_voltage_b_pu: Phase B voltage [pu]
        grid_voltage_c_pu: Phase C voltage [pu]
        grid_current_magnitude_pu: Grid current magnitude [pu]
        grid_current_a_pu: Phase A current [pu]
        grid_current_b_pu: Phase B current [pu]
        grid_current_c_pu: Phase C current [pu]
        dc_link_current_pu: DC link current [pu]
        is_commuting: Thyristor commutation status
    """
    grid_voltage_magnitude_pu: float
    grid_voltage_a_pu: float
    grid_voltage_b_pu: float
    grid_voltage_c_pu: float
    grid_current_magnitude_pu: float
    grid_current_a_pu: float
    grid_current_b_pu: float
    grid_current_c_pu: float
    dc_link_current_pu: float
    is_commuting: bool


@dataclass
class GridObserverGroup:
    """Grid-side observer estimates.
    
    Size: 12 bytes (3 floats)
    Flag: OBSERVER
    
    Attributes:
        estimated_voltage_magnitude_pu: Estimated grid voltage magnitude [pu]
        estimated_frequency_pu: Estimated grid frequency [pu]
        estimated_angle_rad: Estimated grid angle [radians]
    """
    estimated_voltage_magnitude_pu: float
    estimated_frequency_pu: float
    estimated_angle_rad: float


@dataclass
class GridModulatorGroup:
    """Grid-side modulator outputs.
    
    Size: 16 bytes (2 bools + 2 floats with padding)
    Flag: MODULATOR
    
    Attributes:
        is_enabled: Modulator enable status
        firing_angle_input_pu: Commanded firing angle [pu]
        firing_angle_output_pu: Actual firing angle (after limits) [pu]
        force_commutation_ack: Force commutation acknowledgment flag
    """
    is_enabled: bool
    firing_angle_input_pu: float
    firing_angle_output_pu: float
    force_commutation_ack: bool


@dataclass
class GridControllerGroup:
    """Current controller state.
    
    Size: 20 bytes (4 floats + 4 uint8_t)
    Flag: CONTROLLER
    
    Attributes:
        dc_link_current_reference: Control reference [pu]
        dc_link_current_measured: Control measured [pu]
        current_error_pu: Control error [pu]
        controller_output_pu: PI controller output [pu]
        operating_mode: 0=forced, 1=natural commutation
        controller_saturated: Saturation flag
        freeze_integrator: Integrator freeze flag
        reserved: Reserved byte
    """
    dc_link_current_reference: float
    dc_link_current_measured: float
    current_error_pu: float
    controller_output_pu: float
    operating_mode: int
    controller_saturated: int
    freeze_integrator: int
    reserved: int


@dataclass
class StateMachineGroup:
    """State machine status.
    
    Size: 8 bytes (1 uint8_t + 7 bools, packed)
    Flag: STATE_MACHINE
    
    Attributes:
        current_state: Current state enum value
        pulse_releases: Pulses enabled flag
        auxiliaries_are_on: Auxiliaries status
        breakers_are_on: Breakers status
        start_command: Start command status
        stop_command: Stop command status
        is_current_extinguished: Current extinguished status
        is_shutdown_requested: Shutdown requested flag
    """
    current_state: int
    pulse_releases: bool
    auxiliaries_are_on: bool
    breakers_are_on: bool
    start_command: bool
    stop_command: bool
    is_current_extinguished: bool
    is_shutdown_requested: bool

@dataclass
class GridSideTelemetryPacket:
    """Complete grid-side telemetry packet.
    
    Contains header plus all available signal groups.
    Only groups indicated by header.flags are populated.
    
    Attributes:
        header: Packet header with metadata
        measurements: Grid and DC measurements (optional)
        observer: Observer estimates (optional)
        modulator: Modulator outputs (optional)
        controller: Controller state (optional)
        state_machine: State machine status (optional)
    """
    header: TelemetryPacketHeader
    measurements: Optional[GridMeasurementsGroup] = None
    observer: Optional[GridObserverGroup] = None
    modulator: Optional[GridModulatorGroup] = None
    controller: Optional[GridControllerGroup] = None
    state_machine: Optional[StateMachineGroup] = None


class TelemetryDecoder:
    """Decoder for binary telemetry protocol.
    
    Parses binary data received from embedded controller and
    creates structured packet objects with all signal data.
    
    Example:
        decoder = TelemetryDecoder()
        raw_data = receive_from_tcp()
        packet = decoder.decode(raw_data)
        if packet and packet.measurements:
            voltage = packet.measurements.grid_voltage_magnitude_pu
    """
    
    # Struct format strings (little-endian)
    HEADER_FORMAT = '<BBHIII'            # 16 bytes (B+B+H+I+I+I = 1+1+2+4+4+4)
    MEASUREMENTS_FORMAT = '<fffffffffBxxx'   # 40 bytes (9 floats + 1 bool + 3 padding bytes)
    OBSERVER_FORMAT = '<fff'             # 12 bytes (3 floats)
    MODULATOR_FORMAT = '<Bxxxffbxxx'     # 16 bytes (bool + 3 pad + 2 floats + bool + 3 pad)
    CONTROLLER_FORMAT = '<ffffBBBB'      # 20 bytes (4 floats + 4 uint8)
    STATE_MACHINE_FORMAT = '<BBBBBBBB'   # 8 bytes (1 uint8 + 7 bools)
    
    HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
    MEASUREMENTS_SIZE = struct.calcsize(MEASUREMENTS_FORMAT)
    OBSERVER_SIZE = struct.calcsize(OBSERVER_FORMAT)
    MODULATOR_SIZE = struct.calcsize(MODULATOR_FORMAT)
    CONTROLLER_SIZE = struct.calcsize(CONTROLLER_FORMAT)
    STATE_MACHINE_SIZE = struct.calcsize(STATE_MACHINE_FORMAT)
    
    def decode(self, data: bytes, verbose: bool = False) -> Optional[GridSideTelemetryPacket]:
        """Decode binary telemetry data into structured packet.
        
        Args:
            data: Raw binary data received from TCP socket
            verbose: Enable detailed logging for debugging
            
        Returns:
            GridSideTelemetryPacket if decode successful, None if invalid
            
        Example:
            packet = decoder.decode(raw_bytes)
            if packet:
                print(f"Timestamp: {packet.header.timestamp_ms} ms")
        """
        if len(data) < self.HEADER_SIZE:
            if verbose:
                print(f"[DECODE ERROR] Data too short: {len(data)} bytes, expected >= {self.HEADER_SIZE} bytes")
            return None
            
        # Decode header
        try:
            header_data = struct.unpack(self.HEADER_FORMAT, data[:self.HEADER_SIZE])
        except struct.error as e:
            if verbose:
                print(f"[DECODE ERROR] Failed to unpack header: {e}")
                print(f"[DECODE ERROR] Data (hex): {data[:self.HEADER_SIZE].hex()}")
            return None
            
        header = TelemetryPacketHeader(
            packet_type=header_data[0],
            protocol_version=header_data[1],
            payload_length=header_data[2],
            flags=header_data[3],
            timestamp_ms=header_data[4],
            sequence_number=header_data[5]
        )
        
        if verbose:
            print(f"[DECODE] Header decoded successfully:")
            print(f"  packet_type: 0x{header.packet_type:02X} (expected 0x{PacketType.GRID_SIDE_TELEMETRY:02X})")
            print(f"  protocol_version: {header.protocol_version} (expected {PROTOCOL_VERSION})")
            print(f"  payload_length: {header.payload_length} bytes")
            print(f"  flags: 0x{header.flags:08X}")
            print(f"  timestamp_ms: {header.timestamp_ms}")
            print(f"  sequence_number: {header.sequence_number}")
            print(f"  total_data_length: {len(data)} bytes")
        
        # Validate header
        if header.protocol_version != PROTOCOL_VERSION:
            if verbose:
                print(f"[DECODE ERROR] Protocol version mismatch: got {header.protocol_version}, expected {PROTOCOL_VERSION}")
            return None
        if header.packet_type != PacketType.GRID_SIDE_TELEMETRY:
            if verbose:
                print(f"[DECODE ERROR] Packet type mismatch: got 0x{header.packet_type:02X}, expected 0x{PacketType.GRID_SIDE_TELEMETRY:02X}")
            return None
            
        # Calculate expected size
        expected_size = self.calculate_expected_size(header.flags)
        if verbose:
            print(f"[DECODE] Expected packet size: {expected_size} bytes (header {self.HEADER_SIZE} + payload {expected_size - self.HEADER_SIZE})")
            print(f"[DECODE] Actual packet size: {len(data)} bytes")
            if len(data) < expected_size:
                print(f"[DECODE WARNING] Packet too short! Missing {expected_size - len(data)} bytes")
        
        # Create packet with header
        packet = GridSideTelemetryPacket(header=header)
        
        # Decode signal groups based on flags
        offset = self.HEADER_SIZE
        
        # Measurements group
        if header.flags & TelemetryFlags.MEASUREMENTS:
            if offset + self.MEASUREMENTS_SIZE > len(data):
                if verbose:
                    print(f"[DECODE ERROR] MEASUREMENTS group: insufficient data at offset {offset}, need {self.MEASUREMENTS_SIZE} bytes, have {len(data) - offset} bytes")
                return None
            try:
                meas_data = struct.unpack(
                    self.MEASUREMENTS_FORMAT,
                    data[offset:offset + self.MEASUREMENTS_SIZE]
                )
            except struct.error as e:
                if verbose:
                    print(f"[DECODE ERROR] MEASUREMENTS group unpack failed: {e}")
                return None
                
            packet.measurements = GridMeasurementsGroup(
                grid_voltage_magnitude_pu=meas_data[0],
                grid_voltage_a_pu=meas_data[1],
                grid_voltage_b_pu=meas_data[2],
                grid_voltage_c_pu=meas_data[3],
                grid_current_magnitude_pu=meas_data[4],
                grid_current_a_pu=meas_data[5],
                grid_current_b_pu=meas_data[6],
                grid_current_c_pu=meas_data[7],
                dc_link_current_pu=meas_data[8],
                is_commuting=bool(meas_data[9])
            )
            if verbose:
                print(f"[DECODE] MEASUREMENTS decoded at offset {offset}: v_mag={meas_data[0]:.3f}, v_a={meas_data[1]:.3f}, v_b={meas_data[2]:.3f}, v_c={meas_data[3]:.3f}, i_mag={meas_data[4]:.3f}, i_a={meas_data[5]:.3f}, i_b={meas_data[6]:.3f}, i_c={meas_data[7]:.3f}, dc={meas_data[8]:.3f}, commuting={meas_data[9]}")
            offset += self.MEASUREMENTS_SIZE
            
        # Observer group
        if header.flags & TelemetryFlags.OBSERVER:
            if offset + self.OBSERVER_SIZE > len(data):
                if verbose:
                    print(f"[DECODE ERROR] OBSERVER group: insufficient data at offset {offset}, need {self.OBSERVER_SIZE} bytes, have {len(data) - offset} bytes")
                return None
            try:
                obs_data = struct.unpack(
                    self.OBSERVER_FORMAT,
                    data[offset:offset + self.OBSERVER_SIZE]
                )
            except struct.error as e:
                if verbose:
                    print(f"[DECODE ERROR] OBSERVER group unpack failed: {e}")
                return None
                
            packet.observer = GridObserverGroup(
                estimated_voltage_magnitude_pu=obs_data[0],
                estimated_frequency_pu=obs_data[1],
                estimated_angle_rad=obs_data[2]
            )
            if verbose:
                print(f"[DECODE] OBSERVER decoded at offset {offset}: voltage_mag={obs_data[0]:.3f}, freq={obs_data[1]:.3f}, angle={obs_data[2]:.3f}")
            offset += self.OBSERVER_SIZE
            
        # Modulator group
        if header.flags & TelemetryFlags.MODULATOR:
            if offset + self.MODULATOR_SIZE > len(data):
                if verbose:
                    print(f"[DECODE ERROR] MODULATOR group: insufficient data at offset {offset}, need {self.MODULATOR_SIZE} bytes, have {len(data) - offset} bytes")
                return None
            try:
                mod_data = struct.unpack(
                    self.MODULATOR_FORMAT,
                    data[offset:offset + self.MODULATOR_SIZE]
                )
            except struct.error as e:
                if verbose:
                    print(f"[DECODE ERROR] MODULATOR group unpack failed: {e}")
                return None
                
            packet.modulator = GridModulatorGroup(
                is_enabled=bool(mod_data[0]),
                firing_angle_input_pu=mod_data[1],
                firing_angle_output_pu=mod_data[2],
                force_commutation_ack=bool(mod_data[3])
            )
            if verbose:
                print(f"[DECODE] MODULATOR decoded at offset {offset}: enabled={mod_data[0]}, input_angle={mod_data[1]:.3f}, output_angle={mod_data[2]:.3f}, force_comm={mod_data[3]}")
            offset += self.MODULATOR_SIZE
            
        # Controller group
        if header.flags & TelemetryFlags.CONTROLLER:
            if offset + self.CONTROLLER_SIZE > len(data):
                if verbose:
                    print(f"[DECODE ERROR] CONTROLLER group: insufficient data at offset {offset}, need {self.CONTROLLER_SIZE} bytes, have {len(data) - offset} bytes")
                return None
            try:
                ctrl_data = struct.unpack(
                    self.CONTROLLER_FORMAT,
                    data[offset:offset + self.CONTROLLER_SIZE]
                )
            except struct.error as e:
                if verbose:
                    print(f"[DECODE ERROR] CONTROLLER group unpack failed: {e}")
                return None
                
            packet.controller = GridControllerGroup(
                dc_link_current_reference=ctrl_data[0],
                dc_link_current_measured=ctrl_data[1],
                current_error_pu=ctrl_data[2],
                controller_output_pu=ctrl_data[3],
                operating_mode=ctrl_data[4],
                controller_saturated=ctrl_data[5],
                freeze_integrator=ctrl_data[6],
                reserved=ctrl_data[7]
            )
            if verbose:
                print(f"[DECODE] CONTROLLER decoded at offset {offset}: ref={ctrl_data[0]:.3f}, meas={ctrl_data[1]:.3f}, error={ctrl_data[2]:.3f}, output={ctrl_data[3]:.3f}, mode={ctrl_data[4]}")
            offset += self.CONTROLLER_SIZE
            
        # State machine group
        if header.flags & TelemetryFlags.STATE_MACHINE:
            if offset + self.STATE_MACHINE_SIZE > len(data):
                if verbose:
                    print(f"[DECODE ERROR] STATE_MACHINE group: insufficient data at offset {offset}, need {self.STATE_MACHINE_SIZE} bytes, have {len(data) - offset} bytes")
                return None
            try:
                state_data = struct.unpack(
                    self.STATE_MACHINE_FORMAT,
                    data[offset:offset + self.STATE_MACHINE_SIZE]
                )
            except struct.error as e:
                if verbose:
                    print(f"[DECODE ERROR] STATE_MACHINE group unpack failed: {e}")
                return None
                
            packet.state_machine = StateMachineGroup(
                current_state=state_data[0],
                pulse_releases=bool(state_data[1]),
                auxiliaries_are_on=bool(state_data[2]),
                breakers_are_on=bool(state_data[3]),
                start_command=bool(state_data[4]),
                stop_command=bool(state_data[5]),
                is_current_extinguished=bool(state_data[6]),
                is_shutdown_requested=bool(state_data[7])
            )
            if verbose:
                print(f"[DECODE] STATE_MACHINE decoded at offset {offset}: state={state_data[0]}, pulses={state_data[1]}, aux_on={state_data[2]}, brk_on={state_data[3]}, start={state_data[4]}, stop={state_data[5]}, curr_ext={state_data[6]}, shutdown={state_data[7]}")
            offset += self.STATE_MACHINE_SIZE
        
        if verbose:
            print(f"[DECODE SUCCESS] Packet decoded successfully, total bytes consumed: {offset}")
            
        return packet
    
    def calculate_expected_size(self, flags: int) -> int:
        """Calculate expected packet size based on flags.
        
        Args:
            flags: Bit flags indicating which groups are present
            
        Returns:
            Expected total packet size in bytes (header + payload)
        """
        size = self.HEADER_SIZE
        
        if flags & TelemetryFlags.MEASUREMENTS:
            size += self.MEASUREMENTS_SIZE
        if flags & TelemetryFlags.OBSERVER:
            size += self.OBSERVER_SIZE
        if flags & TelemetryFlags.MODULATOR:
            size += self.MODULATOR_SIZE
        if flags & TelemetryFlags.CONTROLLER:
            size += self.CONTROLLER_SIZE
        if flags & TelemetryFlags.STATE_MACHINE:
            size += self.STATE_MACHINE_SIZE
            
        return size
