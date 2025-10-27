#!/usr/bin/env python3
"""Telemetry protocol decoder for SFC control system."""

import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional, List, Tuple


# Protocol constants
PROTOCOL_VERSION = 1
MAX_SIGNALS = 256
MAX_PACKET_SIZE = 16 + (MAX_SIGNALS * 4)


class PacketType(IntEnum):
    """Telemetry packet type identifiers."""
    GRID_SIDE_TELEMETRY = 0x01
    MACHINE_SIDE_TELEMETRY = 0x02
    SYSTEM_STATUS = 0x10
    CONFIGURATION = 0x20
    COMMAND = 0x30
    PARAMETER_SET_REQUEST = 0x40
    PARAMETER_SET_RESPONSE = 0x41
    PARAMETER_GET_REQUEST = 0x42
    PARAMETER_GET_RESPONSE = 0x43
    PARAMETER_COUNT_REQUEST = 0x46
    PARAMETER_COUNT_RESPONSE = 0x47
    PARAMETER_GET_BY_INDEX_REQUEST = 0x48
    PARAMETER_GET_BY_INDEX_RESPONSE = 0x49


class SignalIndex(IntEnum):
    """Signal array indices matching C++ network_telemetry_builder.cpp."""
    GRID_VOLTAGE_MAGNITUDE_FILTERED = 0
    GRID_VOLTAGE_A = 1
    GRID_VOLTAGE_B = 2
    GRID_VOLTAGE_C = 3
    GRID_CURRENT_MAGNITUDE = 4
    GRID_CURRENT_A = 5
    GRID_CURRENT_B = 6
    GRID_CURRENT_C = 7
    DC_LINK_CURRENT = 8
    IS_COMMUTING = 9
    GRID_VOLTAGE_ALPHA = 10
    GRID_VOLTAGE_BETA = 11
    GRID_VOLTAGE_MAGNITUDE_FILTERED_DUP = 12
    GRID_SIDE_FREQUENCY = 13
    ESTIMATED_VOLTAGE_MAGNITUDE = 14
    ESTIMATED_FREQUENCY_PU = 15
    ESTIMATED_ANGLE_RAD = 16
    ESTIMATED_FREQUENCY_HZ = 17
    ESTIMATED_VOLTAGE_ALPHA = 18
    ESTIMATED_VOLTAGE_BETA = 19
    FIRING_ANGLE_RAD = 20
    MODULATOR_ENABLED = 21
    FORCE_COMMUTATION_ACK = 22
    IS_REQUESTING_PI_RESET = 23
    DC_LINK_CURRENT_REFERENCE = 24
    DC_LINK_CURRENT_MEASURED = 25
    CURRENT_ERROR = 26
    CONTROLLER_OUTPUT = 27
    OPERATING_MODE = 28
    CONTROLLER_SATURATED = 29
    FREEZE_INTEGRATOR = 30
    STATE_MACHINE_STATE = 31
    PULSE_RELEASES = 32
    AUXILIARIES_ON = 33
    BREAKERS_ON = 34
    START_COMMAND = 35
    STOP_COMMAND = 36
    CURRENT_EXTINGUISHED = 37
    SHUTDOWN_REQUESTED = 38
    OPERATION_MODE = 39
    MODE_CHANGED = 40
    CURRENT_ZERO = 41
    DISCONTINUOUS_MODE = 42
    FIRING_PULSE_1 = 43
    FIRING_PULSE_2 = 44
    FIRING_PULSE_3 = 45
    FIRING_PULSE_4 = 46
    FIRING_PULSE_5 = 47
    FIRING_PULSE_6 = 48
    PULSE_GENERATOR_ACTIVE = 49


@dataclass
class TelemetryPacketHeader:
    """Packet header for array-based telemetry (16 bytes)."""
    packet_type: int
    protocol_version: int
    signal_count: int
    timestamp_ms: int
    sequence_number: int
    reserved: int


@dataclass
class GridMeasurements:
    """Grid-side measurements."""
    grid_voltage_magnitude_pu: float = 0.0
    grid_voltage_a_pu: float = 0.0
    grid_voltage_b_pu: float = 0.0
    grid_voltage_c_pu: float = 0.0
    grid_current_magnitude_pu: float = 0.0
    grid_current_a_pu: float = 0.0
    grid_current_b_pu: float = 0.0
    grid_current_c_pu: float = 0.0
    dc_link_current_pu: float = 0.0
    is_commuting: bool = False
    grid_voltage_alpha_pu: float = 0.0
    grid_voltage_beta_pu: float = 0.0
    grid_side_frequency_hz: float = 0.0


@dataclass
class ObserverData:
    """Observer estimates."""
    estimated_voltage_magnitude_pu: float = 0.0
    estimated_frequency_pu: float = 0.0
    estimated_angle_rad: float = 0.0
    estimated_frequency_hz: float = 0.0
    estimated_voltage_alpha_pu: float = 0.0
    estimated_voltage_beta_pu: float = 0.0


@dataclass
class ModulatorData:
    """Modulator control signals."""
    is_enabled: bool = False
    firing_angle_input_pu: float = 0.0
    firing_angle_output_pu: float = 0.0
    force_commutation_ack: bool = False
    is_requesting_pi_reset: bool = False


@dataclass
class ControllerData:
    """Current controller state."""
    dc_link_current_reference: float = 0.0
    dc_link_current_measured: float = 0.0
    current_error_pu: float = 0.0
    controller_output_pu: float = 0.0
    operating_mode: bool = False
    controller_saturated: bool = False
    freeze_integrator: bool = False


@dataclass
class StateMachineData:
    """State machine flags."""
    current_state: int = 0
    pulse_releases: bool = False
    auxiliaries_are_on: bool = False
    breakers_are_on: bool = False
    start_command: bool = False
    stop_command: bool = False
    is_current_extinguished: bool = False
    is_shutdown_requested: bool = False


@dataclass
class OperationModeMonitorData:
    """Operation mode monitor."""
    mode: int = 0
    mode_changed: bool = False
    current_zero: bool = False
    discontinuous_mode: bool = False


@dataclass
class PulseGeneratorData:
    """Pulse generator outputs."""
    firing_pulse_1: bool = False
    firing_pulse_2: bool = False
    firing_pulse_3: bool = False
    firing_pulse_4: bool = False
    firing_pulse_5: bool = False
    firing_pulse_6: bool = False
    is_active: bool = False


@dataclass
class GridSideTelemetryPacket:
    """Grid-side telemetry packet with variable-length signal array."""
    header: TelemetryPacketHeader
    signals: List[float]
    
    def get_signal(self, index: int, default: float = 0.0) -> float:
        """Get signal by index with bounds checking."""
        if 0 <= index < len(self.signals):
            return self.signals[index]
        return default
    
    def get_bool_signal(self, index: int, default: bool = False) -> bool:
        """Get boolean signal (converts 1.0/0.0 to True/False)."""
        return bool(self.get_signal(index, 1.0 if default else 0.0))
    
    @property
    def measurements(self) -> GridMeasurements:
        """Extract grid measurements from signal array."""
        return GridMeasurements(
            grid_voltage_magnitude_pu=self.get_signal(SignalIndex.GRID_VOLTAGE_MAGNITUDE_FILTERED),
            grid_voltage_a_pu=self.get_signal(SignalIndex.GRID_VOLTAGE_A),
            grid_voltage_b_pu=self.get_signal(SignalIndex.GRID_VOLTAGE_B),
            grid_voltage_c_pu=self.get_signal(SignalIndex.GRID_VOLTAGE_C),
            grid_current_magnitude_pu=self.get_signal(SignalIndex.GRID_CURRENT_MAGNITUDE),
            grid_current_a_pu=self.get_signal(SignalIndex.GRID_CURRENT_A),
            grid_current_b_pu=self.get_signal(SignalIndex.GRID_CURRENT_B),
            grid_current_c_pu=self.get_signal(SignalIndex.GRID_CURRENT_C),
            dc_link_current_pu=self.get_signal(SignalIndex.DC_LINK_CURRENT),
            is_commuting=self.get_bool_signal(SignalIndex.IS_COMMUTING),
            grid_voltage_alpha_pu=self.get_signal(SignalIndex.GRID_VOLTAGE_ALPHA),
            grid_voltage_beta_pu=self.get_signal(SignalIndex.GRID_VOLTAGE_BETA),
            grid_side_frequency_hz=self.get_signal(SignalIndex.GRID_SIDE_FREQUENCY),
        )
    
    @property
    def observer(self) -> ObserverData:
        """Extract observer estimates from signal array."""
        return ObserverData(
            estimated_voltage_magnitude_pu=self.get_signal(SignalIndex.ESTIMATED_VOLTAGE_MAGNITUDE),
            estimated_frequency_pu=self.get_signal(SignalIndex.ESTIMATED_FREQUENCY_PU),
            estimated_angle_rad=self.get_signal(SignalIndex.ESTIMATED_ANGLE_RAD),
            estimated_frequency_hz=self.get_signal(SignalIndex.ESTIMATED_FREQUENCY_HZ),
            estimated_voltage_alpha_pu=self.get_signal(SignalIndex.ESTIMATED_VOLTAGE_ALPHA),
            estimated_voltage_beta_pu=self.get_signal(SignalIndex.ESTIMATED_VOLTAGE_BETA),
        )
    
    @property
    def modulator(self) -> ModulatorData:
        """Extract modulator signals from signal array."""
        return ModulatorData(
            is_enabled=self.get_bool_signal(SignalIndex.MODULATOR_ENABLED),
            firing_angle_input_pu=self.get_signal(SignalIndex.CONTROLLER_OUTPUT),
            firing_angle_output_pu=self.get_signal(SignalIndex.FIRING_ANGLE_RAD),
            force_commutation_ack=self.get_bool_signal(SignalIndex.FORCE_COMMUTATION_ACK),
            is_requesting_pi_reset=self.get_bool_signal(SignalIndex.IS_REQUESTING_PI_RESET),
        )
    
    @property
    def controller(self) -> ControllerData:
        """Extract controller state from signal array."""
        return ControllerData(
            dc_link_current_reference=self.get_signal(SignalIndex.DC_LINK_CURRENT_REFERENCE),
            dc_link_current_measured=self.get_signal(SignalIndex.DC_LINK_CURRENT_MEASURED),
            current_error_pu=self.get_signal(SignalIndex.CURRENT_ERROR),
            controller_output_pu=self.get_signal(SignalIndex.CONTROLLER_OUTPUT),
            operating_mode=self.get_bool_signal(SignalIndex.OPERATING_MODE),
            controller_saturated=self.get_bool_signal(SignalIndex.CONTROLLER_SATURATED),
            freeze_integrator=self.get_bool_signal(SignalIndex.FREEZE_INTEGRATOR),
        )
    
    @property
    def state_machine(self) -> StateMachineData:
        """Extract state machine flags from signal array."""
        return StateMachineData(
            current_state=int(self.get_signal(SignalIndex.STATE_MACHINE_STATE)),
            pulse_releases=self.get_bool_signal(SignalIndex.PULSE_RELEASES),
            auxiliaries_are_on=self.get_bool_signal(SignalIndex.AUXILIARIES_ON),
            breakers_are_on=self.get_bool_signal(SignalIndex.BREAKERS_ON),
            start_command=self.get_bool_signal(SignalIndex.START_COMMAND),
            stop_command=self.get_bool_signal(SignalIndex.STOP_COMMAND),
            is_current_extinguished=self.get_bool_signal(SignalIndex.CURRENT_EXTINGUISHED),
            is_shutdown_requested=self.get_bool_signal(SignalIndex.SHUTDOWN_REQUESTED),
        )
    
    @property
    def operation_mode_monitor(self) -> OperationModeMonitorData:
        """Extract operation mode monitor from signal array."""
        return OperationModeMonitorData(
            mode=int(self.get_signal(SignalIndex.OPERATION_MODE)),
            mode_changed=self.get_bool_signal(SignalIndex.MODE_CHANGED),
            current_zero=self.get_bool_signal(SignalIndex.CURRENT_ZERO),
            discontinuous_mode=self.get_bool_signal(SignalIndex.DISCONTINUOUS_MODE),
        )
    
    @property
    def pulse_generator(self) -> PulseGeneratorData:
        """Extract pulse generator outputs from signal array."""
        return PulseGeneratorData(
            firing_pulse_1=self.get_bool_signal(SignalIndex.FIRING_PULSE_1),
            firing_pulse_2=self.get_bool_signal(SignalIndex.FIRING_PULSE_2),
            firing_pulse_3=self.get_bool_signal(SignalIndex.FIRING_PULSE_3),
            firing_pulse_4=self.get_bool_signal(SignalIndex.FIRING_PULSE_4),
            firing_pulse_5=self.get_bool_signal(SignalIndex.FIRING_PULSE_5),
            firing_pulse_6=self.get_bool_signal(SignalIndex.FIRING_PULSE_6),
            is_active=self.get_bool_signal(SignalIndex.PULSE_GENERATOR_ACTIVE),
        )


class TelemetryDecoder:
    """Decoder for array-based binary telemetry protocol."""
    
    HEADER_FORMAT = '<BBHIII'
    HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
    SIGNAL_SIZE = 4
    
    def decode(self, data: bytes, verbose: bool = False) -> Optional[Tuple[GridSideTelemetryPacket, int]]:
        """Decode binary telemetry data into structured packet."""
        if len(data) < self.HEADER_SIZE:
            if verbose:
                hex_preview = data[:min(16, len(data))].hex() if len(data) > 0 else "(empty)"
                print(f"[DECODE ERROR] Data too short: {len(data)} bytes, expected >= {self.HEADER_SIZE} bytes. Data (hex): {hex_preview}")
            return None
            
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
            signal_count=header_data[2],
            timestamp_ms=header_data[3],
            sequence_number=header_data[4],
            reserved=header_data[5]
        )
        
        if verbose:
            print(f"[DECODE] Header decoded successfully:")
            print(f"  packet_type: 0x{header.packet_type:02X} (expected 0x{PacketType.GRID_SIDE_TELEMETRY:02X})")
            print(f"  protocol_version: {header.protocol_version} (expected {PROTOCOL_VERSION})")
            print(f"  signal_count: {header.signal_count}")
            print(f"  timestamp_ms: {header.timestamp_ms}")
            print(f"  sequence_number: {header.sequence_number}")
        
        if header.packet_type != PacketType.GRID_SIDE_TELEMETRY:
            if verbose:
                print(f"[DECODE ERROR] Packet type mismatch: got 0x{header.packet_type:02X}, expected 0x{PacketType.GRID_SIDE_TELEMETRY:02X}")
            return None
        
        if header.protocol_version != PROTOCOL_VERSION:
            if verbose:
                print(f"[DECODE ERROR] Protocol version mismatch: got {header.protocol_version}, expected {PROTOCOL_VERSION}")
            return None
        
        if header.signal_count > MAX_SIGNALS or header.signal_count == 0:
            if verbose:
                print(f"[DECODE ERROR] Invalid signal count: {header.signal_count} (max: {MAX_SIGNALS})")
            return None
        
        expected_payload_size = header.signal_count * self.SIGNAL_SIZE
        expected_total_size = self.HEADER_SIZE + expected_payload_size
        
        if len(data) < expected_total_size:
            if verbose:
                print(f"[DECODE ERROR] Insufficient data: have {len(data)} bytes, need {expected_total_size} bytes")
            return None
        
        signals = []
        offset = self.HEADER_SIZE
        
        for i in range(header.signal_count):
            if offset + self.SIGNAL_SIZE > len(data):
                if verbose:
                    print(f"[DECODE ERROR] Insufficient data for signal {i} at offset {offset}")
                return None
            
            try:
                signal_value = struct.unpack('<f', data[offset:offset + self.SIGNAL_SIZE])[0]
                signals.append(signal_value)
                offset += self.SIGNAL_SIZE
            except struct.error as e:
                if verbose:
                    print(f"[DECODE ERROR] Failed to unpack signal {i}: {e}")
                return None
        
        if verbose:
            print(f"[DECODE SUCCESS] Decoded {len(signals)} signals, total bytes consumed: {offset}")
            if len(signals) > 0:
                print(f"[DECODE] First few signals: {signals[:min(10, len(signals))]}")
        
        packet = GridSideTelemetryPacket(
            header=header,
            signals=signals
        )
        
        return (packet, offset)
