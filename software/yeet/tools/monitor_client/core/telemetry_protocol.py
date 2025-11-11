#!/usr/bin/env python3
"""Telemetry protocol decoder for SFC control system.

This module handles the low-level binary telemetry protocol decoding and packet
structure. Signal indices and data models are imported from configuration modules:
  - config.grid_side_telemetry_protocol: Grid-side signals and dataclasses
  - config.machine_side_telemetry_protocol: Machine-side signals and dataclasses
"""

import struct
import zlib
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional, List, Tuple, Union

# Import signal indices and dataclasses from config
from ..config.grid_side_telemetry_protocol import (
    GridSignalIndex,
    GridMeasurements,
    GridObserverData,
    GridModulatorData,
    GridControllerData,
    StateMachineData,
    OperationModeMonitorData,
    PulseGeneratorData,
)
from ..config.machine_side_telemetry_protocol import (
    MachineSignalIndex,
    MachineMeasurements,
    MachineObserverData,
    MachineModulatorData,
    MachineControllerData,
    TorqueControllerInputsData,
    TorqueControllerOutputsData,
    SpeedControllerData,
    StartupSequencerData,
    ExcitationControllerData,
)


# Protocol constants
PROTOCOL_VERSION = 1
MAX_SIGNALS = 256
MAX_PACKET_SIZE = 16 + (MAX_SIGNALS * 4)
SYNC_MAGIC = 0xDEADBEEF  # Magic number for stream synchronization


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


# For backward compatibility
SignalIndex = GridSignalIndex


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
            grid_voltage_magnitude_pu=self.get_signal(GridSignalIndex.GRID_VOLTAGE_MAGNITUDE_FILTERED),
            grid_voltage_a_pu=self.get_signal(GridSignalIndex.GRID_VOLTAGE_A),
            grid_voltage_b_pu=self.get_signal(GridSignalIndex.GRID_VOLTAGE_B),
            grid_voltage_c_pu=self.get_signal(GridSignalIndex.GRID_VOLTAGE_C),
            grid_current_magnitude_pu=self.get_signal(GridSignalIndex.GRID_CURRENT_MAGNITUDE),
            grid_current_a_pu=self.get_signal(GridSignalIndex.GRID_CURRENT_A),
            grid_current_b_pu=self.get_signal(GridSignalIndex.GRID_CURRENT_B),
            grid_current_c_pu=self.get_signal(GridSignalIndex.GRID_CURRENT_C),
            dc_link_current_pu=self.get_signal(GridSignalIndex.DC_LINK_CURRENT),
        )
    
    @property
    def observer(self) -> GridObserverData:
        """Extract observer estimates from signal array."""
        return GridObserverData(
            estimated_voltage_magnitude_pu=self.get_signal(GridSignalIndex.ESTIMATED_VOLTAGE_MAGNITUDE),
            estimated_frequency_pu=self.get_signal(GridSignalIndex.ESTIMATED_FREQUENCY_PU),
            estimated_reference_angle_pu=self.get_signal(GridSignalIndex.ESTIMATED_REFERENCE_ANGLE),
        )
    
    @property
    def modulator(self) -> GridModulatorData:
        """Extract modulator signals from signal array."""
        return GridModulatorData(
            release_pulses_input=self.get_bool_signal(GridSignalIndex.MODULATOR_RELEASE_PULSES_INPUT),
            firing_angle_input_pu=self.get_signal(GridSignalIndex.MODULATOR_FIRING_ANGLE_INPUT),
            firing_angle_output_pu=self.get_signal(GridSignalIndex.MODULATOR_FIRING_ANGLE_OUTPUT),
            operating_mode=int(self.get_signal(GridSignalIndex.MODULATOR_OPERATING_MODE)),
            request_extinguish_dc=self.get_bool_signal(GridSignalIndex.MODULATOR_REQUEST_EXTINGUISH_DC),
            request_force_commutation=self.get_bool_signal(GridSignalIndex.MODULATOR_REQUEST_FORCE_COMMUTATION),
            modulator_state=int(self.get_signal(GridSignalIndex.MODULATOR_STATE)),
            release_pulses_output=self.get_bool_signal(GridSignalIndex.MODULATOR_RELEASE_PULSES_OUTPUT),
        )
    
    @property
    def controller(self) -> GridControllerData:
        """Extract controller state from signal array."""
        return GridControllerData(
            current_reference=self.get_signal(GridSignalIndex.CURRENT_REFERENCE),
            current_measured=self.get_signal(GridSignalIndex.CURRENT_MEASURED),
            freeze_integrator=self.get_bool_signal(GridSignalIndex.CURRENT_FREEZE_INTEGRATOR),
            preset_request=self.get_bool_signal(GridSignalIndex.CURRENT_PRESET_REQUEST),
            current_reference_output=self.get_signal(GridSignalIndex.CURRENT_REFERENCE_OUTPUT),
            current_error_pu=self.get_signal(GridSignalIndex.CURRENT_ERROR),
            pi_output_pu=self.get_signal(GridSignalIndex.CURRENT_PI_OUTPUT),
            is_active=self.get_bool_signal(GridSignalIndex.CURRENT_IS_ACTIVE),
            is_saturated=self.get_bool_signal(GridSignalIndex.CURRENT_IS_SATURATED),
        )
    
    @property
    def state_machine(self) -> StateMachineData:
        """Extract state machine flags from signal array."""
        return StateMachineData(
            current_state=int(self.get_signal(GridSignalIndex.STATE_MACHINE_STATE)),
            release_pulses=self.get_bool_signal(GridSignalIndex.STATE_MACHINE_RELEASE_PULSES),
            auxiliaries_are_on=self.get_bool_signal(GridSignalIndex.STATE_MACHINE_AUXILIARIES_ON),
            breakers_are_on=self.get_bool_signal(GridSignalIndex.STATE_MACHINE_BREAKERS_ON),
            start_command=self.get_bool_signal(GridSignalIndex.STATE_MACHINE_START_COMMAND),
            stop_command=self.get_bool_signal(GridSignalIndex.STATE_MACHINE_STOP_COMMAND),
            is_current_extinguished=self.get_bool_signal(GridSignalIndex.STATE_MACHINE_CURRENT_EXTINGUISHED),
            is_shutdown_requested=self.get_bool_signal(GridSignalIndex.STATE_MACHINE_SHUTDOWN_REQUEST),
        )
    
    @property
    def operation_mode_monitor(self) -> OperationModeMonitorData:
        """Extract operation mode monitor from signal array."""
        return OperationModeMonitorData(
            preset_request=self.get_bool_signal(GridSignalIndex.STATE_MACHINE_PRESET_REQUEST),
            has_mode_changed=self.get_bool_signal(GridSignalIndex.MODE_MONITOR_HAS_MODE_CHANGED),
            current_is_zero=self.get_bool_signal(GridSignalIndex.MODE_MONITOR_CURRENT_IS_ZERO),
            is_discontinuous_mode=self.get_bool_signal(GridSignalIndex.MODE_MONITOR_IS_DISCONTINUOUS_MODE),
        )
    
    @property
    def pulse_generator(self) -> PulseGeneratorData:
        """Extract pulse generator outputs from signal array."""
        return PulseGeneratorData(
            release_pulses=self.get_bool_signal(GridSignalIndex.PULSE_GENERATOR_RELEASE_PULSES),
            actual_firing_angle_pu=self.get_signal(GridSignalIndex.PULSE_GENERATOR_ACTUAL_FIRING_ANGLE),
            release_pulses_input=self.get_bool_signal(GridSignalIndex.PULSE_GENERATOR_RELEASE_PULSES_INPUT),
        )


@dataclass
class MachineSideTelemetryPacket:
    """Machine-side telemetry packet with variable-length signal array."""
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
    def measurements(self) -> MachineMeasurements:
        """Extract machine measurements from signal array."""
        return MachineMeasurements(
            machine_voltage_abc_a_pu=self.get_signal(MachineSignalIndex.MACHINE_VOLTAGE_ABC_A),
            machine_voltage_abc_b_pu=self.get_signal(MachineSignalIndex.MACHINE_VOLTAGE_ABC_B),
            machine_voltage_abc_c_pu=self.get_signal(MachineSignalIndex.MACHINE_VOLTAGE_ABC_C),
            machine_voltage_magnitude_filtered_pu=self.get_signal(MachineSignalIndex.MACHINE_VOLTAGE_MAGNITUDE_FILTERED),
            estimated_speed_pu=self.get_signal(MachineSignalIndex.ESTIMATED_SPEED),
            estimated_voltage_magnitude_pu=self.get_signal(MachineSignalIndex.ESTIMATED_VOLTAGE_MAGNITUDE),
            estimated_frequency_pu=self.get_signal(MachineSignalIndex.ESTIMATED_FREQUENCY_PU),
            estimated_reference_angle_pu=self.get_signal(MachineSignalIndex.ESTIMATED_REFERENCE_ANGLE),
            estimated_frequency_hz=self.get_signal(MachineSignalIndex.ESTIMATED_FREQUENCY),
        )
    
    @property
    def observer(self) -> MachineObserverData:
        """Extract machine observer estimates from signal array."""
        return MachineObserverData(
            estimated_frequency_pu=self.get_signal(MachineSignalIndex.ESTIMATED_FREQUENCY_PU),
            estimated_reference_angle_pu=self.get_signal(MachineSignalIndex.ESTIMATED_REFERENCE_ANGLE),
            estimated_frequency_hz=self.get_signal(MachineSignalIndex.ESTIMATED_FREQUENCY),
        )
    
    @property
    def modulator(self) -> MachineModulatorData:
        """Extract machine modulator signals from signal array."""
        return MachineModulatorData(
            release_pulses_input=self.get_bool_signal(MachineSignalIndex.MODULATOR_RELEASE_PULSES_INPUT),
            operating_mode=int(self.get_signal(MachineSignalIndex.MODULATOR_OPERATING_MODE)),
            release_pulses_output=self.get_bool_signal(MachineSignalIndex.MODULATOR_RELEASE_PULSES_OUTPUT),
            request_force_commutation=self.get_bool_signal(MachineSignalIndex.MODULATOR_REQUEST_FORCE_COMMUTATION),
            force_commutation_acknowledged=self.get_bool_signal(MachineSignalIndex.MODULATOR_FORCE_COMMUTATION_ACK),
            firing_angle_pu=self.get_signal(MachineSignalIndex.MODULATOR_FIRING_ANGLE),
            current_rotor_sector_normalized=self.get_signal(MachineSignalIndex.MODULATOR_CURRENT_ROTOR_SECTOR),
            commutation_state=int(self.get_signal(MachineSignalIndex.MODULATOR_COMMUTATION_STATE)),
        )
    
    @property
    def controller(self) -> MachineControllerData:
        """Extract machine controller state from signal array."""
        return MachineControllerData(
            firing_angle_rad=self.get_signal(MachineSignalIndex.CONTROLLER_FIRING_ANGLE),
            dc_link_current_pu=self.get_signal(MachineSignalIndex.CONTROLLER_DC_LINK_CURRENT),
            machine_speed_pu=self.get_signal(MachineSignalIndex.CONTROLLER_MACHINE_SPEED),
        )
    
    @property
    def torque_controller(self) -> TorqueControllerOutputsData:
        """Extract torque controller state from signal array."""
        return TorqueControllerOutputsData(
            current_reference_pu=self.get_signal(MachineSignalIndex.TORQUE_CURRENT_REFERENCE),
            is_saturated=self.get_bool_signal(MachineSignalIndex.TORQUE_IS_SATURATED_OUTPUT),
            is_active=self.get_bool_signal(MachineSignalIndex.TORQUE_IS_ACTIVE),
            is_breakaway_boost_active=self.get_bool_signal(MachineSignalIndex.TORQUE_IS_BREAKAWAY_BOOST_ACTIVE),
        )
    
    @property
    def speed_controller(self) -> SpeedControllerData:
        """Extract speed controller state from signal array."""
        return SpeedControllerData(
            speed_reference_pu=self.get_signal(MachineSignalIndex.SPEED_SPEED_REFERENCE),
            measured_speed_pu=self.get_signal(MachineSignalIndex.SPEED_MEASURED_SPEED),
            torque_reference_pu=self.get_signal(MachineSignalIndex.SPEED_TORQUE_REFERENCE),
            pi_output_pu=self.get_signal(MachineSignalIndex.SPEED_PI_OUTPUT),
            is_saturated=self.get_bool_signal(MachineSignalIndex.SPEED_IS_SATURATED),
        )
    
    @property
    def startup_sequencer(self) -> StartupSequencerData:
        """Extract startup sequencer state from signal array."""
        return StartupSequencerData(
            speed_reference_pu=self.get_signal(MachineSignalIndex.STARTUP_SPEED_REFERENCE),
            virtual_rotor_frequency_pu=self.get_signal(MachineSignalIndex.STARTUP_VIRTUAL_ROTOR_FREQUENCY),
            virtual_rotor_angle_pu=self.get_signal(MachineSignalIndex.STARTUP_VIRTUAL_ROTOR_ANGLE),
            virtual_torque_reference_pu=self.get_signal(MachineSignalIndex.STARTUP_VIRTUAL_TORQUE_REFERENCE),
            release_speed_controller=self.get_bool_signal(MachineSignalIndex.STARTUP_RELEASE_SPEED_CONTROLLER),
            release_excitation=self.get_bool_signal(MachineSignalIndex.STARTUP_RELEASE_EXCITATION),
            release_torque_controller=self.get_bool_signal(MachineSignalIndex.STARTUP_RELEASE_TORQUE_CONTROLLER),
            release_machine_side_pulses=self.get_bool_signal(MachineSignalIndex.STARTUP_RELEASE_MACHINE_SIDE_PULSES),
            is_startup_complete=self.get_bool_signal(MachineSignalIndex.STARTUP_IS_STARTUP_COMPLETE),
        )
    
    @property
    def excitation_controller(self) -> ExcitationControllerData:
        """Extract excitation controller state from signal array."""
        return ExcitationControllerData(
            field_current_reference_pu=self.get_signal(MachineSignalIndex.EXCITATION_FIELD_CURRENT_REFERENCE),
            prefluxing_elapsed_time_s=self.get_signal(MachineSignalIndex.EXCITATION_PREFLUXING_ELAPSED_TIME),
        )
    
    @property
    def state_machine(self) -> StateMachineData:
        """Extract machine state machine flags from signal array (empty for machine side)."""
        return StateMachineData(
            current_state=0,
            pulse_releases=False,
            auxiliaries_are_on=False,
            breakers_are_on=False,
            start_command=False,
            stop_command=False,
            is_current_extinguished=False,
            is_shutdown_requested=False,
        )
    
    @property
    def operation_mode_monitor(self) -> OperationModeMonitorData:
        """Extract machine operation mode monitor (not applicable, return defaults)."""
        return OperationModeMonitorData(
            mode=0,
            mode_changed=False,
            current_zero=False,
            discontinuous_mode=False,
        )
    
    # @property
    # def pulse_generator(self) -> PulseGeneratorData:
    #     """Extract machine pulse generator outputs from signal array."""
    #     return PulseGeneratorData(
    #         firing_pulse_1=self.get_bool_signal(MachineSignalIndex.PULSE_P1),
    #         firing_pulse_2=self.get_bool_signal(MachineSignalIndex.PULSE_P2),
    #         firing_pulse_3=self.get_bool_signal(MachineSignalIndex.PULSE_P3),
    #         firing_pulse_4=self.get_bool_signal(MachineSignalIndex.PULSE_P4),
    #         firing_pulse_5=self.get_bool_signal(MachineSignalIndex.PULSE_P5),
    #         firing_pulse_6=False,  # Not available on machine side (only 5 pulses)
    #     )


class TelemetryDecoder:
    """Decoder for array-based binary telemetry protocol."""
    
    HEADER_FORMAT = '<BBHIII'
    HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
    SIGNAL_SIZE = 4
    CRC_SIZE = 4  # CRC32 = 4 bytes
    SYNC_FRAME_SIZE = 4  # Sync magic number = 4 bytes
    
    def __init__(self, require_crc: bool = False):
        """Initialize telemetry decoder.
        
        Args:
            require_crc: If True, require and validate CRC32 at end of packet.
                         If False, support both CRC and non-CRC packets (for backward compatibility).
        """
        self.require_crc = require_crc
    
    @staticmethod
    def is_sync_frame(data: bytes) -> bool:
        """Check if data starts with sync magic number (0xDEADBEEF).
        
        Args:
            data: Data buffer to check
            
        Returns:
            True if first 4 bytes contain sync magic, False otherwise
        """
        if len(data) < 4:
            return False
        
        magic = struct.unpack('<I', data[:4])[0]
        return magic == SYNC_MAGIC
    
    @staticmethod
    def decode_sync_frame(data: bytes) -> tuple[int, bool]:
        """Decode and validate sync frame.
        
        Args:
            data: Data buffer containing sync frame
            
        Returns:
            Tuple of (bytes_consumed, is_valid)
        """
        if len(data) < 4:
            return (0, False)
        
        magic = struct.unpack('<I', data[:4])[0]
        is_valid = (magic == SYNC_MAGIC)
        return (4, is_valid)
    
    def decode(self, data: bytes, verbose: bool = False) -> Optional[Tuple[Union[GridSideTelemetryPacket, MachineSideTelemetryPacket], int]]:
        """Decode binary telemetry data into structured packet (grid or machine side)."""
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
            print(f"  packet_type: 0x{header.packet_type:02X} (GRID=0x{PacketType.GRID_SIDE_TELEMETRY:02X}, MACHINE=0x{PacketType.MACHINE_SIDE_TELEMETRY:02X})")
            print(f"  protocol_version: {header.protocol_version} (expected {PROTOCOL_VERSION})")
            print(f"  signal_count: {header.signal_count}")
            print(f"  timestamp_ms: {header.timestamp_ms}")
            print(f"  sequence_number: {header.sequence_number}")
        
        # Determine packet type (support both grid and machine side)
        if header.packet_type not in (PacketType.GRID_SIDE_TELEMETRY, PacketType.MACHINE_SIDE_TELEMETRY):
            if verbose:
                print(f"[DECODE ERROR] Unsupported packet type: 0x{header.packet_type:02X}")
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
        expected_total_size_without_crc = self.HEADER_SIZE + expected_payload_size
        expected_total_size_with_crc = expected_total_size_without_crc + self.CRC_SIZE
        
        # Determine if packet has CRC based on available data
        has_crc = False
        if len(data) >= expected_total_size_with_crc:
            # Might have CRC - we'll verify it
            has_crc = True
            expected_total_size = expected_total_size_with_crc
        else:
            expected_total_size = expected_total_size_without_crc
        
        if self.require_crc and not has_crc:
            if verbose:
                print(f"[DECODE ERROR] CRC required but packet too short: have {len(data)} bytes, need {expected_total_size_with_crc} bytes")
            return None
        
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
        
        # Validate CRC if present
        if has_crc:
            # CRC is calculated over header + signals (everything except the CRC itself)
            packet_data_for_crc = data[:offset]
            received_crc = struct.unpack('<I', data[offset:offset + self.CRC_SIZE])[0]
            calculated_crc = zlib.crc32(packet_data_for_crc) & 0xFFFFFFFF
            
            if received_crc != calculated_crc:
                if verbose:
                    print(f"[DECODE ERROR] CRC mismatch: received=0x{received_crc:08X}, calculated=0x{calculated_crc:08X}")
                    print(f"[DECODE ERROR] Packet corrupt or misaligned, rejecting")
                return None
            
            if verbose:
                print(f"[DECODE] CRC32 validated: 0x{calculated_crc:08X}")
            
            offset += self.CRC_SIZE
        elif verbose:
            print(f"[DECODE] No CRC present (legacy packet or backward compatibility mode)")
        
        if verbose:
            print(f"[DECODE SUCCESS] Decoded {len(signals)} signals, total bytes consumed: {offset}")
            if len(signals) > 0:
                print(f"[DECODE] First few signals: {signals[:min(10, len(signals))]}")
        
        # Create appropriate packet type
        if header.packet_type == PacketType.GRID_SIDE_TELEMETRY:
            packet: Union[GridSideTelemetryPacket, MachineSideTelemetryPacket] = GridSideTelemetryPacket(
                header=header,
                signals=signals
            )
        else:  # MACHINE_SIDE_TELEMETRY
            packet = MachineSideTelemetryPacket(
                header=header,
                signals=signals
            )
        
        return (packet, offset)
