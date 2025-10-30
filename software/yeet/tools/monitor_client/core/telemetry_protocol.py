#!/usr/bin/env python3
"""Telemetry protocol decoder for SFC control system."""

import struct
import zlib
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional, List, Tuple, Union


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


class GridSignalIndex(IntEnum):
    """Signal array indices for GRID_SIDE_TELEMETRY matching C++ grid_side_telemetry_builder.cpp."""
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


class MachineSignalIndex(IntEnum):
    """Signal array indices for MACHINE_SIDE_TELEMETRY matching C++ machine_side_telemetry_builder.cpp."""
    # Machine Measurements Group (0-9)
    MACHINE_VOLTAGE_MAGNITUDE_FILTERED = 0
    MACHINE_VOLTAGE_A = 1
    MACHINE_VOLTAGE_B = 2
    MACHINE_VOLTAGE_C = 3
    MACHINE_SIDE_SPEED = 4
    MACHINE_VOLTAGE_MAGNITUDE = 5
    IS_COMMUTING = 6
    MACHINE_VOLTAGE_ALPHA = 7
    MACHINE_VOLTAGE_BETA = 8
    MACHINE_VOLTAGE_MAGNITUDE_FILTERED_2 = 9
    
    # Observer Group (10-14)
    ESTIMATED_VOLTAGE_MAGNITUDE = 10
    ESTIMATED_FREQUENCY_PU = 11
    ESTIMATED_ANGLE_RAD = 12
    ESTIMATED_FREQUENCY_HZ = 13
    ESTIMATED_VOLTAGE_ALPHA = 14
    
    # Modulator Group (15-19)
    FIRING_ANGLE_RAD = 15
    MODULATOR_ENABLED = 16
    RELEASE_PULSES = 17
    REQUEST_FORCE_COMMUTATION = 18
    CURRENT_ROTOR_SECTOR = 19
    
    # Machine-Side Controller Group (20-24)
    CONTROLLER_FIRING_ANGLE = 20
    CONTROLLER_MACHINE_SPEED = 21
    CONTROLLER_DC_LINK_CURRENT = 22
    CONTROLLER_RESERVED_1 = 23
    CONTROLLER_RESERVED_2 = 24
    
    # Torque Controller Group (25-29)
    TORQUE_CURRENT_REFERENCE = 25
    TORQUE_BASE_REFERENCE = 26
    TORQUE_TOTAL_REFERENCE = 27
    TORQUE_LIMITED_REFERENCE = 28
    TORQUE_IS_SATURATED = 29
    
    # Speed Controller Group (30-34)
    SPEED_TORQUE_REFERENCE = 30
    SPEED_MEASURED_SPEED_PU = 31
    SPEED_ERROR = 32
    SPEED_PI_OUTPUT = 33
    SPEED_IS_SATURATED = 34
    
    # Speed Generator Group (35-39)
    GENERATOR_SPEED_REFERENCE = 35
    GENERATOR_TORQUE_LIMIT_MAX = 36
    GENERATOR_TORQUE_LIMIT_MIN = 37
    GENERATOR_IS_PROFILE_COMPLETE = 38
    GENERATOR_IS_BREAKAWAY_BOOST_ACTIVE = 39
    
    # Excitation Controller Group (40-44)
    EXCITATION_VOLTAGE_SETPOINT = 40
    EXCITATION_ENABLE = 41
    EXCITATION_IS_FIELD_ESTABLISHED = 42
    EXCITATION_IS_RUNNING = 43
    EXCITATION_RESERVED = 44
    
    # Pulse Generator Group (45-49)
    PULSE_P1 = 45
    PULSE_P2 = 46
    PULSE_P3 = 47
    PULSE_P4 = 48
    PULSE_P5 = 49


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


# Machine-side specific data classes
@dataclass
class MachineMeasurements:
    """Machine-side measurements."""
    machine_voltage_magnitude_filtered_pu: float = 0.0
    machine_voltage_a_pu: float = 0.0
    machine_voltage_b_pu: float = 0.0
    machine_voltage_c_pu: float = 0.0
    machine_side_speed: float = 0.0
    machine_voltage_magnitude: float = 0.0
    is_commuting: bool = False
    machine_voltage_alpha_pu: float = 0.0
    machine_voltage_beta_pu: float = 0.0


@dataclass
class MachineControllerData:
    """Machine-side controller data."""
    controller_firing_angle: float = 0.0
    controller_machine_speed: float = 0.0
    controller_dc_link_current: float = 0.0


@dataclass
class TorqueControllerData:
    """Torque controller data."""
    torque_current_reference_pu: float = 0.0
    torque_base_reference: float = 0.0
    torque_total_reference: float = 0.0
    torque_limited_reference: float = 0.0
    torque_is_saturated: bool = False


@dataclass
class SpeedControllerData:
    """Speed controller data."""
    speed_torque_reference: float = 0.0
    speed_measured_speed_pu: float = 0.0
    speed_error: float = 0.0
    speed_pi_output: float = 0.0
    speed_is_saturated: bool = False


@dataclass
class SpeedGeneratorData:
    """Speed generator data."""
    generator_speed_reference_pu: float = 0.0
    generator_torque_limit_max_pu: float = 0.0
    generator_torque_limit_min_pu: float = 0.0
    generator_is_profile_complete: bool = False
    generator_is_breakaway_boost_active: bool = False


@dataclass
class ExcitationControllerData:
    """Excitation controller data."""
    excitation_voltage_setpoint_pu: float = 0.0
    excitation_enable: bool = False
    excitation_is_field_established: bool = False
    excitation_is_running: bool = False


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
            is_commuting=self.get_bool_signal(GridSignalIndex.IS_COMMUTING),
            grid_voltage_alpha_pu=self.get_signal(GridSignalIndex.GRID_VOLTAGE_ALPHA),
            grid_voltage_beta_pu=self.get_signal(GridSignalIndex.GRID_VOLTAGE_BETA),
            grid_side_frequency_hz=self.get_signal(GridSignalIndex.GRID_SIDE_FREQUENCY),
        )
    
    @property
    def observer(self) -> ObserverData:
        """Extract observer estimates from signal array."""
        return ObserverData(
            estimated_voltage_magnitude_pu=self.get_signal(GridSignalIndex.ESTIMATED_VOLTAGE_MAGNITUDE),
            estimated_frequency_pu=self.get_signal(GridSignalIndex.ESTIMATED_FREQUENCY_PU),
            estimated_angle_rad=self.get_signal(GridSignalIndex.ESTIMATED_ANGLE_RAD),
            estimated_frequency_hz=self.get_signal(GridSignalIndex.ESTIMATED_FREQUENCY_HZ),
            estimated_voltage_alpha_pu=self.get_signal(GridSignalIndex.ESTIMATED_VOLTAGE_ALPHA),
            estimated_voltage_beta_pu=self.get_signal(GridSignalIndex.ESTIMATED_VOLTAGE_BETA),
        )
    
    @property
    def modulator(self) -> ModulatorData:
        """Extract modulator signals from signal array."""
        return ModulatorData(
            is_enabled=self.get_bool_signal(GridSignalIndex.MODULATOR_ENABLED),
            firing_angle_input_pu=self.get_signal(GridSignalIndex.CONTROLLER_OUTPUT),
            firing_angle_output_pu=self.get_signal(GridSignalIndex.FIRING_ANGLE_RAD),
            force_commutation_ack=self.get_bool_signal(GridSignalIndex.FORCE_COMMUTATION_ACK),
            is_requesting_pi_reset=self.get_bool_signal(GridSignalIndex.IS_REQUESTING_PI_RESET),
        )
    
    @property
    def controller(self) -> ControllerData:
        """Extract controller state from signal array."""
        return ControllerData(
            dc_link_current_reference=self.get_signal(GridSignalIndex.DC_LINK_CURRENT_REFERENCE),
            dc_link_current_measured=self.get_signal(GridSignalIndex.DC_LINK_CURRENT_MEASURED),
            current_error_pu=self.get_signal(GridSignalIndex.CURRENT_ERROR),
            controller_output_pu=self.get_signal(GridSignalIndex.CONTROLLER_OUTPUT),
            operating_mode=self.get_bool_signal(GridSignalIndex.OPERATING_MODE),
            controller_saturated=self.get_bool_signal(GridSignalIndex.CONTROLLER_SATURATED),
            freeze_integrator=self.get_bool_signal(GridSignalIndex.FREEZE_INTEGRATOR),
        )
    
    @property
    def state_machine(self) -> StateMachineData:
        """Extract state machine flags from signal array."""
        return StateMachineData(
            current_state=int(self.get_signal(GridSignalIndex.STATE_MACHINE_STATE)),
            pulse_releases=self.get_bool_signal(GridSignalIndex.PULSE_RELEASES),
            auxiliaries_are_on=self.get_bool_signal(GridSignalIndex.AUXILIARIES_ON),
            breakers_are_on=self.get_bool_signal(GridSignalIndex.BREAKERS_ON),
            start_command=self.get_bool_signal(GridSignalIndex.START_COMMAND),
            stop_command=self.get_bool_signal(GridSignalIndex.STOP_COMMAND),
            is_current_extinguished=self.get_bool_signal(GridSignalIndex.CURRENT_EXTINGUISHED),
            is_shutdown_requested=self.get_bool_signal(GridSignalIndex.SHUTDOWN_REQUESTED),
        )
    
    @property
    def operation_mode_monitor(self) -> OperationModeMonitorData:
        """Extract operation mode monitor from signal array."""
        return OperationModeMonitorData(
            mode=int(self.get_signal(GridSignalIndex.OPERATION_MODE)),
            mode_changed=self.get_bool_signal(GridSignalIndex.MODE_CHANGED),
            current_zero=self.get_bool_signal(GridSignalIndex.CURRENT_ZERO),
            discontinuous_mode=self.get_bool_signal(GridSignalIndex.DISCONTINUOUS_MODE),
        )
    
    @property
    def pulse_generator(self) -> PulseGeneratorData:
        """Extract pulse generator outputs from signal array."""
        return PulseGeneratorData(
            firing_pulse_1=self.get_bool_signal(GridSignalIndex.FIRING_PULSE_1),
            firing_pulse_2=self.get_bool_signal(GridSignalIndex.FIRING_PULSE_2),
            firing_pulse_3=self.get_bool_signal(GridSignalIndex.FIRING_PULSE_3),
            firing_pulse_4=self.get_bool_signal(GridSignalIndex.FIRING_PULSE_4),
            firing_pulse_5=self.get_bool_signal(GridSignalIndex.FIRING_PULSE_5),
            firing_pulse_6=self.get_bool_signal(GridSignalIndex.FIRING_PULSE_6),
            is_active=self.get_bool_signal(GridSignalIndex.PULSE_GENERATOR_ACTIVE),
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
            machine_voltage_magnitude_filtered_pu=self.get_signal(MachineSignalIndex.MACHINE_VOLTAGE_MAGNITUDE_FILTERED),
            machine_voltage_a_pu=self.get_signal(MachineSignalIndex.MACHINE_VOLTAGE_A),
            machine_voltage_b_pu=self.get_signal(MachineSignalIndex.MACHINE_VOLTAGE_B),
            machine_voltage_c_pu=self.get_signal(MachineSignalIndex.MACHINE_VOLTAGE_C),
            machine_side_speed=self.get_signal(MachineSignalIndex.MACHINE_SIDE_SPEED),
            machine_voltage_magnitude=self.get_signal(MachineSignalIndex.MACHINE_VOLTAGE_MAGNITUDE),
            is_commuting=self.get_bool_signal(MachineSignalIndex.IS_COMMUTING),
            machine_voltage_alpha_pu=self.get_signal(MachineSignalIndex.MACHINE_VOLTAGE_ALPHA),
            machine_voltage_beta_pu=self.get_signal(MachineSignalIndex.MACHINE_VOLTAGE_BETA),
        )
    
    @property
    def observer(self) -> ObserverData:
        """Extract machine observer estimates from signal array."""
        return ObserverData(
            estimated_voltage_magnitude_pu=self.get_signal(MachineSignalIndex.ESTIMATED_VOLTAGE_MAGNITUDE),
            estimated_frequency_pu=self.get_signal(MachineSignalIndex.ESTIMATED_FREQUENCY_PU),
            estimated_angle_rad=self.get_signal(MachineSignalIndex.ESTIMATED_ANGLE_RAD),
            estimated_frequency_hz=self.get_signal(MachineSignalIndex.ESTIMATED_FREQUENCY_HZ),
            estimated_voltage_alpha_pu=self.get_signal(MachineSignalIndex.ESTIMATED_VOLTAGE_ALPHA),
            estimated_voltage_beta_pu=self.get_signal(MachineSignalIndex.ESTIMATED_VOLTAGE_ALPHA),
        )
    
    @property
    def modulator(self) -> ModulatorData:
        """Extract machine modulator signals from signal array."""
        return ModulatorData(
            is_enabled=self.get_bool_signal(MachineSignalIndex.MODULATOR_ENABLED),
            firing_angle_input_pu=self.get_signal(MachineSignalIndex.CURRENT_ROTOR_SECTOR),
            firing_angle_output_pu=self.get_signal(MachineSignalIndex.FIRING_ANGLE_RAD),
            force_commutation_ack=self.get_bool_signal(MachineSignalIndex.REQUEST_FORCE_COMMUTATION),
            is_requesting_pi_reset=self.get_bool_signal(MachineSignalIndex.RELEASE_PULSES),
        )
    
    @property
    def controller(self) -> MachineControllerData:
        """Extract machine controller state from signal array."""
        return MachineControllerData(
            controller_firing_angle=self.get_signal(MachineSignalIndex.CONTROLLER_FIRING_ANGLE),
            controller_machine_speed=self.get_signal(MachineSignalIndex.CONTROLLER_MACHINE_SPEED),
            controller_dc_link_current=self.get_signal(MachineSignalIndex.CONTROLLER_DC_LINK_CURRENT),
        )
    
    @property
    def torque_controller(self) -> TorqueControllerData:
        """Extract torque controller state from signal array."""
        return TorqueControllerData(
            torque_current_reference_pu=self.get_signal(MachineSignalIndex.TORQUE_CURRENT_REFERENCE),
            torque_base_reference=self.get_signal(MachineSignalIndex.TORQUE_BASE_REFERENCE),
            torque_total_reference=self.get_signal(MachineSignalIndex.TORQUE_TOTAL_REFERENCE),
            torque_limited_reference=self.get_signal(MachineSignalIndex.TORQUE_LIMITED_REFERENCE),
            torque_is_saturated=self.get_bool_signal(MachineSignalIndex.TORQUE_IS_SATURATED),
        )
    
    @property
    def speed_controller(self) -> SpeedControllerData:
        """Extract speed controller state from signal array."""
        return SpeedControllerData(
            speed_torque_reference=self.get_signal(MachineSignalIndex.SPEED_TORQUE_REFERENCE),
            speed_measured_speed_pu=self.get_signal(MachineSignalIndex.SPEED_MEASURED_SPEED_PU),
            speed_error=self.get_signal(MachineSignalIndex.SPEED_ERROR),
            speed_pi_output=self.get_signal(MachineSignalIndex.SPEED_PI_OUTPUT),
            speed_is_saturated=self.get_bool_signal(MachineSignalIndex.SPEED_IS_SATURATED),
        )
    
    @property
    def speed_generator(self) -> SpeedGeneratorData:
        """Extract speed generator state from signal array."""
        return SpeedGeneratorData(
            generator_speed_reference_pu=self.get_signal(MachineSignalIndex.GENERATOR_SPEED_REFERENCE),
            generator_torque_limit_max_pu=self.get_signal(MachineSignalIndex.GENERATOR_TORQUE_LIMIT_MAX),
            generator_torque_limit_min_pu=self.get_signal(MachineSignalIndex.GENERATOR_TORQUE_LIMIT_MIN),
            generator_is_profile_complete=self.get_bool_signal(MachineSignalIndex.GENERATOR_IS_PROFILE_COMPLETE),
            generator_is_breakaway_boost_active=self.get_bool_signal(MachineSignalIndex.GENERATOR_IS_BREAKAWAY_BOOST_ACTIVE),
        )
    
    @property
    def excitation_controller(self) -> ExcitationControllerData:
        """Extract excitation controller state from signal array."""
        return ExcitationControllerData(
            excitation_voltage_setpoint_pu=self.get_signal(MachineSignalIndex.EXCITATION_VOLTAGE_SETPOINT),
            excitation_enable=self.get_bool_signal(MachineSignalIndex.EXCITATION_ENABLE),
            excitation_is_field_established=self.get_bool_signal(MachineSignalIndex.EXCITATION_IS_FIELD_ESTABLISHED),
            excitation_is_running=self.get_bool_signal(MachineSignalIndex.EXCITATION_IS_RUNNING),
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
    
    @property
    def pulse_generator(self) -> PulseGeneratorData:
        """Extract machine pulse generator outputs from signal array."""
        return PulseGeneratorData(
            firing_pulse_1=self.get_bool_signal(MachineSignalIndex.PULSE_P1),
            firing_pulse_2=self.get_bool_signal(MachineSignalIndex.PULSE_P2),
            firing_pulse_3=self.get_bool_signal(MachineSignalIndex.PULSE_P3),
            firing_pulse_4=self.get_bool_signal(MachineSignalIndex.PULSE_P4),
            firing_pulse_5=self.get_bool_signal(MachineSignalIndex.PULSE_P5),
            firing_pulse_6=False,  # Not available on machine side (only 5 pulses)
        )


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
