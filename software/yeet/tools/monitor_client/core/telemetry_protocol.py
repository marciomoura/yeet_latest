#!/usr/bin/env python3
"""Telemetry protocol decoder for SFC control system.

This module handles the low-level binary telemetry protocol decoding and packet
structure. Signal indices and data models are imported from configuration modules:
  - config.grid_side_telemetry_protocol: Grid-side signals and dataclasses
  - config.machine_side_telemetry_protocol: Machine-side signals and dataclasses
"""

import struct
import zlib
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Optional, List, Tuple, Union
import logging
import os
from pathlib import Path

# Set up logger (inherits level from root logger, controlled by --verbose flag)
logger = logging.getLogger(__name__)


def enable_telemetry_debug_log(log_file: Optional[str] = None) -> bool:
    """Enable telemetry debug logging to file.
    
    Args:
        log_file: Path to log file. If None, uses './telemetry_debug.log' in current directory.
    
    Returns:
        True if log file handler was successfully added, False otherwise.
    """
    if log_file is None:
        log_file = os.path.join(os.getcwd(), 'telemetry_debug.log')
    
    try:
        log_path = Path(log_file)
        log_path.parent.mkdir(parents=True, exist_ok=True)
        
        fh = logging.FileHandler(str(log_path), mode='w')
        fh.setLevel(logging.DEBUG)
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        fh.setFormatter(formatter)
        logger.addHandler(fh)
        logger.info("="*80)
        logger.info(f"TELEMETRY DECODER DEBUG LOG INITIALIZED: {log_path}")
        logger.info("="*80)
        return True
    except Exception as e:
        # Log to console if file creation fails
        logger.warning(f"Could not create telemetry debug log file '{log_file}': {e}")
        return False

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
    GridProtectionsData,
    FiberOpticRxFromMachineData,
    GridHardwareDebugData,
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
    MachineProtectionsData,
    MachinePulseGeneratorData,
    MachineFiberOpticRxData,
)


# Protocol constants
PROTOCOL_VERSION = 2  # v2: signals are uint32 (floats bit-cast, booleans can be packed)
MAX_SIGNALS = 256
MAX_PACKET_SIZE = 16 + (MAX_SIGNALS * 4)
SYNC_MAGIC = 0xDEADBEEF  # Magic number for stream synchronization


def uint32_to_float(raw: int) -> float:
    """Reinterpret uint32 bits as IEEE 754 float.
    
    Protocol v2 transmits all signals as uint32_t. Floats are bit-cast
    on the C++ side and need to be reinterpreted here.
    """
    return struct.unpack('<f', struct.pack('<I', raw))[0]


def unpack_bools(packed: int, count: int = 32) -> list[bool]:
    """Unpack a uint32 into a list of booleans.
    
    Protocol v2 can pack up to 32 booleans into a single uint32.
    Bit 0 is the first boolean, bit 31 is the last.
    """
    return [(packed >> i) & 1 == 1 for i in range(count)]


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
    """Grid-side telemetry packet with variable-length signal array.
    
    Protocol v2 with grouped boolean packing:
    - Total signals: 46 (indices 0-45)
    - Indices 44-45: Grouped boolean packs (40 bools → 2 uint32)
    """
    header: TelemetryPacketHeader
    signals: List[float]
    raw_signals: List[int] = field(default_factory=list)  # Raw uint32 values before float conversion
    
    def __post_init__(self):
        """Automatically expand signals array with unpacked booleans for backward compatibility."""
        self._expand_signals_with_bools()

    def _expand_signals_with_bools(self):
        """Unpack booleans from packs and append them to the signals array.
        
        This allows the UI and other components to access packed booleans
        via virtual indices (46+ for grid).
        """
        # Unpack booleans
        bools1 = self._get_bool_pack_1()
        bools2 = self._get_bool_pack_2()
        all_bools = bools1 + bools2
        
        # Append as floats (1.0 or 0.0)
        for b in all_bools:
            self.signals.append(1.0 if b else 0.0)

    def get_signal(self, index: int, default: float = 0.0) -> float:
        """Get signal by index with bounds checking."""
        if 0 <= index < len(self.signals):
            return self.signals[index]
        return default
    
    def get_raw_signal(self, index: int, default: int = 0) -> int:
        """Get raw uint32 signal by index (for boolean packs)."""
        if self.raw_signals and 0 <= index < len(self.raw_signals):
            return self.raw_signals[index]
        return default
    
    def get_bool_signal(self, index: int, default: bool = False) -> bool:
        """Get boolean signal (converts 1.0/0.0 to True/False)."""
        return bool(self.get_signal(index, 1.0 if default else 0.0))
    
    def _get_bool_pack_1(self) -> list[bool]:
        """Unpack boolean pack #1 (index 42, bits 0-31).
        
        Returns 32 booleans in order:
        - Bits 0-2: Modulator control (release_pulses, force_comm_ack, freeze_integrator)
        - Bits 3-4: Current controller status (is_active, is_saturated)
        - Bits 5-10: State machine flags (6 flags)
        - Bits 11-13: Mode monitor (has_mode_changed, current_is_zero, is_discontinuous_mode)
        - Bits 14-17: Pulse generator (release_pulses_output, release_pulses_input, virtual_ref_active, individual_override)
        - Bits 18-23: Firing pulses (p1-p6)
        - Bits 24-30: Protections part 1 (is_tripped, is_inhibited, any_fault_active, overcurrent_tripped, overvoltage_tripped, undervoltage_tripped, overcurrent_warning)
        """
        raw = self.get_raw_signal(GridSignalIndex.BOOL_PACK_1, 0)
        logger.debug(f"_get_bool_pack_1: raw_signal[42] = 0x{raw:08X} ({raw})")
        bools = unpack_bools(raw, 32)
        logger.debug(f"_get_bool_pack_1: unpacked {len(bools)} bools: {bools[:8]}...")
        return bools
    
    def _get_bool_pack_2(self) -> list[bool]:
        """Unpack boolean pack #2 (index 43, bits 0-19).
        
        Returns 20 booleans in order:
        - Bits 0-3: Protections part 2 (overvoltage_warning, undervoltage_warning, comm_loss_tripped, comm_loss_warning)
        - Bits 4-6: Fiber optic RX (request_force_comm, machine_protection_tripped, protection_inhibit)
        - Bits 7-10: Extra trips (door_open, external_fault, emergency_off, machine_side)
        - Bits 11-18: Alarms (overcurrent, overvoltage, undervoltage, comm_loss, door_open, external_fault, emergency_off, machine_side)
        - Bit 19: Emergency Off reset pulse
        """
        raw = self.get_raw_signal(GridSignalIndex.BOOL_PACK_2, 0)
        logger.debug(f"_get_bool_pack_2: raw_signal[43] = 0x{raw:08X} ({raw})")
        bools = unpack_bools(raw, 20)
        logger.debug(f"_get_bool_pack_2: unpacked {len(bools)} bools: {bools}")
        return bools
    
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
        bools1 = self._get_bool_pack_1()
        return GridModulatorData(
            release_pulses_input=self.get_bool_signal(GridSignalIndex.MODULATOR_RELEASE_PULSES_INPUT),
            firing_angle_input_pu=self.get_signal(GridSignalIndex.MODULATOR_FIRING_ANGLE_INPUT),
            firing_angle_output_pu=self.get_signal(GridSignalIndex.MODULATOR_FIRING_ANGLE_OUTPUT),
            operating_mode=int(self.get_signal(GridSignalIndex.MODULATOR_OPERATING_MODE)),
            request_extinguish_dc=self.get_bool_signal(GridSignalIndex.MODULATOR_REQUEST_EXTINGUISH_DC),
            request_force_commutation=self.get_bool_signal(GridSignalIndex.MODULATOR_REQUEST_FORCE_COMMUTATION),
            modulator_state=int(self.get_signal(GridSignalIndex.MODULATOR_STATE)),
            release_pulses_output=bools1[0],  # Bit 0: Modulator release_pulses (output)
        )
    
    @property
    def controller(self) -> GridControllerData:
        """Extract controller state from signal array."""
        bools1 = self._get_bool_pack_1()
        return GridControllerData(
            current_reference=self.get_signal(GridSignalIndex.CURRENT_REFERENCE),
            current_measured=self.get_signal(GridSignalIndex.CURRENT_MEASURED),
            freeze_integrator=self.get_bool_signal(GridSignalIndex.CURRENT_FREEZE_INTEGRATOR),
            preset_request=self.get_bool_signal(GridSignalIndex.CURRENT_PRESET_REQUEST),
            current_reference_output=self.get_signal(GridSignalIndex.CURRENT_REFERENCE_OUTPUT),
            current_error_pu=self.get_signal(GridSignalIndex.CURRENT_ERROR),
            pi_output_pu=self.get_signal(GridSignalIndex.CURRENT_PI_OUTPUT),
            is_active=bools1[3],  # Bit 3: Current controller is_active
            is_saturated=bools1[4],  # Bit 4: Current controller is_saturated
        )
    
    @property
    def state_machine(self) -> StateMachineData:
        """Extract state machine flags from signal array."""
        bools1 = self._get_bool_pack_1()
        return StateMachineData(
            current_state=int(self.get_signal(GridSignalIndex.STATE_MACHINE_STATE)),
            release_pulses=bools1[5],  # Bit 5: State machine release_pulses
            auxiliaries_are_on=bools1[6],  # Bit 6: State machine auxiliaries_on
            mcb_is_closed=bools1[7],  # Bit 7: State machine breaker_closed
            start_command=bools1[8],  # Bit 8: State machine start_cmd
            stop_command=bools1[9],  # Bit 9: State machine stop_cmd
            is_current_extinguished=bools1[10],  # Bit 10: State machine current_extinguished
        )
    
    @property
    def operation_mode_monitor(self) -> OperationModeMonitorData:
        """Extract operation mode monitor from signal array."""
        bools1 = self._get_bool_pack_1()
        return OperationModeMonitorData(
            preset_request=self.get_bool_signal(GridSignalIndex.STATE_MACHINE_PRESET_REQUEST),
            has_mode_changed=bools1[11],  # Bit 11: Mode monitor mode_changed
            current_is_zero=bools1[12],  # Bit 12: Mode monitor current_zero
            is_discontinuous_mode=bools1[13],  # Bit 13: Mode monitor discontinuous_mode
        )
    
    @property
    def pulse_generator(self) -> PulseGeneratorData:
        """Extract pulse generator outputs from signal array."""
        bools1 = self._get_bool_pack_1()
        return PulseGeneratorData(
            release_pulses=bools1[14],  # Bit 14: Pulse gen release_pulses (output)
            actual_firing_angle_pu=self.get_signal(GridSignalIndex.PULSE_GENERATOR_ACTUAL_FIRING_ANGLE),
            release_pulses_input=bools1[15],  # Bit 15: Pulse gen release_pulses (input)
            virtual_reference_active=bools1[16],  # Bit 16: Pulse gen virtual_ref_active
            virtual_reference_angle_pu=self.get_signal(GridSignalIndex.PULSE_GENERATOR_VIRTUAL_REFERENCE_ANGLE),
            individual_override_active=bools1[17],  # Bit 17: Pulse gen individual_override
            p1=bools1[18],  # Bit 18: Firing pulse p1
            p2=bools1[19],  # Bit 19: Firing pulse p2
            p3=bools1[20],  # Bit 20: Firing pulse p3
            p4=bools1[21],  # Bit 21: Firing pulse p4
            p5=bools1[22],  # Bit 22: Firing pulse p5
            p6=bools1[23],  # Bit 23: Firing pulse p6
        )
    
    @property
    def protections(self) -> GridProtectionsData:
        """Extract protection flags from signal array."""
        bools1 = self._get_bool_pack_1()
        bools2 = self._get_bool_pack_2()
        return GridProtectionsData(
            is_tripped=bools1[24],  # Bit 24
            is_inhibited=bools1[25],  # Bit 25
            any_fault_active=bools1[26],  # Bit 26
            first_trip_source=int(self.get_signal(GridSignalIndex.FIRST_TRIP_SOURCE)),
            overcurrent_tripped=bools1[27],  # Bit 27
            overvoltage_tripped=bools1[28],  # Bit 28
            undervoltage_tripped=bools1[29],  # Bit 29
            overcurrent_warning=bools1[30],  # Bit 30
            overvoltage_warning=bools2[0],  # Pack 2, Bit 0
            undervoltage_warning=bools2[1],  # Pack 2, Bit 1
            communication_loss_tripped=bools2[2],  # Pack 2, Bit 2
            communication_loss_warning=bools2[3],  # Pack 2, Bit 3
            door_open_tripped=bools2[7],     # Pack 2, Bit 7
            external_fault_tripped=bools2[8], # Pack 2, Bit 8
            emergency_off_tripped=bools2[9],  # Pack 2, Bit 9
            machine_side_protection_tripped=bools2[10], # Pack 2, Bit 10
            emergency_off_reset=bools2[19]    # Pack 2, Bit 19
        )
    
    @property
    def fiber_rx(self) -> FiberOpticRxFromMachineData:
        """Extract fiber optic RX signals from signal array."""
        bools2 = self._get_bool_pack_2()
        return FiberOpticRxFromMachineData(
            measured_speed_pu=self.get_signal(GridSignalIndex.FIBER_RX_MEASURED_SPEED),
            dc_link_current_reference_pu=self.get_signal(GridSignalIndex.FIBER_RX_DC_LINK_CURRENT_REFERENCE),
            machine_voltage_magnitude_pu=self.get_signal(GridSignalIndex.FIBER_RX_MACHINE_VOLTAGE_MAGNITUDE),
            force_commutation_request=bools2[4],  # Pack 2, Bit 4
            machine_protection_tripped=bools2[5],  # Pack 2, Bit 5
            protection_inhibit=bools2[6],  # Pack 2, Bit 6
        )

    @property
    def hardware_debug(self) -> GridHardwareDebugData:
        """Extract hardware debug signals from signal array."""
        return GridHardwareDebugData(
            voltage_ab_counts_peak=self.get_signal(GridSignalIndex.VOLTAGE_AB_COUNTS_PEAK),
            voltage_bc_counts_peak=self.get_signal(GridSignalIndex.VOLTAGE_BC_COUNTS_PEAK),
            current_a_counts_peak=self.get_signal(GridSignalIndex.CURRENT_A_COUNTS_PEAK),
            current_c_counts_peak=self.get_signal(GridSignalIndex.CURRENT_C_COUNTS_PEAK),
        )


@dataclass
class MachineSideTelemetryPacket:
    """Machine-side telemetry packet with variable-length signal array.
    
    Protocol v2 with grouped boolean packing:
    - Total signals: 52 (indices 0-51)
    - Indices 50-51: Grouped boolean packs (37 bools → 2 uint32)
    """
    header: TelemetryPacketHeader
    signals: List[float]
    raw_signals: List[int] = field(default_factory=list)  # Raw uint32 values before float conversion
    
    def __post_init__(self):
        """Automatically expand signals array with unpacked booleans for backward compatibility."""
        self._expand_signals_with_bools()

    def _expand_signals_with_bools(self):
        """Unpack booleans from packs and append them to the signals array.
        
        This allows the UI and other components to access packed booleans
        via virtual indices (52+ for machine).
        """
        # Unpack booleans
        bools1 = self._get_bool_pack_1()
        bools2 = self._get_bool_pack_2()
        all_bools = bools1 + bools2
        
        # Append as floats (1.0 or 0.0)
        for b in all_bools:
            self.signals.append(1.0 if b else 0.0)

    def get_signal(self, index: int, default: float = 0.0) -> float:
        """Get signal by index with bounds checking."""
        if 0 <= index < len(self.signals):
            return self.signals[index]
        return default
    
    def get_raw_signal(self, index: int, default: int = 0) -> int:
        """Get raw uint32 signal by index (for boolean packs)."""
        if self.raw_signals and 0 <= index < len(self.raw_signals):
            return self.raw_signals[index]
        return default
    
    def get_bool_signal(self, index: int, default: bool = False) -> bool:
        """Get boolean signal (converts 1.0/0.0 to True/False)."""
        return bool(self.get_signal(index, 1.0 if default else 0.0))

    def _get_bool_pack_1(self) -> list[bool]:
        """Unpack boolean pack #1 (index 51, bits 0-31).
        
        Returns 32 booleans in order:
        - Bits 0-2: Modulator status (release_pulses, request_force_comm, force_comm_ack)
        - Bits 3-7: Torque controller (enable_boost, is_sat_in, is_sat_out, is_active, is_boost_active)
        - Bit 8: Speed controller (is_saturated)
        - Bits 9-13: Startup sequencer (rel_speed, rel_exc, rel_torque, rel_pulses, is_complete)
        - Bits 14-15: Pulse generator (virtual_ref_active, individual_override)
        - Bits 16-21: Firing pulses (p1-p6)
        - Bits 22-31: Protections part 1 (is_tripped, is_inhibited, any_fault, over_flux_t, under_flux_t, stall_t, over_flux_w, under_flux_w, stall_w, comm_loss_t)
        """
        raw = self.get_raw_signal(MachineSignalIndex.BOOL_PACK_1, 0)
        return unpack_bools(raw, 32)
    
    def _get_bool_pack_2(self) -> list[bool]:
        """Unpack boolean pack #2 (index 52, bits 0-18).
        
        Returns 19 booleans in order:
        - Bit 0: Protection part 2 (comm_loss_w)
        - Bits 1-4: Fiber optic RX (is_forced_comm, force_comm_ack, release_pulses, grid_rel_prot)
        - Bits 5-14: Protections part 3 (overspeed_t, overspeed_a, overvoltage_t, overvoltage_a, loe_t, loe_a, ext_fault_t, ext_fault_a, machine_ov_w, spare)
        - Bits 15-18: Alarms (over_flux_a, under_flux_a, stall_a, comm_loss_a)
        """
        raw = self.get_raw_signal(MachineSignalIndex.BOOL_PACK_2, 0)
        return unpack_bools(raw, 19)

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
            estimated_frequency_hz=self.get_signal(MachineSignalIndex.ESTIMATED_FREQUENCY_HZ),
            v_ab_counts_peak=self.get_signal(MachineSignalIndex.MACHINE_V_AB_COUNTS_PEAK),
            v_bc_counts_peak=self.get_signal(MachineSignalIndex.MACHINE_V_BC_COUNTS_PEAK),
        )
    
    @property
    def observer(self) -> MachineObserverData:
        """Extract machine observer estimates from signal array."""
        return MachineObserverData(
            estimated_frequency_pu=self.get_signal(MachineSignalIndex.ESTIMATED_FREQUENCY_PU),
            estimated_reference_angle_pu=self.get_signal(MachineSignalIndex.ESTIMATED_REFERENCE_ANGLE),
            estimated_frequency_hz=self.get_signal(MachineSignalIndex.ESTIMATED_FREQUENCY_HZ),
        )
    
    @property
    def modulator(self) -> MachineModulatorData:
        """Extract machine modulator signals from signal array."""
        bools1 = self._get_bool_pack_1()
        return MachineModulatorData(
            release_pulses_input=self.get_signal(MachineSignalIndex.MODULATOR_RELEASE_PULSES_INPUT) > 0.5,
            operating_mode=int(self.get_signal(MachineSignalIndex.MODULATOR_OPERATING_MODE)),
            release_pulses_output=bools1[0],
            request_force_commutation=bools1[1],
            force_commutation_acknowledged=bools1[2],
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
        bools1 = self._get_bool_pack_1()
        return TorqueControllerOutputsData(
            current_reference_pu=self.get_signal(MachineSignalIndex.TORQUE_CURRENT_REFERENCE),
            is_saturated=bools1[5],
            is_active=bools1[6],
            is_breakaway_boost_active=bools1[7],
        )
    
    @property
    def speed_controller(self) -> SpeedControllerData:
        """Extract speed controller state from signal array."""
        bools1 = self._get_bool_pack_1()
        return SpeedControllerData(
            speed_reference_pu=self.get_signal(MachineSignalIndex.SPEED_SPEED_REFERENCE),
            measured_speed_pu=self.get_signal(MachineSignalIndex.SPEED_MEASURED_SPEED),
            torque_reference_pu=self.get_signal(MachineSignalIndex.SPEED_TORQUE_REFERENCE),
            pi_output_pu=self.get_signal(MachineSignalIndex.SPEED_PI_OUTPUT),
            is_saturated=bools1[8],
        )
    
    @property
    def startup_sequencer(self) -> StartupSequencerData:
        """Extract startup sequencer state from signal array."""
        bools1 = self._get_bool_pack_1()
        return StartupSequencerData(
            speed_reference_pu=self.get_signal(MachineSignalIndex.STARTUP_SPEED_REFERENCE),
            virtual_rotor_frequency_pu=self.get_signal(MachineSignalIndex.STARTUP_VIRTUAL_ROTOR_FREQUENCY),
            virtual_rotor_angle_pu=self.get_signal(MachineSignalIndex.STARTUP_VIRTUAL_ROTOR_ANGLE),
            virtual_torque_reference_pu=self.get_signal(MachineSignalIndex.STARTUP_VIRTUAL_TORQUE_REFERENCE),
            release_speed_controller=bools1[9],
            release_excitation=bools1[10],
            release_torque_controller=bools1[11],
            release_machine_side_pulses=bools1[12],
            is_startup_complete=bools1[13],
        )
    
    @property
    def excitation_controller(self) -> ExcitationControllerData:
        """Extract excitation controller state from signal array."""
        return ExcitationControllerData(
            field_current_reference_pu=self.get_signal(MachineSignalIndex.EXCITATION_FIELD_CURRENT_REFERENCE),
            prefluxing_elapsed_time_s=self.get_signal(MachineSignalIndex.EXCITATION_PREFLUXING_ELAPSED_TIME),
            machine_voltage_reference_pu=self.get_signal(MachineSignalIndex.EXCITATION_MACHINE_VOLTAGE_REFERENCE_PU),
            machine_voltage_magnitude_pu=self.get_signal(MachineSignalIndex.EXCITATION_MACHINE_VOLTAGE_MAGNITUDE_PU),
            machine_voltage_error_pu=self.get_signal(MachineSignalIndex.EXCITATION_MACHINE_VOLTAGE_ERROR_PU),
        )
    
    @property
    def state_machine(self) -> StateMachineData:
        """Extract machine state machine flags from signal array (empty for machine side)."""
        return StateMachineData(
            current_state=0,
            pulse_releases=False,
            auxiliaries_are_on=False,
            mcb_is_closed=False,
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
            is_discontinuous_mode=False,
        )

    @property
    def protections(self) -> MachineProtectionsData:
        """Extract machine protection status from signal array."""
        bools1 = self._get_bool_pack_1()
        bools2 = self._get_bool_pack_2()
        return MachineProtectionsData(
            is_tripped=bools1[22],
            is_inhibited=bools1[23],
            any_fault_active=bools1[24],
            first_trip_source=int(self.get_signal(MachineSignalIndex.FIRST_TRIP_SOURCE)),
            over_flux_tripped=bools1[25],
            under_flux_tripped=bools1[26],
            stall_tripped=bools1[27],
            over_flux_warning=bools1[28],
            under_flux_warning=bools1[29],
            stall_warning=bools1[30],
            communication_loss_tripped=bools1[31],
            communication_loss_warning=bools2[0],
            overspeed_tripped=bools2[5],
            overspeed_alarmed=bools2[6],
            machine_overvoltage_tripped=bools2[7],
            machine_overvoltage_alarmed=bools2[8],
            loss_of_excitation_tripped=bools2[9],
            loss_of_excitation_alarmed=bools2[10],
            external_fault_tripped=bools2[11],
            external_fault_alarmed=bools2[12],
            emergency_off_tripped=bools2[13],
            emergency_off_alarmed=bools2[14],
        )

    @property
    def pulse_generator(self) -> MachinePulseGeneratorData:
        """Extract machine pulse generator state from signal array."""
        bools1 = self._get_bool_pack_1()
        return MachinePulseGeneratorData(
            virtual_reference_active=bools1[14],
            virtual_reference_angle_pu=self.get_signal(MachineSignalIndex.PULSE_GENERATOR_VIRTUAL_REFERENCE_ANGLE),
            individual_override_active=bools1[15],
            p1=bools1[16],
            p2=bools1[17],
            p3=bools1[18],
            p4=bools1[19],
            p5=bools1[20],
            p6=bools1[21],
        )

    @property
    def fiber_optic(self) -> MachineFiberOpticRxData:
        """Extract fiber optic RX data from signal array."""
        bools2 = self._get_bool_pack_2()
        return MachineFiberOpticRxData(
            dc_link_current_pu=self.get_signal(MachineSignalIndex.FIBER_RX_DC_LINK_CURRENT),
            grid_voltage_magnitude_pu=self.get_signal(MachineSignalIndex.FIBER_RX_GRID_VOLTAGE_MAGNITUDE),
            current_state_id=int(self.get_signal(MachineSignalIndex.FIBER_RX_STATE_ID)),
            is_forced_commutation=bools2[1],
            force_commutation_acknowledged=bools2[2],
            release_pulses=bools2[3],
            grid_release_protections=bools2[4],
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
    
    def __init__(self, require_crc: bool = False, debug_log: Optional[str] = None):
        """Initialize telemetry decoder.
        
        Args:
            require_crc: If True, require and validate CRC32 at end of packet.
                         If False, support both CRC and non-CRC packets (for backward compatibility).
            debug_log: Path to debug log file. If provided, enables detailed decoding logs.
        """
        self.require_crc = require_crc
        
        # Enable debug logging if requested
        if debug_log:
            enable_telemetry_debug_log(debug_log)
    
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
        logger.debug(f"decode() called with {len(data)} bytes")
        
        if len(data) < self.HEADER_SIZE:
            logger.warning(f"Data too short: {len(data)} < {self.HEADER_SIZE}")
            if verbose:
                hex_preview = data[:min(16, len(data))].hex() if len(data) > 0 else "(empty)"
                # print(f"[DECODE ERROR] Data too short: {len(data)} bytes, expected >= {self.HEADER_SIZE} bytes. Data (hex): {hex_preview}")
                pass
            return None
            
        try:
            header_data = struct.unpack(self.HEADER_FORMAT, data[:self.HEADER_SIZE])
            logger.debug(f"Header unpacked: {header_data}")
        except struct.error as e:
            logger.error(f"Header unpack failed: {e}")
            if verbose:
                # print(f"[DECODE ERROR] Failed to unpack header: {e}")
                # print(f"[DECODE ERROR] Data (hex): {data[:self.HEADER_SIZE].hex()}")
                pass
            return None
            
        header = TelemetryPacketHeader(
            packet_type=header_data[0],
            protocol_version=header_data[1],
            signal_count=header_data[2],
            timestamp_ms=header_data[3],
            sequence_number=header_data[4],
            reserved=header_data[5]
        )
        
        logger.debug(f"Decoded header: type=0x{header.packet_type:02X}, ver={header.protocol_version}, signals={header.signal_count}, seq={header.sequence_number}")
        
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
        
        # Only support protocol v2 (uint32 with bit-cast floats and packed bools)
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
        raw_signals = []  # Keep raw uint32 values for boolean packs
        offset = self.HEADER_SIZE
        
        logger.debug(f"Decoding {header.signal_count} signals starting at offset {offset}")
        
        for i in range(header.signal_count):
            if offset + self.SIGNAL_SIZE > len(data):
                logger.error(f"Insufficient data for signal {i} at offset {offset}")
                if verbose:
                    print(f"[DECODE ERROR] Insufficient data for signal {i} at offset {offset}")
                return None
            
            try:
                # Protocol v2: signals are uint32, reinterpret as float
                raw_uint32 = struct.unpack('<I', data[offset:offset + self.SIGNAL_SIZE])[0]
                raw_signals.append(raw_uint32)  # Store raw value
                signal_value = uint32_to_float(raw_uint32)  # Convert to float
                signals.append(signal_value)
                
                # Log boolean packs specifically
                if i == 44 or i == 45:
                    logger.debug(f"  Signal[{i}] (BOOL_PACK): raw=0x{raw_uint32:08X}, as_float={signal_value}")
                
                offset += self.SIGNAL_SIZE
            except struct.error as e:
                logger.error(f"Failed to unpack signal {i}: {e}")
                if verbose:
                    print(f"[DECODE ERROR] Failed to unpack signal {i}: {e}")
                return None
        
        logger.debug(f"Successfully decoded {len(signals)} signals, {len(raw_signals)} raw values")
        
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
            logger.debug("Creating GridSideTelemetryPacket")
            packet: Union[GridSideTelemetryPacket, MachineSideTelemetryPacket] = GridSideTelemetryPacket(
                header=header,
                signals=signals,
                raw_signals=raw_signals
            )
        else:  # MACHINE_SIDE_TELEMETRY
            logger.debug("Creating MachineSideTelemetryPacket")
            packet = MachineSideTelemetryPacket(
                header=header,
                signals=signals,
                raw_signals=raw_signals
            )
        
        logger.debug(f"Packet created successfully, returning {offset} bytes consumed")
        return (packet, offset)
