#!/usr/bin/env python3
"""Machine-side telemetry signal indices and data models.

This module defines signal array indices and dataclasses for machine-side telemetry,
matching the C++ machine_side_telemetry_builder.cpp packet structure.

The signal indices define the position of each signal in the telemetry packet's
signal array (0-49), and the dataclasses provide strongly-typed access to groups of signals.
"""

from dataclasses import dataclass
from enum import IntEnum


class MachineSignalIndex(IntEnum):
    """Signal array indices for MACHINE_SIDE_TELEMETRY matching C++ machine_side_telemetry_builder.cpp."""
    # Machine Measurements Group (0-6)
    MACHINE_VOLTAGE_ABC_A = 0
    MACHINE_VOLTAGE_ABC_B = 1
    MACHINE_VOLTAGE_ABC_C = 2
    MACHINE_VOLTAGE_MAGNITUDE_FILTERED = 3
    ESTIMATED_SPEED = 4
    ESTIMATED_VOLTAGE_MAGNITUDE = 5
    
    # Observer Group (6-8)
    ESTIMATED_FREQUENCY_PU = 6
    ESTIMATED_REFERENCE_ANGLE = 7
    ESTIMATED_FREQUENCY = 8
    
    # Modulator Group (9-16)
    MODULATOR_RELEASE_PULSES_INPUT = 9
    MODULATOR_OPERATING_MODE = 10
    MODULATOR_RELEASE_PULSES_OUTPUT = 11
    MODULATOR_REQUEST_FORCE_COMMUTATION = 12
    MODULATOR_FORCE_COMMUTATION_ACK = 13
    MODULATOR_FIRING_ANGLE = 14
    MODULATOR_CURRENT_ROTOR_SECTOR = 15
    MODULATOR_COMMUTATION_STATE = 16
    
    # Machine-Side Controller Group (17-19)
    CONTROLLER_FIRING_ANGLE = 17
    CONTROLLER_DC_LINK_CURRENT = 18
    CONTROLLER_MACHINE_SPEED = 19
    
    # Torque Controller Group (20-28)
    TORQUE_TORQUE_REFERENCE = 20
    TORQUE_DYNAMIC_MAX_LIMIT = 21
    TORQUE_DYNAMIC_MIN_LIMIT = 22
    TORQUE_ENABLE_BREAKAWAY_BOOST = 23
    TORQUE_IS_SATURATED_INPUT = 24
    TORQUE_CURRENT_REFERENCE = 25
    TORQUE_IS_SATURATED_OUTPUT = 26
    TORQUE_IS_ACTIVE = 27
    TORQUE_IS_BREAKAWAY_BOOST_ACTIVE = 28
    
    # Speed Controller Group (29-33)
    SPEED_SPEED_REFERENCE = 29
    SPEED_MEASURED_SPEED = 30
    SPEED_TORQUE_REFERENCE = 31
    SPEED_PI_OUTPUT = 32
    SPEED_IS_SATURATED = 33
    
    # Speed Generator Group (34-38)
    SPEED_GEN_SPEED_REFERENCE = 34
    SPEED_GEN_TORQUE_LIMIT_MAX = 35
    SPEED_GEN_TORQUE_LIMIT_MIN = 36
    SPEED_GEN_IS_PROFILE_COMPLETE = 37
    SPEED_GEN_IS_BREAKAWAY_BOOST_ACTIVE = 38
    
    # Excitation Controller Group (39-42)
    EXCITATION_VOLTAGE_SETPOINT = 39
    EXCITATION_ENABLE = 40
    EXCITATION_IS_FIELD_ESTABLISHED = 41
    EXCITATION_IS_RUNNING = 42


@dataclass
class MachineMeasurements:
    """Machine-side voltage and observer data (0-8)."""
    machine_voltage_abc_a_pu: float = 0.0
    machine_voltage_abc_b_pu: float = 0.0
    machine_voltage_abc_c_pu: float = 0.0
    machine_voltage_magnitude_filtered_pu: float = 0.0
    estimated_speed_pu: float = 0.0
    estimated_voltage_magnitude_pu: float = 0.0
    estimated_frequency_pu: float = 0.0
    estimated_reference_angle_pu: float = 0.0
    estimated_frequency_hz: float = 0.0


@dataclass
class MachineObserverData:
    """Machine-side observer estimates."""
    estimated_frequency_pu: float = 0.0
    estimated_reference_angle_pu: float = 0.0
    estimated_frequency_hz: float = 0.0


@dataclass
class MachineModulatorData:
    """Machine-side modulator control signals (9-16)."""
    release_pulses_input: bool = False
    operating_mode: int = 0
    release_pulses_output: bool = False
    request_force_commutation: bool = False
    force_commutation_acknowledged: bool = False
    firing_angle_pu: float = 0.0
    current_rotor_sector_normalized: float = 0.0
    commutation_state: int = 0


@dataclass
class MachineControllerData:
    """Machine-side controller outputs (17-19)."""
    firing_angle_rad: float = 0.0
    dc_link_current_pu: float = 0.0
    machine_speed_pu: float = 0.0


@dataclass
class TorqueControllerInputsData:
    """Torque controller inputs (20-24)."""
    torque_reference_pu: float = 0.0
    dynamic_max_torque_limit_pu: float = 0.0
    dynamic_min_torque_limit_pu: float = 0.0
    enable_breakaway_boost: bool = False
    is_saturated: bool = False


@dataclass
class TorqueControllerOutputsData:
    """Torque controller outputs and state (25-28)."""
    current_reference_pu: float = 0.0
    is_saturated: bool = False
    is_active: bool = False
    is_breakaway_boost_active: bool = False


@dataclass
class SpeedControllerData:
    """Speed controller state and outputs (29-33)."""
    speed_reference_pu: float = 0.0
    measured_speed_pu: float = 0.0
    torque_reference_pu: float = 0.0
    pi_output_pu: float = 0.0
    is_saturated: bool = False


@dataclass
class SpeedGeneratorData:
    """Speed generator configuration and state (34-38)."""
    speed_reference_pu: float = 0.0
    torque_limit_max_pu: float = 0.0
    torque_limit_min_pu: float = 0.0
    is_profile_complete: bool = False
    is_breakaway_boost_active: bool = False


@dataclass
class ExcitationControllerData:
    """Excitation controller state and outputs (39-42)."""
    voltage_setpoint_pu: float = 0.0
    enable: bool = False
    is_field_established: bool = False
    is_running: bool = False
