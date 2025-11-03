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
    # Machine Measurements Group (0-8)
    MACHINE_VOLTAGE_ABC_A = 0
    MACHINE_VOLTAGE_ABC_B = 1
    MACHINE_VOLTAGE_ABC_C = 2
    MACHINE_VOLTAGE_UNMASKED_A = 3
    MACHINE_VOLTAGE_UNMASKED_B = 4
    MACHINE_VOLTAGE_UNMASKED_C = 5
    MACHINE_VOLTAGE_MAGNITUDE = 6
    MACHINE_VOLTAGE_MAGNITUDE_FILTERED = 7
    IS_COMMUTING = 8
    
    # Observer Group (9-12)
    ESTIMATED_FREQUENCY_PU = 9
    ESTIMATED_POSITION_PU = 10
    FILTERED_VOLTAGE_ALPHA = 11
    FILTERED_VOLTAGE_BETA = 12
    
    # Modulator Group (13-17)
    FIRING_ANGLE_RAD = 13
    MODULATOR_ENABLED = 14
    RELEASE_PULSES = 15
    REQUEST_FORCE_COMMUTATION = 16
    CURRENT_ROTOR_SECTOR = 17
    
    # Machine-Side Controller Group (18-20)
    CONTROLLER_FIRING_ANGLE = 18
    CONTROLLER_DC_LINK_CURRENT = 19
    CONTROLLER_MACHINE_SPEED = 20
    
    # Torque Controller Inputs (21-25)
    TORQUE_REF_FROM_SPEED_CONTROLLER = 21
    TORQUE_DYNAMIC_MAX_LIMIT = 22
    TORQUE_DYNAMIC_MIN_LIMIT = 23
    TORQUE_ENABLE_BREAKAWAY_BOOST = 24
    TORQUE_CURRENT_CONTROLLER_SATURATED = 25
    
    # Torque Controller Outputs (26-33)
    TORQUE_CURRENT_REFERENCE = 26
    TORQUE_BASE_REFERENCE = 27
    TORQUE_TOTAL_REFERENCE = 28
    TORQUE_LIMITED_REFERENCE = 29
    TORQUE_IS_SATURATED = 30
    TORQUE_IS_ACTIVE = 31
    TORQUE_IS_BREAKAWAY_BOOST_ACTIVE = 32
    TORQUE_DOWNSTREAM_SATURATION = 33
    
    # Speed Controller Group (34-41)
    SPEED_REFERENCE_PU = 34
    SPEED_MEASURED_SPEED_PU = 35
    SPEED_REFERENCE_LIMITED_PU = 36
    SPEED_TORQUE_REFERENCE = 37
    SPEED_ERROR = 38
    SPEED_PI_OUTPUT = 39
    SPEED_IS_ACTIVE = 40
    SPEED_IS_SATURATED = 41
    
    # Speed Generator Group (42-46)
    SPEED_REF_PU = 42
    SPEED_TORQUE_LIMIT_MAX_PU = 43
    SPEED_TORQUE_LIMIT_MIN_PU = 44
    SPEED_IS_PROFILE_COMPLETE = 45
    SPEED_IS_BREAKAWAY_BOOST_ACTIVE = 46
    
    # Excitation Controller Group (47-50)
    EXCITATION_VOLTAGE_SETPOINT_PU = 47
    EXCITATION_ENABLE = 48
    EXCITATION_IS_FIELD_ESTABLISHED = 49
    EXCITATION_IS_RUNNING = 50


@dataclass
class MachineMeasurements:
    """Machine-side voltage and state measurements (0-8)."""
    machine_voltage_abc_a_pu: float = 0.0
    machine_voltage_abc_b_pu: float = 0.0
    machine_voltage_abc_c_pu: float = 0.0
    machine_voltage_unmasked_a_pu: float = 0.0
    machine_voltage_unmasked_b_pu: float = 0.0
    machine_voltage_unmasked_c_pu: float = 0.0
    machine_voltage_magnitude_pu: float = 0.0
    machine_voltage_magnitude_filtered_pu: float = 0.0
    is_commuting: bool = False


@dataclass
class MachineObserverData:
    """Machine-side observer estimates."""
    estimated_frequency_pu: float = 0.0
    estimated_position_pu: float = 0.0
    filtered_voltage_alpha_pu: float = 0.0
    filtered_voltage_beta_pu: float = 0.0


@dataclass
class MachineModulatorData:
    """Machine-side modulator control signals (13-17)."""
    firing_angle_rad: float = 0.0
    is_enabled: bool = False
    release_pulses: bool = False
    request_force_commutation: bool = False
    current_rotor_sector: float = 0.0


@dataclass
class MachineControllerData:
    """Machine-side controller outputs."""
    firing_angle_rad: float = 0.0
    dc_link_current_pu: float = 0.0
    machine_speed_pu: float = 0.0


@dataclass
class TorqueControllerInputsData:
    """Torque controller inputs (21-25)."""
    torque_reference_from_speed_controller_pu: float = 0.0
    dynamic_max_torque_limit_pu: float = 0.0
    dynamic_min_torque_limit_pu: float = 0.0
    enable_breakaway_boost: bool = False
    current_controller_is_saturated: bool = False


@dataclass
class TorqueControllerOutputsData:
    """Torque controller outputs and state (26-33)."""
    current_reference_pu: float = 0.0
    base_torque_reference_pu: float = 0.0
    total_torque_reference_pu: float = 0.0
    limited_torque_reference_pu: float = 0.0
    is_saturated: bool = False
    is_active: bool = False
    is_breakaway_boost_active: bool = False
    downstream_saturation: bool = False


@dataclass
class SpeedControllerData:
    """Speed controller state and outputs (34-41)."""
    speed_reference_pu: float = 0.0
    measured_speed_pu: float = 0.0
    speed_reference_limited_pu: float = 0.0
    torque_reference_pu: float = 0.0
    speed_error_pu: float = 0.0
    pi_output_pu: float = 0.0
    is_active: bool = False
    is_saturated: bool = False


@dataclass
class SpeedGeneratorData:
    """Speed generator configuration and state (42-46)."""
    speed_ref_pu: float = 0.0
    torque_limit_max_pu: float = 0.0
    torque_limit_min_pu: float = 0.0
    is_profile_complete: bool = False
    is_breakaway_boost_active: bool = False


@dataclass
class ExcitationControllerData:
    """Excitation controller state and outputs (47-50)."""
    voltage_setpoint_pu: float = 0.0
    enable: bool = False
    is_field_established: bool = False
    is_running: bool = False
