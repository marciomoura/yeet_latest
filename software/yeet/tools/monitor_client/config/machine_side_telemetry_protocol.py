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
    
    # Startup Sequencer Group (34-42)
    STARTUP_SPEED_REFERENCE = 34
    STARTUP_VIRTUAL_ROTOR_FREQUENCY = 35
    STARTUP_VIRTUAL_ROTOR_ANGLE = 36
    STARTUP_VIRTUAL_TORQUE_REFERENCE = 37
    STARTUP_RELEASE_SPEED_CONTROLLER = 38
    STARTUP_RELEASE_EXCITATION = 39
    STARTUP_RELEASE_TORQUE_CONTROLLER = 40
    STARTUP_RELEASE_MACHINE_SIDE_PULSES = 41
    STARTUP_IS_STARTUP_COMPLETE = 42
    
    # Excitation Controller Group (43-44)
    EXCITATION_FIELD_CURRENT_REFERENCE = 43
    EXCITATION_PREFLUXING_ELAPSED_TIME = 44
    
    # === Pulse Generator Group (45-53) ===
    PULSE_GENERATOR_VIRTUAL_REFERENCE_ACTIVE = 45
    PULSE_GENERATOR_VIRTUAL_REFERENCE_ANGLE = 46
    PULSE_GENERATOR_INDIVIDUAL_OVERRIDE_ACTIVE = 47
    PULSE_GENERATOR_P1 = 48
    PULSE_GENERATOR_P2 = 49
    PULSE_GENERATOR_P3 = 50
    PULSE_GENERATOR_P4 = 51
    PULSE_GENERATOR_P5 = 52
    PULSE_GENERATOR_P6 = 53
    
    # === Machine-Side Protections Group (54-64) ===
    PROTECTION_IS_TRIPPED = 54
    PROTECTION_IS_INHIBITED = 55
    PROTECTION_ANY_FAULT_ACTIVE = 56
    PROTECTION_OVER_FLUX_TRIPPED = 57
    PROTECTION_UNDER_FLUX_TRIPPED = 58
    PROTECTION_STALL_TRIPPED = 59
    PROTECTION_OVER_FLUX_WARNING = 60
    PROTECTION_UNDER_FLUX_WARNING = 61
    PROTECTION_STALL_WARNING = 62
    PROTECTION_COMMUNICATION_LOSS_TRIPPED = 63
    PROTECTION_COMMUNICATION_LOSS_WARNING = 64
    
    # === Fiber Optic RX from Grid-Side Group (65-75) ===
    FIBER_RX_DC_LINK_CURRENT = 65
    FIBER_RX_GRID_VOLTAGE_MAGNITUDE = 66
    FIBER_RX_IS_FORCED_COMMUTATION = 67
    FIBER_RX_FORCE_COMMUTATION_ACKNOWLEDGED = 68
    FIBER_RX_RELEASE_PULSES = 69
    FIBER_RX_GRID_RELEASE_PROTECTIONS = 70
    FIBER_RX_GRID_TRIP_ACKNOWLEDGED = 71
    FIBER_RX_STATE_ID = 72
    FIBER_RX_GRID_VOLTAGE_MAGNITUDE_DUP = 73
    FIBER_RX_DC_LINK_CURRENT_DUP = 74
    FIBER_RX_STATE_ID_DUP = 75


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
class StartupSequencerData:
    """Startup sequencer configuration and state (34-42)."""
    speed_reference_pu: float = 0.0
    virtual_rotor_frequency_pu: float = 0.0
    virtual_rotor_angle_pu: float = 0.0
    virtual_torque_reference_pu: float = 0.0
    release_speed_controller: bool = False
    release_excitation: bool = False
    release_torque_controller: bool = False
    release_machine_side_pulses: bool = False
    is_startup_complete: bool = False


@dataclass
class ExcitationControllerData:
    """Excitation controller state and outputs (43-44)."""
    field_current_reference_pu: float = 0.0
    prefluxing_elapsed_time_s: float = 0.0


@dataclass
class PulseGeneratorData:
    """Pulse generator outputs (45-53)."""
    virtual_reference_active: bool = False
    virtual_reference_angle_pu: float = 0.0
    individual_override_active: bool = False
    p1: bool = False
    p2: bool = False
    p3: bool = False
    p4: bool = False
    p5: bool = False
    p6: bool = False


@dataclass
class MachineProtectionsData:
    """Machine-side protection system (54-64)."""
    is_tripped: bool = False
    is_inhibited: bool = False
    any_fault_active: bool = False
    over_flux_tripped: bool = False
    under_flux_tripped: bool = False
    stall_tripped: bool = False
    over_flux_warning: bool = False
    under_flux_warning: bool = False
    stall_warning: bool = False
    communication_loss_tripped: bool = False
    communication_loss_warning: bool = False


@dataclass
class FiberOpticRxFromGridData:
    """Fiber optic signals received from grid-side (65-75)."""
    dc_link_current_pu: float = 0.0
    grid_voltage_magnitude_pu: float = 0.0
    is_forced_commutation: bool = False
    force_commutation_acknowledged: bool = False
    release_pulses: bool = False
    grid_release_protections: bool = False
    grid_trip_acknowledged: bool = False
    state_id: int = 0
