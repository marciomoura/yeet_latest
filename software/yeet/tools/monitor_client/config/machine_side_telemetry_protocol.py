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
    
    # Observer Group (6-9)
    ESTIMATED_FREQUENCY_PU = 6
    ESTIMATED_REFERENCE_ANGLE = 7
    ESTIMATED_FREQUENCY = 8
    OBSERVER_ERROR = 9
    
    # Modulator Group (10-17)
    MODULATOR_RELEASE_PULSES_INPUT = 10
    MODULATOR_OPERATING_MODE = 11
    MODULATOR_RELEASE_PULSES_OUTPUT = 12
    MODULATOR_REQUEST_FORCE_COMMUTATION = 13
    MODULATOR_FORCE_COMMUTATION_ACK = 14
    MODULATOR_FIRING_ANGLE = 15
    MODULATOR_CURRENT_ROTOR_SECTOR = 16
    MODULATOR_COMMUTATION_STATE = 17
    
    # Machine-Side Controller Group (18-20)
    CONTROLLER_FIRING_ANGLE = 18
    CONTROLLER_DC_LINK_CURRENT = 19
    CONTROLLER_MACHINE_SPEED = 20
    
    # Torque Controller Group (21-29)
    TORQUE_TORQUE_REFERENCE = 21
    TORQUE_DYNAMIC_MAX_LIMIT = 22
    TORQUE_DYNAMIC_MIN_LIMIT = 23
    TORQUE_ENABLE_BREAKAWAY_BOOST = 24
    TORQUE_IS_SATURATED_INPUT = 25
    TORQUE_CURRENT_REFERENCE = 26
    TORQUE_IS_SATURATED_OUTPUT = 27
    TORQUE_IS_ACTIVE = 28
    TORQUE_IS_BREAKAWAY_BOOST_ACTIVE = 29
    
    # Speed Controller Group (30-34)
    SPEED_SPEED_REFERENCE = 30
    SPEED_MEASURED_SPEED = 31
    SPEED_TORQUE_REFERENCE = 32
    SPEED_PI_OUTPUT = 33
    SPEED_IS_SATURATED = 34
    
    # Startup Sequencer Group (35-43)
    STARTUP_SPEED_REFERENCE = 35
    STARTUP_VIRTUAL_ROTOR_FREQUENCY = 36
    STARTUP_VIRTUAL_ROTOR_ANGLE = 37
    STARTUP_VIRTUAL_TORQUE_REFERENCE = 38
    STARTUP_RELEASE_SPEED_CONTROLLER = 39
    STARTUP_RELEASE_EXCITATION = 40
    STARTUP_RELEASE_TORQUE_CONTROLLER = 41
    STARTUP_RELEASE_MACHINE_SIDE_PULSES = 42
    STARTUP_IS_STARTUP_COMPLETE = 43
    
    # Excitation Controller Group (44-45)
    EXCITATION_FIELD_CURRENT_REFERENCE = 44
    EXCITATION_PREFLUXING_ELAPSED_TIME = 45

    # Excitation Voltage Control Diagnostics (77-79)
    EXCITATION_MACHINE_VOLTAGE_REFERENCE_PU = 77
    EXCITATION_MACHINE_VOLTAGE_MAGNITUDE_PU = 78
    EXCITATION_MACHINE_VOLTAGE_ERROR_PU = 79
    
    # === Pulse Generator Group (46-54) ===
    PULSE_GENERATOR_VIRTUAL_REFERENCE_ACTIVE = 46
    PULSE_GENERATOR_VIRTUAL_REFERENCE_ANGLE = 47
    PULSE_GENERATOR_INDIVIDUAL_OVERRIDE_ACTIVE = 48
    PULSE_GENERATOR_P1 = 49
    PULSE_GENERATOR_P2 = 50
    PULSE_GENERATOR_P3 = 51
    PULSE_GENERATOR_P4 = 52
    PULSE_GENERATOR_P5 = 53
    PULSE_GENERATOR_P6 = 54
    
    # === Machine-Side Protections Group (55-65) ===
    PROTECTION_IS_TRIPPED = 55
    PROTECTION_IS_INHIBITED = 56
    PROTECTION_ANY_FAULT_ACTIVE = 57
    PROTECTION_OVER_FLUX_TRIPPED = 58
    PROTECTION_UNDER_FLUX_TRIPPED = 59
    PROTECTION_STALL_TRIPPED = 60
    PROTECTION_OVER_FLUX_WARNING = 61
    PROTECTION_UNDER_FLUX_WARNING = 62
    PROTECTION_STALL_WARNING = 63
    PROTECTION_COMMUNICATION_LOSS_TRIPPED = 64
    PROTECTION_COMMUNICATION_LOSS_WARNING = 65
    
    # === Fiber Optic RX from Grid-Side Group (66-76) ===
    FIBER_RX_DC_LINK_CURRENT = 66
    FIBER_RX_GRID_VOLTAGE_MAGNITUDE = 67
    FIBER_RX_IS_FORCED_COMMUTATION = 68
    FIBER_RX_FORCE_COMMUTATION_ACKNOWLEDGED = 69
    FIBER_RX_RELEASE_PULSES = 70
    FIBER_RX_GRID_RELEASE_PROTECTIONS = 71
    FIBER_RX_GRID_TRIP_ACKNOWLEDGED = 72
    FIBER_RX_STATE_ID = 73
    FIBER_RX_GRID_VOLTAGE_MAGNITUDE_DUP = 74
    FIBER_RX_DC_LINK_CURRENT_DUP = 75
    FIBER_RX_STATE_ID_DUP = 76

    # === Machine-Side Protections Severity/Status Group (77-82) ===
    PROTECTION_OVER_FLUX_STATUS = 77
    PROTECTION_UNDER_FLUX_STATUS = 78
    PROTECTION_STALL_STATUS = 79
    PROTECTION_COMMUNICATION_LOSS_STATUS = 80
    PROTECTION_OVERSPEED_STATUS = 81
    PROTECTION_MACHINE_OVERVOLTAGE_STATUS = 82

    # === Commutation Notch Duration Group (83-84) ===
    COMMUTATION_NOTCH_DURATION_LAST_S = 83
    COMMUTATION_NOTCH_DURATION_FILTERED_S = 84
    
    # === Line Voltages (Unmasked) Group (85-87) ===
    MACHINE_VOLTAGE_LINE_AB = 85
    MACHINE_VOLTAGE_LINE_BC = 86
    MACHINE_VOLTAGE_LINE_CA = 87
    
    # === Filtered Voltage Group (88-91) ===
    MACHINE_VOLTAGE_ABC_A_FILTERED = 88
    MACHINE_VOLTAGE_ABC_B_FILTERED = 89
    MACHINE_VOLTAGE_ABC_C_FILTERED = 90
    MACHINE_VOLTAGE_MAGNITUDE_FILTERED_SI = 91


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
    """Excitation controller state and outputs (43-44, 76-78)."""
    field_current_reference_pu: float = 0.0
    prefluxing_elapsed_time_s: float = 0.0

    machine_voltage_reference_pu: float = 0.0
    machine_voltage_magnitude_pu: float = 0.0
    machine_voltage_error_pu: float = 0.0


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
