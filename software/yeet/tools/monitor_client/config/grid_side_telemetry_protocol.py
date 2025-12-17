#!/usr/bin/env python3
"""Grid-side telemetry signal indices and data models.

This module defines signal array indices and dataclasses for grid-side telemetry,
matching the C++ grid_side_telemetry_builder.cpp packet structure.

The signal indices define the position of each signal in the telemetry packet's
signal array, and the dataclasses provide strongly-typed access to groups of signals.
"""

from dataclasses import dataclass
from enum import IntEnum
from typing import List


class GridSignalIndex(IntEnum):
    """Signal array indices for GRID_SIDE_TELEMETRY matching C++ grid_side_telemetry_builder.cpp."""
    # === Grid Measurements Group (0-8) ===
    GRID_VOLTAGE_MAGNITUDE_FILTERED = 0
    GRID_VOLTAGE_A = 1
    GRID_VOLTAGE_B = 2
    GRID_VOLTAGE_C = 3
    GRID_CURRENT_MAGNITUDE = 4
    GRID_CURRENT_A = 5
    GRID_CURRENT_B = 6
    GRID_CURRENT_C = 7
    DC_LINK_CURRENT = 8
    
    # === Observer Group (9-12) ===
    ESTIMATED_VOLTAGE_MAGNITUDE = 9
    ESTIMATED_FREQUENCY_PU = 10
    ESTIMATED_REFERENCE_ANGLE = 11
    OBSERVER_ERROR = 12
    
    # === Modulator Group (13-20) ===
    MODULATOR_RELEASE_PULSES_INPUT = 13
    MODULATOR_FIRING_ANGLE_INPUT = 14
    MODULATOR_FIRING_ANGLE_OUTPUT = 15
    MODULATOR_OPERATING_MODE = 16
    MODULATOR_REQUEST_EXTINGUISH_DC = 17
    MODULATOR_REQUEST_FORCE_COMMUTATION = 18
    MODULATOR_STATE = 19
    MODULATOR_RELEASE_PULSES_OUTPUT = 20
    
    # === Modulator Control Group (21-23) ===
    MODULATOR_FORCE_COMMUTATION_ACK = 21
    MODULATOR_FREEZE_INTEGRATOR = 22
    MODE_MONITOR_OPERATING_MODE = 23
    
    # === Current Controller Group (24-32) ===
    CURRENT_REFERENCE = 24
    CURRENT_MEASURED = 25
    CURRENT_FREEZE_INTEGRATOR = 26
    CURRENT_PRESET_REQUEST = 27
    CURRENT_REFERENCE_OUTPUT = 28
    CURRENT_ERROR = 29
    CURRENT_PI_OUTPUT = 30
    CURRENT_IS_ACTIVE = 31
    CURRENT_IS_SATURATED = 32
    
    # === State Machine Group (33-40) ===
    STATE_MACHINE_STATE = 33
    STATE_MACHINE_RELEASE_PULSES = 34
    STATE_MACHINE_AUXILIARIES_ON = 35
    STATE_MACHINE_BREAKERS_CLOSED = 36
    STATE_MACHINE_START_COMMAND = 37
    STATE_MACHINE_STOP_COMMAND = 38
    STATE_MACHINE_CURRENT_EXTINGUISHED = 39
    STATE_MACHINE_SHUTDOWN_REQUEST = 40
    
    # === Operation Mode Monitor Group (41-44) ===
    STATE_MACHINE_PRESET_REQUEST = 41
    MODE_MONITOR_HAS_MODE_CHANGED = 42
    MODE_MONITOR_CURRENT_IS_ZERO = 43
    MODE_MONITOR_IS_DISCONTINUOUS_MODE = 44
    
    # === Pulse Generator Group (45-56) ===
    PULSE_GENERATOR_RELEASE_PULSES = 45
    PULSE_GENERATOR_ACTUAL_FIRING_ANGLE = 46
    PULSE_GENERATOR_RELEASE_PULSES_INPUT = 47
    PULSE_GENERATOR_VIRTUAL_REFERENCE_ACTIVE = 48
    PULSE_GENERATOR_VIRTUAL_REFERENCE_ANGLE = 49
    PULSE_GENERATOR_INDIVIDUAL_OVERRIDE_ACTIVE = 50
    PULSE_GENERATOR_P1 = 51
    PULSE_GENERATOR_P2 = 52
    PULSE_GENERATOR_P3 = 53
    PULSE_GENERATOR_P4 = 54
    PULSE_GENERATOR_P5 = 55
    PULSE_GENERATOR_P6 = 56
    
    # === Grid-Side Protections Group (57-67) ===
    PROTECTION_IS_TRIPPED = 57
    PROTECTION_IS_INHIBITED = 58
    PROTECTION_ANY_FAULT_ACTIVE = 59
    PROTECTION_OVERCURRENT_TRIPPED = 60
    PROTECTION_OVERVOLTAGE_TRIPPED = 61
    PROTECTION_UNDERVOLTAGE_TRIPPED = 62
    PROTECTION_OVERCURRENT_WARNING = 63
    PROTECTION_OVERVOLTAGE_WARNING = 64
    PROTECTION_UNDERVOLTAGE_WARNING = 65
    PROTECTION_COMMUNICATION_LOSS_TRIPPED = 66
    PROTECTION_COMMUNICATION_LOSS_WARNING = 67
    
    # === Fiber Optic RX from Machine-Side Group (68-75) ===
    FIBER_RX_MEASURED_SPEED = 68
    FIBER_RX_DC_LINK_CURRENT_REFERENCE = 69
    FIBER_RX_MACHINE_VOLTAGE_MAGNITUDE = 70
    FIBER_RX_FORCE_COMMUTATION_REQUEST = 71
    FIBER_RX_SHUTDOWN_REQUEST = 72
    FIBER_RX_MACHINE_PROTECTION_TRIPPED = 73
    FIBER_RX_PROTECTION_INHIBIT = 74
    FIBER_RX_MACHINE_TRIP_ACKNOWLEDGED = 75

    # === Grid-Side Protections Severity/Status Group (76-80) ===
    PROTECTION_OVERCURRENT_STATUS = 76
    PROTECTION_OVERVOLTAGE_STATUS = 77
    PROTECTION_UNDERVOLTAGE_STATUS = 78
    PROTECTION_COMMUNICATION_LOSS_STATUS = 79
    PROTECTION_DOOR_OPEN_STATUS = 80

    # === Commutation Notch Duration Group (81-82) ===
    COMMUTATION_NOTCH_DURATION_LAST_S = 81
    COMMUTATION_NOTCH_DURATION_FILTERED_S = 82
    
    # === Line Voltages (Unmasked) Group (83-85) ===
    GRID_VOLTAGE_LINE_AB = 83
    GRID_VOLTAGE_LINE_BC = 84
    GRID_VOLTAGE_LINE_CA = 85
    
    # === Phase Currents Group (86-88) ===
    GRID_CURRENT_A_SI = 86
    GRID_CURRENT_B_SI = 87
    GRID_CURRENT_C_SI = 88
    
    # === Filtered Voltage/Current Group (89-94) ===
    GRID_VOLTAGE_ABC_A_FILTERED = 89
    GRID_VOLTAGE_ABC_B_FILTERED = 90
    GRID_VOLTAGE_ABC_C_FILTERED = 91
    GRID_VOLTAGE_MAGNITUDE_FILTERED_SI = 92


@dataclass
class GridMeasurements:
    """Grid-side measurements (0-8)."""
    grid_voltage_magnitude_pu: float = 0.0
    grid_voltage_a_pu: float = 0.0
    grid_voltage_b_pu: float = 0.0
    grid_voltage_c_pu: float = 0.0
    grid_current_magnitude_pu: float = 0.0
    grid_current_a_pu: float = 0.0
    grid_current_b_pu: float = 0.0
    grid_current_c_pu: float = 0.0
    dc_link_current_pu: float = 0.0


@dataclass
class GridObserverData:
    """Grid-side observer estimates (9-11)."""
    estimated_voltage_magnitude_pu: float = 0.0
    estimated_frequency_pu: float = 0.0
    estimated_reference_angle_pu: float = 0.0


@dataclass
class GridModulatorData:
    """Grid-side modulator control signals (12-19)."""
    release_pulses_input: bool = False
    firing_angle_input_pu: float = 0.0
    firing_angle_output_pu: float = 0.0
    operating_mode: int = 0
    request_extinguish_dc: bool = False
    request_force_commutation: bool = False
    modulator_state: int = 0
    release_pulses_output: bool = False


@dataclass
class GridModulatorControlData:
    """Grid-side modulator control (20-22)."""
    force_commutation_acknowledged: bool = False
    freeze_integrator: bool = False
    operating_mode: int = 0


@dataclass
class GridControllerData:
    """Grid-side current controller state (23-31)."""
    current_reference: float = 0.0
    current_measured: float = 0.0
    freeze_integrator: bool = False
    preset_request: bool = False
    current_reference_output: float = 0.0
    current_error_pu: float = 0.0
    pi_output_pu: float = 0.0
    is_active: bool = False
    is_saturated: bool = False


@dataclass
class StateMachineData:
    """State machine flags (32-39)."""
    current_state: int = 0
    release_pulses: bool = False
    auxiliaries_are_on: bool = False
    mcb_is_closed: bool = False
    start_command: bool = False
    stop_command: bool = False
    is_current_extinguished: bool = False
    is_shutdown_requested: bool = False


@dataclass
class OperationModeMonitorData:
    """Operation mode monitor (40-43)."""
    preset_request: bool = False
    has_mode_changed: bool = False
    current_is_zero: bool = False
    is_discontinuous_mode: bool = False


@dataclass
class PulseGeneratorData:
    """Pulse generator outputs (44-55)."""
    release_pulses: bool = False
    actual_firing_angle_pu: float = 0.0
    release_pulses_input: bool = False
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
class GridProtectionsData:
    """Grid-side protection system (56-66)."""
    is_tripped: bool = False
    is_inhibited: bool = False
    any_fault_active: bool = False
    overcurrent_tripped: bool = False
    overvoltage_tripped: bool = False
    undervoltage_tripped: bool = False
    overcurrent_warning: bool = False
    overvoltage_warning: bool = False
    undervoltage_warning: bool = False
    communication_loss_tripped: bool = False
    communication_loss_warning: bool = False


@dataclass
class FiberOpticRxFromMachineData:
    """Fiber optic signals received from machine-side (67-74)."""
    measured_speed_pu: float = 0.0
    dc_link_current_reference_pu: float = 0.0
    machine_voltage_magnitude_pu: float = 0.0
    force_commutation_request: bool = False
    shutdown_request: bool = False
    machine_protection_tripped: bool = False
    protection_inhibit: bool = False
    machine_trip_acknowledged: bool = False
