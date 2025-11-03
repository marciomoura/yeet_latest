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
    # === Grid Measurements Group (0-13) ===
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
    
    # === Observer Group (14-19) ===
    ESTIMATED_VOLTAGE_MAGNITUDE = 14
    ESTIMATED_FREQUENCY_PU = 15
    ESTIMATED_ANGLE_RAD = 16
    ESTIMATED_FREQUENCY_HZ = 17
    ESTIMATED_VOLTAGE_ALPHA = 18
    ESTIMATED_VOLTAGE_BETA = 19
    
    # === Modulator Group (20-28) ===
    FIRING_ANGLE_INPUT = 20
    FIRING_ANGLE_OUTPUT = 21
    MODULATOR_OPERATING_MODE = 22
    MODULATOR_REQUEST_EXTINGUISH_DC = 23
    MODULATOR_REQUEST_FORCE_COMMUTATION = 24
    MODULATOR_ENABLED = 25
    FORCE_COMMUTATION_ACK = 26
    IS_REQUESTING_PI_RESET = 27
    MODULATOR_STATE = 28
    
    # === Controller Group (29-37) ===
    DC_LINK_CURRENT_REFERENCE = 29
    DC_LINK_CURRENT_MEASURED = 30
    FREEZE_INTEGRATOR = 31
    MODULATOR_FREEZE_INTEGRATOR = 32
    CURRENT_ERROR = 33
    PI_OUTPUT = 34
    NATURAL_COMMUTATION_MODE = 35
    CONTROLLER_SATURATED = 36
    STATE_MACHINE_FREEZE_INTEGRATOR = 37
    
    # === State Machine Group (38-45) ===
    STATE_MACHINE_STATE = 38
    PULSE_RELEASES = 39
    AUXILIARIES_ON = 40
    BREAKERS_ON = 41
    START_COMMAND = 42
    STOP_COMMAND = 43
    CURRENT_EXTINGUISHED = 44
    SHUTDOWN_REQUESTED = 45
    
    # === Operation Mode Monitor Group (46-49) ===
    OPERATION_MODE = 46
    MODE_CHANGED = 47
    CURRENT_ZERO = 48
    DISCONTINUOUS_MODE = 49
    
    # === Pulse Generator Group (50-57) ===
    FIRING_PULSE_1 = 50
    FIRING_PULSE_2 = 51
    FIRING_PULSE_3 = 52
    FIRING_PULSE_4 = 53
    FIRING_PULSE_5 = 54
    FIRING_PULSE_6 = 55
    PULSE_GENERATOR_ACTIVE = 56
    ACTUAL_FIRING_ANGLE = 57


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
class GridObserverData:
    """Grid-side observer estimates."""
    estimated_voltage_magnitude_pu: float = 0.0
    estimated_frequency_pu: float = 0.0
    estimated_angle_rad: float = 0.0
    estimated_frequency_hz: float = 0.0
    estimated_voltage_alpha_pu: float = 0.0
    estimated_voltage_beta_pu: float = 0.0


@dataclass
class GridModulatorData:
    """Grid-side modulator control signals."""
    firing_angle_input_pu: float = 0.0
    firing_angle_output_pu: float = 0.0
    operating_mode: int = 0
    request_extinguish_dc: bool = False
    request_force_commutation: bool = False
    is_enabled: bool = False
    force_commutation_ack: bool = False
    is_requesting_pi_reset: bool = False
    modulator_state: int = 0


@dataclass
class GridControllerData:
    """Grid-side current controller state."""
    dc_link_current_reference: float = 0.0
    dc_link_current_measured: float = 0.0
    current_error_pu: float = 0.0
    controller_output_pu: float = 0.0
    operating_mode: bool = False
    controller_saturated: bool = False
    freeze_integrator: bool = False


@dataclass
class StateMachineData:
    """State machine flags (grid-side only)."""
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
    """Operation mode monitor (grid-side only)."""
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
