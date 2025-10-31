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
    is_enabled: bool = False
    firing_angle_input_pu: float = 0.0
    firing_angle_output_pu: float = 0.0
    force_commutation_ack: bool = False
    is_requesting_pi_reset: bool = False


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
