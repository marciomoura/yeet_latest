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
    MODE_MONITOR_OPERATING_MODE = 20
    
    # === Current Controller Group (21-27) ===
    CURRENT_REFERENCE = 21
    CURRENT_MEASURED = 22
    CURRENT_FREEZE_INTEGRATOR = 23
    CURRENT_PRESET_REQUEST = 24
    CURRENT_REFERENCE_OUTPUT = 25
    CURRENT_ERROR = 26
    CURRENT_PI_OUTPUT = 27
    
    # === State Machine Group (28-29) ===
    STATE_MACHINE_STATE = 28
    STATE_MACHINE_PRESET_REQUEST = 29
    
    # === Pulse Generator Group (30-31) ===
    PULSE_GENERATOR_ACTUAL_FIRING_ANGLE = 30
    PULSE_GENERATOR_VIRTUAL_REFERENCE_ANGLE = 31
    
    # === Grid-Side Protections Severity/Status Group (32-36) ===
    PROTECTION_OVERCURRENT_STATUS = 32
    PROTECTION_OVERVOLTAGE_STATUS = 33
    PROTECTION_UNDERVOLTAGE_STATUS = 34
    PROTECTION_COMMUNICATION_LOSS_STATUS = 35
    PROTECTION_DOOR_OPEN_STATUS = 36

    # === Fiber Optic RX from Machine-Side Group (37-39) ===
    FIBER_RX_MEASURED_SPEED = 37
    FIBER_RX_DC_LINK_CURRENT_REFERENCE = 38
    FIBER_RX_MACHINE_VOLTAGE_MAGNITUDE = 39

    # === Commutation Notch Duration Group (40-41) ===
    COMMUTATION_NOTCH_DURATION_LAST_S = 40
    COMMUTATION_NOTCH_DURATION_FILTERED_S = 41
    
    # === Grouped Boolean Packs (42-43) ===
    # Word 1 (42): Modulator(3) + CurrentCtrl(2) + StateMachine(6) + ModeMonitor(3) + PulseGen(4) + FiringPulses(6) + Protections(7) = 31 bits
    # Word 2 (43): Protections(4) + FiberOptic(3) = 7 bits
    BOOL_PACK_1 = 42
    BOOL_PACK_2 = 43
    
    # === Line Voltages (Unmasked) Group (44-46) ===
    GRID_VOLTAGE_LINE_AB = 44
    GRID_VOLTAGE_LINE_BC = 45
    GRID_VOLTAGE_LINE_CA = 46
    
    # === Phase Currents (SI) Group (47-49) ===
    GRID_CURRENT_A_SI = 47
    GRID_CURRENT_B_SI = 48
    GRID_CURRENT_C_SI = 49
    
    # === Filtered Voltage/Current Group (50-55) ===
    GRID_VOLTAGE_ABC_A_FILTERED = 50
    GRID_VOLTAGE_ABC_B_FILTERED = 51
    GRID_VOLTAGE_ABC_C_FILTERED = 52
    GRID_VOLTAGE_MAGNITUDE_FILTERED_SI = 53
    # Reserved for future filtered current signals
    # GRID_CURRENT_A_FILTERED = 54
    # GRID_CURRENT_B_FILTERED = 55

    # === Grid-Side Digital Inputs (56-64) ===
    DI_AUXILIARIES_ARE_ON = 56
    DI_MV_BREAKER_IS_CLOSED = 57
    DI_START_COMMAND = 58
    DI_STOP_COMMAND = 59
    DI_MV_BREAKER_IS_TRIPPING = 60
    DI_EMERGENCY_OFF = 61
    DI_DOOR_IS_OPEN = 62
    DI_EXTERNAL_FAULT = 63
    DI_FANS_ARE_RUNNING = 64

    # === Grid-Side Digital Outputs (65-74) ===
    DO_MV_BREAKER_CLOSE = 65
    DO_MV_BREAKER_OPEN = 66
    DO_AUXILIARIES_ON_COMMAND = 67
    DO_ON_COMMAND_ACU_FAN_1 = 68
    DO_ON_COMMAND_ACU_FAN_2 = 69
    DO_ON_COMMAND_ACU_FAN_3 = 70
    DO_ON_COMMAND_ACU_FAN_4 = 71
    DO_SOFTWARE_TRIP_1 = 72
    DO_SOFTWARE_TRIP_2 = 73
    DO_SOFTWARE_TRIP_3 = 74

    # === Additional Protection Status Signals (75-78) ===
    PROTECTION_EXTERNAL_FAULT_STATUS = 75
    PROTECTION_EMERGENCY_OFF_STATUS = 76
    PROTECTION_MACHINE_PROTECTION_STATUS = 77
    PROTECTION_ANY_ALARM_ACTIVE = 78

    # === Additional Fiber Optic RX Boolean Signals (79-82) ===
    FIBER_RX_REQUEST_FORCE_COMMUTATION_BOOL = 79
    FIBER_RX_MACHINE_PROTECTION_TRIPPED_BOOL = 80
    FIBER_RX_PROTECTION_INHIBIT_BOOL = 81
    FIBER_RX_COMMUNICATION_OK = 82

    # === Virtual Boolean Indices (83+) ===
    # These indices point to the expanded signals array after _expand_signals_with_bools()
    # Regular signals end at index 82, so bools1[0] starts at index 83
    # 
    # Unpacked from BOOL_PACK_1 (42) - bits 0-31 appended at indices 83-114
    MODULATOR_RELEASE_PULSES_OUTPUT = 83      # bools1[0]
    FORCE_COMMUTATION_ACKNOWLEDGED = 84       # bools1[1]
    MODULATOR_FREEZE_INTEGRATOR = 85          # bools1[2]
    CURRENT_CONTROLLER_IS_ACTIVE = 86         # bools1[3]
    CURRENT_CONTROLLER_IS_SATURATED = 87      # bools1[4]
    STATE_MACHINE_RELEASE_PULSES = 88         # bools1[5]
    STATE_MACHINE_AUXILIARIES_ON = 89         # bools1[6]
    STATE_MACHINE_BREAKER_CLOSED = 90         # bools1[7]
    STATE_MACHINE_START_CMD = 91              # bools1[8]
    STATE_MACHINE_STOP_CMD = 92               # bools1[9]
    STATE_MACHINE_CURRENT_EXTINGUISHED = 93   # bools1[10]
    MODE_MONITOR_HAS_MODE_CHANGED = 94        # bools1[11]
    MODE_MONITOR_CURRENT_IS_ZERO = 95         # bools1[12]
    MODE_MONITOR_IS_DISCONTINUOUS_MODE = 96   # bools1[13]
    PULSE_GENERATOR_RELEASE_PULSES = 97       # bools1[14]
    PULSE_GENERATOR_RELEASE_PULSES_INPUT = 98 # bools1[15]
    PULSE_GENERATOR_VIRTUAL_REFERENCE_ACTIVE = 99   # bools1[16]
    PULSE_GENERATOR_INDIVIDUAL_OVERRIDE_ACTIVE = 100 # bools1[17]
    PULSE_P1 = 101                            # bools1[18]
    PULSE_P2 = 102                            # bools1[19]
    PULSE_P3 = 103                            # bools1[20]
    PULSE_P4 = 104                            # bools1[21]
    PULSE_P5 = 105                            # bools1[22]
    PULSE_P6 = 106                            # bools1[23]
    PROTECTION_IS_TRIPPED = 107               # bools1[24]
    PROTECTION_IS_INHIBITED = 108             # bools1[25]
    PROTECTION_ANY_ALARM_ACTIVE_UNPACKED = 109  # bools1[26] - has_active_alarm from bool pack
    PROTECTION_OVERCURRENT_TRIPPED = 110      # bools1[27]
    PROTECTION_OVERVOLTAGE_TRIPPED = 111      # bools1[28]
    PROTECTION_UNDERVOLTAGE_TRIPPED = 112     # bools1[29]
    PROTECTION_OVERCURRENT_WARNING = 113      # bools1[30]

    # Unpacked from BOOL_PACK_2 (43) - bits 0-7 appended at indices 115-122
    PROTECTION_OVERVOLTAGE_WARNING = 115      # bools2[0]
    PROTECTION_UNDERVOLTAGE_WARNING = 116     # bools2[1]
    PROTECTION_COMMUNICATION_LOSS_TRIPPED = 117 # bools2[2]
    PROTECTION_COMMUNICATION_LOSS_WARNING = 118 # bools2[3]
    FIBER_RX_REQUEST_FORCE_COMMUTATION = 119  # bools2[4]
    FIBER_RX_MACHINE_PROTECTION_TRIPPED = 120 # bools2[5]
    FIBER_RX_PROTECTION_INHIBIT = 121         # bools2[6]


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
    """Fiber optic signals received from machine-side (68-74)."""
    measured_speed_pu: float = 0.0
    dc_link_current_reference_pu: float = 0.0
    machine_voltage_magnitude_pu: float = 0.0
    force_commutation_request: bool = False
    machine_protection_tripped: bool = False
    protection_inhibit: bool = False
