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
    # Machine Measurements Group (0-5)
    MACHINE_VOLTAGE_ABC_A = 0
    MACHINE_VOLTAGE_ABC_B = 1
    MACHINE_VOLTAGE_ABC_C = 2
    MACHINE_VOLTAGE_MAGNITUDE_FILTERED = 3
    ESTIMATED_SPEED = 4
    ESTIMATED_VOLTAGE_MAGNITUDE = 5
    
    # Observer Group (6-9)
    ESTIMATED_FREQUENCY_PU = 6
    ESTIMATED_REFERENCE_ANGLE = 7
    ESTIMATED_FREQUENCY_HZ = 8
    OBSERVER_ERROR = 9
    
    # Modulator Group (10-15)
    MODULATOR_RELEASE_PULSES_INPUT = 10
    MODULATOR_OPERATING_MODE = 11
    MODULATOR_BOOL_PACK = 12  # Bool pack: release_pulses, request_force_commutation, force_commutation_ack
    MODULATOR_FIRING_ANGLE = 13
    MODULATOR_CURRENT_ROTOR_SECTOR = 14
    MODULATOR_COMMUTATION_STATE = 15
    
    # Machine-Side Controller Group (16-18)
    CONTROLLER_FIRING_ANGLE = 16
    CONTROLLER_DC_LINK_CURRENT = 17
    CONTROLLER_MACHINE_SPEED = 18
    
    # Torque Controller Group (19-23)
    TORQUE_TORQUE_REFERENCE = 19
    TORQUE_DYNAMIC_MAX_LIMIT = 20  # Reserved
    TORQUE_DYNAMIC_MIN_LIMIT = 21  # Reserved
    TORQUE_BOOL_PACK = 22  # Bool pack: enable_breakaway_boost, is_saturated_input, is_saturated_output, is_active, is_breakaway_boost_active
    TORQUE_CURRENT_REFERENCE = 23
    
    # Speed Controller Group (24-28)
    SPEED_SPEED_REFERENCE = 24
    SPEED_MEASURED_SPEED = 25
    SPEED_TORQUE_REFERENCE = 26
    SPEED_PI_OUTPUT = 27
    SPEED_BOOL_PACK = 28  # Bool pack: is_saturated
    
    # Startup Sequencer Group (29-33)
    STARTUP_SPEED_REFERENCE = 29
    STARTUP_VIRTUAL_ROTOR_FREQUENCY = 30
    STARTUP_VIRTUAL_ROTOR_ANGLE = 31
    STARTUP_VIRTUAL_TORQUE_REFERENCE = 32
    STARTUP_BOOL_PACK = 33  # Bool pack: release_speed_controller, release_excitation, release_torque_controller, release_machine_side_pulses, is_startup_complete
    
    # Excitation Controller Group (34-38)
    EXCITATION_FIELD_CURRENT_REFERENCE = 34
    EXCITATION_PREFLUXING_ELAPSED_TIME = 35
    EXCITATION_MACHINE_VOLTAGE_REFERENCE_PU = 36
    EXCITATION_MACHINE_VOLTAGE_MAGNITUDE_PU = 37
    EXCITATION_MACHINE_VOLTAGE_ERROR_PU = 38

    # Pulse Generator Group (39)
    PULSE_GENERATOR_VIRTUAL_REFERENCE_ANGLE = 39
    
    # Machine-Side Protections Severity/Status Group (40-45)
    PROTECTION_OVER_FLUX_STATUS = 40
    PROTECTION_UNDER_FLUX_STATUS = 41
    PROTECTION_STALL_STATUS = 42
    PROTECTION_COMMUNICATION_LOSS_STATUS = 43
    PROTECTION_OVERSPEED_STATUS = 44
    PROTECTION_MACHINE_OVERVOLTAGE_STATUS = 45

    # Fiber Optic RX from Grid-Side Group (46-48)
    FIBER_RX_DC_LINK_CURRENT = 46
    FIBER_RX_GRID_VOLTAGE_MAGNITUDE = 47
    FIBER_RX_STATE_ID = 48

    # Commutation Notch Duration Group (49-50)
    COMMUTATION_NOTCH_DURATION_LAST_S = 49
    COMMUTATION_NOTCH_DURATION_FILTERED_S = 50
    
    # Grouped Boolean Packs (51-52)
    # Word 1 (51): Modulator(3) + Torque(5) + Speed(1) + Startup(5) + PulseGen(2) + FiringPulses(6) + Protections(10)
    # Word 2 (52): Protection(1) + FiberOptic(4)
    BOOL_PACK_1 = 51
    BOOL_PACK_2 = 52
    
    # Line Voltages (Unmasked) Group (53-55)
    MACHINE_VOLTAGE_LINE_AB = 53
    MACHINE_VOLTAGE_LINE_BC = 54
    MACHINE_VOLTAGE_LINE_CA = 55
    
    # Filtered Voltage Group (56-59)
    MACHINE_VOLTAGE_ABC_A_FILTERED = 56
    MACHINE_VOLTAGE_ABC_B_FILTERED = 57
    MACHINE_VOLTAGE_ABC_C_FILTERED = 58
    MACHINE_VOLTAGE_MAGNITUDE_FILTERED_SI = 59

    # === Machine-Side Digital Inputs (60-61) ===
    DI_EXCITATION_STATUS = 60
    DI_EXTERNAL_FAULT = 61

    # === Machine-Side Digital Outputs (62-64) ===
    DO_EXCITATION_ON_COMMAND = 62
    DO_EXCITATION_OFF_COMMAND = 63


    # === Machine-Side Analog Outputs (64) ===
    AO_EXCITATION_CURRENT_REFERENCE = 64

    # === Additional Protection Status Signals (65-66) ===
    PROTECTION_LOSS_OF_EXCITATION_STATUS = 65
    PROTECTION_ANY_ALARM_ACTIVE = 66

    # === Additional Fiber Optic RX Boolean Signals (67-71) ===
    FIBER_RX_IS_FORCED_COMMUTATION_BOOL = 67
    FIBER_RX_FORCE_COMMUTATION_ACKNOWLEDGED_BOOL = 68
    FIBER_RX_RELEASE_PULSES_BOOL = 69
    FIBER_RX_GRID_RELEASE_PROTECTIONS_BOOL = 70
    FIBER_RX_COMMUNICATION_OK = 71

    # === Virtual Boolean Indices (72+) ===
    # After 72 raw signals (0-71), bools are appended starting at index 72
    # Unpacked from BOOL_PACK_1 (51) - 32 bools at indices 72-103
    # Bits 0-2: Modulator (3)
    MODULATOR_RELEASE_PULSES_OUTPUT = 72        # bools1[0]
    MODULATOR_REQUEST_FORCE_COMMUTATION = 73    # bools1[1]
    MODULATOR_FORCE_COMMUTATION_ACKNOWLEDGED = 74  # bools1[2]
    # Bits 3-7: Torque controller (5)
    TORQUE_ENABLE_BREAKAWAY_BOOST = 75          # bools1[3]
    TORQUE_IS_SATURATED_INPUT = 76              # bools1[4]
    TORQUE_IS_SATURATED_OUTPUT = 77             # bools1[5]
    TORQUE_IS_ACTIVE = 78                       # bools1[6]
    TORQUE_IS_BREAKAWAY_BOOST_ACTIVE = 79       # bools1[7]
    # Bit 8: Speed controller (1)
    SPEED_IS_SATURATED = 80                     # bools1[8]
    # Bits 9-13: Startup sequencer (5)
    STARTUP_RELEASE_SPEED_CONTROLLER = 81       # bools1[9]
    STARTUP_RELEASE_EXCITATION = 82             # bools1[10]
    STARTUP_RELEASE_TORQUE_CONTROLLER = 83      # bools1[11]
    STARTUP_RELEASE_MACHINE_SIDE_PULSES = 84    # bools1[12]
    STARTUP_IS_STARTUP_COMPLETE = 85            # bools1[13]
    # Bits 14-15: Pulse generator (2)
    PULSE_GENERATOR_VIRTUAL_REFERENCE_ACTIVE = 86  # bools1[14]
    PULSE_GENERATOR_INDIVIDUAL_OVERRIDE_ACTIVE = 87  # bools1[15]
    # Bits 16-21: Firing pulses (6)
    PULSE_P1 = 88                               # bools1[16]
    PULSE_P2 = 89                               # bools1[17]
    PULSE_P3 = 90                               # bools1[18]
    PULSE_P4 = 91                               # bools1[19]
    PULSE_P5 = 92                               # bools1[20]
    PULSE_P6 = 93                               # bools1[21]
    # Bits 22-31: Protections (10)
    PROTECTION_IS_TRIPPED = 94                  # bools1[22]
    PROTECTION_IS_INHIBITED = 95                # bools1[23]
    PROTECTION_ANY_ALARM_ACTIVE_UNPACKED = 96   # bools1[24] - has_active_alarm from bool pack
    PROTECTION_OVER_FLUX_TRIPPED = 97           # bools1[25]
    PROTECTION_UNDER_FLUX_TRIPPED = 98          # bools1[26]
    PROTECTION_STALL_TRIPPED = 99               # bools1[27]
    PROTECTION_OVER_FLUX_WARNING = 100          # bools1[28]
    PROTECTION_UNDER_FLUX_WARNING = 101         # bools1[29]
    PROTECTION_STALL_WARNING = 102              # bools1[30]
    PROTECTION_COMMUNICATION_LOSS_TRIPPED = 103 # bools1[31]

    # Unpacked from BOOL_PACK_2 (52) - 15 bools at indices 104-118
    PROTECTION_COMMUNICATION_LOSS_WARNING = 104 # bools2[0]
    FIBER_RX_IS_FORCED_COMMUTATION = 105        # bools2[1]
    FIBER_RX_FORCE_COMMUTATION_ACKNOWLEDGED = 106  # bools2[2]
    FIBER_RX_RELEASE_PULSES = 107               # bools2[3]
    FIBER_RX_GRID_RELEASE_PROTECTIONS = 108     # bools2[4]
    PROTECTION_OVERSPEED_TRIPPED = 109          # bools2[5]
    PROTECTION_OVERSPEED_ALARMED = 110          # bools2[6]
    PROTECTION_MACHINE_OVERVOLTAGE_TRIPPED = 111 # bools2[7]
    PROTECTION_MACHINE_OVERVOLTAGE_ALARMED = 112 # bools2[8]
    PROTECTION_LOSS_OF_EXCITATION_TRIPPED = 113 # bools2[9]
    PROTECTION_LOSS_OF_EXCITATION_ALARMED = 114 # bools2[10]
    PROTECTION_EXTERNAL_FAULT_TRIPPED = 115     # bools2[11]
    PROTECTION_EXTERNAL_FAULT_ALARMED = 116     # bools2[12]
    PROTECTION_MACHINE_OVERVOLTAGE_WARNING_PACK2 = 117 # bools2[13]
    PROTECTION_SPARE_118 = 118                  # bools2[14]
    PROTECTION_OVER_FLUX_ALARMED = 119          # bools2[15]
    PROTECTION_UNDER_FLUX_ALARMED = 120         # bools2[16]
    PROTECTION_STALL_ALARMED = 121              # bools2[17]
    PROTECTION_COMM_LOSS_ALARMED = 122          # bools2[18]


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
class MachinePulseGeneratorData:
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
    overspeed_tripped: bool = False
    overspeed_alarmed: bool = False
    machine_overvoltage_tripped: bool = False
    machine_overvoltage_alarmed: bool = False
    loss_of_excitation_tripped: bool = False
    loss_of_excitation_alarmed: bool = False
    external_fault_tripped: bool = False
    external_fault_alarmed: bool = False
    emergency_off_tripped: bool = False
    emergency_off_alarmed: bool = False


@dataclass
class MachineFiberOpticRxData:
    """Fiber optic signals received from grid-side (66-76)."""
    dc_link_current_pu: float = 0.0
    grid_voltage_magnitude_pu: float = 0.0
    current_state_id: int = 0
    is_forced_commutation: bool = False
    force_commutation_acknowledged: bool = False
    release_pulses: bool = False
    grid_release_protections: bool = False
    # Note: grid_trip_acknowledged removed - doesn't exist in C++ fiber struct
    state_id: int = 0
