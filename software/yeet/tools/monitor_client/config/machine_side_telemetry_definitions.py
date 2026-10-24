#!/usr/bin/env python3
"""Machine-side telemetry signal definitions for monitor displays.

This module defines the declarative structure for machine-side telemetry panels,
mapping signal indices from machine_side_telemetry_builder.cpp to human-readable
names and display parameters.

Signal indices correspond to:
  - 0-5:   Machine Measurements (Voltage ABC, magnitude filtered, estimated speed, estimated voltage magnitude)
  - 6-8:   Observer (Frequency pu, reference angle, frequency Hz)
  - 9-16:  Modulator (Release pulses in/out, operating mode, force commutation, firing angle, sector, state)
  - 17-19: Machine Controller (Firing angle, DC current, speed)
  - 20-28: Torque Controller (Torque reference, limits, breakaway boost, current reference, saturation)
  - 29-33: Speed Controller (Speed ref, measured speed, torque ref, PI output, saturation)
  - 34-38: Speed Generator (Speed ref, torque limits, profile complete, breakaway boost active)
    - 43-44: Excitation Controller (Field current ref, prefluxing time)
    - 76-78: Excitation Voltage Control Diagnostics (Vref, Vmeas, Verr)
"""

from typing import Dict, List, Any
from collections import OrderedDict
from .machine_side_telemetry_protocol import MachineSignalIndex


def _get_section_order() -> List[str]:
    """Get the canonical display order for telemetry sections.
    
    This defines the preferred order for section display across all telemetry panels.
    Not all sections may be present in every configuration.
    
    Returns:
        List of section keys in preferred display order
    """
    return [
        'digital_io',
        'system',
        'protections',
        'fiber_optic_rx',
        'measurements',
        'modulator',
        'excitation_controller',
        'startup_sequencer',
        'speed_torque_controller',
        'pulse_generator',
    ]


def get_machine_side_telemetry() -> Dict[str, Dict[str, Any]]:
    """Get telemetry definitions for machine-side monitoring.
    
    Returns:
        OrderedDict with section definitions in canonical order.
    """
    all_sections = {
        'measurements': {
            'title': 'Measurements',
            'fields': [
                {'name': 'voltage mag. (pu)', 'index': MachineSignalIndex.MACHINE_VOLTAGE_MAGNITUDE_FILTERED, 'type': 'float', 'format': '.4f'},
                {'name': 'frequency est. (pu)', 'index': MachineSignalIndex.ESTIMATED_FREQUENCY_PU, 'type': 'float', 'format': '.4f'},
                {'name': 'frequency est. (Hz)', 'index': MachineSignalIndex.ESTIMATED_FREQUENCY_HZ, 'type': 'float', 'format': '.2f'},
                {'name': 'voltage ab counts peak', 'index': MachineSignalIndex.MACHINE_V_AB_COUNTS_PEAK, 'type': 'float', 'format': '.0f'},
                {'name': 'voltage bc counts peak', 'index': MachineSignalIndex.MACHINE_V_BC_COUNTS_PEAK, 'type': 'float', 'format': '.0f'}
            ]
        },
        'modulator': {
            'title': 'Modulator',
            'fields': [
                {'name': 'operating mode', 'index': MachineSignalIndex.MODULATOR_OPERATING_MODE, 'type': 'int', 'format': '.0f'},
                {'name': 'firing angle (pu)', 'index': MachineSignalIndex.MODULATOR_FIRING_ANGLE, 'type': 'float', 'format': '.4f'},
                {'name': 'request force commutation', 'index': MachineSignalIndex.MODULATOR_REQUEST_FORCE_COMMUTATION, 'type': 'bool', 'format': 'bool'},
                {'name': 'release pulses (out)', 'index': MachineSignalIndex.MODULATOR_RELEASE_PULSES_OUTPUT, 'type': 'bool', 'format': 'bool'},
                {'name': 'commutation state', 'index': MachineSignalIndex.MODULATOR_COMMUTATION_STATE, 'type': 'int', 'format': '.0f'},
                {'name': 'rotor sector', 'index': MachineSignalIndex.MODULATOR_CURRENT_ROTOR_SECTOR, 'type': 'float', 'format': '.2f'}
            ]
        },
        'speed_torque_controller': {
            'title': 'Speed + Torque Controller',
            'fields': [
                {'name': 'speed reference', 'index': MachineSignalIndex.SPEED_SPEED_REFERENCE, 'type': 'float', 'format': '.4f'},
                {'name': 'measured speed', 'index': MachineSignalIndex.SPEED_MEASURED_SPEED, 'type': 'float', 'format': '.4f'},
                {'name': 'pi output', 'index': MachineSignalIndex.SPEED_PI_OUTPUT, 'type': 'float', 'format': '.4f'},
                {'name': 'is saturated', 'index': MachineSignalIndex.SPEED_IS_SATURATED, 'type': 'bool', 'format': 'bool'},
                {'name': 'torque reference', 'index': MachineSignalIndex.TORQUE_TORQUE_REFERENCE, 'type': 'float', 'format': '.4f'},
                {'name': 'current reference', 'index': MachineSignalIndex.TORQUE_CURRENT_REFERENCE, 'type': 'float', 'format': '.4f'},
                {'name': 'is saturated', 'index': MachineSignalIndex.TORQUE_IS_SATURATED_OUTPUT, 'type': 'bool', 'format': 'bool'},
                {'name': 'is active', 'index': MachineSignalIndex.TORQUE_IS_ACTIVE, 'type': 'bool', 'format': 'bool'},
            ]
        },
        'excitation_controller': {
            'title': 'Stator Voltage Control',
            'fields': [
                {'name': 'field current ref.', 'index': MachineSignalIndex.EXCITATION_FIELD_CURRENT_REFERENCE, 'type': 'float', 'format': '.4f'},
                {'name': 'prefluxing elapsed time', 'index': MachineSignalIndex.EXCITATION_PREFLUXING_ELAPSED_TIME, 'type': 'float', 'format': '.4f'},
                {'name': 'voltage ref. (pu)', 'index': MachineSignalIndex.EXCITATION_MACHINE_VOLTAGE_REFERENCE_PU, 'type': 'float', 'format': '.4f'},
                {'name': 'voltage meas. (pu)', 'index': MachineSignalIndex.EXCITATION_MACHINE_VOLTAGE_MAGNITUDE_PU, 'type': 'float', 'format': '.4f'},
                {'name': 'voltage error (pu)', 'index': MachineSignalIndex.EXCITATION_MACHINE_VOLTAGE_ERROR_PU, 'type': 'float', 'format': '.4f'}
            ]
        },
        'pulse_generator': {
            'title': 'Pulse Generator',
            'fields': [
                {'name': 'virtual ref. active', 'index': MachineSignalIndex.PULSE_GENERATOR_VIRTUAL_REFERENCE_ACTIVE, 'type': 'bool', 'format': 'bool'},
                {'name': 'virtual ref. angle (pu)', 'index': MachineSignalIndex.PULSE_GENERATOR_VIRTUAL_REFERENCE_ANGLE, 'type': 'float', 'format': '.4f'},
                {'name': 'individual override active', 'index': MachineSignalIndex.PULSE_GENERATOR_INDIVIDUAL_OVERRIDE_ACTIVE, 'type': 'bool', 'format': 'bool'},
                {'name': 'pulse 1', 'index': MachineSignalIndex.PULSE_P1, 'type': 'bool', 'format': 'bool'},
                {'name': 'pulse 2', 'index': MachineSignalIndex.PULSE_P2, 'type': 'bool', 'format': 'bool'},
                {'name': 'pulse 3', 'index': MachineSignalIndex.PULSE_P3, 'type': 'bool', 'format': 'bool'},
                {'name': 'pulse 4', 'index': MachineSignalIndex.PULSE_P4, 'type': 'bool', 'format': 'bool'},
                {'name': 'pulse 5', 'index': MachineSignalIndex.PULSE_P5, 'type': 'bool', 'format': 'bool'},
                {'name': 'pulse 6', 'index': MachineSignalIndex.PULSE_P6, 'type': 'bool', 'format': 'bool'},
            ]
        },
        'protections': {
            'title': 'Machine-Side Protections',
            'fields': [
                {'name': 'trip source', 'index': MachineSignalIndex.FIRST_TRIP_SOURCE, 'type': 'enum', 'format': 'enum',
                 'enum_map': {
                     0: 'NONE',
                     1: 'OVER FLUX',
                     2: 'UNDER FLUX',
                     3: 'STALL',
                     4: 'COMM LOSS',
                     5: 'OVERSPEED',
                     6: 'OVERVOLTAGE',
                     7: 'LOSS OF EXCITATION',
                     8: 'EXTERNAL FAULT'
                 }},
                {'name': 'is tripped', 'index': MachineSignalIndex.PROTECTION_IS_TRIPPED, 'type': 'bool', 'format': 'bool'},
                {'name': 'is inhibited', 'index': MachineSignalIndex.PROTECTION_IS_INHIBITED, 'type': 'bool', 'format': 'bool'},
                {'name': 'has active alarm', 'index': MachineSignalIndex.PROTECTION_ANY_ALARM_ACTIVE_UNPACKED, 'type': 'bool', 'format': 'bool'},
                {'name': 'over flux', 'index': MachineSignalIndex.PROTECTION_OVER_FLUX_STATUS, 'type': 'enum', 'format': 'enum',
                 'enum_map': {0: 'TRIP OFF', 1: 'TRIP ON', 2: 'ALARM OFF', 3: 'ALARM ON'}},
                {'name': 'under flux', 'index': MachineSignalIndex.PROTECTION_UNDER_FLUX_STATUS, 'type': 'enum', 'format': 'enum',
                 'enum_map': {0: 'TRIP OFF', 1: 'TRIP ON', 2: 'ALARM OFF', 3: 'ALARM ON'}},
                {'name': 'stall', 'index': MachineSignalIndex.PROTECTION_STALL_STATUS, 'type': 'enum', 'format': 'enum',
                 'enum_map': {0: 'TRIP OFF', 1: 'TRIP ON', 2: 'ALARM OFF', 3: 'ALARM ON'}},
                {'name': 'over flux warning', 'index': MachineSignalIndex.PROTECTION_OVER_FLUX_WARNING, 'type': 'bool', 'format': 'bool'},
                {'name': 'under flux warning', 'index': MachineSignalIndex.PROTECTION_UNDER_FLUX_WARNING, 'type': 'bool', 'format': 'bool'},
                {'name': 'stall warning', 'index': MachineSignalIndex.PROTECTION_STALL_WARNING, 'type': 'bool', 'format': 'bool'},
                {'name': 'comm loss', 'index': MachineSignalIndex.PROTECTION_COMMUNICATION_LOSS_STATUS, 'type': 'enum', 'format': 'enum',
                 'enum_map': {0: 'TRIP OFF', 1: 'TRIP ON', 2: 'ALARM OFF', 3: 'ALARM ON'}},
                {'name': 'comm loss warning', 'index': MachineSignalIndex.PROTECTION_COMMUNICATION_LOSS_WARNING, 'type': 'bool', 'format': 'bool'},
                {'name': 'overspeed', 'index': MachineSignalIndex.PROTECTION_OVERSPEED_STATUS, 'type': 'enum', 'format': 'enum',
                 'enum_map': {0: 'TRIP OFF', 1: 'TRIP ON', 2: 'ALARM OFF', 3: 'ALARM ON'}},
                {'name': 'overspeed trip', 'index': MachineSignalIndex.PROTECTION_OVERSPEED_TRIPPED, 'type': 'bool', 'format': 'bool'},
                {'name': 'machine overvoltage', 'index': MachineSignalIndex.PROTECTION_MACHINE_OVERVOLTAGE_STATUS, 'type': 'enum', 'format': 'enum',
                 'enum_map': {0: 'TRIP OFF', 1: 'TRIP ON', 2: 'ALARM OFF', 3: 'ALARM ON'}},
                {'name': 'machine ov trip', 'index': MachineSignalIndex.PROTECTION_MACHINE_OVERVOLTAGE_TRIPPED, 'type': 'bool', 'format': 'bool'},
                {'name': 'loss of excitation', 'index': MachineSignalIndex.PROTECTION_LOSS_OF_EXCITATION_STATUS, 'type': 'enum', 'format': 'enum',
                 'enum_map': {0: 'TRIP OFF', 1: 'TRIP ON', 2: 'ALARM OFF', 3: 'ALARM ON'}},
                {'name': 'loss of excitation trip', 'index': MachineSignalIndex.PROTECTION_LOSS_OF_EXCITATION_TRIPPED, 'type': 'bool', 'format': 'bool'},
                {'name': 'external fault trip', 'index': MachineSignalIndex.PROTECTION_EXTERNAL_FAULT_TRIPPED, 'type': 'bool', 'format': 'bool'},
            ]
        },
        'fiber_optic_rx': {
            'title': 'Fiber Optic RX (from Grid)',
            'fields': [
                {'name': 'dc link current', 'index': MachineSignalIndex.FIBER_RX_DC_LINK_CURRENT, 'type': 'float', 'format': '.4f'},
                {'name': 'grid voltage mag.', 'index': MachineSignalIndex.FIBER_RX_GRID_VOLTAGE_MAGNITUDE, 'type': 'float', 'format': '.4f'},
                {'name': 'is forced commutation', 'index': MachineSignalIndex.FIBER_RX_IS_FORCED_COMMUTATION_BOOL, 'type': 'bool', 'format': 'bool'},
                {'name': 'force comm. ack', 'index': MachineSignalIndex.FIBER_RX_FORCE_COMMUTATION_ACKNOWLEDGED_BOOL, 'type': 'bool', 'format': 'bool'},
                {'name': 'release pulses', 'index': MachineSignalIndex.FIBER_RX_RELEASE_PULSES_BOOL, 'type': 'bool', 'format': 'bool'},
                {'name': 'grid release protections', 'index': MachineSignalIndex.FIBER_RX_GRID_RELEASE_PROTECTIONS_BOOL, 'type': 'bool', 'format': 'bool'},
                {'name': 'communication ok', 'index': MachineSignalIndex.FIBER_RX_COMMUNICATION_OK, 'type': 'bool', 'format': 'bool'},
                {'name': 'state machine ID', 'index': MachineSignalIndex.FIBER_RX_STATE_ID, 'type': 'int', 'format': '.0f'},
            ]
        },
        'digital_io': {
            'title': 'Digital I/O',
            'fields': [
                {'name': 'DI: Excitation Status', 'index': MachineSignalIndex.DI_EXCITATION_STATUS, 'type': 'bool', 'format': 'bool'},
                {'name': 'DI: External Fault', 'index': MachineSignalIndex.DI_EXTERNAL_FAULT, 'type': 'bool', 'format': 'bool'},
                {'name': 'DO: Excitation On Cmd', 'index': MachineSignalIndex.DO_EXCITATION_ON_COMMAND, 'type': 'bool', 'format': 'bool'},
                {'name': 'DO: Excitation Off Cmd', 'index': MachineSignalIndex.DO_EXCITATION_OFF_COMMAND, 'type': 'bool', 'format': 'bool'},

                {'name': 'AO: Excitation Current Ref', 'index': MachineSignalIndex.AO_EXCITATION_CURRENT_REFERENCE, 'type': 'float', 'format': '.4f'},
            ]
        }
    }
    
    # Return sections in canonical order
    section_order = _get_section_order()
    ordered_result = OrderedDict()
    for section_key in section_order:
        if section_key in all_sections:
            ordered_result[section_key] = all_sections[section_key]
    
    return ordered_result


def get_default_visible_sections() -> list:
    """Get default visible sections for machine-side telemetry, in canonical order.
    
    Returns:
        List of section names that should be visible by default
    """
    section_order = _get_section_order()
    visible = ['speed_torque_controller', 'measurements', 'observer']
    return [s for s in section_order if s in visible]
