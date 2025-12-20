#!/usr/bin/env python3
"""Grid-side telemetry signal definitions for monitor displays.

This module defines the declarative structure for grid-side telemetry panels,
mapping signal indices from grid-side telemetry packets to human-readable names
and display parameters.
"""

from typing import Dict, List, Any
from collections import OrderedDict
from .grid_side_telemetry_protocol import GridSignalIndex


def _get_section_order() -> List[str]:
    """Get the canonical display order for telemetry sections.
    
    This defines the preferred order for section display across all telemetry panels.
    Not all sections may be present in every configuration.
    
    Returns:
        List of section keys in preferred display order
    """
    return [
        'digital_io',
        'state_machine',
        'protections',
        'fiber_optic_rx',
        'modulator',
        'measurements',
        'controller',
        'pulse_generator'
    ]


def get_grid_side_telemetry() -> Dict[str, Dict[str, Any]]:
    """Get telemetry definitions for grid-side monitoring.
    
    Returns:
        OrderedDict with section definitions in canonical order.
    """
    all_sections = {
        'measurements': {
            'title': 'Measurements',
            'fields': [
                {'name': 'voltage magnitude (pu)', 'index': GridSignalIndex.GRID_VOLTAGE_MAGNITUDE_FILTERED, 'type': 'float', 'format': '.4f'},
                {'name': 'current magnitude (pu)', 'index': GridSignalIndex.GRID_CURRENT_MAGNITUDE, 'type': 'float', 'format': '.4f'},
                {'name': 'dc link current (pu)', 'index': GridSignalIndex.DC_LINK_CURRENT, 'type': 'float', 'format': '.4f'},
                {'name': 'voltage magnitude est.', 'index': GridSignalIndex.ESTIMATED_VOLTAGE_MAGNITUDE, 'type': 'float', 'format': '.4f'},
                {'name': 'frequency est.', 'index': GridSignalIndex.ESTIMATED_FREQUENCY_PU, 'type': 'float', 'format': '.4f'},
                {'name': 'reference angle (pu)', 'index': GridSignalIndex.ESTIMATED_REFERENCE_ANGLE, 'type': 'float', 'format': '.4f'}
            ]
        },
        'controller': {
            'title': 'Current Controller',
            'fields': [
                {'name': 'current reference', 'index': GridSignalIndex.CURRENT_REFERENCE_OUTPUT, 'type': 'float', 'format': '.4f'},
                {'name': 'current measured', 'index': GridSignalIndex.CURRENT_MEASURED, 'type': 'float', 'format': '.4f'},
                {'name': 'preset request', 'index': GridSignalIndex.CURRENT_PRESET_REQUEST, 'type': 'bool', 'format': 'bool'},
                # NOTE: 'is saturated' is now packed in BOOL_PACK_1 (bit 4)
            ]
        },
        'modulator': {
            'title': 'Modulator',
            'fields': [
                {'name': 'firing angle (pu)', 'index': GridSignalIndex.MODULATOR_FIRING_ANGLE_OUTPUT, 'type': 'float', 'format': '.4f'},
                {'name': 'request extinguish dc', 'index': GridSignalIndex.MODULATOR_REQUEST_EXTINGUISH_DC, 'type': 'bool', 'format': 'bool'},
                {'name': 'request force commutation', 'index': GridSignalIndex.MODULATOR_REQUEST_FORCE_COMMUTATION, 'type': 'bool', 'format': 'bool'},
                {'name': 'release pulses (out)', 'index': GridSignalIndex.MODULATOR_RELEASE_PULSES_OUTPUT, 'type': 'bool', 'format': 'bool'},
                {'name': 'force comm. ack', 'index': GridSignalIndex.FORCE_COMMUTATION_ACKNOWLEDGED, 'type': 'bool', 'format': 'bool'},
            ]
        },
        'pulse_generator': {
            'title': 'Pulse Generator',
            'fields': [
                {'name': 'virtual ref. angle (pu)', 'index': GridSignalIndex.PULSE_GENERATOR_VIRTUAL_REFERENCE_ANGLE, 'type': 'float', 'format': '.4f'},
                {'name': 'actual firing angle (pu)', 'index': GridSignalIndex.PULSE_GENERATOR_ACTUAL_FIRING_ANGLE, 'type': 'float', 'format': '.4f'},
                {'name': 'pulse 1', 'index': GridSignalIndex.PULSE_P1, 'type': 'bool', 'format': 'bool'},
                {'name': 'pulse 2', 'index': GridSignalIndex.PULSE_P2, 'type': 'bool', 'format': 'bool'},
                {'name': 'pulse 3', 'index': GridSignalIndex.PULSE_P3, 'type': 'bool', 'format': 'bool'},
                {'name': 'pulse 4', 'index': GridSignalIndex.PULSE_P4, 'type': 'bool', 'format': 'bool'},
                {'name': 'pulse 5', 'index': GridSignalIndex.PULSE_P5, 'type': 'bool', 'format': 'bool'},
                {'name': 'pulse 6', 'index': GridSignalIndex.PULSE_P6, 'type': 'bool', 'format': 'bool'},
            ]
        },
        'state_machine': {
            'title': 'State Machine',
            'fields': [
                {'name': 'current state', 'index': GridSignalIndex.STATE_MACHINE_STATE, 'type': 'enum', 'format': 'enum',
                 'enum_map': {0: 'IDLE', 1: 'AUXILIARIES_ON', 2: 'BREAKERS_OFF', 3: 'BREAKERS_ON', 4: 'OPERATION_OFF', 5: 'OPERATION_ON', 6: 'SHUTDOWN_REQUESTED'}},
                {'name': 'preset request', 'index': GridSignalIndex.STATE_MACHINE_PRESET_REQUEST, 'type': 'bool', 'format': 'bool'},
                {'name': 'release pulses', 'index': GridSignalIndex.STATE_MACHINE_RELEASE_PULSES, 'type': 'bool', 'format': 'bool'},
                {'name': 'auxiliaries on', 'index': GridSignalIndex.STATE_MACHINE_AUXILIARIES_ON, 'type': 'bool', 'format': 'bool'},
                {'name': 'breaker closed', 'index': GridSignalIndex.STATE_MACHINE_BREAKER_CLOSED, 'type': 'bool', 'format': 'bool'},
                {'name': 'start command', 'index': GridSignalIndex.STATE_MACHINE_START_CMD, 'type': 'bool', 'format': 'bool'},
                {'name': 'stop command', 'index': GridSignalIndex.STATE_MACHINE_STOP_CMD, 'type': 'bool', 'format': 'bool'},
                {'name': 'current extinguished', 'index': GridSignalIndex.STATE_MACHINE_CURRENT_EXTINGUISHED, 'type': 'bool', 'format': 'bool'},
                {'name': 'shutdown request', 'index': GridSignalIndex.STATE_MACHINE_SHUTDOWN_REQUEST, 'type': 'bool', 'format': 'bool'},
                {'name': 'operating mode', 'index': GridSignalIndex.MODE_MONITOR_OPERATING_MODE, 'type': 'enum', 'format': 'enum','enum_map': {0: 'NATURAL_COMMUTATION', 1: 'FORCED_COMMUTATION'}}
            ]
        },
        'protections': {
            'title': 'Grid-Side Protections',
            'fields': [
                {'name': 'is tripped', 'index': GridSignalIndex.PROTECTION_IS_TRIPPED, 'type': 'bool', 'format': 'bool'},
                {'name': 'is inhibited', 'index': GridSignalIndex.PROTECTION_IS_INHIBITED, 'type': 'bool', 'format': 'bool'},
                {'name': 'any fault active', 'index': GridSignalIndex.PROTECTION_ANY_FAULT_ACTIVE, 'type': 'bool', 'format': 'bool'},
                {'name': 'overcurrent', 'index': GridSignalIndex.PROTECTION_OVERCURRENT_STATUS, 'type': 'enum', 'format': 'enum',
                 'enum_map': {0: 'TRIP OFF', 1: 'TRIP ON', 2: 'ALARM OFF', 3: 'ALARM ON'}},
                {'name': 'overvoltage', 'index': GridSignalIndex.PROTECTION_OVERVOLTAGE_STATUS, 'type': 'enum', 'format': 'enum',
                 'enum_map': {0: 'TRIP OFF', 1: 'TRIP ON', 2: 'ALARM OFF', 3: 'ALARM ON'}},
                {'name': 'undervoltage', 'index': GridSignalIndex.PROTECTION_UNDERVOLTAGE_STATUS, 'type': 'enum', 'format': 'enum',
                 'enum_map': {0: 'TRIP OFF', 1: 'TRIP ON', 2: 'ALARM OFF', 3: 'ALARM ON'}},
                {'name': 'comm loss', 'index': GridSignalIndex.PROTECTION_COMMUNICATION_LOSS_STATUS, 'type': 'enum', 'format': 'enum',
                 'enum_map': {0: 'TRIP OFF', 1: 'TRIP ON', 2: 'ALARM OFF', 3: 'ALARM ON'}},
                {'name': 'door open', 'index': GridSignalIndex.PROTECTION_DOOR_OPEN_STATUS, 'type': 'enum', 'format': 'enum',
                 'enum_map': {0: 'TRIP OFF', 1: 'TRIP ON', 2: 'ALARM OFF', 3: 'ALARM ON'}},
            ]
        },
        'fiber_optic_rx': {
            'title': 'Fiber Optic RX (from Machine)',
            'fields': [
                {'name': 'measured speed', 'index': GridSignalIndex.FIBER_RX_MEASURED_SPEED, 'type': 'float', 'format': '.4f'},
                {'name': 'current reference', 'index': GridSignalIndex.FIBER_RX_DC_LINK_CURRENT_REFERENCE, 'type': 'float', 'format': '.4f'},
                {'name': 'machine voltage mag.', 'index': GridSignalIndex.FIBER_RX_MACHINE_VOLTAGE_MAGNITUDE, 'type': 'float', 'format': '.4f'},
                # NOTE: Fiber optic RX booleans are now packed in BOOL_PACK_2 (bits 4-7)
            ]
        },
        'digital_io': {
            'title': 'Digital I/O',
            'fields': [
                {'name': 'DI: Auxiliaries On', 'index': GridSignalIndex.DI_AUXILIARIES_ARE_ON, 'type': 'bool', 'format': 'bool'},
                {'name': 'DI: MV Breaker Closed', 'index': GridSignalIndex.DI_MV_BREAKER_IS_CLOSED, 'type': 'bool', 'format': 'bool'},
                {'name': 'DI: Start Command', 'index': GridSignalIndex.DI_START_COMMAND, 'type': 'bool', 'format': 'bool'},
                {'name': 'DI: Stop Command', 'index': GridSignalIndex.DI_STOP_COMMAND, 'type': 'bool', 'format': 'bool'},
                {'name': 'DI: MV Breaker Tripping', 'index': GridSignalIndex.DI_MV_BREAKER_IS_TRIPPING, 'type': 'bool', 'format': 'bool'},
                {'name': 'DI: Emergency Off', 'index': GridSignalIndex.DI_EMERGENCY_OFF, 'type': 'bool', 'format': 'bool'},
                {'name': 'DI: Door Open', 'index': GridSignalIndex.DI_DOOR_IS_OPEN, 'type': 'bool', 'format': 'bool'},
                {'name': 'DI: External Fault', 'index': GridSignalIndex.DI_EXTERNAL_FAULT, 'type': 'bool', 'format': 'bool'},
                {'name': 'DI: Fans Running', 'index': GridSignalIndex.DI_FANS_ARE_RUNNING, 'type': 'bool', 'format': 'bool'},
                {'name': 'DO: MV Breaker Close', 'index': GridSignalIndex.DO_MV_BREAKER_CLOSE, 'type': 'bool', 'format': 'bool'},
                {'name': 'DO: MV Breaker Open', 'index': GridSignalIndex.DO_MV_BREAKER_OPEN, 'type': 'bool', 'format': 'bool'},
                {'name': 'DO: Auxiliaries On Cmd', 'index': GridSignalIndex.DO_AUXILIARIES_ON_COMMAND, 'type': 'bool', 'format': 'bool'},
                {'name': 'DO: ACU Fan 1', 'index': GridSignalIndex.DO_ON_COMMAND_ACU_FAN_1, 'type': 'bool', 'format': 'bool'},
                {'name': 'DO: ACU Fan 2', 'index': GridSignalIndex.DO_ON_COMMAND_ACU_FAN_2, 'type': 'bool', 'format': 'bool'},
                {'name': 'DO: ACU Fan 3', 'index': GridSignalIndex.DO_ON_COMMAND_ACU_FAN_3, 'type': 'bool', 'format': 'bool'},
                {'name': 'DO: ACU Fan 4', 'index': GridSignalIndex.DO_ON_COMMAND_ACU_FAN_4, 'type': 'bool', 'format': 'bool'},
                {'name': 'DO: Software Trip 1', 'index': GridSignalIndex.DO_SOFTWARE_TRIP_1, 'type': 'bool', 'format': 'bool'},
                {'name': 'DO: Software Trip 2', 'index': GridSignalIndex.DO_SOFTWARE_TRIP_2, 'type': 'bool', 'format': 'bool'},
                {'name': 'DO: Software Trip 3', 'index': GridSignalIndex.DO_SOFTWARE_TRIP_3, 'type': 'bool', 'format': 'bool'},
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
    """Get default visible sections for grid-side telemetry, in canonical order.
    
    Returns:
        List of section names that should be visible by default
    """
    section_order = _get_section_order()
    # Grid-side has: system, controller, observer, measurements, modulator, operation_mode
    # (no speed_controller or torque_controller)
    visible = ['controller', 'measurements']
    return [s for s in section_order if s in visible]
