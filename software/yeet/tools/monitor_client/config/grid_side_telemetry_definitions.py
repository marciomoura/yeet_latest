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
        'state_machine',
        'system',
        'protections',
        'operation_mode',
        'modulator',
        'pulse_generator',
        'measurements',
        'controller',
        'observer',
        'fiber_optic_rx',
    ]


def get_grid_side_telemetry() -> Dict[str, Dict[str, Any]]:
    """Get telemetry definitions for grid-side monitoring.
    
    Returns:
        OrderedDict with section definitions in canonical order.
    """
    all_sections = {
        'system': {
            'title': 'System',
            'fields': [
                {'name': 'state machine release pulses', 'index': GridSignalIndex.STATE_MACHINE_RELEASE_PULSES, 'type': 'bool', 'format': 'bool'},
                {'name': 'modulator release pulses', 'index': GridSignalIndex.MODULATOR_RELEASE_PULSES_INPUT, 'type': 'bool', 'format': 'bool'},
                {'name': 'pulse gen. release pulses', 'index': GridSignalIndex.PULSE_GENERATOR_RELEASE_PULSES, 'type': 'bool', 'format': 'bool'}
            ]
        },
        'measurements': {
            'title': 'Measurements',
            'fields': [
                {'name': 'voltage magnitude', 'index': GridSignalIndex.GRID_VOLTAGE_MAGNITUDE_FILTERED, 'type': 'float', 'format': '.4f'},
                {'name': 'current magnitude', 'index': GridSignalIndex.GRID_CURRENT_MAGNITUDE, 'type': 'float', 'format': '.4f'},
                {'name': 'dc link current', 'index': GridSignalIndex.DC_LINK_CURRENT, 'type': 'float', 'format': '.4f'}
            ]
        },
        'observer': {
            'title': 'Observer',
            'fields': [
                {'name': 'voltage magnitude est.', 'index': GridSignalIndex.ESTIMATED_VOLTAGE_MAGNITUDE, 'type': 'float', 'format': '.4f'},
                {'name': 'frequency est.', 'index': GridSignalIndex.ESTIMATED_FREQUENCY_PU, 'type': 'float', 'format': '.4f'},
                {'name': 'reference angle (pu)', 'index': GridSignalIndex.ESTIMATED_REFERENCE_ANGLE, 'type': 'float', 'format': '.4f'},
            ]
        },
        'controller': {
            'title': 'Current Controller',
            'fields': [
                {'name': 'current reference', 'index': GridSignalIndex.CURRENT_REFERENCE_OUTPUT, 'type': 'float', 'format': '.4f'},
                {'name': 'current measured', 'index': GridSignalIndex.CURRENT_MEASURED, 'type': 'float', 'format': '.4f'},
                {'name': 'preset request', 'index': GridSignalIndex.CURRENT_PRESET_REQUEST, 'type': 'bool', 'format': 'bool'},
                {'name': 'is saturated', 'index': GridSignalIndex.CURRENT_IS_SATURATED, 'type': 'bool', 'format': 'bool'},
            ]
        },
        'modulator': {
            'title': 'Modulator',
            'fields': [
                {'name': 'firing angle (pu)', 'index': GridSignalIndex.MODULATOR_FIRING_ANGLE_OUTPUT, 'type': 'float', 'format': '.4f'},
          
                {'name': 'request extinguish dc', 'index': GridSignalIndex.MODULATOR_REQUEST_EXTINGUISH_DC, 'type': 'bool', 'format': 'bool'},
                {'name': 'request force commutation', 'index': GridSignalIndex.MODULATOR_REQUEST_FORCE_COMMUTATION, 'type': 'bool', 'format': 'bool'},
                {'name': 'force commutation ack', 'index': GridSignalIndex.MODULATOR_FORCE_COMMUTATION_ACK, 'type': 'bool', 'format': 'bool'},
            ]
        },
        'pulse_generator': {
            'title': 'Pulse Generator',
            'fields': [
                {'name': 'release pulses', 'index': GridSignalIndex.PULSE_GENERATOR_RELEASE_PULSES, 'type': 'bool', 'format': 'bool'},
                {'name': 'virtual ref. active', 'index': GridSignalIndex.PULSE_GENERATOR_VIRTUAL_REFERENCE_ACTIVE, 'type': 'bool', 'format': 'bool'},
                {'name': 'virtual ref. angle (pu)', 'index': GridSignalIndex.PULSE_GENERATOR_VIRTUAL_REFERENCE_ANGLE, 'type': 'float', 'format': '.4f'},
                {'name': 'individual override active', 'index': GridSignalIndex.PULSE_GENERATOR_INDIVIDUAL_OVERRIDE_ACTIVE, 'type': 'bool', 'format': 'bool'},
                {'name': 'pulse 1', 'index': GridSignalIndex.PULSE_GENERATOR_P1, 'type': 'bool', 'format': 'bool'},
                {'name': 'pulse 2', 'index': GridSignalIndex.PULSE_GENERATOR_P2, 'type': 'bool', 'format': 'bool'},
                {'name': 'pulse 3', 'index': GridSignalIndex.PULSE_GENERATOR_P3, 'type': 'bool', 'format': 'bool'},
                {'name': 'pulse 4', 'index': GridSignalIndex.PULSE_GENERATOR_P4, 'type': 'bool', 'format': 'bool'},
                {'name': 'pulse 5', 'index': GridSignalIndex.PULSE_GENERATOR_P5, 'type': 'bool', 'format': 'bool'},
                {'name': 'pulse 6', 'index': GridSignalIndex.PULSE_GENERATOR_P6, 'type': 'bool', 'format': 'bool'},
            ]
        },
        'operation_mode': {
            'title': 'Operation Mode',
            'fields': [
                {'name': 'operating mode', 'index': GridSignalIndex.MODULATOR_OPERATING_MODE, 'type': 'enum', 'format': 'enum',
                 'enum_map': {0: 'NATURAL_COMMUTATION', 1: 'FORCED_COMMUTATION'}},
                {'name': 'state machine preset request', 'index': GridSignalIndex.STATE_MACHINE_PRESET_REQUEST, 'type': 'bool', 'format': 'bool'},
                {'name': 'has mode changed', 'index': GridSignalIndex.MODE_MONITOR_HAS_MODE_CHANGED, 'type': 'bool', 'format': 'bool'},
                {'name': 'is current zero', 'index': GridSignalIndex.MODE_MONITOR_CURRENT_IS_ZERO, 'type': 'bool', 'format': 'bool'},
                {'name': 'is discontinuous mode', 'index': GridSignalIndex.MODE_MONITOR_IS_DISCONTINUOUS_MODE, 'type': 'bool', 'format': 'bool'},
            ]
        },
        'state_machine': {
            'title': 'State Machine',
            'fields': [
                {'name': 'current state', 'index': GridSignalIndex.STATE_MACHINE_STATE, 'type': 'enum', 'format': 'enum',
                 'enum_map': {0: 'IDLE', 1: 'AUXILIARIES_ON', 2: 'BREAKERS_OFF', 3: 'BREAKERS_ON', 4: 'OPERATION_OFF', 5: 'OPERATION_ON', 6: 'SHUTDOWN_REQUESTED'}},
                {'name': 'start command', 'index': GridSignalIndex.STATE_MACHINE_START_COMMAND, 'type': 'bool', 'format': 'bool'},
                {'name': 'auxiliaries on', 'index': GridSignalIndex.STATE_MACHINE_AUXILIARIES_ON, 'type': 'bool', 'format': 'bool'},
                {'name': 'breakers closed', 'index': GridSignalIndex.STATE_MACHINE_BREAKERS_CLOSED, 'type': 'bool', 'format': 'bool'},           
                {'name': 'current extinguished', 'index': GridSignalIndex.STATE_MACHINE_CURRENT_EXTINGUISHED, 'type': 'bool', 'format': 'bool'},
                {'name': 'shutdown request', 'index': GridSignalIndex.STATE_MACHINE_SHUTDOWN_REQUEST, 'type': 'bool', 'format': 'bool'},
                {'name': 'stop command', 'index': GridSignalIndex.STATE_MACHINE_STOP_COMMAND, 'type': 'bool', 'format': 'bool'},
            ]
        },
        'protections': {
            'title': 'Grid-Side Protections',
            'fields': [
                {'name': 'is tripped', 'index': GridSignalIndex.PROTECTION_IS_TRIPPED, 'type': 'bool', 'format': 'bool'},
                {'name': 'is inhibited', 'index': GridSignalIndex.PROTECTION_IS_INHIBITED, 'type': 'bool', 'format': 'bool'},
                {'name': 'any fault active', 'index': GridSignalIndex.PROTECTION_ANY_FAULT_ACTIVE, 'type': 'bool', 'format': 'bool'},
                {'name': 'overcurrent tripped', 'index': GridSignalIndex.PROTECTION_OVERCURRENT_TRIPPED, 'type': 'bool', 'format': 'bool'},
                {'name': 'overvoltage tripped', 'index': GridSignalIndex.PROTECTION_OVERVOLTAGE_TRIPPED, 'type': 'bool', 'format': 'bool'},
                {'name': 'undervoltage tripped', 'index': GridSignalIndex.PROTECTION_UNDERVOLTAGE_TRIPPED, 'type': 'bool', 'format': 'bool'},
                {'name': 'overcurrent warning', 'index': GridSignalIndex.PROTECTION_OVERCURRENT_WARNING, 'type': 'bool', 'format': 'bool'},
                {'name': 'overvoltage warning', 'index': GridSignalIndex.PROTECTION_OVERVOLTAGE_WARNING, 'type': 'bool', 'format': 'bool'},
                {'name': 'undervoltage warning', 'index': GridSignalIndex.PROTECTION_UNDERVOLTAGE_WARNING, 'type': 'bool', 'format': 'bool'},
                {'name': 'comm loss tripped', 'index': GridSignalIndex.PROTECTION_COMMUNICATION_LOSS_TRIPPED, 'type': 'bool', 'format': 'bool'},
                {'name': 'comm loss warning', 'index': GridSignalIndex.PROTECTION_COMMUNICATION_LOSS_WARNING, 'type': 'bool', 'format': 'bool'},
            ]
        },
        'fiber_optic_rx': {
            'title': 'Fiber Optic RX (from Machine)',
            'fields': [
                {'name': 'measured speed', 'index': GridSignalIndex.FIBER_RX_MEASURED_SPEED, 'type': 'float', 'format': '.4f'},
                {'name': 'current reference', 'index': GridSignalIndex.FIBER_RX_DC_LINK_CURRENT_REFERENCE, 'type': 'float', 'format': '.4f'},
                {'name': 'machine voltage mag.', 'index': GridSignalIndex.FIBER_RX_MACHINE_VOLTAGE_MAGNITUDE, 'type': 'float', 'format': '.4f'},
                {'name': 'force comm. request', 'index': GridSignalIndex.FIBER_RX_FORCE_COMMUTATION_REQUEST, 'type': 'bool', 'format': 'bool'},
                {'name': 'shutdown request', 'index': GridSignalIndex.FIBER_RX_SHUTDOWN_REQUEST, 'type': 'bool', 'format': 'bool'},
                {'name': 'machine protection tripped', 'index': GridSignalIndex.FIBER_RX_MACHINE_PROTECTION_TRIPPED, 'type': 'bool', 'format': 'bool'},
                {'name': 'protection inhibit', 'index': GridSignalIndex.FIBER_RX_PROTECTION_INHIBIT, 'type': 'bool', 'format': 'bool'},
                {'name': 'machine trip ack', 'index': GridSignalIndex.FIBER_RX_MACHINE_TRIP_ACKNOWLEDGED, 'type': 'bool', 'format': 'bool'},
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
    visible = ['system', 'controller', 'observer', 'measurements']
    return [s for s in section_order if s in visible]
