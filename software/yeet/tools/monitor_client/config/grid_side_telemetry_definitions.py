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
        'system',
        'speed_controller',
        'torque_controller',
        'controller',
        'observer',
        'measurements',
        'modulator',
        'operation_mode',
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
                {'name': 'pulse releases', 'index': GridSignalIndex.PULSE_RELEASES, 'type': 'bool', 'format': 'bool'},
                {'name': 'is commuting', 'index': GridSignalIndex.IS_COMMUTING, 'type': 'bool', 'format': 'bool'},
                {'name': 'modulator enabled', 'index': GridSignalIndex.MODULATOR_ENABLED, 'type': 'bool', 'format': 'bool'},
                {'name': 'modulator state', 'index': GridSignalIndex.MODULATOR_STATE, 'type': 'int', 'format': '.0f'},
                {'name': 'auxiliaries on', 'index': GridSignalIndex.AUXILIARIES_ON, 'type': 'bool', 'format': 'bool'},
                {'name': 'breakers on', 'index': GridSignalIndex.BREAKERS_ON, 'type': 'bool', 'format': 'bool'},
                {'name': 'current state', 'index': GridSignalIndex.STATE_MACHINE_STATE, 'type': 'int', 'format': '.0f'},
                {'name': 'operating mode', 'index': GridSignalIndex.OPERATION_MODE, 'type': 'int', 'format': '.0f'},
            ]
        },
        'measurements': {
            'title': 'Measurements',
            'fields': [
                {'name': 'voltage magnitude (pu)', 'index': GridSignalIndex.GRID_VOLTAGE_MAGNITUDE_FILTERED, 'type': 'float', 'format': '.4f'},
                {'name': 'voltage magnitude filtered (pu)', 'index': GridSignalIndex.GRID_VOLTAGE_MAGNITUDE_FILTERED_DUP, 'type': 'float', 'format': '.4f'},
                {'name': 'current magnitude (pu)', 'index': GridSignalIndex.GRID_CURRENT_MAGNITUDE, 'type': 'float', 'format': '.4f'},
                {'name': 'dc link current (pu)', 'index': GridSignalIndex.DC_LINK_CURRENT, 'type': 'float', 'format': '.4f'},
                {'name': 'frequency (hz)', 'index': GridSignalIndex.GRID_SIDE_FREQUENCY, 'type': 'float', 'format': '.2f'},
                {'name': 'voltage alpha (pu)', 'index': GridSignalIndex.GRID_VOLTAGE_ALPHA, 'type': 'float', 'format': '.4f'},
                {'name': 'voltage beta (pu)', 'index': GridSignalIndex.GRID_VOLTAGE_BETA, 'type': 'float', 'format': '.4f'},
            ]
        },
        'observer': {
            'title': 'Observer',
            'fields': [
                {'name': 'voltage magnitude est. (pu)', 'index': GridSignalIndex.ESTIMATED_VOLTAGE_MAGNITUDE, 'type': 'float', 'format': '.4f'},
                {'name': 'frequency est. (pu)', 'index': GridSignalIndex.ESTIMATED_FREQUENCY_PU, 'type': 'float', 'format': '.4f'},
                {'name': 'frequency est. (hz)', 'index': GridSignalIndex.ESTIMATED_FREQUENCY_HZ, 'type': 'float', 'format': '.2f'},
                {'name': 'angle est. (rad)', 'index': GridSignalIndex.ESTIMATED_ANGLE_RAD, 'type': 'float', 'format': '.4f'},
                {'name': 'voltage alpha est. (pu)', 'index': GridSignalIndex.ESTIMATED_VOLTAGE_ALPHA, 'type': 'float', 'format': '.4f'},
                {'name': 'voltage beta est. (pu)', 'index': GridSignalIndex.ESTIMATED_VOLTAGE_BETA, 'type': 'float', 'format': '.4f'},
            ]
        },
        'controller': {
            'title': 'Controller',
            'fields': [
                {'name': 'dc current ref. (pu)', 'index': GridSignalIndex.DC_LINK_CURRENT_REFERENCE, 'type': 'float', 'format': '.4f'},
                {'name': 'dc current meas. (pu)', 'index': GridSignalIndex.DC_LINK_CURRENT_MEASURED, 'type': 'float', 'format': '.4f'},
                {'name': 'current error (pu)', 'index': GridSignalIndex.CURRENT_ERROR, 'type': 'float', 'format': '.4f'},
                {'name': 'pi output (pu)', 'index': GridSignalIndex.PI_OUTPUT, 'type': 'float', 'format': '.4f'},
                {'name': 'freeze integrator', 'index': GridSignalIndex.FREEZE_INTEGRATOR, 'type': 'bool', 'format': 'bool'},
                {'name': 'natural commutation mode', 'index': GridSignalIndex.NATURAL_COMMUTATION_MODE, 'type': 'bool', 'format': 'bool'},
                {'name': 'controller saturated', 'index': GridSignalIndex.CONTROLLER_SATURATED, 'type': 'bool', 'format': 'bool'},
            ]
        },
        'modulator': {
            'title': 'Modulator',
            'fields': [
                {'name': 'firing angle input (pu)', 'index': GridSignalIndex.FIRING_ANGLE_INPUT, 'type': 'float', 'format': '.4f'},
                {'name': 'firing angle output (pu)', 'index': GridSignalIndex.FIRING_ANGLE_OUTPUT, 'type': 'float', 'format': '.4f'},
                {'name': 'actual firing angle (pu)', 'index': GridSignalIndex.ACTUAL_FIRING_ANGLE, 'type': 'float', 'format': '.4f'},
                {'name': 'request extinguish dc', 'index': GridSignalIndex.MODULATOR_REQUEST_EXTINGUISH_DC, 'type': 'float', 'format': '.4f'},
                {'name': 'request force commutation', 'index': GridSignalIndex.MODULATOR_REQUEST_FORCE_COMMUTATION, 'type': 'bool', 'format': 'bool'},
                {'name': 'force commutation ack', 'index': GridSignalIndex.FORCE_COMMUTATION_ACK, 'type': 'bool', 'format': 'bool'},
                {'name': 'is requesting pi reset', 'index': GridSignalIndex.IS_REQUESTING_PI_RESET, 'type': 'bool', 'format': 'bool'},
            ]
        },
        'operation_mode': {
            'title': 'Operation Mode',
            'fields': [
                {'name': 'has mode changed', 'index': GridSignalIndex.MODE_CHANGED, 'type': 'bool', 'format': 'bool'},
                {'name': 'is current zero', 'index': GridSignalIndex.CURRENT_ZERO, 'type': 'bool', 'format': 'bool'},
                {'name': 'is discontinuous mode', 'index': GridSignalIndex.DISCONTINUOUS_MODE, 'type': 'bool', 'format': 'bool'},
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
