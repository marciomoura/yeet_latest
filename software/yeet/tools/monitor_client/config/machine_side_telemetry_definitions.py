#!/usr/bin/env python3
"""Machine-side telemetry signal definitions for monitor displays.

This module defines the declarative structure for machine-side telemetry panels,
mapping signal indices from machine_side_telemetry_builder.cpp to human-readable
names and display parameters.

Signal indices correspond to:
  - 0-8:   Machine Measurements (Voltage ABC masked/raw, magnitudes)
  - 9-12:  Observer (Frequency, position, voltage alpha/beta)
  - 13-17: Modulator (Firing angle, enable, pulses, commutation, sector)
  - 18-20: Machine Controller (Firing angle, DC current, speed)
  - 21-33: Torque Controller (References, limits, saturation, boost state)
  - 34-41: Speed Controller (Speed/torque ref, limited speed ref, error, PI output, saturation)
  - 42-46: Speed Generator (Speed ref, torque limits, profile state)
  - 47-50: Excitation Controller (Voltage setpoint, enable, field state, running)
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
        'system',
        'speed_controller',
        'torque_controller',
        'observer',
        'measurements',
        'modulator',
        'operation_mode',
    ]


def get_machine_side_telemetry() -> Dict[str, Dict[str, Any]]:
    """Get telemetry definitions for machine-side monitoring.
    
    Returns:
        OrderedDict with section definitions in canonical order.
    """
    all_sections = {
        'system': {
            'title': 'System',
            'fields': [
                {'name': 'modulator enabled', 'index': MachineSignalIndex.MODULATOR_ENABLED, 'type': 'bool', 'format': 'bool'},
                {'name': 'release pulses', 'index': MachineSignalIndex.RELEASE_PULSES, 'type': 'bool', 'format': 'bool'},
                {'name': 'torque is active', 'index': MachineSignalIndex.TORQUE_IS_ACTIVE, 'type': 'bool', 'format': 'bool'},
                {'name': 'speed is active', 'index': MachineSignalIndex.SPEED_IS_ACTIVE, 'type': 'bool', 'format': 'bool'},
                {'name': 'excitation enable', 'index': MachineSignalIndex.EXCITATION_ENABLE, 'type': 'bool', 'format': 'bool'},
                {'name': 'excitation is running', 'index': MachineSignalIndex.EXCITATION_IS_RUNNING, 'type': 'bool', 'format': 'bool'},
                {'name': 'field established', 'index': MachineSignalIndex.EXCITATION_IS_FIELD_ESTABLISHED, 'type': 'bool', 'format': 'bool'},
                {'name': 'rotor sector', 'index': MachineSignalIndex.CURRENT_ROTOR_SECTOR, 'type': 'int', 'format': '.0f'},
            ]
        },
        'measurements': {
            'title': 'Measurements',
            'fields': [
                {'name': 'voltage magnitude (pu)', 'index': MachineSignalIndex.MACHINE_VOLTAGE_MAGNITUDE, 'type': 'float', 'format': '.4f'},
                {'name': 'voltage magnitude filtered (pu)', 'index': MachineSignalIndex.MACHINE_VOLTAGE_MAGNITUDE_FILTERED, 'type': 'float', 'format': '.4f'},
                {'name': 'is commuting', 'index': MachineSignalIndex.IS_COMMUTING, 'type': 'bool', 'format': 'bool'}
            ]
        },
        'observer': {
            'title': 'Observer',
            'fields': [
                {'name': 'frequency est. (pu)', 'index': MachineSignalIndex.ESTIMATED_FREQUENCY_PU, 'type': 'float', 'format': '.4f'},
                {'name': 'position est. (pu)', 'index': MachineSignalIndex.ESTIMATED_POSITION_PU, 'type': 'float', 'format': '.4f'},
                {'name': 'voltage alpha filtered (pu)', 'index': MachineSignalIndex.FILTERED_VOLTAGE_ALPHA, 'type': 'float', 'format': '.4f'},
                {'name': 'voltage beta filtered (pu)', 'index': MachineSignalIndex.FILTERED_VOLTAGE_BETA, 'type': 'float', 'format': '.4f'},
            ]
        },
        'modulator': {
            'title': 'Modulator',
            'fields': [
                {'name': 'firing angle (rad)', 'index': MachineSignalIndex.FIRING_ANGLE_RAD, 'type': 'float', 'format': '.4f'},
                {'name': 'firing angle out (rad)', 'index': MachineSignalIndex.CONTROLLER_FIRING_ANGLE, 'type': 'float', 'format': '.4f'},
                {'name': 'force commutation', 'index': MachineSignalIndex.REQUEST_FORCE_COMMUTATION, 'type': 'bool', 'format': 'bool'},
            ]
        },
        'torque_controller': {
            'title': 'Torque Controller',
            'fields': [
                {'name': 'torque ref. from speed (pu)', 'index': MachineSignalIndex.TORQUE_REF_FROM_SPEED_CONTROLLER, 'type': 'float', 'format': '.4f'},
                {'name': 'max torque limit (pu)', 'index': MachineSignalIndex.TORQUE_DYNAMIC_MAX_LIMIT, 'type': 'float', 'format': '.4f'},
                {'name': 'min torque limit (pu)', 'index': MachineSignalIndex.TORQUE_DYNAMIC_MIN_LIMIT, 'type': 'float', 'format': '.4f'},
                {'name': 'breakaway boost enabled', 'index': MachineSignalIndex.TORQUE_ENABLE_BREAKAWAY_BOOST, 'type': 'bool', 'format': 'bool'},
                {'name': 'current controller saturated', 'index': MachineSignalIndex.TORQUE_CURRENT_CONTROLLER_SATURATED, 'type': 'bool', 'format': 'bool'},
                {'name': 'current reference (pu)', 'index': MachineSignalIndex.TORQUE_CURRENT_REFERENCE, 'type': 'float', 'format': '.4f'},
                {'name': 'base torque ref. (pu)', 'index': MachineSignalIndex.TORQUE_BASE_REFERENCE, 'type': 'float', 'format': '.4f'},
                {'name': 'total torque ref. (pu)', 'index': MachineSignalIndex.TORQUE_TOTAL_REFERENCE, 'type': 'float', 'format': '.4f'},
                {'name': 'limited torque ref. (pu)', 'index': MachineSignalIndex.TORQUE_LIMITED_REFERENCE, 'type': 'float', 'format': '.4f'},
                {'name': 'is saturated', 'index': MachineSignalIndex.TORQUE_IS_SATURATED, 'type': 'bool', 'format': 'bool'},
                {'name': 'breakaway boost active', 'index': MachineSignalIndex.TORQUE_IS_BREAKAWAY_BOOST_ACTIVE, 'type': 'bool', 'format': 'bool'},
                {'name': 'downstream saturated', 'index': MachineSignalIndex.TORQUE_DOWNSTREAM_SATURATION, 'type': 'bool', 'format': 'bool'},
            ]
        },
        'speed_controller': {
            'title': 'Speed Controller',
            'fields': [
                {'name': 'speed reference (pu)', 'index': MachineSignalIndex.SPEED_REFERENCE_PU, 'type': 'float', 'format': '.4f'},
                {'name': 'measured speed (pu)', 'index': MachineSignalIndex.SPEED_MEASURED_SPEED_PU, 'type': 'float', 'format': '.4f'},
                {'name': 'speed reference limited (pu)', 'index': MachineSignalIndex.SPEED_REFERENCE_LIMITED_PU, 'type': 'float', 'format': '.4f'},
                {'name': 'torque reference (pu)', 'index': MachineSignalIndex.SPEED_TORQUE_REFERENCE, 'type': 'float', 'format': '.4f'},
                {'name': 'speed error (pu)', 'index': MachineSignalIndex.SPEED_ERROR, 'type': 'float', 'format': '.4f'},
                {'name': 'pi output (pu)', 'index': MachineSignalIndex.SPEED_PI_OUTPUT, 'type': 'float', 'format': '.4f'},
                {'name': 'is saturated', 'index': MachineSignalIndex.SPEED_IS_SATURATED, 'type': 'bool', 'format': 'bool'},
            ]
        },
        'speed_generator': {
            'title': 'Speed Generator',
            'fields': [
                {'name': 'speed ref. (pu)', 'index': MachineSignalIndex.SPEED_REF_PU, 'type': 'float', 'format': '.4f'},
                {'name': 'torque limit max (pu)', 'index': MachineSignalIndex.SPEED_TORQUE_LIMIT_MAX_PU, 'type': 'float', 'format': '.4f'},
                {'name': 'torque limit min (pu)', 'index': MachineSignalIndex.SPEED_TORQUE_LIMIT_MIN_PU, 'type': 'float', 'format': '.4f'},
                {'name': 'profile complete', 'index': MachineSignalIndex.SPEED_IS_PROFILE_COMPLETE, 'type': 'bool', 'format': 'bool'},
                {'name': 'breakaway boost active', 'index': MachineSignalIndex.SPEED_IS_BREAKAWAY_BOOST_ACTIVE, 'type': 'bool', 'format': 'bool'},
            ]
        },
        'excitation_controller': {
            'title': 'Excitation Controller',
            'fields': [
                {'name': 'excitation voltage setpoint (pu)', 'index': MachineSignalIndex.EXCITATION_VOLTAGE_SETPOINT_PU, 'type': 'float', 'format': '.4f'},
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
    # Machine-side has: system, speed_controller, torque_controller, measurements, observer, modulator
    # (no controller or operation_mode at this side)
    visible = ['system', 'speed_controller', 'torque_controller', 'measurements', 'observer']
    return [s for s in section_order if s in visible]
