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
  - 39-42: Excitation Controller (Voltage setpoint, enable, field established, running)
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
        'controller',
        'observer',
        'measurements',
        'modulator',
        'speed_generator',
        'excitation_controller',
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
                {'name': 'modulator release pulses', 'index': MachineSignalIndex.MODULATOR_RELEASE_PULSES_INPUT, 'type': 'bool', 'format': 'bool'},
                {'name': 'torque is active', 'index': MachineSignalIndex.TORQUE_IS_ACTIVE, 'type': 'bool', 'format': 'bool'},
                {'name': 'excitation enable', 'index': MachineSignalIndex.EXCITATION_ENABLE, 'type': 'bool', 'format': 'bool'},
                {'name': 'excitation is running', 'index': MachineSignalIndex.EXCITATION_IS_RUNNING, 'type': 'bool', 'format': 'bool'},
                {'name': 'field established', 'index': MachineSignalIndex.EXCITATION_IS_FIELD_ESTABLISHED, 'type': 'bool', 'format': 'bool'}
            ]
        },
        'measurements': {
            'title': 'Measurements',
            'fields': [
                {'name': 'voltage magnitude filtered (pu)', 'index': MachineSignalIndex.MACHINE_VOLTAGE_MAGNITUDE_FILTERED, 'type': 'float', 'format': '.4f'},
                {'name': 'estimated speed (pu)', 'index': MachineSignalIndex.ESTIMATED_SPEED, 'type': 'float', 'format': '.4f'},
                {'name': 'estimated voltage magnitude (pu)', 'index': MachineSignalIndex.ESTIMATED_VOLTAGE_MAGNITUDE, 'type': 'float', 'format': '.4f'}
            ]
        },
        'observer': {
            'title': 'Observer',
            'fields': [
                {'name': 'frequency est. (pu)', 'index': MachineSignalIndex.ESTIMATED_FREQUENCY_PU, 'type': 'float', 'format': '.4f'},
                {'name': 'reference angle (pu)', 'index': MachineSignalIndex.ESTIMATED_REFERENCE_ANGLE, 'type': 'float', 'format': '.4f'},
                {'name': 'frequency (Hz)', 'index': MachineSignalIndex.ESTIMATED_FREQUENCY, 'type': 'float', 'format': '.2f'},
            ]
        },
        'modulator': {
            'title': 'Modulator',
            'fields': [
                {'name': 'operating mode', 'index': MachineSignalIndex.MODULATOR_OPERATING_MODE, 'type': 'int', 'format': '.0f'},
                {'name': 'firing angle (pu)', 'index': MachineSignalIndex.MODULATOR_FIRING_ANGLE, 'type': 'float', 'format': '.4f'},
                {'name': 'request force commutation', 'index': MachineSignalIndex.MODULATOR_REQUEST_FORCE_COMMUTATION, 'type': 'bool', 'format': 'bool'},
                {'name': 'force commutation ack', 'index': MachineSignalIndex.MODULATOR_FORCE_COMMUTATION_ACK, 'type': 'bool', 'format': 'bool'},
                {'name': 'commutation state', 'index': MachineSignalIndex.MODULATOR_COMMUTATION_STATE, 'type': 'int', 'format': '.0f'},
                {'name': 'rotor sector', 'index': MachineSignalIndex.MODULATOR_CURRENT_ROTOR_SECTOR, 'type': 'float', 'format': '.2f'}
            ]
        },
        'controller': {
            'title': 'Machine Controller',
            'fields': [
                {'name': 'firing angle (rad)', 'index': MachineSignalIndex.CONTROLLER_FIRING_ANGLE, 'type': 'float', 'format': '.4f'},
                {'name': 'dc link current (pu)', 'index': MachineSignalIndex.CONTROLLER_DC_LINK_CURRENT, 'type': 'float', 'format': '.4f'},
                {'name': 'machine speed (pu)', 'index': MachineSignalIndex.CONTROLLER_MACHINE_SPEED, 'type': 'float', 'format': '.4f'},
            ]
        },
        'torque_controller': {
            'title': 'Torque Controller',
            'fields': [
                {'name': 'torque reference (pu)', 'index': MachineSignalIndex.TORQUE_TORQUE_REFERENCE, 'type': 'float', 'format': '.4f'},
                {'name': 'max torque limit (pu)', 'index': MachineSignalIndex.TORQUE_DYNAMIC_MAX_LIMIT, 'type': 'float', 'format': '.4f'},
                {'name': 'min torque limit (pu)', 'index': MachineSignalIndex.TORQUE_DYNAMIC_MIN_LIMIT, 'type': 'float', 'format': '.4f'},
                {'name': 'breakaway boost enabled', 'index': MachineSignalIndex.TORQUE_ENABLE_BREAKAWAY_BOOST, 'type': 'bool', 'format': 'bool'},
                {'name': 'is saturated (input)', 'index': MachineSignalIndex.TORQUE_IS_SATURATED_INPUT, 'type': 'bool', 'format': 'bool'},
                {'name': 'current reference (pu)', 'index': MachineSignalIndex.TORQUE_CURRENT_REFERENCE, 'type': 'float', 'format': '.4f'},
                {'name': 'is saturated (output)', 'index': MachineSignalIndex.TORQUE_IS_SATURATED_OUTPUT, 'type': 'bool', 'format': 'bool'},
                {'name': 'is active', 'index': MachineSignalIndex.TORQUE_IS_ACTIVE, 'type': 'bool', 'format': 'bool'},
                {'name': 'breakaway boost active', 'index': MachineSignalIndex.TORQUE_IS_BREAKAWAY_BOOST_ACTIVE, 'type': 'bool', 'format': 'bool'},
            ]
        },
        'speed_controller': {
            'title': 'Speed Controller',
            'fields': [
                {'name': 'speed reference (pu)', 'index': MachineSignalIndex.SPEED_SPEED_REFERENCE, 'type': 'float', 'format': '.4f'},
                {'name': 'measured speed (pu)', 'index': MachineSignalIndex.SPEED_MEASURED_SPEED, 'type': 'float', 'format': '.4f'},
                {'name': 'torque reference (pu)', 'index': MachineSignalIndex.SPEED_TORQUE_REFERENCE, 'type': 'float', 'format': '.4f'},
                {'name': 'pi output (pu)', 'index': MachineSignalIndex.SPEED_PI_OUTPUT, 'type': 'float', 'format': '.4f'},
                {'name': 'is saturated', 'index': MachineSignalIndex.SPEED_IS_SATURATED, 'type': 'bool', 'format': 'bool'},
            ]
        },
        'speed_generator': {
            'title': 'Speed Generator',
            'fields': [
                {'name': 'speed ref. (pu)', 'index': MachineSignalIndex.SPEED_GEN_SPEED_REFERENCE, 'type': 'float', 'format': '.4f'},
                {'name': 'torque limit max (pu)', 'index': MachineSignalIndex.SPEED_GEN_TORQUE_LIMIT_MAX, 'type': 'float', 'format': '.4f'},
                {'name': 'torque limit min (pu)', 'index': MachineSignalIndex.SPEED_GEN_TORQUE_LIMIT_MIN, 'type': 'float', 'format': '.4f'},
                {'name': 'profile complete', 'index': MachineSignalIndex.SPEED_GEN_IS_PROFILE_COMPLETE, 'type': 'bool', 'format': 'bool'},
                {'name': 'breakaway boost active', 'index': MachineSignalIndex.SPEED_GEN_IS_BREAKAWAY_BOOST_ACTIVE, 'type': 'bool', 'format': 'bool'},
            ]
        },
        'excitation_controller': {
            'title': 'Excitation Controller',
            'fields': [
                {'name': 'excitation voltage setpoint (pu)', 'index': MachineSignalIndex.EXCITATION_VOLTAGE_SETPOINT, 'type': 'float', 'format': '.4f'},
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
