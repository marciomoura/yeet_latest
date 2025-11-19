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
        'measurements',
        'protections',
        'startup_sequencer',
        'speed_controller',
        'torque_controller',
        'controller',
        'modulator',
        'excitation_controller',
        'pulse_generator',
        'fiber_optic_rx',
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
                {'name': 'startup complete', 'index': MachineSignalIndex.STARTUP_IS_STARTUP_COMPLETE, 'type': 'bool', 'format': 'bool'}
            ]
        },
        'measurements': {
            'title': 'Measurements',
            'fields': [
                {'name': 'voltage mag.', 'index': MachineSignalIndex.MACHINE_VOLTAGE_MAGNITUDE_FILTERED, 'type': 'float', 'format': '.4f'},
                {'name': 'frequency est.', 'index': MachineSignalIndex.ESTIMATED_FREQUENCY_PU, 'type': 'float', 'format': '.4f'},
                {'name': 'frequency est. (Hz)', 'index': MachineSignalIndex.ESTIMATED_FREQUENCY, 'type': 'float', 'format': '.2f'},
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
                {'name': 'firing angle (pu)', 'index': MachineSignalIndex.CONTROLLER_FIRING_ANGLE, 'type': 'float', 'format': '.4f'},
                {'name': 'current reference', 'index': MachineSignalIndex.TORQUE_CURRENT_REFERENCE, 'type': 'float', 'format': '.4f'},
                {'name': 'current measured', 'index': MachineSignalIndex.CONTROLLER_DC_LINK_CURRENT, 'type': 'float', 'format': '.4f'},
                {'name': 'machine speed', 'index': MachineSignalIndex.CONTROLLER_MACHINE_SPEED, 'type': 'float', 'format': '.4f'},
            ]
        },
        'torque_controller': {
            'title': 'Torque Controller',
            'fields': [
                {'name': 'torque reference', 'index': MachineSignalIndex.TORQUE_TORQUE_REFERENCE, 'type': 'float', 'format': '.4f'},
                {'name': 'max torque limit', 'index': MachineSignalIndex.TORQUE_DYNAMIC_MAX_LIMIT, 'type': 'float', 'format': '.4f'},
                {'name': 'min torque limit', 'index': MachineSignalIndex.TORQUE_DYNAMIC_MIN_LIMIT, 'type': 'float', 'format': '.4f'},
                {'name': 'breakaway boost enabled', 'index': MachineSignalIndex.TORQUE_ENABLE_BREAKAWAY_BOOST, 'type': 'bool', 'format': 'bool'},
                {'name': 'is saturated', 'index': MachineSignalIndex.TORQUE_IS_SATURATED_INPUT, 'type': 'bool', 'format': 'bool'},
                {'name': 'is active', 'index': MachineSignalIndex.TORQUE_IS_ACTIVE, 'type': 'bool', 'format': 'bool'},
            ]
        },
        'speed_controller': {
            'title': 'Speed Controller',
            'fields': [
                {'name': 'speed reference', 'index': MachineSignalIndex.SPEED_SPEED_REFERENCE, 'type': 'float', 'format': '.4f'},
                {'name': 'measured speed', 'index': MachineSignalIndex.SPEED_MEASURED_SPEED, 'type': 'float', 'format': '.4f'},
                {'name': 'pi output', 'index': MachineSignalIndex.SPEED_PI_OUTPUT, 'type': 'float', 'format': '.4f'},
                {'name': 'is saturated', 'index': MachineSignalIndex.SPEED_IS_SATURATED, 'type': 'bool', 'format': 'bool'},
            ]
        },
        'startup_sequencer': {
            'title': 'Startup Sequencer',
            'fields': [
                {'name': 'speed ref.', 'index': MachineSignalIndex.STARTUP_SPEED_REFERENCE, 'type': 'float', 'format': '.4f'},
                {'name': 'virtual rotor freq.', 'index': MachineSignalIndex.STARTUP_VIRTUAL_ROTOR_FREQUENCY, 'type': 'float', 'format': '.4f'},
                {'name': 'virtual rotor angle', 'index': MachineSignalIndex.STARTUP_VIRTUAL_ROTOR_ANGLE, 'type': 'float', 'format': '.4f'},
                {'name': 'virtual torque ref.', 'index': MachineSignalIndex.STARTUP_VIRTUAL_TORQUE_REFERENCE, 'type': 'float', 'format': '.4f'},
                {'name': 'release speed ctrl', 'index': MachineSignalIndex.STARTUP_RELEASE_SPEED_CONTROLLER, 'type': 'bool', 'format': 'bool'},
                {'name': 'release excitation', 'index': MachineSignalIndex.STARTUP_RELEASE_EXCITATION, 'type': 'bool', 'format': 'bool'},
                {'name': 'release torque ctrl', 'index': MachineSignalIndex.STARTUP_RELEASE_TORQUE_CONTROLLER, 'type': 'bool', 'format': 'bool'},
                {'name': 'release pulses', 'index': MachineSignalIndex.STARTUP_RELEASE_MACHINE_SIDE_PULSES, 'type': 'bool', 'format': 'bool'},
                {'name': 'startup complete', 'index': MachineSignalIndex.STARTUP_IS_STARTUP_COMPLETE, 'type': 'bool', 'format': 'bool'},
            ]
        },
        'excitation_controller': {
            'title': 'Excitation Controller',
            'fields': [
                {'name': 'field current ref.', 'index': MachineSignalIndex.EXCITATION_FIELD_CURRENT_REFERENCE, 'type': 'float', 'format': '.4f'},
                {'name': 'prefluxing elapsed time', 'index': MachineSignalIndex.EXCITATION_PREFLUXING_ELAPSED_TIME, 'type': 'float', 'format': '.4f'}
            ]
        },
        'pulse_generator': {
            'title': 'Pulse Generator',
            'fields': [
                {'name': 'virtual ref. active', 'index': MachineSignalIndex.PULSE_GENERATOR_VIRTUAL_REFERENCE_ACTIVE, 'type': 'bool', 'format': 'bool'},
                {'name': 'virtual ref. angle (pu)', 'index': MachineSignalIndex.PULSE_GENERATOR_VIRTUAL_REFERENCE_ANGLE, 'type': 'float', 'format': '.4f'},
                {'name': 'individual override active', 'index': MachineSignalIndex.PULSE_GENERATOR_INDIVIDUAL_OVERRIDE_ACTIVE, 'type': 'bool', 'format': 'bool'},
                {'name': 'pulse 1', 'index': MachineSignalIndex.PULSE_GENERATOR_P1, 'type': 'bool', 'format': 'bool'},
                {'name': 'pulse 2', 'index': MachineSignalIndex.PULSE_GENERATOR_P2, 'type': 'bool', 'format': 'bool'},
                {'name': 'pulse 3', 'index': MachineSignalIndex.PULSE_GENERATOR_P3, 'type': 'bool', 'format': 'bool'},
                {'name': 'pulse 4', 'index': MachineSignalIndex.PULSE_GENERATOR_P4, 'type': 'bool', 'format': 'bool'},
                {'name': 'pulse 5', 'index': MachineSignalIndex.PULSE_GENERATOR_P5, 'type': 'bool', 'format': 'bool'},
                {'name': 'pulse 6', 'index': MachineSignalIndex.PULSE_GENERATOR_P6, 'type': 'bool', 'format': 'bool'},
            ]
        },
        'protections': {
            'title': 'Machine-Side Protections',
            'fields': [
                {'name': 'is tripped', 'index': MachineSignalIndex.PROTECTION_IS_TRIPPED, 'type': 'bool', 'format': 'bool'},
                {'name': 'is inhibited', 'index': MachineSignalIndex.PROTECTION_IS_INHIBITED, 'type': 'bool', 'format': 'bool'},
                {'name': 'any fault active', 'index': MachineSignalIndex.PROTECTION_ANY_FAULT_ACTIVE, 'type': 'bool', 'format': 'bool'},
                {'name': 'over flux tripped', 'index': MachineSignalIndex.PROTECTION_OVER_FLUX_TRIPPED, 'type': 'bool', 'format': 'bool'},
                {'name': 'under flux tripped', 'index': MachineSignalIndex.PROTECTION_UNDER_FLUX_TRIPPED, 'type': 'bool', 'format': 'bool'},
                {'name': 'stall tripped', 'index': MachineSignalIndex.PROTECTION_STALL_TRIPPED, 'type': 'bool', 'format': 'bool'},
                {'name': 'over flux warning', 'index': MachineSignalIndex.PROTECTION_OVER_FLUX_WARNING, 'type': 'bool', 'format': 'bool'},
                {'name': 'under flux warning', 'index': MachineSignalIndex.PROTECTION_UNDER_FLUX_WARNING, 'type': 'bool', 'format': 'bool'},
                {'name': 'stall warning', 'index': MachineSignalIndex.PROTECTION_STALL_WARNING, 'type': 'bool', 'format': 'bool'},
                {'name': 'comm loss tripped', 'index': MachineSignalIndex.PROTECTION_COMMUNICATION_LOSS_TRIPPED, 'type': 'bool', 'format': 'bool'},
                {'name': 'comm loss warning', 'index': MachineSignalIndex.PROTECTION_COMMUNICATION_LOSS_WARNING, 'type': 'bool', 'format': 'bool'},
            ]
        },
        'fiber_optic_rx': {
            'title': 'Fiber Optic RX (from Grid)',
            'fields': [
                {'name': 'dc link current', 'index': MachineSignalIndex.FIBER_RX_DC_LINK_CURRENT, 'type': 'float', 'format': '.4f'},
                {'name': 'grid voltage mag.', 'index': MachineSignalIndex.FIBER_RX_GRID_VOLTAGE_MAGNITUDE, 'type': 'float', 'format': '.4f'},
                {'name': 'is forced commutation', 'index': MachineSignalIndex.FIBER_RX_IS_FORCED_COMMUTATION, 'type': 'bool', 'format': 'bool'},
                {'name': 'force comm. ack', 'index': MachineSignalIndex.FIBER_RX_FORCE_COMMUTATION_ACKNOWLEDGED, 'type': 'bool', 'format': 'bool'},
                {'name': 'release pulses', 'index': MachineSignalIndex.FIBER_RX_RELEASE_PULSES, 'type': 'bool', 'format': 'bool'},
                {'name': 'grid release protections', 'index': MachineSignalIndex.FIBER_RX_GRID_RELEASE_PROTECTIONS, 'type': 'bool', 'format': 'bool'},
                {'name': 'grid trip ack', 'index': MachineSignalIndex.FIBER_RX_GRID_TRIP_ACKNOWLEDGED, 'type': 'bool', 'format': 'bool'},
                {'name': 'state machine ID', 'index': MachineSignalIndex.FIBER_RX_STATE_ID, 'type': 'int', 'format': '.0f'},
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
    # Machine-side has: system, startup_sequencer, speed_controller, torque_controller, measurements, observer, modulator
    visible = ['system', 'startup_sequencer', 'speed_controller', 'torque_controller', 'measurements', 'observer']
    return [s for s in section_order if s in visible]
