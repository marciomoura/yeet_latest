#!/usr/bin/env python3
"""Grid-side telemetry signal definitions for monitor displays.

This module defines the declarative structure for grid-side telemetry panels,
mapping signal indices from grid-side telemetry packets to human-readable names
and display parameters.

Organized by functional area:
  - State Machine: Current operating state and commands
  - Measurements: Voltage and current measurements
  - Observer: Estimated grid parameters
  - Controller: Current and power regulation
  - Modulator: Firing angle and pulse control
"""

from typing import Dict, List, Any


def get_grid_side_telemetry() -> Dict[str, Dict[str, List[Dict[str, Any]]]]:
    """Get telemetry definitions for grid-side monitoring.
    
    Returns:
        Dictionary containing section definitions with structure:
        {
            'section_name': {
                'fields': [
                    {'name': str, 'index': int, 'type': 'float'|'bool'},
                    ...
                ]
            }
        }
    """
    return {
        'measurements': {
            'fields': [
                {'name': 'Grid Voltage Magnitude (pu)', 'index': 0, 'type': 'float'},
                {'name': 'Grid Voltage A (pu)', 'index': 1, 'type': 'float'},
                {'name': 'Grid Voltage B (pu)', 'index': 2, 'type': 'float'},
                {'name': 'Grid Voltage C (pu)', 'index': 3, 'type': 'float'},
                {'name': 'Grid Current Magnitude (pu)', 'index': 4, 'type': 'float'},
                {'name': 'Grid Current A (pu)', 'index': 5, 'type': 'float'},
                {'name': 'Grid Current B (pu)', 'index': 6, 'type': 'float'},
                {'name': 'Grid Current C (pu)', 'index': 7, 'type': 'float'},
                {'name': 'DC Link Current (pu)', 'index': 8, 'type': 'float'},
                {'name': 'Is Commuting', 'index': 9, 'type': 'bool'},
                {'name': 'Grid Voltage Alpha (pu)', 'index': 10, 'type': 'float'},
                {'name': 'Grid Voltage Beta (pu)', 'index': 11, 'type': 'float'},
                {'name': 'Grid Voltage Mag Filtered (pu)', 'index': 12, 'type': 'float'},
                {'name': 'Grid Frequency (Hz)', 'index': 13, 'type': 'float'},
            ]
        },
        'observer': {
            'fields': [
                {'name': 'Est Voltage Magnitude (pu)', 'index': 14, 'type': 'float'},
                {'name': 'Est Frequency (pu)', 'index': 15, 'type': 'float'},
                {'name': 'Est Angle (pu)', 'index': 16, 'type': 'float'},
                {'name': 'Est Frequency (Hz)', 'index': 17, 'type': 'float'},
                {'name': 'Est Voltage Alpha (pu)', 'index': 18, 'type': 'float'},
                {'name': 'Est Voltage Beta (pu)', 'index': 19, 'type': 'float'},
            ]
        },
        'modulator': {
            'fields': [
                {'name': 'Firing Angle (rad)', 'index': 20, 'type': 'float'},
                {'name': 'Is Enabled', 'index': 21, 'type': 'bool'},
                {'name': 'Force Commutation Ack', 'index': 22, 'type': 'bool'},
                {'name': 'Is Requesting PI Reset', 'index': 23, 'type': 'bool'},
            ]
        },
        'controller': {
            'fields': [
                {'name': 'DC Link Current Ref (pu)', 'index': 24, 'type': 'float'},
                {'name': 'DC Link Current Meas (pu)', 'index': 25, 'type': 'float'},
                {'name': 'Current Error (pu)', 'index': 26, 'type': 'float'},
                {'name': 'Controller Output (rad)', 'index': 27, 'type': 'float'},
                {'name': 'Operating Mode', 'index': 28, 'type': 'float'},
                {'name': 'Controller Saturated', 'index': 29, 'type': 'bool'},
                {'name': 'Freeze Integrator', 'index': 30, 'type': 'bool'},
            ]
        },
        'state_machine': {
            'fields': [
                {'name': 'Current State', 'index': 31, 'type': 'float'},
                {'name': 'Pulse Releases', 'index': 32, 'type': 'bool'},
                {'name': 'Auxiliaries On', 'index': 33, 'type': 'bool'},
                {'name': 'Breakers On', 'index': 34, 'type': 'bool'},
                {'name': 'Start Command', 'index': 35, 'type': 'bool'},
                {'name': 'Stop Command', 'index': 36, 'type': 'bool'},
                {'name': 'Current Extinguished', 'index': 37, 'type': 'bool'},
                {'name': 'Shutdown Requested', 'index': 38, 'type': 'bool'},
            ]
        },
        'operation_mode': {
            'fields': [
                {'name': 'Operating Mode', 'index': 39, 'type': 'float'},
                {'name': 'Has Mode Changed', 'index': 40, 'type': 'bool'},
                {'name': 'Is Current Zero', 'index': 41, 'type': 'bool'},
                {'name': 'Is Discontinuous Mode', 'index': 42, 'type': 'bool'},
            ]
        },
        'pulse_generator': {
            'fields': [
                {'name': 'Pulse 1', 'index': 43, 'type': 'bool'},
                {'name': 'Pulse 2', 'index': 44, 'type': 'bool'},
                {'name': 'Pulse 3', 'index': 45, 'type': 'bool'},
                {'name': 'Pulse 4', 'index': 46, 'type': 'bool'},
                {'name': 'Pulse 5', 'index': 47, 'type': 'bool'},
                {'name': 'Pulse 6', 'index': 48, 'type': 'bool'},
                {'name': 'Is Active', 'index': 49, 'type': 'bool'},
            ]
        }
    }


def get_default_visible_sections() -> list:
    """Get default visible sections for grid-side telemetry.
    
    Returns:
        List of section names that should be visible by default
    """
    return ['measurements', 'observer', 'controller', 'modulator']
