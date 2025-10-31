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
        'state_machine': {
            'fields': [
                {'name': 'Current State', 'index': 0, 'type': 'float'},
                {'name': 'Pulse Releases', 'index': 1, 'type': 'bool'},
                {'name': 'Auxiliaries On', 'index': 2, 'type': 'bool'},
                {'name': 'Breakers On', 'index': 3, 'type': 'bool'},
                {'name': 'Start Command', 'index': 4, 'type': 'bool'},
                {'name': 'Stop Command', 'index': 5, 'type': 'bool'},
                {'name': 'Current Extinguished', 'index': 6, 'type': 'bool'},
                {'name': 'Shutdown Requested', 'index': 7, 'type': 'bool'},
            ]
        },
        'measurements': {
            'fields': [
                {'name': 'Grid Voltage Magnitude (pu)', 'index': 1, 'type': 'float'},
                {'name': 'Grid Voltage A (pu)', 'index': 2, 'type': 'float'},
                {'name': 'Grid Voltage B (pu)', 'index': 3, 'type': 'float'},
                {'name': 'Grid Voltage C (pu)', 'index': 4, 'type': 'float'},
                {'name': 'Grid Current Magnitude (pu)', 'index': 5, 'type': 'float'},
                {'name': 'Grid Current A (pu)', 'index': 6, 'type': 'float'},
                {'name': 'Grid Current B (pu)', 'index': 7, 'type': 'float'},
                {'name': 'Grid Current C (pu)', 'index': 8, 'type': 'float'},
                {'name': 'DC Link Current (pu)', 'index': 9, 'type': 'float'},
                {'name': 'Is Commuting', 'index': 10, 'type': 'bool'},
            ]
        },
        'observer': {
            'fields': [
                {'name': 'Est Voltage Magnitude (pu)', 'index': 11, 'type': 'float'},
                {'name': 'Est Frequency (pu)', 'index': 12, 'type': 'float'},
                {'name': 'Est Angle (rad)', 'index': 13, 'type': 'float'},
            ]
        },
        'controller': {
            'fields': [
                {'name': 'DC Link Current Ref (pu)', 'index': 14, 'type': 'float'},
                {'name': 'DC Link Current Meas (pu)', 'index': 15, 'type': 'float'},
                {'name': 'Current Error (pu)', 'index': 16, 'type': 'float'},
                {'name': 'Controller Output (pu)', 'index': 17, 'type': 'float'},
                {'name': 'Operating Mode', 'index': 18, 'type': 'float'},
                {'name': 'Controller Saturated', 'index': 19, 'type': 'bool'},
                {'name': 'Freeze Integrator', 'index': 20, 'type': 'bool'},
            ]
        },
        'modulator': {
            'fields': [
                {'name': 'Is Enabled', 'index': 21, 'type': 'bool'},
                {'name': 'Firing Angle In (rad)', 'index': 22, 'type': 'float'},
                {'name': 'Firing Angle Out (rad)', 'index': 23, 'type': 'float'},
                {'name': 'Force Commutation Ack', 'index': 24, 'type': 'bool'},
            ]
        }
    }


def get_default_visible_sections() -> list:
    """Get default visible sections for grid-side telemetry.
    
    Returns:
        List of section names that should be visible by default
    """
    return ['measurements', 'observer', 'controller', 'modulator']
