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
                {'name': 'Est Angle (rad)', 'index': 16, 'type': 'float'},
                {'name': 'Est Frequency (Hz)', 'index': 17, 'type': 'float'},
                {'name': 'Est Voltage Alpha (pu)', 'index': 18, 'type': 'float'},
                {'name': 'Est Voltage Beta (pu)', 'index': 19, 'type': 'float'},
            ]
        },
        'modulator': {
            'fields': [
                {'name': 'Firing Angle Input (pu)', 'index': 20, 'type': 'float'},
                {'name': 'Firing Angle Output (pu)', 'index': 21, 'type': 'float'},
                {'name': 'Operating Mode', 'index': 22, 'type': 'float'},
                {'name': 'Request Extinguish DC', 'index': 23, 'type': 'float'},
                {'name': 'Request Force Commutation', 'index': 24, 'type': 'bool'},
                {'name': 'Is Enabled', 'index': 25, 'type': 'bool'},
                {'name': 'Force Commutation Ack', 'index': 26, 'type': 'bool'},
                {'name': 'Is Requesting PI Reset', 'index': 27, 'type': 'bool'},
                {'name': 'Modulator State', 'index': 28, 'type': 'float'},
            ]
        },
        'controller': {
            'fields': [
                {'name': 'DC Link Current Ref (pu)', 'index': 29, 'type': 'float'},
                {'name': 'DC Link Current Meas (pu)', 'index': 30, 'type': 'float'},
                {'name': 'Freeze Integrator', 'index': 31, 'type': 'bool'},
                {'name': 'Modulator Freeze Integrator', 'index': 32, 'type': 'bool'},
                {'name': 'Current Error (pu)', 'index': 33, 'type': 'float'},
                {'name': 'PI Output (pu)', 'index': 34, 'type': 'float'},
                {'name': 'Natural Commutation Mode', 'index': 35, 'type': 'bool'},
                {'name': 'Controller Saturated', 'index': 36, 'type': 'bool'},
                {'name': 'State Machine Freeze', 'index': 37, 'type': 'bool'},
            ]
        },
        'state_machine': {
            'fields': [
                {'name': 'Current State', 'index': 38, 'type': 'float'},
                {'name': 'Pulse Releases', 'index': 39, 'type': 'bool'},
                {'name': 'Auxiliaries On', 'index': 40, 'type': 'bool'},
                {'name': 'Breakers On', 'index': 41, 'type': 'bool'},
                {'name': 'Start Command', 'index': 42, 'type': 'bool'},
                {'name': 'Stop Command', 'index': 43, 'type': 'bool'},
                {'name': 'Current Extinguished', 'index': 44, 'type': 'bool'},
                {'name': 'Shutdown Requested', 'index': 45, 'type': 'bool'},
            ]
        },
        'operation_mode': {
            'fields': [
                {'name': 'Operating Mode', 'index': 46, 'type': 'float'},
                {'name': 'Has Mode Changed', 'index': 47, 'type': 'bool'},
                {'name': 'Is Current Zero', 'index': 48, 'type': 'bool'},
                {'name': 'Is Discontinuous Mode', 'index': 49, 'type': 'bool'},
            ]
        },
        'pulse_generator': {
            'fields': [
                {'name': 'Pulse 1', 'index': 50, 'type': 'bool'},
                {'name': 'Pulse 2', 'index': 51, 'type': 'bool'},
                {'name': 'Pulse 3', 'index': 52, 'type': 'bool'},
                {'name': 'Pulse 4', 'index': 53, 'type': 'bool'},
                {'name': 'Pulse 5', 'index': 54, 'type': 'bool'},
                {'name': 'Pulse 6', 'index': 55, 'type': 'bool'},
                {'name': 'Is Active', 'index': 56, 'type': 'bool'},
                {'name': 'Actual Firing Angle (pu)', 'index': 57, 'type': 'float'},
            ]
        }
    }


def get_default_visible_sections() -> list:
    """Get default visible sections for grid-side telemetry.
    
    Returns:
        List of section names that should be visible by default
    """
    return ['measurements', 'observer', 'controller', 'modulator']
