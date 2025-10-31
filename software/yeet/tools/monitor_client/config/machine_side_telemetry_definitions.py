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
  - 34-39: Speed Controller (Torque ref, speed, error, PI output, saturation)
  - 40-44: Speed Generator (Speed ref, torque limits, profile state)
  - 45-49: Excitation Controller (Voltage setpoint, enable, field state, running)
"""

from typing import Dict, List, Any


def get_machine_side_telemetry() -> Dict[str, Dict[str, List[Dict[str, Any]]]]:
    """Get telemetry definitions for machine-side monitoring.
    
    Maps signal indices from machine_side_telemetry_builder.cpp to
    human-readable names and display parameters.
    
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
                {'name': 'Voltage Magnitude (pu)', 'index': 6, 'type': 'float'},
                {'name': 'Voltage Magnitude Filtered (pu)', 'index': 7, 'type': 'float'}
            ]
        },
        'observer': {
            'fields': [
                {'name': 'Frequency Est (pu)', 'index': 9, 'type': 'float'},
                {'name': 'Position Est (pu)', 'index': 10, 'type': 'float'},
                {'name': 'Voltage Alpha Filtered (pu)', 'index': 11, 'type': 'float'},
                {'name': 'Voltage Beta Filtered (pu)', 'index': 12, 'type': 'float'},
            ]
        },
        'modulator': {
            'fields': [
                {'name': 'Firing Angle (rad)', 'index': 13, 'type': 'float'},
                {'name': 'Modulator Enabled', 'index': 14, 'type': 'bool'},
                {'name': 'Release Pulses', 'index': 15, 'type': 'bool'},
                {'name': 'Force Commutation', 'index': 16, 'type': 'bool'},
                {'name': 'Rotor Sector', 'index': 17, 'type': 'float'},
                {'name': 'Is Commuting', 'index': 8, 'type': 'bool'}
            ]
        },
        'machine_controller': {
            'fields': [
                {'name': 'Firing Angle Out (rad)', 'index': 18, 'type': 'float'},
                {'name': 'DC Link Current (pu)', 'index': 19, 'type': 'float'},
                {'name': 'Machine Speed (pu)', 'index': 20, 'type': 'float'},
            ]
        },
        'torque_controller_inputs': {
            'fields': [
                {'name': 'Torque Ref from Speed (pu)', 'index': 21, 'type': 'float'},
                {'name': 'Max Torque Limit (pu)', 'index': 22, 'type': 'float'},
                {'name': 'Min Torque Limit (pu)', 'index': 23, 'type': 'float'},
                {'name': 'Breakaway Boost Enabled', 'index': 24, 'type': 'bool'},
                {'name': 'Current Controller Saturated', 'index': 25, 'type': 'bool'},
            ]
        },
        'torque_controller_outputs': {
            'fields': [
                {'name': 'Current Reference (pu)', 'index': 26, 'type': 'float'},
                {'name': 'Base Torque Ref (pu)', 'index': 27, 'type': 'float'},
                {'name': 'Total Torque Ref (pu)', 'index': 28, 'type': 'float'},
                {'name': 'Limited Torque Ref (pu)', 'index': 29, 'type': 'float'},
                {'name': 'Is Saturated', 'index': 30, 'type': 'bool'},
                {'name': 'Is Active', 'index': 31, 'type': 'bool'},
                {'name': 'Breakaway Boost Active', 'index': 32, 'type': 'bool'},
                {'name': 'Downstream Saturated', 'index': 33, 'type': 'bool'},
            ]
        },
        'speed_controller': {
            'fields': [
                {'name': 'Torque Reference (pu)', 'index': 34, 'type': 'float'},
                {'name': 'Measured Speed (pu)', 'index': 35, 'type': 'float'},
                {'name': 'Speed Error (pu)', 'index': 36, 'type': 'float'},
                {'name': 'PI Output (pu)', 'index': 37, 'type': 'float'},
                {'name': 'Is Active', 'index': 38, 'type': 'bool'},
                {'name': 'Is Saturated', 'index': 39, 'type': 'bool'},
            ]
        },
        'speed_generator': {
            'fields': [
                {'name': 'Speed Ref (pu)', 'index': 40, 'type': 'float'},
                {'name': 'Torque Limit Max (pu)', 'index': 41, 'type': 'float'},
                {'name': 'Torque Limit Min (pu)', 'index': 42, 'type': 'float'},
                {'name': 'Profile Complete', 'index': 43, 'type': 'bool'},
                {'name': 'Breakaway Boost Active', 'index': 44, 'type': 'bool'},
            ]
        },
        'excitation_controller': {
            'fields': [
                {'name': 'Excitation V Setpoint (pu)', 'index': 45, 'type': 'float'},
                {'name': 'Enable Excitation', 'index': 46, 'type': 'bool'},
                {'name': 'Field Established', 'index': 47, 'type': 'bool'},
                {'name': 'Is Running', 'index': 48, 'type': 'bool'},
                {'name': 'Reserved', 'index': 49, 'type': 'float'},
            ]
        }
    }


def get_default_visible_sections() -> list:
    """Get default visible sections for machine-side telemetry.
    
    Returns:
        List of section names that should be visible by default
    """
    return [
        'measurements',
        'observer',
        'torque_controller_outputs',
        'speed_controller',
        'machine_controller'
    ]
