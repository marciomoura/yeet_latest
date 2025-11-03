#!/usr/bin/env python3
"""Plot definitions for grid-side and machine-side oscilloscope displays.

This module contains the declarative plot configurations that define what signals
are displayed in the oscilloscope for each monitoring target (grid or machine).
Keeping these separate from the UI code makes them easier to maintain and modify.
"""

from typing import Dict, Any


def get_grid_side_plots() -> Dict[str, Dict[str, Any]]:
    """Get plot definitions for grid-side monitoring.
    
    Returns:
        Dictionary containing plot group definitions with structure:
        {
            'group_name': {
                'plot_name': {
                    'title': str,
                    'y_label': str,
                    'signals': [{'name': str, 'color': str, 'index': int}, ...]
                }
            }
        }
    """
    return {
        'measurements': {
            'voltage_and_current': {
                'title': 'Grid Measurements',
                'y_label': 'Magnitude (pu)',
                'signals': [
                    {'name': 'Voltage Magnitude', 'color': 'g', 'index': 0},
                    {'name': 'Current Magnitude', 'color': 'r', 'index': 4},
                    {'name': 'Current A', 'color': 'c', 'index': 5},
                    {'name': 'Current B', 'color': 'm', 'index': 6},
                    {'name': 'Current C', 'color': 'y', 'index': 7},
                ]
            }
        },
        'observer': {
            'voltage_freq': {
                'title': 'Observer Estimates',
                'y_label': 'Magnitude (pu) / Frequency (Hz) / Angle (rad)',
                'signals': [
                    {'name': 'Voltage Magnitude (pu)', 'color': 'g', 'index': 14},
                    {'name': 'Frequency (Hz)', 'color': 'r', 'index': 17},
                    {'name': 'Angle (rad)', 'color': 'c', 'index': 16}
                ]
            }
        },
        'controller': {
            'dclink': {
                'title': 'DC Link Current Control',
                'y_label': 'Current (pu)',
                'signals': [
                    {'name': 'Reference', 'color': 'g', 'index': 29},
                    {'name': 'Measured', 'color': 'r', 'index': 30},
                    {'name': 'Error', 'color': 'c', 'index': 33},
                ]
            }
        },
        'modulator': {
            'firing_angle_control': {
                'title': 'Firing Angle Control',
                'y_label': 'Angle (pu) / Mode / Requests / State',
                'signals': [
                    {'name': 'Firing Angle Input', 'color': 'g', 'index': 20},
                    {'name': 'Firing Angle Output', 'color': 'r', 'index': 21},
                    {'name': 'Operating Mode', 'color': 'c', 'index': 22},
                    {'name': 'Request Extinguish DC', 'color': 'm', 'index': 23},
                    {'name': 'Request Force Commutation', 'color': 'y', 'index': 24},
                    {'name': 'Enable', 'color': 'w', 'index': 25},
                    {'name': 'Force Commutation Ack', 'color': '#ff7f00', 'index': 26},
                    {'name': 'PI Reset Request', 'color': '#ff00ff', 'index': 27},
                    {'name': 'Modulator State', 'color': 'k', 'index': 28},
                ]
            }
        },
        'pulse_generator': {
            'firing_pulses': {
                'title': 'Firing Pulses',
                'y_label': 'Pulse State',
                'signals': [
                    {'name': 'P1', 'color': 'r', 'index': 50},
                    {'name': 'P2', 'color': 'g', 'index': 51},
                    {'name': 'P3', 'color': 'c', 'index': 52},
                    {'name': 'P4', 'color': 'm', 'index': 53},
                    {'name': 'P5', 'color': 'y', 'index': 54},
                    {'name': 'P6', 'color': 'w', 'index': 55}
                ]
            }
        }
    }


def get_machine_side_plots() -> Dict[str, Dict[str, Any]]:
    """Get plot definitions for machine-side monitoring.
    
    Returns:
        Dictionary containing plot group definitions with structure:
        {
            'group_name': {
                'plot_name': {
                    'title': str,
                    'y_label': str,
                    'signals': [{'name': str, 'color': str, 'index': int}, ...]
                }
            }
        }
    """
    return {
        'measurements': {
            'voltage_measurements': {
                'title': 'Machine Voltage Measurements',
                'y_label': 'Voltage (pu)',
                'signals': [
                    {'name': 'V_A (Unmasked)', 'color': 'y', 'index': 3},
                    {'name': 'V_B (Unmasked)', 'color': 'w', 'index': 4},
                    {'name': 'V_C (Unmasked)', 'color': '#ff00ff', 'index': 5},
                    {'name': 'V_A (Masked)', 'color': 'r', 'index': 0},
                    {'name': 'V_B (Masked)', 'color': 'c', 'index': 1},
                    {'name': 'V_C (Masked)', 'color': 'm', 'index': 2},
                    {'name': 'V Magnitude', 'color': 'g', 'index': 6}
                ]
            }
        },
        'observer': {
            'observer_estimates': {
                'title': 'Observer Estimates',
                'y_label': 'Frequency (pu) / Angle (pu)',
                'signals': [
                    {'name': 'Frequency (pu)', 'color': 'g', 'index': 9},
                    {'name': 'Position (pu)', 'color': 'r', 'index': 10}
                ]
            }
        },
        'modulator': {
            'modulator_control': {
                'title': 'Modulator Control',
                'y_label': 'Angle (rad) / State',
                'signals': [
                    {'name': 'Firing Angle (rad)', 'color': 'g', 'index': 13},
                    {'name': 'Enabled', 'color': 'r', 'index': 14},
                    {'name': 'Release Pulses', 'color': 'c', 'index': 15},
                    {'name': 'Force Commutation', 'color': 'm', 'index': 16},
                    {'name': 'Rotor Sector', 'color': 'y', 'index': 17},
                ]
            }
        },
        'torque_controller': {
            'torque_control': {
                'title': 'Torque Controller',
                'y_label': 'Reference (pu) / Limit (pu) / State',
                'signals': [
                    {'name': 'Torque Reference (from Speed)', 'color': 'g', 'index': 21},
                    {'name': 'Current Controller Saturated', 'color': 'y', 'index': 25},
                    {'name': 'Current Reference', 'color': 'c', 'index': 26},
                    {'name': 'Limited Torque Ref', 'color': 'm', 'index': 29},
                    {'name': 'Is Saturated', 'color': 'w', 'index': 30}
                ]
            },
        },
        'speed_controller': {
            'speed_control': {
                'title': 'Speed Controller',
                'y_label': 'Speed Ref (pu) / Measured Speed (pu) / Torque Ref (pu) / Error / PI Output',
                'signals': [
                    {'name': 'Speed Reference', 'color': 'g', 'index': 34},
                    {'name': 'Measured Speed', 'color': 'r', 'index': 35},
                    {'name': 'Speed Ref Limited', 'color': 'c', 'index': 36},
                    {'name': 'Torque Reference', 'color': 'm', 'index': 37},
                    {'name': 'Speed Error', 'color': 'y', 'index': 38},
                    {'name': 'PI Output', 'color': 'w', 'index': 39},
                    {'name': 'Is Active', 'color': '#ff7f00', 'index': 40},
                    {'name': 'Is Saturated', 'color': '#ff00ff', 'index': 41},
                ]
            }
        },
        'speed_generator': {
            'speed_gen_outputs': {
                'title': 'Speed Generator',
                'y_label': 'Speed Ref (pu) / Torque Limit (pu) / State',
                'signals': [
                    {'name': 'Speed Ref (pu)', 'color': 'g', 'index': 42},
                    {'name': 'Torque Limit Max', 'color': 'r', 'index': 43},
                    {'name': 'Torque Limit Min', 'color': 'c', 'index': 44},
                    {'name': 'Profile Complete', 'color': 'm', 'index': 45},
                    {'name': 'Breakaway Boost Active', 'color': 'y', 'index': 46},
                ]
            }
        },
        'excitation': {
            'excitation_control': {
                'title': 'Excitation Controller',
                'y_label': 'Voltage (pu) / State',
                'signals': [
                    {'name': 'Excitation V Setpoint', 'color': 'g', 'index': 47},
                    {'name': 'Enable Excitation', 'color': 'r', 'index': 48},
                    {'name': 'Field Established', 'color': 'c', 'index': 49},
                    {'name': 'Is Running', 'color': 'm', 'index': 50},
                ]
            }
        }
    }


def get_default_visible_groups(target: str) -> list:
    """Get default visible plot groups for a given target.
    
    Args:
        target: Either 'grid' or 'machine'
        
    Returns:
        List of group names that should be visible by default
    """
    if target == 'grid':
        return ['measurements', 'observer', 'current_controller']
    if target == 'machine':
        return ['measurements', 'observer', 'torque_controller', 'speed_controller']
    return []