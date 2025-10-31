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
                    {'name': 'Voltage', 'color': 'g', 'index': 1},
                    {'name': 'Current A', 'color': 'r', 'index': 2},
                    {'name': 'Current B', 'color': 'c', 'index': 3},
                    {'name': 'Current C', 'color': 'm', 'index': 4}
                ]
            }
        },
        'observer': {
            'voltage_freq': {
                'title': 'Observer',
                'y_label': 'Observer',
                'signals': [
                    {'name': 'V Est (pu)', 'color': 'c', 'index': 1},
                    {'name': 'Freq (pu)', 'color': 'm', 'index': 2}
                ]
            }
        },
        'current_controller': {
            'dclink': {
                'title': 'DC Link Control',
                'y_label': 'Current (pu)',
                'signals': [
                    {'name': 'Reference', 'color': 'y', 'index': 1},
                    {'name': 'Measured', 'color': 'w', 'index': 2}
                ]
            }
        },
        'modulator': {
            'firing_angle_control': {
                'title': 'Firing Angle Control',
                'y_label': 'Angle (rad)',
                'signals': [
                    {'name': 'Input', 'color': 'y', 'index': 1},
                    {'name': 'Output', 'color': 'w', 'index': 2},
                    {'name': 'Enabled', 'color': 'c', 'index': 3}
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
                    {'name': 'Current Reference', 'color': 'g', 'index': 26},
                    {'name': 'Limited Torque Ref', 'color': 'm', 'index': 29},
                    {'name': 'Is Saturated', 'color': 'y', 'index': 30}
                ]
            },
        },
        'speed_controller': {
            'speed_control': {
                'title': 'Speed Controller',
                'y_label': 'Torque Ref (pu) / Speed (pu) / Error / Output',
                'signals': [
                    {'name': 'Torque Reference', 'color': 'g', 'index': 34},
                    {'name': 'Measured Speed (pu)', 'color': 'r', 'index': 35},
                    {'name': 'Speed Error', 'color': 'c', 'index': 36},
                    {'name': 'PI Output', 'color': 'm', 'index': 37},
                    {'name': 'Is Active', 'color': 'y', 'index': 38},
                    {'name': 'Is Saturated', 'color': '#ff00ff', 'index': 39},
                ]
            }
        },
        'speed_generator': {
            'speed_gen_outputs': {
                'title': 'Speed Generator',
                'y_label': 'Speed Ref (pu) / Torque Limit (pu) / State',
                'signals': [
                    {'name': 'Speed Ref (pu)', 'color': 'g', 'index': 40},
                    {'name': 'Torque Limit Max', 'color': 'r', 'index': 41},
                    {'name': 'Torque Limit Min', 'color': 'c', 'index': 42},
                    {'name': 'Profile Complete', 'color': 'm', 'index': 43},
                    {'name': 'Breakaway Boost Active', 'color': 'y', 'index': 44},
                ]
            }
        },
        'excitation': {
            'excitation_control': {
                'title': 'Excitation Controller',
                'y_label': 'Voltage (pu) / State',
                'signals': [
                    {'name': 'Excitation V Setpoint', 'color': 'g', 'index': 45},
                    {'name': 'Enable Excitation', 'color': 'r', 'index': 46},
                    {'name': 'Field Established', 'color': 'c', 'index': 47},
                    {'name': 'Is Running', 'color': 'm', 'index': 48},
                    {'name': 'Reserved', 'color': 'y', 'index': 49},
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