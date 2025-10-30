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
            'all_measurements': {
                'title': 'Machine Measurements',
                'y_label': 'Magnitude (pu) / Speed (rpm) / State',
                'signals': [
                    {'name': 'V Filtered', 'color': 'g', 'index': 0},
                    {'name': 'V_A', 'color': 'r', 'index': 1},
                    {'name': 'V_B', 'color': 'c', 'index': 2},
                    {'name': 'V_C', 'color': 'm', 'index': 3},
                    {'name': 'Speed', 'color': 'y', 'index': 4},
                    {'name': 'V Magnitude', 'color': '#00ff00', 'index': 5},
                    {'name': 'Commuting', 'color': 'w', 'index': 6},
                ]
            }
        },
        'observer': {
            'all_observer': {
                'title': 'Observer Estimates',
                'y_label': 'Voltage (pu) / Frequency (pu/Hz) / Angle (rad)',
                'signals': [
                    {'name': 'Est V Mag', 'color': 'g', 'index': 7},
                    {'name': 'Freq (pu)', 'color': 'r', 'index': 8},
                    {'name': 'Freq (Hz)', 'color': 'c', 'index': 9},
                    {'name': 'Angle', 'color': 'm', 'index': 10},
                    {'name': 'V Alpha', 'color': 'y', 'index': 11},
                ]
            }
        },
        'speed_controller': {
            'all_speed_control': {
                'title': 'Speed Controller',
                'y_label': 'Torque (pu) / Speed (rpm) / Error / Output',
                'signals': [
                    {'name': 'Torque Ref', 'color': 'g', 'index': 12},
                    {'name': 'Meas Speed', 'color': 'r', 'index': 13},
                    {'name': 'Speed Error', 'color': 'c', 'index': 14},
                    {'name': 'PI Output', 'color': 'm', 'index': 15},
                    {'name': 'Saturated', 'color': 'y', 'index': 16},
                ]
            }
        },
        'torque_controller': {
            'all_torque_control': {
                'title': 'Torque Controller',
                'y_label': 'Current / Reference (pu)',
                'signals': [
                    {'name': 'Current Ref', 'color': 'g', 'index': 17},
                    {'name': 'Base Ref', 'color': 'r', 'index': 18},
                    {'name': 'Total Ref', 'color': 'c', 'index': 19},
                    {'name': 'Limited Ref', 'color': 'm', 'index': 20},
                    {'name': 'Saturated', 'color': 'y', 'index': 21},
                ]
            }
        },
        'speed_generator': {
            'all_speed_gen': {
                'title': 'Speed Generator',
                'y_label': 'Speed (rpm) / Torque (pu) / State',
                'signals': [
                    {'name': 'Speed Ref', 'color': 'g', 'index': 22},
                    {'name': 'Torque Max', 'color': 'r', 'index': 23},
                    {'name': 'Torque Min', 'color': 'c', 'index': 24},
                    {'name': 'Profile Complete', 'color': 'm', 'index': 25},
                    {'name': 'Breakaway Boost', 'color': 'y', 'index': 26},
                ]
            }
        },
        'modulator': {
            'all_modulator': {
                'title': 'Modulator',
                'y_label': 'Angle (deg) / State',
                'signals': [
                    {'name': 'Firing In', 'color': 'g', 'index': 27},
                    {'name': 'Firing Out', 'color': 'r', 'index': 28},
                    {'name': 'Enabled', 'color': 'c', 'index': 29},
                    {'name': 'Force Comm', 'color': 'm', 'index': 30},
                    {'name': 'Rotor Sector', 'color': 'y', 'index': 31},
                ]
            }
        },
        'excitation': {
            'all_excitation': {
                'title': 'Excitation Controller',
                'y_label': 'Voltage (pu) / State',
                'signals': [
                    {'name': 'V Setpoint', 'color': 'g', 'index': 32},
                    {'name': 'Enable', 'color': 'r', 'index': 33},
                    {'name': 'Field Est', 'color': 'c', 'index': 34},
                    {'name': 'Running', 'color': 'm', 'index': 35},
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