#!/usr/bin/env python3
"""Plot definitions for grid-side and machine-side oscilloscope displays.

This module contains the declarative plot configurations that define what signals
are displayed in the oscilloscope for each monitoring target (grid or machine).
Keeping these separate from the UI code makes them easier to maintain and modify.
"""

from typing import Dict, Any
from .grid_side_telemetry_protocol import GridSignalIndex
from .machine_side_telemetry_protocol import MachineSignalIndex


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
            'grid_voltages': {
                'title': 'Grid Voltages',
                'y_label': 'Magnitude (pu)',
                'signals': [
                    {'name': 'voltage a', 'color': 'c', 'index': GridSignalIndex.GRID_VOLTAGE_A},
                    {'name': 'voltage b', 'color': 'm', 'index': GridSignalIndex.GRID_VOLTAGE_B},
                    {'name': 'voltage c', 'color': 'y', 'index': GridSignalIndex.GRID_VOLTAGE_C},
                    {'name': 'voltage mag.', 'color': 'g', 'index': GridSignalIndex.GRID_VOLTAGE_MAGNITUDE_FILTERED},
                ]
            },
            'grid_currents': {
                'title': 'Grid Currents',
                'y_label': 'Magnitude (pu)',
                'signals': [
                    {'name': 'current a', 'color': 'c', 'index': GridSignalIndex.GRID_CURRENT_A},
                    {'name': 'current b', 'color': 'm', 'index': GridSignalIndex.GRID_CURRENT_B},
                    {'name': 'current c', 'color': 'y', 'index': GridSignalIndex.GRID_CURRENT_C},
                    {'name': 'current mag.', 'color': 'g', 'index': GridSignalIndex.GRID_CURRENT_MAGNITUDE},
                ]
            }

        },
        'observer': {
            'voltage_freq': {
                'title': 'Observer Estimates',
                'y_label': 'Magnitude (pu) / Frequency (Hz) / Angle (rad)',
                'signals': [
                    {'name': 'voltage mag.', 'color': 'c', 'index': GridSignalIndex.ESTIMATED_VOLTAGE_MAGNITUDE},
                    {'name': 'frequency (pu)', 'color': 'm', 'index': GridSignalIndex.ESTIMATED_FREQUENCY_PU},
                    {'name': 'angle (pu)', 'color': 'y', 'index': GridSignalIndex.ESTIMATED_ANGLE_RAD}
                ]
            }
        },
        'controller': {
            'dclink': {
                'title': 'DC Link Current Control',
                'y_label': 'Current (pu)',
                'signals': [
                    {'name': 'current ref.', 'color': 'g', 'index': GridSignalIndex.DC_LINK_CURRENT_REFERENCE},
                    {'name': 'current meas.', 'color': 'r', 'index': GridSignalIndex.DC_LINK_CURRENT_MEASURED}
                ]
            }
        },
        'modulator': {
            'firing_angle_control': {
                'title': 'Firing Angle Control',
                'y_label': 'Angle (pu) / Mode / Requests / State',
                'signals': [
                    {'name': 'firing angle in', 'color': 'g', 'index': GridSignalIndex.FIRING_ANGLE_INPUT},
                    {'name': 'firing angle out', 'color': 'r', 'index': GridSignalIndex.FIRING_ANGLE_OUTPUT},
                ]
            },
            'forced_commutation': {
                'title': 'Forced Commutation Control',
                'y_label': 'Angle (pu) / Mode / Requests / State',
                'signals': [
                    {'name': 'request force commutation', 'color': 'c', 'index': GridSignalIndex.MODULATOR_REQUEST_FORCE_COMMUTATION},
                    {'name': 'request extinguish dc', 'color': 'm', 'index': GridSignalIndex.MODULATOR_REQUEST_EXTINGUISH_DC},
                    {'name': 'force commutation ack', 'color': 'y', 'index': GridSignalIndex.FORCE_COMMUTATION_ACK},
                    {'name': 'pi reset request', 'color': 'g', 'index': GridSignalIndex.IS_REQUESTING_PI_RESET}
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
                    {'name': 'voltage a', 'color': 'c', 'index': MachineSignalIndex.MACHINE_VOLTAGE_ABC_A},
                    {'name': 'voltage b', 'color': 'm', 'index': MachineSignalIndex.MACHINE_VOLTAGE_ABC_B},
                    {'name': 'voltage c', 'color': 'y', 'index': MachineSignalIndex.MACHINE_VOLTAGE_ABC_C},
                    {'name': 'voltage magnitude', 'color': 'g', 'index': MachineSignalIndex.MACHINE_VOLTAGE_MAGNITUDE}
                ]
            }
        },
        'observer': {
            'observer_estimates': {
                'title': 'Observer Estimates',
                'y_label': 'Frequency (pu) / Angle (pu)',
                'signals': [
                    {'name': 'voltage mag.', 'color': 'c', 'index': MachineSignalIndex.MACHINE_VOLTAGE_MAGNITUDE_FILTERED},
                    {'name': 'frequency (pu)', 'color': 'm', 'index': MachineSignalIndex.ESTIMATED_FREQUENCY_PU},
                    {'name': 'angle (pu)', 'color': 'y', 'index': MachineSignalIndex.ESTIMATED_POSITION_PU}
                ]
            }
        },
        'modulator': {
            'modulator_control': {
                'title': 'Modulator Control',
                'y_label': 'Angle (rad) / State',
                'signals': [
                    {'name': 'firing angle (rad)', 'color': 'g', 'index': MachineSignalIndex.FIRING_ANGLE_RAD},
                    {'name': 'enabled', 'color': 'r', 'index': MachineSignalIndex.MODULATOR_ENABLED},
                    {'name': 'release pulses', 'color': 'c', 'index': MachineSignalIndex.RELEASE_PULSES},
                    {'name': 'force commutation', 'color': 'm', 'index': MachineSignalIndex.REQUEST_FORCE_COMMUTATION},
                    {'name': 'rotor sector', 'color': 'y', 'index': MachineSignalIndex.CURRENT_ROTOR_SECTOR},
                ]
            }
        },
        'torque_controller': {
            'torque_control': {
                'title': 'Torque Controller',
                'y_label': 'Reference (pu) / Limit (pu) / State',
                'signals': [
                    {'name': 'torque ref.', 'color': 'g', 'index': MachineSignalIndex.TORQUE_REF_FROM_SPEED_CONTROLLER},
                    {'name': 'current controller saturated', 'color': 'y', 'index': MachineSignalIndex.TORQUE_CURRENT_CONTROLLER_SATURATED},
                    {'name': 'current reference', 'color': 'c', 'index': MachineSignalIndex.TORQUE_CURRENT_REFERENCE},
                    {'name': 'limited torque ref.', 'color': 'm', 'index': MachineSignalIndex.TORQUE_LIMITED_REFERENCE},
                    {'name': 'is saturated', 'color': 'w', 'index': MachineSignalIndex.TORQUE_IS_SATURATED}
                ]
            },
        },
        'speed_controller': {
            'speed_control': {
                'title': 'Speed Controller',
                'y_label': 'Speed Ref (pu) / Measured Speed (pu) / Torque Ref (pu) / Error',
                'signals': [
                    {'name': 'speed ref.', 'color': 'g', 'index': MachineSignalIndex.SPEED_REFERENCE_PU},
                    {'name': 'measured speed', 'color': 'r', 'index': MachineSignalIndex.SPEED_MEASURED_SPEED_PU},
                    {'name': 'speed ref. limited', 'color': 'c', 'index': MachineSignalIndex.SPEED_REFERENCE_LIMITED_PU},
                ]
            }
        },
        'speed_generator': {
            'speed_gen_outputs': {
                'title': 'Speed Generator',
                'y_label': 'Speed Ref (pu) / Torque Limit (pu) / State',
                'signals': [
                    {'name': 'speed ref.', 'color': 'g', 'index': MachineSignalIndex.SPEED_REF_PU},
                    {'name': 'torque limit max', 'color': 'r', 'index': MachineSignalIndex.SPEED_TORQUE_LIMIT_MAX_PU},
                    {'name': 'torque limit min', 'color': 'c', 'index': MachineSignalIndex.SPEED_TORQUE_LIMIT_MIN_PU},
                    {'name': 'profile complete', 'color': 'm', 'index': MachineSignalIndex.SPEED_IS_PROFILE_COMPLETE},
                    {'name': 'breakaway boost active', 'color': 'y', 'index': MachineSignalIndex.SPEED_IS_BREAKAWAY_BOOST_ACTIVE},
                ]
            }
        },
        'excitation': {
            'excitation_control': {
                'title': 'Excitation Controller',
                'y_label': 'Voltage (pu) / State',
                'signals': [
                    {'name': 'excitation v setpoint', 'color': 'g', 'index': MachineSignalIndex.EXCITATION_VOLTAGE_SETPOINT_PU},
                    {'name': 'enable excitation', 'color': 'r', 'index': MachineSignalIndex.EXCITATION_ENABLE},
                    {'name': 'field established', 'color': 'c', 'index': MachineSignalIndex.EXCITATION_IS_FIELD_ESTABLISHED},
                    {'name': 'is running', 'color': 'm', 'index': MachineSignalIndex.EXCITATION_IS_RUNNING},
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
        return ['observer', 'measurements', 'controller'] 
    if target == 'machine':
        return ['torque_controller', 'speed_controller', 'speed_generator']
    return []