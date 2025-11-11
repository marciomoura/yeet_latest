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
                    {'name': 'angle (pu)', 'color': 'y', 'index': GridSignalIndex.ESTIMATED_REFERENCE_ANGLE}
                ]
            }
        },
        'controller': {
            'dclink': {
                'title': 'DC Link Current Control',
                'y_label': 'Current (pu)',
                'signals': [
                    {'name': 'current ref.', 'color': 'g', 'index': GridSignalIndex.CURRENT_REFERENCE},
                    {'name': 'current meas.', 'color': 'r', 'index': GridSignalIndex.CURRENT_MEASURED},
                    {'name': 'current error', 'color': 'c', 'index': GridSignalIndex.CURRENT_ERROR},
                    {'name': 'pi output', 'color': 'm', 'index': GridSignalIndex.CURRENT_PI_OUTPUT}
                ]
            }
        },
        'modulator': {
            'firing_angle_control': {
                'title': 'Firing Angle Control',
                'y_label': 'Angle (pu) / Mode / State',
                'signals': [
                    {'name': 'firing angle in', 'color': 'g', 'index': GridSignalIndex.MODULATOR_FIRING_ANGLE_INPUT},
                    {'name': 'firing angle out', 'color': 'r', 'index': GridSignalIndex.MODULATOR_FIRING_ANGLE_OUTPUT},
                ]
            },
            'forced_commutation': {
                'title': 'Forced Commutation Control',
                'y_label': 'Requests / State',
                'signals': [
                    {'name': 'request force commutation', 'color': 'c', 'index': GridSignalIndex.MODULATOR_REQUEST_FORCE_COMMUTATION},
                    {'name': 'request extinguish dc', 'color': 'm', 'index': GridSignalIndex.MODULATOR_REQUEST_EXTINGUISH_DC},
                    {'name': 'force commutation ack', 'color': 'y', 'index': GridSignalIndex.MODULATOR_FORCE_COMMUTATION_ACK},
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
                    {'name': 'voltage magnitude filtered', 'color': 'g', 'index': MachineSignalIndex.MACHINE_VOLTAGE_MAGNITUDE_FILTERED}
                ]
            }
        },
        'observer': {
            'observer_estimates': {
                'title': 'Observer Estimates',
                'y_label': 'Frequency (pu) / Angle (pu)',
                'signals': [
                    {'name': 'estimated voltage mag.', 'color': 'b', 'index': MachineSignalIndex.ESTIMATED_VOLTAGE_MAGNITUDE},
                    {'name': 'estimated frequency (pu)', 'color': 'm', 'index': MachineSignalIndex.ESTIMATED_SPEED},
                    {'name': 'estimated reference angle (pu)', 'color': 'y', 'index': MachineSignalIndex.ESTIMATED_REFERENCE_ANGLE}
                ]
            }
        },
        'modulator': {
            'modulator_control': {
                'title': 'Modulator Control',
                'y_label': 'Angle (pu) / State',
                'signals': [
                    {'name': 'firing angle (pu)', 'color': 'g', 'index': MachineSignalIndex.MODULATOR_FIRING_ANGLE},
                    {'name': 'release pulses input', 'color': 'r', 'index': MachineSignalIndex.MODULATOR_RELEASE_PULSES_INPUT},
                    {'name': 'release pulses output', 'color': 'c', 'index': MachineSignalIndex.MODULATOR_RELEASE_PULSES_OUTPUT},
                    {'name': 'force commutation', 'color': 'm', 'index': MachineSignalIndex.MODULATOR_REQUEST_FORCE_COMMUTATION}
                ]
            }
        },
        'controller': {
            'machine_controller': {
                'title': 'Machine-Side Controller',
                'y_label': 'Firing Angle (rad) / Current (pu) / Speed (pu)',
                'signals': [
                    {'name': 'firing angle (rad)', 'color': 'g', 'index': MachineSignalIndex.CONTROLLER_FIRING_ANGLE},
                    {'name': 'dc link current', 'color': 'r', 'index': MachineSignalIndex.CONTROLLER_DC_LINK_CURRENT},
                    {'name': 'machine speed', 'color': 'c', 'index': MachineSignalIndex.CONTROLLER_MACHINE_SPEED},
                ]
            }
        },
        'torque_controller': {
            'torque_control': {
                'title': 'Torque Controller',
                'y_label': 'Reference (pu) / Limit (pu) / State',
                'signals': [
                    {'name': 'torque ref.', 'color': 'g', 'index': MachineSignalIndex.TORQUE_TORQUE_REFERENCE},
                    {'name': 'current reference', 'color': 'c', 'index': MachineSignalIndex.TORQUE_CURRENT_REFERENCE},
                    {'name': 'is saturated', 'color': 'w', 'index': MachineSignalIndex.TORQUE_IS_SATURATED_OUTPUT},
                    {'name': 'is active', 'color': 'y', 'index': MachineSignalIndex.TORQUE_IS_ACTIVE}
                ]
            },
        },
        'speed_controller': {
            'speed_control': {
                'title': 'Speed Controller',
                'y_label': 'Speed Ref (pu) / Measured Speed (pu) / Torque Ref (pu)',
                'signals': [
                    {'name': 'speed ref.', 'color': 'g', 'index': MachineSignalIndex.SPEED_SPEED_REFERENCE},
                    {'name': 'measured speed', 'color': 'r', 'index': MachineSignalIndex.SPEED_MEASURED_SPEED},
                    {'name': 'torque reference', 'color': 'c', 'index': MachineSignalIndex.SPEED_TORQUE_REFERENCE},
                ]
            }
        },
        'startup_sequencer': {
            'startup_outputs': {
                'title': 'Startup Sequencer',
                'y_label': 'Speed Ref (pu) / Virtual Freq (pu) / Virtual Torque (pu) / State',
                'signals': [
                    {'name': 'speed ref.', 'color': 'g', 'index': MachineSignalIndex.STARTUP_SPEED_REFERENCE},
                    {'name': 'virtual rotor freq.', 'color': 'r', 'index': MachineSignalIndex.STARTUP_VIRTUAL_ROTOR_FREQUENCY},
                    {'name': 'virtual rotor angle', 'color': 'b', 'index': MachineSignalIndex.STARTUP_VIRTUAL_ROTOR_ANGLE},
                    {'name': 'virtual torque ref.', 'color': 'c', 'index': MachineSignalIndex.STARTUP_VIRTUAL_TORQUE_REFERENCE},
                    {'name': 'release speed ctrl', 'color': 'm', 'index': MachineSignalIndex.STARTUP_RELEASE_SPEED_CONTROLLER},
                    {'name': 'release excitation', 'color': 'y', 'index': MachineSignalIndex.STARTUP_RELEASE_EXCITATION},
                    {'name': 'release torque ctrl', 'color': 'w', 'index': MachineSignalIndex.STARTUP_RELEASE_TORQUE_CONTROLLER},
                    {'name': 'release pulses', 'color': 'cyan', 'index': MachineSignalIndex.STARTUP_RELEASE_MACHINE_SIDE_PULSES},
                    {'name': 'startup complete', 'color': 'orange', 'index': MachineSignalIndex.STARTUP_IS_STARTUP_COMPLETE},
                ]
            }
        },
        'excitation': {
            'excitation_control': {
                'title': 'Excitation Controller',
                'y_label': 'Current (pu) / Time (s)',
                'signals': [
                    {'name': 'field current ref.', 'color': 'g', 'index': MachineSignalIndex.EXCITATION_FIELD_CURRENT_REFERENCE},
                    {'name': 'prefluxing elapsed time', 'color': 'r', 'index': MachineSignalIndex.EXCITATION_PREFLUXING_ELAPSED_TIME},
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
        return ['startup_sequencer', 'torque_controller', 'speed_controller']
    return []