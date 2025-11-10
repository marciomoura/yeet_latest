#!/usr/bin/env python3
"""Unified telemetry-to-plot mapper for machine-side monitoring.

This module consolidates:
1. Plot definitions (what to display and with what colors)
2. Data extraction logic (how to map telemetry fields to plot signals)
3. Buffer organization (group_plot_name → signal indices)

This eliminates the disconnect between plot_definitions.py and main_window.py,
ensuring that adding a new signal requires only one place to update.

Architecture:
- Each plot group/plot has a "mapper" function that extracts data from telemetry packet
- Mapper functions return tuples of (sample_index, signal1, signal2, ...)
- Main window simply calls the mapper and adds buffer via oscilloscope widget
"""

from typing import Dict, Callable, Tuple, Any, Optional
from dataclasses import dataclass


@dataclass
class PlotConfig:
    """Configuration for a single plot."""
    title: str
    y_label: str
    signals: list  # List of {'name': str, 'color': str, 'index': int}
    mapper: Optional[Callable] = None  # Function to extract data from packet


class TelemetryPlotMapper:
    """Unified mapper for telemetry packets to oscilloscope buffers."""
    
    @staticmethod
    def get_machine_side_plots() -> Dict[str, Dict[str, PlotConfig]]:
        """Get complete machine-side plot configuration with data mappers.
        
        Returns:
            Dictionary with structure:
            {
                'group_name': {
                    'plot_name': PlotConfig(title, y_label, signals, mapper_func)
                }
            }
        """
        return {
            'measurements': {
                'voltage_measurements': PlotConfig(
                    title='Machine Voltage Measurements',
                    y_label='Voltage (pu)',
                    signals=[
                        {'name': 'V_A', 'color': 'r', 'index': 0},
                        {'name': 'V_B', 'color': 'c', 'index': 1},
                        {'name': 'V_C', 'color': 'm', 'index': 2},
                        {'name': 'V Magnitude (Filtered)', 'color': 'g', 'index': 3},
                        {'name': 'Estimated Speed', 'color': 'y', 'index': 4},
                        {'name': 'Estimated Voltage Mag', 'color': 'w', 'index': 5},
                    ],
                    mapper=lambda sample_idx, pkt: (
                        sample_idx,
                        pkt.measurements.machine_voltage_abc_a_pu,
                        pkt.measurements.machine_voltage_abc_b_pu,
                        pkt.measurements.machine_voltage_abc_c_pu,
                        pkt.measurements.machine_voltage_magnitude_filtered_pu,
                        pkt.measurements.estimated_speed_pu,
                        pkt.measurements.estimated_voltage_magnitude_pu,
                    )
                )
            },
            'observer': {
                'observer_estimates': PlotConfig(
                    title='Observer Estimates',
                    y_label='Frequency (pu) / Angle (pu)',
                    signals=[
                        {'name': 'Frequency (pu)', 'color': 'g', 'index': 6},
                        {'name': 'Reference Angle (pu)', 'color': 'r', 'index': 7},
                        {'name': 'Frequency (Hz)', 'color': 'c', 'index': 8},
                    ],
                    mapper=lambda sample_idx, pkt: (
                        sample_idx,
                        pkt.observer.estimated_frequency_pu,
                        pkt.observer.estimated_reference_angle_pu,
                        pkt.observer.estimated_frequency_hz,
                    )
                )
            },
            'modulator': {
                'modulator_control': PlotConfig(
                    title='Modulator Control',
                    y_label='Angle (pu) / State',
                    signals=[
                        {'name': 'Release Pulses In', 'color': 'g', 'index': 9},
                        {'name': 'Operating Mode', 'color': 'r', 'index': 10},
                        {'name': 'Release Pulses Out', 'color': 'c', 'index': 11},
                        {'name': 'Force Commutation', 'color': 'm', 'index': 12},
                        {'name': 'Force Comm Ack', 'color': 'y', 'index': 13},
                        {'name': 'Firing Angle (pu)', 'color': 'w', 'index': 14},
                        {'name': 'Rotor Sector', 'color': '#ff00ff', 'index': 15},
                        {'name': 'Commutation State', 'color': '#00ff00', 'index': 16},
                    ],
                    mapper=lambda sample_idx, pkt: (
                        sample_idx,
                        float(pkt.modulator.release_pulses_input),
                        float(pkt.modulator.operating_mode),
                        float(pkt.modulator.release_pulses_output),
                        float(pkt.modulator.request_force_commutation),
                        float(pkt.modulator.force_commutation_acknowledged),
                        pkt.modulator.firing_angle_pu,
                        pkt.modulator.current_rotor_sector_normalized,
                        float(pkt.modulator.commutation_state),
                    )
                )
            },
            'torque_controller': {
                'torque_control': PlotConfig(
                    title='Torque Controller',
                    y_label='Reference (pu) / State',
                    signals=[
                        {'name': 'Current Reference', 'color': 'g', 'index': 25},
                        {'name': 'Is Saturated', 'color': 'r', 'index': 26},
                        {'name': 'Is Active', 'color': 'c', 'index': 27},
                        {'name': 'Breakaway Boost Active', 'color': 'm', 'index': 28},
                    ],
                    mapper=lambda sample_idx, pkt: (
                        sample_idx,
                        pkt.torque_controller.current_reference_pu,
                        float(pkt.torque_controller.is_saturated),
                        float(pkt.torque_controller.is_active),
                        float(pkt.torque_controller.is_breakaway_boost_active),
                    )
                )
            },
            'speed_controller': {
                'speed_control': PlotConfig(
                    title='Speed Controller',
                    y_label='Speed (pu) / Torque (pu) / PI Output',
                    signals=[
                        {'name': 'Speed Reference', 'color': 'g', 'index': 29},
                        {'name': 'Measured Speed', 'color': 'r', 'index': 30},
                        {'name': 'Torque Reference', 'color': 'c', 'index': 31},
                        {'name': 'PI Output', 'color': 'm', 'index': 32},
                        {'name': 'Is Saturated', 'color': 'y', 'index': 33},
                    ],
                    mapper=lambda sample_idx, pkt: (
                        sample_idx,
                        pkt.speed_controller.speed_reference_pu,
                        pkt.speed_controller.measured_speed_pu,
                        pkt.speed_controller.torque_reference_pu,
                        pkt.speed_controller.pi_output_pu,
                        float(pkt.speed_controller.is_saturated),
                    )
                )
            },
            'startup_sequencer': {
                'startup_sequencer_outputs': PlotConfig(
                    title='Startup Sequencer',
                    y_label='Speed Ref (pu) / Virtual Torque / Release Flags',
                    signals=[
                        {'name': 'Speed Ref (pu)', 'color': 'g', 'index': 34},
                        {'name': 'Virtual Rotor Freq (pu)', 'color': 'b', 'index': 35},
                        {'name': 'Virtual Rotor Angle (pu)', 'color': 'c', 'index': 36},
                        {'name': 'Virtual Torque Ref (pu)', 'color': 'r', 'index': 37},
                        {'name': 'Release Speed Ctrl', 'color': 'm', 'index': 38},
                        {'name': 'Release Excitation', 'color': 'y', 'index': 39},
                        {'name': 'Release Torque Ctrl', 'color': 'orange', 'index': 40},
                        {'name': 'Release Machine Pulses', 'color': 'purple', 'index': 41},
                        {'name': 'Startup Complete', 'color': 'lime', 'index': 42},
                    ],
                    mapper=lambda sample_idx, pkt: (
                        sample_idx,
                        pkt.startup_sequencer.speed_reference_pu,
                        pkt.startup_sequencer.virtual_rotor_frequency_pu,
                        pkt.startup_sequencer.virtual_rotor_angle_pu,
                        pkt.startup_sequencer.virtual_torque_reference_pu,
                        float(pkt.startup_sequencer.release_speed_controller),
                        float(pkt.startup_sequencer.release_excitation),
                        float(pkt.startup_sequencer.release_torque_controller),
                        float(pkt.startup_sequencer.release_machine_side_pulses),
                        float(pkt.startup_sequencer.is_startup_complete),
                    )
                )
            },
            'excitation': {
                'excitation_control': PlotConfig(
                    title='Excitation Controller',
                    y_label='Field Current (pu) / Time (s)',
                    signals=[
                        {'name': 'Field Current Reference', 'color': 'g', 'index': 43},
                        {'name': 'Prefluxing Elapsed Time', 'color': 'b', 'index': 44},
                    ],
                    mapper=lambda sample_idx, pkt: (
                        sample_idx,
                        pkt.excitation_controller.field_current_reference_pu,
                        pkt.excitation_controller.prefluxing_elapsed_time_s,
                    )
                )
            },
        }
    
    @staticmethod
    def extract_plot_definitions_only() -> Dict[str, Dict[str, Dict]]:
        """Extract just the plot definitions (without mappers) for backward compatibility.
        
        Useful for oscilloscope_widget.py which only needs the visual definitions.
        """
        plots = TelemetryPlotMapper.get_machine_side_plots()
        return {
            group_name: {
                plot_name: {
                    'title': config.title,
                    'y_label': config.y_label,
                    'signals': config.signals,
                }
                for plot_name, config in plots_dict.items()
            }
            for group_name, plots_dict in plots.items()
        }
    
    @staticmethod
    def get_default_visible_groups(target: str) -> list:
        """Get default visible plot groups for a given target."""
        if target == 'grid':
            return ['measurements', 'observer', 'current_controller']
        if target == 'machine':
            return ['startup_sequencer', 'torque_controller', 'speed_controller']
        return []
