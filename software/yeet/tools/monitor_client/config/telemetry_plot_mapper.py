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
            'speed_generator': {
                'speed_gen_outputs': PlotConfig(
                    title='Speed Generator',
                    y_label='Speed Ref (pu) / Torque Limit (pu) / State',
                    signals=[
                        {'name': 'Speed Ref (pu)', 'color': 'g', 'index': 34},
                        {'name': 'Torque Limit Max', 'color': 'r', 'index': 35},
                        {'name': 'Torque Limit Min', 'color': 'c', 'index': 36},
                        {'name': 'Profile Complete', 'color': 'm', 'index': 37},
                        {'name': 'Breakaway Boost Active', 'color': 'y', 'index': 38},
                    ],
                    mapper=lambda sample_idx, pkt: (
                        sample_idx,
                        pkt.speed_generator.speed_reference_pu,
                        pkt.speed_generator.torque_limit_max_pu,
                        pkt.speed_generator.torque_limit_min_pu,
                        float(pkt.speed_generator.is_profile_complete),
                        float(pkt.speed_generator.is_breakaway_boost_active),
                    )
                )
            },
            'excitation': {
                'excitation_control': PlotConfig(
                    title='Excitation Controller',
                    y_label='Voltage (pu) / State',
                    signals=[
                        {'name': 'Excitation V Setpoint', 'color': 'g', 'index': 39},
                        {'name': 'Enable Excitation', 'color': 'r', 'index': 40},
                        {'name': 'Field Established', 'color': 'c', 'index': 41},
                        {'name': 'Is Running', 'color': 'm', 'index': 42},
                    ],
                    mapper=lambda sample_idx, pkt: (
                        sample_idx,
                        pkt.excitation_controller.voltage_setpoint_pu,
                        float(pkt.excitation_controller.enable),
                        float(pkt.excitation_controller.is_field_established),
                        float(pkt.excitation_controller.is_running),
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
            return ['measurements', 'observer', 'torque_controller', 'speed_controller']
        return []
