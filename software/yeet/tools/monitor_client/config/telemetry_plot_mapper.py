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
                        {'name': 'V_A (Unmasked)', 'color': 'y', 'index': 3},
                        {'name': 'V_B (Unmasked)', 'color': 'w', 'index': 4},
                        {'name': 'V_C (Unmasked)', 'color': '#ff00ff', 'index': 5},
                        {'name': 'V_A (Masked)', 'color': 'r', 'index': 0},
                        {'name': 'V_B (Masked)', 'color': 'c', 'index': 1},
                        {'name': 'V_C (Masked)', 'color': 'm', 'index': 2},
                        {'name': 'V Magnitude', 'color': 'g', 'index': 6},
                    ],
                    mapper=lambda sample_idx, pkt: (
                        sample_idx,
                        pkt.measurements.machine_voltage_abc_a_pu,
                        pkt.measurements.machine_voltage_abc_b_pu,
                        pkt.measurements.machine_voltage_abc_c_pu,
                        pkt.measurements.machine_voltage_unmasked_a_pu,
                        pkt.measurements.machine_voltage_unmasked_b_pu,
                        pkt.measurements.machine_voltage_unmasked_c_pu,
                        pkt.measurements.machine_voltage_magnitude_pu,
                    )
                )
            },
            'observer': {
                'observer_estimates': PlotConfig(
                    title='Observer Estimates',
                    y_label='Frequency (pu) / Angle (pu)',
                    signals=[
                        {'name': 'Frequency (pu)', 'color': 'g', 'index': 9},
                        {'name': 'Position (pu)', 'color': 'r', 'index': 10},
                    ],
                    mapper=lambda sample_idx, pkt: (
                        sample_idx,
                        pkt.observer.estimated_frequency_pu,
                        pkt.observer.estimated_position_pu,
                    )
                )
            },
            'modulator': {
                'modulator_control': PlotConfig(
                    title='Modulator Control',
                    y_label='Angle (rad) / State',
                    signals=[
                        {'name': 'Firing Angle (rad)', 'color': 'g', 'index': 13},
                        {'name': 'Enabled', 'color': 'r', 'index': 14},
                        {'name': 'Release Pulses', 'color': 'c', 'index': 15},
                        {'name': 'Force Commutation', 'color': 'm', 'index': 16},
                        {'name': 'Rotor Sector', 'color': 'y', 'index': 17},
                    ],
                    mapper=lambda sample_idx, pkt: (
                        sample_idx,
                        pkt.modulator.firing_angle_rad,
                        float(pkt.modulator.is_enabled),
                        float(pkt.modulator.release_pulses),
                        float(pkt.modulator.request_force_commutation),
                        pkt.modulator.current_rotor_sector,
                    )
                )
            },
            'torque_controller': {
                'torque_control': PlotConfig(
                    title='Torque Controller',
                    y_label='Reference (pu) / Limit (pu) / State',
                    signals=[
                        {'name': 'Torque Reference (from Speed)', 'color': 'g', 'index': 21},
                        {'name': 'Current Controller Saturated', 'color': 'y', 'index': 25},
                        {'name': 'Current Reference', 'color': 'g', 'index': 26},
                        {'name': 'Limited Torque Ref', 'color': 'm', 'index': 29},
                        {'name': 'Is Saturated', 'color': 'y', 'index': 30},
                    ],
                    mapper=lambda sample_idx, pkt: (
                        sample_idx,
                        pkt.torque_controller.base_torque_reference_pu,
                        float(pkt.torque_controller.is_saturated),
                        pkt.torque_controller.current_reference_pu,
                        pkt.torque_controller.limited_torque_reference_pu,
                        float(pkt.torque_controller.is_saturated),
                    )
                )
            },
            'speed_controller': {
                'speed_control': PlotConfig(
                    title='Speed Controller',
                    y_label='Speed Ref (pu) / Measured Speed (pu) / Torque Ref (pu) / Error / PI Output',
                    signals=[
                        {'name': 'Speed Reference', 'color': 'g', 'index': 34},
                        {'name': 'Measured Speed', 'color': 'r', 'index': 35},
                        {'name': 'Speed Ref Limited', 'color': 'b', 'index': 36},
                        {'name': 'Torque Reference', 'color': 'c', 'index': 37},
                        {'name': 'Speed Error', 'color': 'm', 'index': 38},
                        {'name': 'PI Output', 'color': 'y', 'index': 39},
                        {'name': 'Is Active', 'color': 'w', 'index': 40},
                        {'name': 'Is Saturated', 'color': '#ff00ff', 'index': 41},
                    ],
                    mapper=lambda sample_idx, pkt: (
                        sample_idx,
                        pkt.speed_controller.speed_reference_pu,
                        pkt.speed_controller.measured_speed_pu,
                        pkt.speed_controller.speed_reference_limited_pu,
                        pkt.speed_controller.torque_reference_pu,
                        pkt.speed_controller.speed_error_pu,
                        pkt.speed_controller.pi_output_pu,
                        float(pkt.speed_controller.is_active),
                        float(pkt.speed_controller.is_saturated),
                    )
                )
            },
            'speed_generator': {
                'speed_gen_outputs': PlotConfig(
                    title='Speed Generator',
                    y_label='Speed Ref (pu) / Torque Limit (pu) / State',
                    signals=[
                        {'name': 'Speed Ref (pu)', 'color': 'g', 'index': 42},
                        {'name': 'Torque Limit Max', 'color': 'r', 'index': 43},
                        {'name': 'Torque Limit Min', 'color': 'c', 'index': 44},
                        {'name': 'Profile Complete', 'color': 'm', 'index': 45},
                        {'name': 'Breakaway Boost Active', 'color': 'y', 'index': 46},
                    ],
                    mapper=lambda sample_idx, pkt: (
                        sample_idx,
                        pkt.speed_generator.speed_ref_pu,
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
                        {'name': 'Excitation V Setpoint', 'color': 'g', 'index': 47},
                        {'name': 'Enable Excitation', 'color': 'r', 'index': 48},
                        {'name': 'Field Established', 'color': 'c', 'index': 49},
                        {'name': 'Is Running', 'color': 'm', 'index': 50},
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
