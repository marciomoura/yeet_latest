#!/usr/bin/env python3
"""Automatic telemetry buffer mapping utility.

This module provides automatic mapping from telemetry signal arrays to plot buffers
based on plot_definitions.py. This eliminates manual buffer packing in main_window.py.

Architecture:
- Reads plot definitions to understand which signals belong to which plots
- Provides a mapper that extracts the correct values from telemetry signal arrays
- Automatically creates buffer tuples with (sample_index, signal1, signal2, ...)
"""

from typing import Dict, List, Tuple, Any
import logging

from monitor_client.config import get_grid_side_plots, get_machine_side_plots


logger = logging.getLogger(__name__)


class TelemetryBufferMapper:
    """Maps telemetry signal arrays to plot buffers automatically.
    
    This class reads plot_definitions.py and creates mappings that extract
    the correct signals from telemetry arrays and pack them into buffer tuples.
    """
    
    def __init__(self, target: str = 'grid'):
        """Initialize mapper for the specified target.
        
        Args:
            target: Either 'grid' or 'machine'
        """
        self.target = target
        self.mappings: Dict[str, List[int]] = {}
        self._build_mappings()
    
    def _build_mappings(self):
        """Build signal index mappings from plot definitions.
        
        Creates a dictionary mapping buffer names to lists of signal indices.
        For example: 'pulse_generator_firing_pulses' -> [50, 51, 52, 53, 54, 55]
        """
        # Get plot definitions for target
        if self.target == 'machine':
            plot_definitions = get_machine_side_plots()
        else:
            plot_definitions = get_grid_side_plots()
        
        # Build mappings
        for group_name, group_plots in plot_definitions.items():
            for plot_name, plot_config in group_plots.items():
                buffer_key = f"{group_name}_{plot_name}"
                
                # Extract signal indices in order
                signal_indices = [signal['index'] for signal in plot_config['signals']]
                
                self.mappings[buffer_key] = signal_indices
                
                logger.debug(f"Mapped buffer '{buffer_key}' to indices {signal_indices}")
    
    def map_signals_to_buffer(self, buffer_name: str, sample_index: int, 
                             signal_array: List[float]) -> Tuple:
        """Map signals from telemetry array to a buffer tuple.
        
        Args:
            buffer_name: Name of the buffer (e.g., 'pulse_generator_firing_pulses')
            sample_index: Current sample index for X-axis
            signal_array: Complete telemetry signal array
            
        Returns:
            Tuple of (sample_index, signal1, signal2, ...) ready for buffer
            
        Example:
            >>> mapper = TelemetryBufferMapper('grid')
            >>> signals = [0.0] * 100
            >>> signals[50] = 1.0  # firing_pulse_1
            >>> signals[51] = 0.0  # firing_pulse_2
            >>> tuple_data = mapper.map_signals_to_buffer(
            ...     'pulse_generator_firing_pulses', 1000, signals)
            >>> # Returns: (1000, 1.0, 0.0, ...)
        """
        if buffer_name not in self.mappings:
            logger.warning(f"Buffer '{buffer_name}' not found in mappings")
            return (sample_index,)
        
        indices = self.mappings[buffer_name]
        
        # Extract values from signal array
        values = []
        for idx in indices:
            if idx < len(signal_array):
                values.append(signal_array[idx])
            else:
                logger.warning(f"Signal index {idx} out of range for array length {len(signal_array)}")
                values.append(0.0)
        
        # Return tuple with sample_index first, then all signal values
        return (sample_index, *values)
    
    def get_all_buffer_names(self) -> List[str]:
        """Get list of all buffer names for this target.
        
        Returns:
            List of buffer name strings
        """
        return list(self.mappings.keys())
    
    def get_signal_indices(self, buffer_name: str) -> List[int]:
        """Get the signal indices for a specific buffer.
        
        Args:
            buffer_name: Name of the buffer
            
        Returns:
            List of signal indices, or empty list if buffer not found
        """
        return self.mappings.get(buffer_name, [])
