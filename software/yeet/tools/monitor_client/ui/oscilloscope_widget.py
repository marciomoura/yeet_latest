#!/usr/bin/env python3
"""Dedicated oscilloscope plot widget for telemetry visualization.

This widget encapsulates all plot management, data buffering, and rendering logic.
It is independent from networking, parameters, and data persistence.

Architecture:
- PyQtGraph-based plotting with dark theme
- Data buffers (deques) for each plot
- Plot groups for organizing related plots
- View toggling for show/hide of plot groups
- Support for 1xN (vertical) and 2x2 (grid) layouts
"""

import logging
from collections import deque
from typing import Dict, Optional, List, Tuple

import numpy as np
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg


logger = logging.getLogger(__name__)


class OscilloscopeWidget(QtWidgets.QWidget):
    """Self-contained oscilloscope widget for real-time data visualization.
    
    Manages:
    - PyQtGraph plot creation and configuration
    - Data buffers (deques) for each plot
    - Plot updates and rendering
    - View group visibility toggling
    - Layout switching (1xN vs 2x2)
    
    External code provides data via add_sample_to_buffer(), and calls update_plots()
    to render the latest data.
    """
    
    def __init__(self, parent=None, window_size: int = 2, layout_mode: str = "1xN"):
        """Initialize oscilloscope widget.
        
        Args:
            parent: Parent widget
            window_size: Number of samples to display on X-axis
            layout_mode: '1xN' for vertical stack or '2x2' for grid
        """
        super().__init__(parent)
        
        self.window_size = window_size
        self.layout_mode = layout_mode
        
        # Data buffers - one per plot group/plot pair
        # Key: "group_name_plot_name", Value: deque of tuples
        self.buffers: Dict[str, deque] = {}
        
        # Plot structure
        self.plots_container: Optional[pg.GraphicsLayoutWidget] = None
        
        # Store plot items by group
        # Structure: {group_name: {plot_name: plot_item}}
        self.plot_groups: Dict[str, Dict[str, pg.PlotItem]] = {}
        
        # Store curve items by plot
        # Structure: {group_name_plot_name: [curve1, curve2, ...]}
        self.plot_curves: Dict[str, List[pg.PlotCurveItem]] = {}
        
        # View visibility configuration
        # Structure: {group_name: {'visible': bool, ...}}
        self.view_configs: Dict[str, Dict] = {}
        
        # Order of visible views (for layout organization)
        self.visible_views_order: List[str] = []
        
        # Layout tracking for reorganization
        self._plot_layout_map: Dict[Tuple[int, int], str] = {}  # (row, col) -> plot_key
        
        # Initialize UI
        self._init_ui()
    
    def _init_ui(self):
        """Initialize widget layout and create plots container."""
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        self.plots_container = pg.GraphicsLayoutWidget()
        self.plots_container.setBackground('#1e1e1e')
        layout.addWidget(self.plots_container)
        
        self.setLayout(layout)
    
    def create_standard_plots(self):
        """Create the standard plot set for telemetry monitoring.
        
        Plots:
        - Measurements: Voltage (magnitude), Current (phase A/B/C)
        - Observer: Estimated voltage and frequency
        - DC Link: Reference and measured currents
        - Current Measurements: Detailed ABC + DC-link
        - Observer Voltage Analysis: Measured vs observed voltages
        - Observer Frequency & Angle: Frequency and phase angle
        """
        
        # Define plot structure: group -> plot_name -> config
        plot_definitions = {
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
            'current_measurements': {
                'abc_dclink': {
                    'title': 'Current Measurements (Grid ABC + DC-Link)',
                    'y_label': 'Current (pu)',
                    'signals': [
                        {'name': 'Grid A', 'color': 'r', 'index': 1},
                        {'name': 'Grid B', 'color': 'g', 'index': 2},
                        {'name': 'Grid C', 'color': 'b', 'index': 3},
                        {'name': 'DC Link', 'color': 'w', 'index': 4}
                    ]
                }
            },
            'observer_voltage_analysis': {
                'voltages': {
                    'title': 'Voltage Analysis (Measured vs Observed)',
                    'y_label': 'Voltage (pu)',
                    'signals': [
                        {'name': 'Grid A', 'color': 'r', 'index': 1},
                        {'name': 'Grid B', 'color': 'g', 'index': 2},
                        {'name': 'Grid C', 'color': 'b', 'index': 3},
                        {'name': 'Est. Voltage', 'color': 'c', 'index': 4}
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
            },
            'operation_mode_monitor': {
                'controller_status': {
                    'title': 'Current Controller',
                    'y_label': 'Status',
                    'signals': [
                        {'name': 'Error', 'color': 'r', 'index': 1},
                        {'name': 'Output', 'color': 'g', 'index': 2},
                        {'name': 'Saturated', 'color': 'y', 'index': 3}
                    ]
                }
            },
            'state_machine': {
                'state_monitoring': {
                    'title': 'State Machine & Pulses',
                    'y_label': 'State/Flag',
                    'signals': [
                        {'name': 'Current State', 'color': 'c', 'index': 1},
                        {'name': 'Pulse Releases', 'color': 'm', 'index': 2}
                    ]
                }
            }
        }
        
        # Create plots from definitions
        self.visible_views_order = []
        for group_name, group_plots in plot_definitions.items():
            self.plot_groups[group_name] = {}
            self.view_configs[group_name] = {'visible': group_name in ['measurements', 'observer', 'current_controller']}
            
            if self.view_configs[group_name]['visible']:
                self.visible_views_order.append(group_name)
            
            for plot_name, plot_config in group_plots.items():
                self._create_plot(group_name, plot_name, plot_config)
        
        # Initial layout organization
        self._reorganize_subplot_layout()
    
    def _create_plot(self, group_name: str, plot_name: str, config: Dict):
        """Create a single plot with its curves.
        
        Args:
            group_name: Plot group (e.g., 'measurements', 'observer')
            plot_name: Specific plot name (e.g., 'voltage', 'current')
            config: Dictionary with 'title', 'y_label', and 'signals' list
        """
        # Create plot item - add to a temporary container first, then add to main container in layout
        plot = pg.PlotItem(title=config['title'])
        plot.setLabel('left', config['y_label'], color='#ffffff')
        plot.setLabel('bottom', 'Sample Index', color='#ffffff')
        plot.showGrid(x=True, y=True, alpha=0.3)
        
        # Add curves for each signal
        curves = []
        for signal in config['signals']:
            curve = plot.plot(
                pen=pg.mkPen(color=signal['color'], width=2),
                name=signal['name']
            )
            curves.append(curve)
        
        # Add legend AFTER all curves have been added
        # The legend must be populated manually with each named curve
        legend = plot.addLegend(offset=(15, 15), spacing=5, frameon=True)
        
        # Manually add each curve to the legend with its name
        for curve in curves:
            if curve.name():
                legend.addItem(curve, curve.name())
        
        # Style the legend
        if legend is not None:
            legend.setBrush(pg.mkBrush(50, 50, 50, 240))  # Dark background with high opacity
            legend.setPen(pg.mkPen(color=(220, 220, 220), width=2))  # Bright border
        
        # Store references
        self.plot_groups[group_name][plot_name] = plot
        buffer_key = f"{group_name}_{plot_name}"
        self.plot_curves[buffer_key] = curves
        
        # Create buffer
        self.buffers[buffer_key] = deque(maxlen=10000)
        
        # Store in layout map (will be populated by _reorganize_subplot_layout)
        self._plot_layout_map[len(self._plot_layout_map)] = buffer_key
    
    def add_sample_to_buffer(self, buffer_name: str, sample: Tuple):
        """Add a data sample to the named buffer.
        
        Args:
            buffer_name: Buffer identifier (e.g., 'measurements_voltage')
            sample: Tuple of (sample_index, value1, value2, ...)
        """
        if buffer_name not in self.buffers:
            self.buffers[buffer_name] = deque(maxlen=10000)
        
        self.buffers[buffer_name].append(sample)
    
    def get_buffer(self, buffer_name: str) -> Optional[deque]:
        """Get direct reference to a buffer for external manipulation.
        
        Args:
            buffer_name: Buffer identifier
            
        Returns:
            Reference to deque, or None if buffer doesn't exist
        """
        return self.buffers.get(buffer_name)
    
    def clear_buffers(self):
        """Clear all data buffers."""
        for buffer in self.buffers.values():
            buffer.clear()
    
    def update_plots(self):
        """Update all visible plots with current buffer data.
        
        Extracts X and Y data from each visible plot's buffer and renders
        the curves. Also updates X-axis range based on window_size.
        """
        try:
            for group_name, plots in self.plot_groups.items():
                # Skip if group not visible
                if group_name not in self.view_configs or not self.view_configs[group_name].get('visible', True):
                    continue
                
                for plot_name, plot_item in plots.items():
                    buffer_key = f"{group_name}_{plot_name}"
                    buffer = self.buffers.get(buffer_key)
                    
                    if not buffer or len(buffer) == 0:
                        continue
                    
                    # Extract X data (sample indices from first column)
                    x_data = np.array([p[0] for p in buffer])
                    
                    # Update each curve
                    curves = self.plot_curves.get(buffer_key, [])
                    for curve_idx, curve in enumerate(curves):
                        # Data index = curve_idx + 1 (since index 0 is X data)
                        y_data = np.array([p[curve_idx + 1] for p in buffer])
                        curve.setData(x_data, y_data)
                    
                    # Update X-axis range to show sliding window
                    if len(x_data) > 0:
                        latest = x_data[-1]
                        plot_item.setXRange(
                            max(0, latest - self.window_size),
                            latest,
                            padding=0
                        )
        
        except Exception as e:
            logger.error(f"Plot update error: {e}", exc_info=True)
    
    def toggle_view_group(self, group_name: str, visible: bool):
        """Toggle visibility of a plot group.
        
        Args:
            group_name: Name of the group (e.g., 'measurements', 'observer')
            visible: Whether to show the group
        """
        if group_name in self.view_configs:
            self.view_configs[group_name]['visible'] = visible
            
            # Update visible order
            if visible and group_name not in self.visible_views_order:
                self.visible_views_order.append(group_name)
            elif not visible and group_name in self.visible_views_order:
                self.visible_views_order.remove(group_name)
            
            # Reorganize layout
            self._reorganize_subplot_layout()
    
    def set_window_size(self, samples: int):
        """Update visible X-axis window size.
        
        Args:
            samples: Number of samples to display on X-axis
        """
        self.window_size = max(1, samples)
    
    def set_layout_mode(self, mode: str):
        """Switch layout mode and reorganize plots.
        
        Args:
            mode: '1xN' for vertical stack or '2x2' for grid layout
        """
        if mode in ['1xN', '2x2']:
            self.layout_mode = mode
            self._reorganize_subplot_layout()
    
    def _reorganize_subplot_layout(self):
        """Reorganize subplot positions based on visibility and layout mode.
        
        Clears the graphics layout and recreates plots in the configured mode.
        This is called when view groups are toggled or layout mode changes.
        """
        try:
            # Save current plot references
            all_plots = {}
            for group_name in self.visible_views_order:
                if group_name in self.plot_groups:
                    for plot_name, plot_item in self.plot_groups[group_name].items():
                        all_plots[f"{group_name}_{plot_name}"] = (plot_item, group_name, plot_name)
            
            # Clear the layout safely - handle potential tracking issues in pyqtgraph
            try:
                self.plots_container.clear()
            except (ValueError, RuntimeError, AttributeError) as e:
                # If clear() fails, try removing items individually
                logger.debug(f"Layout clear() failed, attempting manual removal: {e}")
                try:
                    # Get list of items currently in layout (GraphicsLayout stores items internally)
                    # We'll iterate through plot groups and try to remove each plot
                    for group_plots in self.plot_groups.values():
                        for plot_item in group_plots.values():
                            try:
                                self.plots_container.removeItem(plot_item)
                            except (ValueError, RuntimeError):
                                # Item may not be in layout, continue
                                pass
                except Exception as e2:
                    logger.warning(f"Manual layout clearing also failed: {e2}")
            
            self.plots_container.setBackground('#1e1e1e')
            
            # Recreate plots in new layout
            if self.layout_mode == "1xN":
                # Vertical stack: one plot per row
                for row, group_name in enumerate(self.visible_views_order):
                    if group_name in self.plot_groups:
                        for plot_name, plot_item in self.plot_groups[group_name].items():
                            self.plots_container.addItem(plot_item, row=row, col=0)
            
            elif self.layout_mode == "2x2":
                # 2x2 grid layout
                plot_items = []
                for group_name in self.visible_views_order:
                    if group_name in self.plot_groups:
                        for plot_name, plot_item in self.plot_groups[group_name].items():
                            plot_items.append(plot_item)
                
                for idx, plot_item in enumerate(plot_items):
                    row = idx // 2
                    col = idx % 2
                    self.plots_container.addItem(plot_item, row=row, col=col)
            
            # Link X-axes for synchronized zooming
            self._link_xaxes()
        
        except Exception as e:
            logger.error(f"Layout reorganization error: {e}", exc_info=True)
    
    def _link_xaxes(self):
        """Link X-axes of all plots for synchronized panning/zooming.
        
        When one plot is panned or zoomed, all others follow.
        """
        try:
            first_plot = None
            for group_name, plots in self.plot_groups.items():
                if group_name in self.view_configs and not self.view_configs[group_name].get('visible', True):
                    continue
                
                for plot_name, plot_item in plots.items():
                    if first_plot is None:
                        first_plot = plot_item
                    else:
                        plot_item.setXLink(first_plot)
        
        except Exception as e:
            logger.error(f"X-axis linking error: {e}", exc_info=True)
    
    def configure_wheel_handler(self, callback):
        """Configure custom wheel event handler for all plot viewboxes.
        
        This allows customizing scroll behavior (e.g., adjusting window size
        instead of zooming).
        
        Args:
            callback: Callable that takes (viewbox, wheel_event) and handles it
        """
        try:
            for group_name, plots in self.plot_groups.items():
                for plot_name, plot_item in plots.items():
                    vb = plot_item.getViewBox()
                    if vb and callback:
                        vb.wheelEvent = lambda event, cb=callback, v=vb: cb(v, event)
        
        except Exception as e:
            logger.error(f"Wheel handler configuration error: {e}", exc_info=True)
    
    def disable_mouse_interactions(self):
        """Disable mouse panning and zooming on all plots.
        
        Useful when custom interactions are preferred (e.g., scroll to adjust
        window size rather than zoom).
        """
        try:
            for group_name, plots in self.plot_groups.items():
                for plot_name, plot_item in plots.items():
                    plot_item.setMouseEnabled(x=False, y=False)
        
        except Exception as e:
            logger.error(f"Mouse interaction configuration error: {e}", exc_info=True)
    
    def get_plot(self, group_name: str, plot_name: str) -> Optional[pg.PlotItem]:
        """Get reference to a specific plot item.
        
        Args:
            group_name: Plot group name
            plot_name: Specific plot name
            
        Returns:
            PlotItem reference or None
        """
        if group_name in self.plot_groups:
            return self.plot_groups[group_name].get(plot_name)
        return None
    
    def get_curve(self, buffer_name: str, curve_index: int = 0) -> Optional[pg.PlotCurveItem]:
        """Get reference to a specific curve within a plot.
        
        Args:
            buffer_name: Buffer identifier (e.g., 'measurements_voltage')
            curve_index: Index of the curve within that plot (0 for first curve)
            
        Returns:
            PlotCurveItem reference or None
        """
        curves = self.plot_curves.get(buffer_name)
        if curves and 0 <= curve_index < len(curves):
            return curves[curve_index]
        return None
