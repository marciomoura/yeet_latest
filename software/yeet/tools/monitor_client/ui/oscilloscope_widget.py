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

from monitor_client.config import get_grid_side_plots, get_machine_side_plots, get_default_visible_groups


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
        main_layout = QtWidgets.QVBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)
        
        # Create toolbar for view toggles
        self.view_button_layout = QtWidgets.QHBoxLayout()
        self.view_button_layout.setContentsMargins(5, 5, 5, 5)
        self.view_button_layout.setSpacing(5)
        self.view_buttons: Dict[str, QtWidgets.QPushButton] = {}
        
        main_layout.addLayout(self.view_button_layout)
        
        self.plots_container = pg.GraphicsLayoutWidget()
        self.plots_container.setBackground('#1e1e1e')
        main_layout.addWidget(self.plots_container)
        
        self.setLayout(main_layout)
    
    def create_plots_from_config(self, target: str = 'grid'):
        """Create plots from configuration for the specified target.
        
        Args:
            target: Either 'grid' or 'machine' to load the appropriate plot definitions
        
        Loads plot definitions from configuration module based on target.
        This is the single entry point for creating all plots from plot_definitions.py.
        """
        # Get plot definitions from config based on target
        if target == 'machine':
            plot_definitions = get_machine_side_plots()
        else:
            plot_definitions = get_grid_side_plots()
        
        default_visible = get_default_visible_groups(target)
        
        # Create plots from definitions
        self.visible_views_order = []
        for group_name, group_plots in plot_definitions.items():
            self.plot_groups[group_name] = {}
            self.view_configs[group_name] = {'visible': group_name in default_visible}
            
            if self.view_configs[group_name]['visible']:
                self.visible_views_order.append(group_name)
            
            for plot_name, plot_config in group_plots.items():
                self._create_plot(group_name, plot_name, plot_config)
        
        # Create view toggle buttons for each defined group
        self._create_view_buttons()
        
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
        self.buffers[buffer_key] = deque(maxlen=max(self.window_size * 2, 1000))
        
        # Store in layout map (will be populated by _reorganize_subplot_layout)
        self._plot_layout_map[len(self._plot_layout_map)] = buffer_key
    
    def _create_view_buttons(self):
        """Create view toggle buttons for each defined plot group.
        
        Only creates buttons for groups that have signal definitions (i.e., are in plot_definitions).
        Buttons appear in the toolbar at the top of the oscilloscope widget.
        """
        # Add label first
        label = QtWidgets.QLabel("Oscilloscope Views:")
        label.setStyleSheet("color: #ffffff; font-size: 10pt; margin-right: 10px;")
        self.view_button_layout.addWidget(label)
        
        # Create a button for each view group (in order of plot_definitions)
        for group_name in self.plot_groups.keys():
            is_visible = self.view_configs[group_name].get('visible', False)
            btn_text = f"{'Hide' if is_visible else 'Show'} {group_name.replace('_', ' ').title()}"
            
            btn = QtWidgets.QPushButton(btn_text)
            btn.setMaximumWidth(150)
            btn.setCheckable(True)
            btn.setChecked(is_visible)
            btn.setToolTip(f"Toggle visibility of {group_name} oscilloscope view")
            btn.clicked.connect(lambda checked, g=group_name: self._on_view_button_clicked(g))
            
            self.view_buttons[group_name] = btn
            self.view_button_layout.addWidget(btn)
        
        # Add separator
        separator = QtWidgets.QLabel("|")
        separator.setStyleSheet("color: #666666;")
        self.view_button_layout.addWidget(separator)
        
        # Add "Hide All" button
        hide_all_btn = QtWidgets.QPushButton("Hide All")
        hide_all_btn.setMaximumWidth(80)
        hide_all_btn.setToolTip("Hide all oscilloscope views")
        hide_all_btn.clicked.connect(self._on_hide_all_clicked)
        self.view_button_layout.addWidget(hide_all_btn)
        
        # Add stretch to push buttons to the left
        self.view_button_layout.addStretch()
    
    def _on_view_button_clicked(self, group_name: str):
        """Handle view toggle button click.
        
        Args:
            group_name: Name of the view group to toggle
        """
        if group_name in self.view_configs:
            current_visible = self.view_configs[group_name].get('visible', False)
            new_visible = not current_visible
            self.toggle_view_group(group_name, new_visible)
            
            # Update button text and state
            btn = self.view_buttons.get(group_name)
            if btn:
                btn.setText(f"{'Hide' if new_visible else 'Show'} {group_name.replace('_', ' ').title()}")
                btn.setChecked(new_visible)
    
    def _on_hide_all_clicked(self):
        """Handle Hide All button click - hide all view groups."""
        for group_name in self.plot_groups.keys():
            self.toggle_view_group(group_name, False)
            
            # Update button text and state
            btn = self.view_buttons.get(group_name)
            if btn:
                btn.setText(f"Show {group_name.replace('_', ' ').title()}")
                btn.setChecked(False)
    
    def add_sample_to_buffer(self, buffer_name: str, sample: Tuple):
        """Add a data sample to the named buffer.
        
        Args:
            buffer_name: Buffer identifier (e.g., 'measurements_voltage')
            sample: Tuple of (sample_index, value1, value2, ...)
        """
        if buffer_name not in self.buffers:
            self.buffers[buffer_name] = deque(maxlen=50000)
        
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
                        try:
                            # Check if buffer samples have enough elements
                            required_idx = curve_idx + 1
                            
                            # Filter out samples that don't have enough data
                            valid_samples = [p for p in buffer if len(p) > required_idx]
                            
                            if not valid_samples:
                                continue
                            
                            y_data = np.array([p[required_idx] for p in valid_samples])
                            
                            # Use only the x_data that corresponds to valid samples
                            if len(valid_samples) < len(buffer):
                                # Some samples were filtered, need matching x_data
                                valid_x_data = np.array([p[0] for p in valid_samples])
                                curve.setData(valid_x_data, y_data)
                            else:
                                curve.setData(x_data, y_data)
                        
                        except (IndexError, ValueError) as e:
                            logger.debug(f"Curve {curve_idx} in {buffer_key}: {e}")
                            continue
                    
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
        """Update visible X-axis window size and resize buffers accordingly.
        
        Args:
            samples: Number of samples to display on X-axis
        """
        self.window_size = max(1, samples)
        
        # Resize all buffers to accommodate the new window size
        # Keep at least 2x window_size or 1000, whichever is larger
        new_maxlen = max(self.window_size * 2, 1000)
        
        for buffer_name, buffer in self.buffers.items():
            # Create new buffer with updated maxlen
            new_buffer = deque(buffer, maxlen=new_maxlen)
            self.buffers[buffer_name] = new_buffer
    
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
                # Try to remove items individually first (safer than clear())
                items_to_remove = []
                for group_plots in self.plot_groups.values():
                    for plot_item in group_plots.values():
                        items_to_remove.append(plot_item)
                
                for plot_item in items_to_remove:
                    try:
                        self.plots_container.removeItem(plot_item)
                    except (ValueError, RuntimeError, KeyError, AttributeError):
                        # Item may not be in layout, continue
                        pass
                
                # Now try clear() to reset internal state
                try:
                    self.plots_container.clear()
                except (ValueError, RuntimeError, KeyError, AttributeError) as e:
                    logger.debug(f"plots_container.clear() failed after removeItem: {e}")
                    
            except Exception as e:
                logger.warning(f"Layout clearing failed: {e}")
            
            self.plots_container.setBackground('#1e1e1e')
            
            # Recreate plots in new layout
            if self.layout_mode == "1xN":
                # Vertical stack: one plot per row
                row = 0
                for group_name in self.visible_views_order:
                    if group_name in self.plot_groups:
                        for plot_name, plot_item in self.plot_groups[group_name].items():
                            self.plots_container.addItem(plot_item, row=row, col=0)
                            row += 1
            
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
