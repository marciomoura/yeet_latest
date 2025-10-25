#!/usr/bin/env python3
"""Combined telemetry monitor with dashboard, oscilloscope, and parameter configuration.

This application provides a comprehensive monitoring and configuration interface combining:
    - Digital dashboard (top left): Real-time signal values with state display
    - Oscilloscope (top right): Continuous real-time waveform plots
    - Parameter panel (bottom): Live parameter list with editing capabilities

Features:
    - Three synchronized oscilloscope subplots
    - Digital displays for all signals organized by functional group
    - Parameter discovery and real-time editing
    - Parameter search and component filtering
    - Auto-scrolling time window with zoom/pan capabilities
    - Connection management and telemetry control

Dependencies:
    pip install PyQt5 pyqtgraph numpy

Usage:
    python telemetry_monitor_with_parameters.py --host 192.168.2.100 --port 7000 --window 5.0

Architecture:
    - Top left panel: Signal dashboard with current values
    - Top right panel: Real-time oscilloscope plots
    - Bottom panel: Parameter configuration with live editing
    - Shared network clients for efficient data handling
"""

import argparse
import json
import logging
import sys
import time
from time import perf_counter
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np
import pyqtgraph as pg
from PyQt5 import QtCore, QtGui, QtWidgets

from monitor_client.core.unified_network_client import UnifiedNetworkClient
from monitor_client.core.telemetry_protocol import GridSideTelemetryPacket
from monitor_client.core.parameter_protocol import ParameterMetadata


# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# UI update intervals
UI_UPDATE_INTERVAL_MS = 75      # ~13 Hz oscilloscope refresh (balanced for smooth display)
DASHBOARD_UPDATE_INTERVAL_MS = 200  # 5 Hz dashboard update (less critical than plots)
STATS_UPDATE_INTERVAL_MS = 500  # 2 Hz statistics
PARAM_REFRESH_INTERVAL_MS = 2000  # 0.5 Hz parameter refresh

# Oscilloscope parameters
DEFAULT_TIME_WINDOW_SEC = 5.0   # Default 5 second window
MAX_POINTS_PER_PLOT = 1500      # Hard cap on samples stored per trace


WINDOW_TRIM_MARGIN = 0.10  # keep a small history margin so trims happen in batches


class SampleBasedBuffer:
    """Sample-based ring buffer with no time-based trimming for stability.
    
    Stores fixed number of most recent samples. When full, oldest samples
    are automatically dropped. No assertions, no time-based logic that can
    fail with packet loss.
    """

    def __init__(self, capacity: int = MAX_POINTS_PER_PLOT):
        self._capacity = max(10, int(capacity))
        self._sample_indices = np.zeros(self._capacity, dtype=np.float64)
        self._values = np.zeros(self._capacity, dtype=np.float64)
        self._count = 0
        self._head = 0  # Index of oldest sample in ring
        self._write_idx = 0
        self._cached_indices = np.array([])
        self._cached_values = np.array([])
        self._dirty = True

    def append(self, sample_idx: float, value: float) -> None:
        """Append sample. Always succeeds, never throws."""
        try:
            self._sample_indices[self._write_idx] = sample_idx
            self._values[self._write_idx] = value

            if self._count == self._capacity:
                # Ring buffer full - overwrite oldest
                self._head = (self._head + 1) % self._capacity
            else:
                self._count += 1

            self._write_idx = (self._write_idx + 1) % self._capacity
            self._dirty = True
        except Exception as e:
            logger.error(f"Buffer append failed: {e}")

    def get_data(self) -> tuple:
        """Get all samples as (indices, values). Never throws, returns empty on error."""
        try:
            if self._count == 0:
                return np.array([]), np.array([])
            
            # Use cache if valid
            if not self._dirty and self._cached_indices.size == self._count:
                return self._cached_indices, self._cached_values

            # Rebuild cache
            ring_indices = (self._head + np.arange(self._count)) % self._capacity
            self._cached_indices = self._sample_indices.take(ring_indices)
            self._cached_values = self._values.take(ring_indices)
            self._dirty = False
            return self._cached_indices, self._cached_values
        except Exception as e:
            logger.error(f"Buffer get_data failed: {e}")
            return np.array([]), np.array([])

    def latest_sample_index(self) -> Optional[float]:
        """Get latest sample index, or None if empty."""
        try:
            if self._count == 0:
                return None
            # Last written sample is at (write_idx - 1)
            last_idx = (self._write_idx - 1) % self._capacity
            return float(self._sample_indices[last_idx])
        except:
            return None

    def clear(self) -> None:
        """Clear all data."""
        self._count = 0
        self._head = 0
        self._write_idx = 0
        self._cached_indices = np.array([])
        self._cached_values = np.array([])
        self._dirty = True
    
    def resize(self, new_capacity: int) -> None:
        """Resize buffer capacity. Safe - never throws."""
        try:
            new_capacity = max(10, int(new_capacity))
            if new_capacity == self._capacity:
                return
            
            # Get current data
            inds, vals = self.get_data()
            
            # Keep only what fits in new capacity
            if inds.size > new_capacity:
                inds = inds[-new_capacity:]
                vals = vals[-new_capacity:]
            
            # Allocate new arrays
            self._capacity = new_capacity
            self._sample_indices = np.zeros(self._capacity, dtype=np.float64)
            self._values = np.zeros(self._capacity, dtype=np.float64)
            
            # Copy data
            self._count = min(inds.size, new_capacity)
            self._head = 0
            self._write_idx = self._count % self._capacity
            
            if self._count > 0:
                self._sample_indices[:self._count] = inds
                self._values[:self._count] = vals
            
            self._cached_indices = np.array([])
            self._cached_values = np.array([])
            self._dirty = True
        except Exception as e:
            logger.error(f"Buffer resize failed: {e}")


class SimpleSampleBuffer:
    """Fixed-capacity ring buffer for sample-index based plotting (simple mode).

    Stores (sample_index, value) pairs without time-based trimming. Capacity can
    be resized by creating a new buffer and copying recent data.
    """

    def __init__(self, capacity: int):
        self.capacity = max(10, int(capacity))
        self.indices = np.zeros(self.capacity, dtype=np.float64)
        self.values = np.zeros(self.capacity, dtype=np.float64)
        self.count = 0
        self.head = 0  # oldest
        self.write_idx = 0
        self._cached_indices = np.array([])
        self._cached_values = np.array([])
        self._dirty = True

    def append(self, idx: float, value: float) -> None:
        self.indices[self.write_idx] = idx
        self.values[self.write_idx] = value
        if self.count == self.capacity:
            self.head = (self.head + 1) % self.capacity
        else:
            self.count += 1
        self.write_idx = (self.write_idx + 1) % self.capacity
        self._dirty = True

    def get_data(self):
        if self.count == 0:
            return np.array([]), np.array([])
        if not self._dirty and self._cached_indices.size == self.count:
            return self._cached_indices, self._cached_values
        indices = (self.head + np.arange(self.count)) % self.capacity
        self._cached_indices = self.indices.take(indices)
        self._cached_values = self.values.take(indices)
        self._dirty = False
        return self._cached_indices, self._cached_values

    def resize(self, new_capacity: int) -> None:
        new_capacity = max(10, int(new_capacity))
        if new_capacity == self.capacity:
            return
        # Copy last new_capacity samples
        inds, vals = self.get_data()
        if inds.size > new_capacity:
            inds = inds[-new_capacity:]
            vals = vals[-new_capacity:]
        self.capacity = new_capacity
        self.indices = np.zeros(self.capacity, dtype=np.float64)
        self.values = np.zeros(self.capacity, dtype=np.float64)
        self.count = inds.size
        self.head = 0
        self.write_idx = self.count % self.capacity
        if self.count > 0:
            self.indices[:self.count] = inds
            self.values[:self.count] = vals
        self._cached_indices = np.array([])
        self._cached_values = np.array([])
        self._dirty = True

    def clear(self) -> None:
        self.count = 0
        self.head = 0
        self.write_idx = 0
        self._cached_indices = np.array([])
        self._cached_values = np.array([])
        self._dirty = True


class SignalPlotGroup:
    """Group of related signals displayed on a single plot."""
    
    def __init__(self, name: str, plot_widget: pg.PlotWidget, capacity: int = MAX_POINTS_PER_PLOT):
        self.name = name
        self.plot_widget = plot_widget
        self.curves: Dict[str, pg.PlotDataItem] = {}
        self.buffers: Dict[str, SampleBasedBuffer] = {}
        self.colors = [
            (255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0),
            (255, 0, 255), (0, 255, 255), (255, 128, 0), (128, 0, 255)
        ]
        self.color_index = 0
        self.data_changed = False  # Track if data has changed since last refresh
        self._capacity = capacity
    
    def add_signal(self, signal_name: str, display_name: str) -> None:
        color = self.colors[self.color_index % len(self.colors)]
        self.color_index += 1
        
        # Enable clipToView for better performance - only render visible data
        curve = self.plot_widget.plot(pen=pg.mkPen(color=color, width=2), name=display_name)
        curve.setClipToView(True)  # Major performance improvement
        curve.setDownsampling(auto=True, method='peak')  # Downsample when zoomed out
        self.curves[signal_name] = curve
        self.buffers[signal_name] = SampleBasedBuffer(capacity=self._capacity)
    
    def update_signal(self, signal_name: str, sample_idx: float, value: float) -> None:
        """Update signal with new sample. Uses sample index, not time."""
        if signal_name in self.buffers:
            self.buffers[signal_name].append(sample_idx, value)
            self.data_changed = True  # Mark that data has changed
    
    def refresh_plots(self) -> int:
        """Refresh plots with safety guards against empty data.
            
        Returns:
            Number of signals actually updated
        """
        # Only refresh if data has changed since last refresh
        if not self.data_changed:
            return 0
        
        updated_count = 0
        
        try:
            # Update all signals that have data
            for signal_name, curve in self.curves.items():
                buffer = self.buffers[signal_name]
                indices, values = buffer.get_data()
                
                # Safety: only update if we have valid data
                if len(indices) > 0 and len(values) > 0 and len(indices) == len(values):
                    curve.setData(indices, values, skipFiniteCheck=True, copy=False)
                    updated_count += 1
        except Exception as e:
            logger.error(f"Plot refresh error in group {self.name}: {e}")
        
        # Clear change tracking after full update
        self.data_changed = False
        
        return updated_count
    
    def clear_all(self) -> None:
        for buffer in self.buffers.values():
            buffer.clear()
        self.data_changed = True  # Force refresh after clear
    
    def resize_buffers(self, new_capacity: int) -> None:
        """Resize all buffers in this group."""
        self._capacity = new_capacity
        for buffer in self.buffers.values():
            buffer.resize(new_capacity)
        self.data_changed = True

    def get_latest_sample_index(self) -> Optional[float]:
        """Get latest sample index across all signals."""
        latest = None
        try:
            for buffer in self.buffers.values():
                buffer_idx = buffer.latest_sample_index()
                if buffer_idx is None:
                    continue
                if latest is None or buffer_idx > latest:
                    latest = buffer_idx
        except Exception as e:
            logger.error(f"Error getting latest sample index: {e}")
        return latest


class SignalDisplayWidget(QtWidgets.QWidget):
    """Widget displaying a single signal with name, value, and unit."""
    
    def __init__(self, display_name: str, unit: str, enum_map: Optional[dict] = None, parent=None):
        super().__init__(parent)
        
        self.display_name = display_name
        self.unit = unit
        self.enum_map = enum_map
        
        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        self.name_label = QtWidgets.QLabel(f"{display_name}:", self)
        self.name_label.setMinimumWidth(150)
        self.name_label.setStyleSheet("font-weight: bold; font-size: 11px;")
        layout.addWidget(self.name_label)
        
        self.value_display = QtWidgets.QLineEdit(self)
        self.value_display.setReadOnly(True)
        self.value_display.setAlignment(QtCore.Qt.AlignCenter)
        self.value_display.setMinimumWidth(90)
        self.value_display.setMaximumWidth(110)
        self.value_display.setStyleSheet("""
            QLineEdit {
                background-color: #1a1a1a;
                color: #00ff00;
                font-family: 'Courier New', monospace;
                font-size: 12px;
                font-weight: bold;
                border: 2px solid #333333;
                border-radius: 3px;
                padding: 2px;
            }
        """)
        self.value_display.setText("--")
        layout.addWidget(self.value_display)
        
        self.unit_label = QtWidgets.QLabel(unit, self)
        self.unit_label.setMinimumWidth(30)
        self.unit_label.setStyleSheet("font-size: 11px;")
        layout.addWidget(self.unit_label)
        
        layout.addStretch()
    
    def set_value(self, value: Optional[float]) -> None:
        if value is None:
            self.value_display.setText("--")
        else:
            if self.enum_map is not None:
                int_value = int(value)
                text = self.enum_map.get(int_value, f"UNKNOWN({int_value})")
            else:
                if abs(value) < 0.001:
                    text = f"{value:.6f}"
                elif abs(value) < 1.0:
                    text = f"{value:.4f}"
                elif abs(value) < 100.0:
                    text = f"{value:.3f}"
                else:
                    text = f"{value:.1f}"
            self.value_display.setText(text)


class SignalGroupWidget(QtWidgets.QGroupBox):
    """Widget displaying a group of related signals."""
    
    def __init__(self, group_name: str, title: str, parent=None):
        super().__init__(title, parent)
        
        self.group_name = group_name
        self.signal_displays = {}
        
        self.layout = QtWidgets.QVBoxLayout(self)
        self.layout.setSpacing(5)
        self.layout.setContentsMargins(12, 12, 12, 12)
        
        # Modern styling already applied by global stylesheet
    
    def add_signal_display(self, signal_name: str, display_name: str, unit: str, 
                           enum_map: Optional[dict] = None) -> None:
        display = SignalDisplayWidget(display_name, unit, enum_map, self)
        self.signal_displays[signal_name] = display
        self.layout.addWidget(display)
    
    def update_signal(self, signal_name: str, value: Optional[float]) -> None:
        display = self.signal_displays.get(signal_name)
        if display:
            display.set_value(value)
    
    def update_all_signals(self, values: dict) -> None:
        for signal_name, value in values.items():
            self.update_signal(signal_name, value)


class ParameterEditDialog(QtWidgets.QDialog):
    """Dialog for editing a single parameter."""
    
    def __init__(self, param: ParameterMetadata, current_value: float, param_panel, parent=None):
        super().__init__(parent)
        self.param = param
        self.param_panel = param_panel
        self.new_value = None
        
        self.setWindowTitle(f"Edit Parameter: {param.name}")
        self.setModal(True)
        self.setMinimumWidth(400)
        
        layout = QtWidgets.QVBoxLayout(self)
        layout.setSpacing(15)
        
        # Parameter info section
        info_group = QtWidgets.QGroupBox("Parameter Information")
        info_layout = QtWidgets.QFormLayout(info_group)
        
        # Full name
        full_name = QtWidgets.QLabel(f"{param.component}.{param.name}")
        full_name.setStyleSheet("font-weight: bold; color: #1f2937;")
        info_layout.addRow("Full Name:", full_name)
        
        # Display name
        display_name = QtWidgets.QLabel(param.display_name)
        info_layout.addRow("Description:", display_name)
        
        # Range
        range_label = QtWidgets.QLabel(f"{param.min_value:.4f} to {param.max_value:.4f}")
        info_layout.addRow("Valid Range:", range_label)
        
        # Unit
        if param.unit:
            unit_label = QtWidgets.QLabel(param.unit)
            info_layout.addRow("Unit:", unit_label)
        
        # Default value (if available)
        default_label = QtWidgets.QLabel(f"{param.default_value:.4f}")
        info_layout.addRow("Default:", default_label)
        
        layout.addWidget(info_group)
        
        # Value input section
        value_group = QtWidgets.QGroupBox("New Value")
        value_layout = QtWidgets.QHBoxLayout(value_group)
        
        self.value_edit = QtWidgets.QDoubleSpinBox()
        self.value_edit.setRange(param.min_value, param.max_value)
        self.value_edit.setDecimals(6)
        self.value_edit.setSingleStep((param.max_value - param.min_value) / 100)
        self.value_edit.setValue(current_value)
        self.value_edit.setMinimumWidth(150)
        value_layout.addWidget(self.value_edit)
        
        if param.unit:
            unit_label = QtWidgets.QLabel(param.unit)
            value_layout.addWidget(unit_label)
        
        value_layout.addStretch()
        layout.addWidget(value_group)
        
        # Buttons
        button_layout = QtWidgets.QHBoxLayout()
        
        restore_button = QtWidgets.QPushButton("Restore Default")
        restore_button.clicked.connect(self._on_restore_default)
        button_layout.addWidget(restore_button)
        
        button_layout.addStretch()
        
        cancel_button = QtWidgets.QPushButton("Cancel")
        cancel_button.clicked.connect(self.reject)
        button_layout.addWidget(cancel_button)
        
        write_button = QtWidgets.QPushButton("Write")
        write_button.setStyleSheet("""
            QPushButton {
                background-color: #10b981;
                color: white;
                font-weight: bold;
                padding: 8px 20px;
            }
            QPushButton:hover {
                background-color: #059669;
            }
        """)
        write_button.clicked.connect(self._on_write)
        button_layout.addWidget(write_button)
        
        layout.addLayout(button_layout)
    
    def _on_restore_default(self):
        """Restore parameter to default value."""
        reply = QtWidgets.QMessageBox.question(
            self,
            "Restore Default",
            f"Restore parameter to default value ({self.param.default_value:.4f})?",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No
        )
        
        if reply == QtWidgets.QMessageBox.Yes:
            self.value_edit.setValue(self.param.default_value)
            self._on_write()
    
    def _on_write(self):
        """Write new value to controller."""
        self.new_value = self.value_edit.value()
        self.accept()


class ParameterItemWidget(QtWidgets.QWidget):
    """Widget for a single parameter with double-click edit capability."""
    
    def __init__(self, param: ParameterMetadata, param_panel, parent=None):
        super().__init__(parent)
        
        self.param = param
        self.param_panel = param_panel  # Reference to ParameterPanelWidget
        
        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(10, 8, 10, 8)
        layout.setSpacing(12)
        
        # Parameter name label (just the name, not component.name)
        self.name_label = QtWidgets.QLabel(param.name)
        self.name_label.setMinimumWidth(250)
        self.name_label.setMaximumWidth(350)
        self.name_label.setStyleSheet("""
            font-size: 12px; 
            color: #374151; 
            font-weight: 500;
            padding: 4px;
        """)
        self.name_label.setCursor(QtCore.Qt.PointingHandCursor)
        layout.addWidget(self.name_label)
        
        # Spacer to push value closer
        layout.addSpacing(20)
        
        # Current value display (right-aligned)
        self.value_label = QtWidgets.QLabel(f"{param.current_value:.4f}")
        self.value_label.setMinimumWidth(90)
        self.value_label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        self.value_label.setStyleSheet("""
            font-size: 13px; 
            font-weight: 600; 
            color: #1f2937;
            background-color: #f3f4f6;
            border: 1px solid #d1d5db;
            border-radius: 4px;
            padding: 4px 8px;
        """)
        self.value_label.setCursor(QtCore.Qt.PointingHandCursor)
        layout.addWidget(self.value_label)
        
        # Unit label (if present)
        if param.unit:
            unit_label = QtWidgets.QLabel(param.unit)
            unit_label.setMinimumWidth(35)
            unit_label.setMaximumWidth(50)
            unit_label.setStyleSheet("""
                font-size: 11px; 
                color: #6b7280; 
                margin-left: 4px;
                padding: 4px;
            """)
            layout.addWidget(unit_label)
        
        layout.addStretch()
        
        # Make entire widget clickable and add hover effect
        self.setCursor(QtCore.Qt.PointingHandCursor)
        self.setStyleSheet("""
            ParameterItemWidget {
                background-color: white;
                border-radius: 6px;
            }
            ParameterItemWidget:hover {
                background-color: #f9fafb;
                border: 1px solid #e5e7eb;
            }
        """)
    
    def mouseDoubleClickEvent(self, event: QtGui.QMouseEvent):
        """Handle double-click to open edit dialog."""
        if event.button() == QtCore.Qt.LeftButton:
            self._open_edit_dialog()
    
    def _open_edit_dialog(self):
        """Open parameter edit dialog."""
        try:
            current_value = float(self.value_label.text())
        except ValueError:
            current_value = self.param.current_value
        
        dialog = ParameterEditDialog(self.param, current_value, self.param_panel, self)
        if dialog.exec_() == QtWidgets.QDialog.Accepted:
            if dialog.new_value is not None:
                self.param_panel.set_parameter_value(self.param, dialog.new_value)
    
    def update_value(self, value: float):
        """Update displayed value."""
        self.value_label.setText(f"{value:.4f}")
        self.param.current_value = value


class ParameterPanelWidget(QtWidgets.QWidget):
    """Panel displaying all parameters with editing capability."""
    
    # Signals for requesting parameter operations from parent
    request_load_parameters = QtCore.pyqtSignal()
    request_set_parameter = QtCore.pyqtSignal(str, str, float)  # component, name, value
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        self.parameter_widgets: Dict[str, ParameterItemWidget] = {}
        self.component_widgets: Dict[str, QtWidgets.QGroupBox] = {}
        self.all_parameters: List[ParameterMetadata] = []
        
        self._build_ui()
    
    def _build_ui(self):
        """Build parameter panel UI."""
        main_layout = QtWidgets.QVBoxLayout(self)
        main_layout.setContentsMargins(15, 15, 15, 15)
        main_layout.setSpacing(12)
        
        # Title
        title = QtWidgets.QLabel("Parameter Configuration")
        title.setStyleSheet("""
            QLabel {
                font-size: 16px;
                font-weight: 600;
                color: #1f2937;
                padding: 10px;
                background-color: #fef3c7;
                border-left: 4px solid #f59e0b;
                border-radius: 6px;
            }
        """)
        main_layout.addWidget(title)
        
        # Top controls
        controls_layout = QtWidgets.QHBoxLayout()
        
        # Search box
        self.search_edit = QtWidgets.QLineEdit(self)
        self.search_edit.setPlaceholderText("🔍 Search parameters...")
        self.search_edit.textChanged.connect(self._filter_parameters)
        controls_layout.addWidget(QtWidgets.QLabel("Search:"))
        controls_layout.addWidget(self.search_edit)
        
        # Load parameters button
        self.load_btn = QtWidgets.QPushButton("📥 Load Parameters", self)
        self.load_btn.clicked.connect(self.load_parameters)
        self.load_btn.setToolTip("Discover and load all parameters from controller")
        controls_layout.addWidget(self.load_btn)
        
        # Refresh values button
        self.refresh_btn = QtWidgets.QPushButton("🔄 Refresh Values", self)
        self.refresh_btn.clicked.connect(self.refresh_parameter_values)
        self.refresh_btn.setToolTip("Refresh current values of all loaded parameters")
        self.refresh_btn.setEnabled(False)  # Disabled until parameters are loaded
        controls_layout.addWidget(self.refresh_btn)
        
        # Save preset button
        self.save_preset_btn = QtWidgets.QPushButton("💾 Save Preset", self)
        self.save_preset_btn.clicked.connect(self.save_preset)
        self.save_preset_btn.setToolTip("Save current parameter values to a JSON file")
        self.save_preset_btn.setEnabled(False)  # Disabled until parameters are loaded
        controls_layout.addWidget(self.save_preset_btn)
        
        # Load preset button
        self.load_preset_btn = QtWidgets.QPushButton("📂 Load Preset", self)
        self.load_preset_btn.clicked.connect(self.load_preset)
        self.load_preset_btn.setToolTip("Load parameter values from a JSON file")
        self.load_preset_btn.setEnabled(False)  # Disabled until parameters are loaded
        controls_layout.addWidget(self.load_preset_btn)
        
        controls_layout.addStretch()
        main_layout.addLayout(controls_layout)
        
        # Scroll area for parameters
        scroll = QtWidgets.QScrollArea(self)
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        
        self.params_widget = QtWidgets.QWidget()
        self.params_layout = QtWidgets.QVBoxLayout(self.params_widget)
        self.params_layout.setSpacing(10)
        
        scroll.setWidget(self.params_widget)
        main_layout.addWidget(scroll)
        
        # Status label
        self.status_label = QtWidgets.QLabel("No parameters loaded. Click 'Load Parameters' to discover parameters.", self)
        self.status_label.setStyleSheet("""
            QLabel {
                color: #6b7280;
                font-style: italic;
                padding: 8px;
                background-color: #f9fafb;
                border-radius: 4px;
            }
        """)
        main_layout.addWidget(self.status_label)
    
    def load_parameters(self):
        """Load parameters from controller (signal-based, non-blocking)."""
        # Disable buttons during load
        self.load_btn.setEnabled(False)
        self.refresh_btn.setEnabled(False)
        
        self.status_label.setText("Requesting parameter count...")
        self.status_label.setStyleSheet("color: #ff8800;")
        
        # Emit signal to parent to start parameter loading
        self.request_load_parameters.emit()
    
    def _on_load_progress(self, message: str):
        """Update status with progress message."""
        self.status_label.setText(message)
        self.status_label.setStyleSheet("color: #ff8800;")
    
    def display_parameters(self, parameters: List[ParameterMetadata]):
        """Display loaded parameters in UI.
        
        Called by parent window when parameters are loaded.
        """
        try:
            # Clear existing
            self.parameter_widgets.clear()
            self.component_widgets.clear()
            for i in reversed(range(self.params_layout.count())):
                widget = self.params_layout.itemAt(i).widget()
                if widget:
                    widget.setParent(None)
            
            # Group by component
            by_component: Dict[str, List[ParameterMetadata]] = {}
            for param in parameters:
                if param.component not in by_component:
                    by_component[param.component] = []
                by_component[param.component].append(param)
            
            # Create widgets
            for component, params in sorted(by_component.items()):
                group_box = QtWidgets.QGroupBox(component.replace('_', ' ').title(), self.params_widget)
                group_box.setStyleSheet("""
                    QGroupBox {
                        font-weight: bold;
                        border: 2px solid #4a9eff;
                        border-radius: 5px;
                        margin-top: 8px;
                        padding-top: 8px;
                    }
                    QGroupBox::title {
                        subcontrol-origin: margin;
                        left: 10px;
                        padding: 0 5px;
                    }
                """)
                
                group_layout = QtWidgets.QVBoxLayout(group_box)
                
                for param in params:
                    param_widget = ParameterItemWidget(param, self, group_box)
                    group_layout.addWidget(param_widget)
                    
                    key = f"{param.component}.{param.name}"
                    self.parameter_widgets[key] = param_widget
                
                self.component_widgets[component] = group_box
                self.params_layout.addWidget(group_box)
            
            self.params_layout.addStretch()
            
            self.status_label.setText(f"✓ Loaded {len(parameters)} parameters from {len(by_component)} components")
            self.status_label.setStyleSheet("color: #00ff00;")
            
            # Enable buttons now that parameters are loaded
            self.refresh_btn.setEnabled(True)
            self.save_preset_btn.setEnabled(True)
            self.load_preset_btn.setEnabled(True)
            
            # Store the loaded parameters list for preset operations
            self.all_parameters = parameters
            
            # Re-enable buttons
            self.load_btn.setEnabled(True)
            
            logger.info(f"Loaded {len(parameters)} parameters")
            
        except Exception as e:
            error_msg = f"Error displaying parameters: {e}"
            self.status_label.setText(error_msg)
            self.status_label.setStyleSheet("color: #ff0000;")
            logger.error(error_msg)
            self.load_btn.setEnabled(True)
    
    def _on_load_error(self, error_msg: str):
        """Handle parameter load error."""
        self.status_label.setText(f"✗ {error_msg}")
        self.status_label.setStyleSheet("color: #ff0000;")
        logger.error(error_msg)
        self.load_btn.setEnabled(True)
    
    def set_parameter_value(self, param: ParameterMetadata, value: float):
        """Set parameter value on controller.
        
        Args:
            param: Parameter metadata
            value: New parameter value
        """
        # Emit signal to parent to handle the actual network operation
        self.request_set_parameter.emit(param.component, param.name, value)
        
        # Optimistically update display
        key = f"{param.component}.{param.name}"
        if key in self.parameter_widgets:
            self.parameter_widgets[key].update_value(value)
        
        self.status_label.setText(f"Setting {param.component}.{param.name} = {value:.4f}...")
        self.status_label.setStyleSheet("color: #ff8800;")
    
    def on_parameter_set_complete(self, component: str, name: str, success: bool, error_msg: str = ""):
        """Called by parent when parameter set operation completes.
        
        Args:
            component: Component name
            name: Parameter name
            success: True if successful, False otherwise
            error_msg: Error message if failed
        """
        if success:
            self.status_label.setText(f"✓ Set {component}.{name}")
            self.status_label.setStyleSheet("color: #00ff00;")
            logger.info(f"Set {component}.{name}")
        else:
            self.status_label.setText(f"✗ Failed to set {component}.{name}: {error_msg}")
            self.status_label.setStyleSheet("color: #ff0000;")
            logger.error(f"Failed to set {component}.{name}: {error_msg}")
    
    def refresh_parameter_values(self):
        """Refresh all parameter values from controller (called by Refresh button only)."""
        self.status_label.setText("Parameter value refresh not yet implemented with unified client")
        self.status_label.setStyleSheet("color: #ff8800;")
        logger.warning("Parameter value refresh not implemented - would require GET requests for each parameter")
        # TODO: Implement batch GET requests via unified client
    
    def _filter_parameters(self, text: str):
        """Filter parameters by search text."""
        text = text.lower()
        
        for component, group_box in self.component_widgets.items():
            component_visible = False
            
            for i in range(group_box.layout().count()):
                item = group_box.layout().itemAt(i)
                if item and item.widget():
                    widget = item.widget()
                    if isinstance(widget, ParameterItemWidget):
                        matches = (text in widget.param.component.lower() or
                                 text in widget.param.name.lower() or
                                 text in widget.param.display_name.lower())
                        widget.setVisible(matches)
                        if matches:
                            component_visible = True
            
            group_box.setVisible(component_visible)
    
    def save_preset(self):
        """Save current parameter values to a JSON file."""
        # Note: Save reads from UI widgets, doesn't need connection
        if not self.parameter_widgets:
            self.status_label.setText("✗ No parameters loaded - cannot save preset")
            self.status_label.setStyleSheet("color: #ff0000;")
            return
        
        # Open file dialog
        default_name = f"sfc_preset_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        file_path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self,
            "Save Parameter Preset",
            default_name,
            "JSON Files (*.json);;All Files (*)"
        )
        
        if not file_path:
            return  # User cancelled
        
        try:
            # Collect current values from all parameter widgets
            preset_data = {
                "timestamp": datetime.now().isoformat(),
                "description": "SFC Parameter Preset",
                "parameters": {}
            }
            
            for key, widget in self.parameter_widgets.items():
                component, name = key.split('.', 1)
                if component not in preset_data["parameters"]:
                    preset_data["parameters"][component] = {}
                
                # Get current value from the widget's text field
                try:
                    current_value = float(widget.value_edit.text())
                    preset_data["parameters"][component][name] = {
                        "value": current_value,
                        "display_name": widget.param.display_name,
                        "unit": widget.param.unit,
                        "min": widget.param.min_value,
                        "max": widget.param.max_value
                    }
                except ValueError:
                    logger.warning(f"Could not parse value for {key}")
            
            # Write to file
            with open(file_path, 'w') as f:
                json.dump(preset_data, f, indent=2)
            
            self.status_label.setText(f"✓ Preset saved to {Path(file_path).name}")
            self.status_label.setStyleSheet("color: #00aa00;")
            logger.info(f"Saved preset to {file_path}")
            
            QtWidgets.QMessageBox.information(
                self,
                "Preset Saved",
                f"Parameter preset saved successfully to:\n{file_path}\n\n"
                f"Saved {len(self.parameter_widgets)} parameters."
            )
            
        except Exception as e:
            error_msg = f"Failed to save preset: {e}"
            self.status_label.setText(f"✗ {error_msg}")
            self.status_label.setStyleSheet("color: #ff0000;")
            logger.error(error_msg)
            QtWidgets.QMessageBox.critical(self, "Save Error", error_msg)
    
    def load_preset(self):
        """Load parameter values from a JSON file."""
        # Ensure parameter client has its own connection for setting values
        if not self.param_client.connected:
            if not self.param_client.connect():
                self.status_label.setText("✗ Failed to connect for preset load")
                self.status_label.setStyleSheet("color: #ff0000;")
                return
        
        # Open file dialog
        file_path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "Load Parameter Preset",
            "",
            "JSON Files (*.json);;All Files (*)"
        )
        
        if not file_path:
            return  # User cancelled
        
        try:
            # Read from file
            with open(file_path, 'r') as f:
                preset_data = json.load(f)
            
            if "parameters" not in preset_data:
                raise ValueError("Invalid preset file format: missing 'parameters' key")
            
            # Ask for confirmation
            param_count = sum(len(params) for params in preset_data["parameters"].values())
            timestamp = preset_data.get("timestamp", "Unknown")
            
            reply = QtWidgets.QMessageBox.question(
                self,
                "Load Preset",
                f"Load preset from:\n{Path(file_path).name}\n\n"
                f"Created: {timestamp}\n"
                f"Parameters: {param_count}\n\n"
                f"This will update parameter values on the controller.\n"
                f"Continue?",
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                QtWidgets.QMessageBox.No
            )
            
            if reply != QtWidgets.QMessageBox.Yes:
                return
            
            # Apply parameters
            self.status_label.setText("Loading preset...")
            self.status_label.setStyleSheet("color: #ff8800;")
            
            success_count = 0
            fail_count = 0
            
            for component, params in preset_data["parameters"].items():
                for name, param_data in params.items():
                    key = f"{component}.{name}"
                    value = param_data["value"]
                    
                    # Find the parameter metadata
                    param_meta = None
                    for p in self.all_parameters:
                        if p.component == component and p.name == name:
                            param_meta = p
                            break
                    
                    if param_meta:
                        # Check range
                        if param_meta.min_value <= value <= param_meta.max_value:
                            # Set parameter on controller
                            success = self.param_client.set_parameter(component, name, value)
                            if success:
                                # Update widget display
                                if key in self.parameter_widgets:
                                    self.parameter_widgets[key].update_value(value)
                                success_count += 1
                                logger.info(f"Set {key} = {value}")
                            else:
                                fail_count += 1
                                logger.warning(f"Failed to set {key}")
                        else:
                            fail_count += 1
                            logger.warning(f"Value {value} out of range for {key}")
                    else:
                        fail_count += 1
                        logger.warning(f"Parameter {key} not found in current configuration")
            
            # Report results
            if fail_count == 0:
                self.status_label.setText(f"✓ Loaded preset: {success_count} parameters applied")
                self.status_label.setStyleSheet("color: #00aa00;")
                QtWidgets.QMessageBox.information(
                    self,
                    "Preset Loaded",
                    f"Successfully loaded preset from:\n{Path(file_path).name}\n\n"
                    f"Applied {success_count} parameters."
                )
            else:
                self.status_label.setText(f"⚠ Loaded preset: {success_count} OK, {fail_count} failed")
                self.status_label.setStyleSheet("color: #ff8800;")
                QtWidgets.QMessageBox.warning(
                    self,
                    "Preset Loaded with Warnings",
                    f"Loaded preset from:\n{Path(file_path).name}\n\n"
                    f"Success: {success_count}\n"
                    f"Failed: {fail_count}\n\n"
                    f"Check log for details."
                )
            
            logger.info(f"Loaded preset from {file_path}: {success_count} success, {fail_count} failed")
            
        except json.JSONDecodeError as e:
            error_msg = f"Invalid JSON file: {e}"
            self.status_label.setText(f"✗ {error_msg}")
            self.status_label.setStyleSheet("color: #ff0000;")
            logger.error(error_msg)
            QtWidgets.QMessageBox.critical(self, "Load Error", error_msg)
        except Exception as e:
            error_msg = f"Failed to load preset: {e}"
            self.status_label.setText(f"✗ {error_msg}")
            self.status_label.setStyleSheet("color: #ff0000;")
            logger.error(error_msg)
            QtWidgets.QMessageBox.critical(self, "Load Error", error_msg)


class TelemetryMonitorWindow(QtWidgets.QMainWindow):
    """Main window with combined dashboard, oscilloscope, and parameter panel.

    Added features:
        - simple_mode: drastically reduced plotting path for diagnostics (single signal only)
        - sample-index x-axis (simple mode) to eliminate stale pre-start time offsets
        - profile instrumentation: lightweight timing of packet processing & plot updates
    """
    
    def __init__(self, host: str, port: int, time_window_sec: float, verbose: bool = False,
                 simple_mode: bool = False, profile: bool = False, simple_sample_window: int = 1500,
                 decimate: int = 1):
        super().__init__()
        
        logger.info(f"=== Initializing Monitor (simple={simple_mode}, decimate={decimate}, verbose={verbose}, profile={profile}) ===")
        
        self.setWindowTitle("SFC Control Center - Monitoring & Configuration")
        self.resize(1400, 900)
        
        self.time_window_sec = time_window_sec
        self.verbose = verbose
        self.simple_mode = simple_mode
        self.profile_enabled = profile
        self.start_time = None
        
        # Decimation: process only 1 out of N packets
        self.decimate_factor = max(1, int(decimate))
        self.packet_counter = 0
        logger.info(f"Decimation factor: {self.decimate_factor} (processing 1 out of every {self.decimate_factor} packets)")
        
        # Sample index tracking (used for x-axis in all modes now)
        self.global_sample_index = 0
        
        if self.simple_mode:
            # Track sample index instead of time for diagnostics (resets on START / CLEAR)
            self.simple_sample_window = max(10, int(simple_sample_window))
            logger.info(f"Simple mode: sample window = {self.simple_sample_window}")
        else:
            # Full mode uses sample-based window too
            self.sample_window = max(10, int(simple_sample_window))  # Can be adjusted via spinner
            logger.info(f"Full mode: sample window = {self.sample_window}")
        
        # State machine enum mapping
        self.sfc_state_map = {
            0: 'IDLE',
            1: 'AUXILIARIES_ON',
            2: 'BREAKERS_OFF',
            3: 'BREAKERS_ON',
            4: 'OPERATION_OFF',
            5: 'OPERATION_ON',
            6: 'SHUTDOWN_REQUESTED'
        }
        
        # Storage for latest values (for dashboard)
        self.latest_values = {
            'measurements': {},
            'observer': {},
            'modulator': {},
            'controller': {},
            'state_machine': {}
        }
        
        # Create unified network client (single-threaded)
        self.network_client = UnifiedNetworkClient(host=host, port=port, verbose=verbose, parent=self)
        
        # Connect signals
        self.network_client.telemetry_packet_received.connect(self._on_packet_received)
        self.network_client.parameter_response_received.connect(self._on_parameter_response)
        self.network_client.connected_changed.connect(self._on_connection_changed)
        self.network_client.error_occurred.connect(self._on_network_error)
        
        # Telemetry status tracking
        self.telemetry_streaming = False
        self.last_packet_time = 0.0
        
        # Packet processing queue (to defer heavy processing)
        self._packet_queue = []
        self._packet_process_timer = QtCore.QTimer(self)
        self._packet_process_timer.timeout.connect(self._process_packet_queue)
        self._packet_process_timer.start(10)  # Process every 10ms
        self._packet_timer_fire_count = 0
        logger.info("Packet processing timer started (10ms interval)")
        
        # Parameter operation state
        self._pending_param_requests = []  # Queue of pending parameter requests
        self._parameter_cache = {}  # Cache loaded parameters
        
        # Performance instrumentation structures (lightweight, only when enabled)
        if self.profile_enabled:
            self._perf_plot_updates = deque(maxlen=400)
            self._perf_packet_batches = deque(maxlen=400)
            self._last_perf_log_time = time.time()
        else:
            self._perf_plot_updates = None
            self._perf_packet_batches = None
            self._last_perf_log_time = 0.0

        # Build UI (may build simplified oscilloscope if simple_mode)
        self._build_ui(host, port)
        
        # Apply modern stylesheet
        self._apply_modern_style()
        
        # Setup update timers with different rates
        self.plot_update_timer = QtCore.QTimer(self)
        self.plot_update_timer.timeout.connect(self._update_oscilloscope)
        self.plot_update_timer.start(UI_UPDATE_INTERVAL_MS)
        
        self.dashboard_update_timer = QtCore.QTimer(self)
        self.dashboard_update_timer.timeout.connect(self._update_dashboard)
        self.dashboard_update_timer.start(DASHBOARD_UPDATE_INTERVAL_MS)
        
        self.stats_timer = QtCore.QTimer(self)
        self.stats_timer.timeout.connect(self._update_statistics)
        self.stats_timer.start(STATS_UPDATE_INTERVAL_MS)
        
        # Watchdog timer to detect stalled telemetry
        self.watchdog_timer = QtCore.QTimer(self)
        self.watchdog_timer.timeout.connect(self._check_telemetry_health)
        self.watchdog_timer.start(1000)  # Check every second
        
        # NOTE: Removed automatic parameter refresh timer to reduce backend load
        # Parameters are now only loaded on user request via "Refresh Parameters" button
        
        mode_note = "(SIMPLE MODE)" if self.simple_mode else ""
        logger.info(f"Telemetry monitor initialized {mode_note}")

    # ----------------------------------------------------------------------------------
    # Performance instrumentation helpers
    # ----------------------------------------------------------------------------------
    def _record_plot_update_time(self, dt: float) -> None:
        if self._perf_plot_updates is not None:
            self._perf_plot_updates.append(dt)
            self._maybe_log_performance()

    def _record_packet_batch_time(self, dt: float) -> None:
        if self._perf_packet_batches is not None:
            self._perf_packet_batches.append(dt)
            self._maybe_log_performance()

    def _maybe_log_performance(self) -> None:
        if not self.profile_enabled:
            return
        now = time.time()
        # Log every ~2 seconds
        if (now - self._last_perf_log_time) < 2.0:
            return
        self._last_perf_log_time = now
        try:
            if self._perf_plot_updates:
                pu = list(self._perf_plot_updates)
                avg_pu = sum(pu)/len(pu)
                max_pu = max(pu)
            else:
                avg_pu = max_pu = 0.0
            if self._perf_packet_batches:
                pb = list(self._perf_packet_batches)
                avg_pb = sum(pb)/len(pb)
                max_pb = max(pb)
            else:
                avg_pb = max_pb = 0.0
            logger.info(f"PERF plot_update avg={avg_pu*1000:.2f}ms max={max_pu*1000:.2f}ms | "
                        f"packet_batch avg={avg_pb*1000:.2f}ms max={max_pb*1000:.2f}ms | "
                        f"queue_len={len(self._packet_queue)}")
        except Exception as e:
            logger.debug(f"Perf logging error: {e}")
    
    def _build_ui(self, host: str, port: int) -> None:
        """Build main UI with dashboard, oscilloscope, and parameter panel."""
        central = QtWidgets.QWidget(self)
        self.setCentralWidget(central)
        main_layout = QtWidgets.QVBoxLayout(central)
        
        # Control panel at top
        control_panel = self._build_control_panel(host, port)
        main_layout.addWidget(control_panel)
        
        # Main splitter for draggable tab areas
        self.main_splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.main_splitter.setHandleWidth(8)  # Wider handle for easier dragging
        self.main_splitter.setChildrenCollapsible(False)  # Prevent panels from collapsing completely
        
        # Create initial tab widget
        self.tab_widget = self._create_tab_widget()
        
        # Tab 1: Telemetry Monitor (Digital Dashboard)
        dashboard_widget = self._build_dashboard()
        self.tab_widget.addTab(dashboard_widget, "📊 Telemetry Monitor")
        
        # Tab 2: Oscilloscope
        oscilloscope_widget = self._build_oscilloscope()
        self.tab_widget.addTab(oscilloscope_widget, "📈 Oscilloscope")
        
        # Tab 3: Parameters
        self.param_panel = ParameterPanelWidget(self)
        self.param_panel.request_load_parameters.connect(self._start_parameter_loading)
        self.param_panel.request_set_parameter.connect(self._handle_parameter_set)
        self.tab_widget.addTab(self.param_panel, "⚙️ Parameters")
        
        # Tab 4: Commands
        commands_widget = self._build_commands_tab()
        self.tab_widget.addTab(commands_widget, "🎮 Commands")
        
        # Add to splitter
        self.main_splitter.addWidget(self.tab_widget)
        
        main_layout.addWidget(self.main_splitter, stretch=1)
        
        self.statusBar().showMessage("Disconnected")
    
    def _create_tab_widget(self, closable: bool = False) -> QtWidgets.QTabWidget:
        """Create a new tab widget with drag-drop enabled."""
        tab_widget = QtWidgets.QTabWidget()
        tab_widget.setDocumentMode(True)
        tab_widget.setMovable(True)  # Allow dragging tabs within same widget
        tab_widget.setTabsClosable(closable)  # Show close buttons on split tabs
        
        # Connect tab close signal
        if closable:
            tab_widget.tabCloseRequested.connect(
                lambda index: self._close_tab(tab_widget, index)
            )
        
        # Create context menu for splitting
        tab_widget.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)
        tab_widget.customContextMenuRequested.connect(
            lambda pos: self._show_tab_context_menu(tab_widget, pos)
        )
        
        return tab_widget
    
    def _show_tab_context_menu(self, tab_widget: QtWidgets.QTabWidget, pos) -> None:
        """Show context menu for tab operations."""
        # Get the tab bar and find which tab was right-clicked
        tab_bar = tab_widget.tabBar()
        global_pos = tab_bar.mapToGlobal(pos)
        
        # Find the tab index at the click position
        tab_index = -1
        for i in range(tab_widget.count()):
            if tab_bar.tabRect(i).contains(pos):
                tab_index = i
                break
        
        if tab_index == -1:
            return
        
        menu = QtWidgets.QMenu(self)
        
        # Split options
        split_right_action = menu.addAction("📐 Split Right")
        split_bottom_action = menu.addAction("📐 Split Bottom")
        
        # Only show merge if there are multiple splitters
        if self.main_splitter.count() > 1:
            menu.addSeparator()
            merge_action = menu.addAction("🔗 Merge All Tabs")
        else:
            merge_action = None
        
        action = menu.exec_(global_pos)
        
        if action == split_right_action:
            self._split_tab(tab_widget, tab_index, QtCore.Qt.Horizontal)
        elif action == split_bottom_action:
            self._split_tab(tab_widget, tab_index, QtCore.Qt.Vertical)
        elif merge_action and action == merge_action:
            self._merge_all_tabs()
    
    def _split_tab(self, source_widget: QtWidgets.QTabWidget, tab_index: int, 
                   orientation: QtCore.Qt.Orientation) -> None:
        """Split a tab into a new tab widget (creates a duplicate view)."""
        # Get the tab content and title (but DON'T remove from source)
        tab_text = source_widget.tabText(tab_index)
        
        # Create new tab widget with the same tab (closable for split tabs)
        new_tab_widget = self._create_tab_widget(closable=True)
        
        # Duplicate the tab content based on which tab it is
        tab_name = tab_text.split(' ', 1)[-1]  # Remove emoji prefix
        
        if "Telemetry Monitor" in tab_name:
            new_content = self._build_dashboard()
        elif "Oscilloscope" in tab_name:
            new_content = self._build_oscilloscope()
        elif "Parameters" in tab_name:
            new_content = ParameterPanelWidget(self)
            # Connect signals for the new parameter panel
            new_content.request_load_parameters.connect(self._start_parameter_loading)
            new_content.request_set_parameter.connect(self._handle_parameter_set)
            # NOTE: Don't auto-load parameters - user clicks "Load Parameters" button
        elif "Commands" in tab_name:
            new_content = self._build_commands_tab()
        else:
            logger.warning(f"Unknown tab type: {tab_name}")
            return
        
        new_tab_widget.addTab(new_content, tab_text)
        
        # Find the parent splitter
        parent_splitter = source_widget.parent()
        
        if isinstance(parent_splitter, QtWidgets.QSplitter):
            # Get the index of the source widget in the parent splitter
            source_index = parent_splitter.indexOf(source_widget)
            
            if parent_splitter.orientation() == orientation:
                # Same orientation - just insert next to it
                parent_splitter.insertWidget(source_index + 1, new_tab_widget)
                
                # Set equal sizes for all widgets in this splitter
                count = parent_splitter.count()
                equal_size = parent_splitter.size().width() // count if orientation == QtCore.Qt.Horizontal else parent_splitter.size().height() // count
                parent_splitter.setSizes([equal_size] * count)
            else:
                # Different orientation - create new splitter
                new_splitter = QtWidgets.QSplitter(orientation)
                new_splitter.setHandleWidth(6)  # Make handle more visible/draggable
                new_splitter.setChildrenCollapsible(False)  # Prevent collapsing
                
                # Remove source from parent
                parent_splitter.widget(source_index).setParent(None)
                
                # Add source and new widget to new splitter
                new_splitter.addWidget(source_widget)
                new_splitter.addWidget(new_tab_widget)
                
                # Insert new splitter into parent at the same position
                parent_splitter.insertWidget(source_index, new_splitter)
                
                # Equal sizes based on orientation
                if orientation == QtCore.Qt.Horizontal:
                    size = parent_splitter.size().width() // 2
                else:
                    size = parent_splitter.size().height() // 2
                new_splitter.setSizes([size, size])
        
        logger.info(f"Split tab '{tab_text}' with orientation {orientation}")
        self.statusBar().showMessage(f"Tab '{tab_name}' split successfully", 2000)
    
    def _close_tab(self, tab_widget: QtWidgets.QTabWidget, tab_index: int) -> None:
        """Close a specific tab and clean up the tab widget if empty."""
        # Get tab name before removing
        tab_text = tab_widget.tabText(tab_index)
        tab_name = tab_text.split(' ', 1)[-1]  # Remove emoji prefix
        
        # Remove the tab
        widget = tab_widget.widget(tab_index)
        tab_widget.removeTab(tab_index)
        if widget:
            widget.setParent(None)
            widget.deleteLater()
        
        logger.info(f"Closed tab '{tab_name}'")
        
        # If this tab widget is now empty, remove it from the splitter
        if tab_widget.count() == 0:
            parent = tab_widget.parent()
            if isinstance(parent, QtWidgets.QSplitter) and parent != self.main_splitter:
                # This is a nested splitter - remove the empty tab widget
                tab_widget.setParent(None)
                tab_widget.deleteLater()
                
                # If the parent splitter now has only one child, collapse it
                if parent.count() == 1:
                    remaining_widget = parent.widget(0)
                    grandparent = parent.parent()
                    
                    if isinstance(grandparent, QtWidgets.QSplitter):
                        parent_index = grandparent.indexOf(parent)
                        remaining_widget.setParent(None)
                        parent.setParent(None)
                        parent.deleteLater()
                        grandparent.insertWidget(parent_index, remaining_widget)
                        logger.info("Collapsed nested splitter")
            elif parent == self.main_splitter and self.main_splitter.count() > 1:
                # Remove from main splitter if there are other tab widgets
                tab_widget.setParent(None)
                tab_widget.deleteLater()
                logger.info("Removed empty tab widget from main splitter")
        
        self.statusBar().showMessage(f"Closed '{tab_name}'", 2000)
    
    def _merge_all_tabs(self) -> None:
        """Merge all split tabs back into a single tab widget."""
        # Collect all unique tab types from all widgets (avoiding duplicates)
        tab_types = set()
        
        def collect_tab_types(widget):
            if isinstance(widget, QtWidgets.QTabWidget):
                for i in range(widget.count()):
                    tab_text = widget.tabText(i)
                    tab_name = tab_text.split(' ', 1)[-1]  # Remove emoji prefix
                    tab_types.add(tab_name)
            elif isinstance(widget, QtWidgets.QSplitter):
                for i in range(widget.count()):
                    collect_tab_types(widget.widget(i))
        
        collect_tab_types(self.main_splitter)
        
        # Clear the main splitter
        while self.main_splitter.count() > 0:
            widget = self.main_splitter.widget(0)
            widget.setParent(None)
            widget.deleteLater()  # Properly clean up old widgets
        
        # Create new tab widget with fresh instances of each unique tab type
        new_tab_widget = self._create_tab_widget()
        
        # Add tabs in standard order (only once each)
        if "Telemetry Monitor" in tab_types:
            dashboard_widget = self._build_dashboard()
            new_tab_widget.addTab(dashboard_widget, "📊 Telemetry Monitor")
        
        if "Oscilloscope" in tab_types:
            oscilloscope_widget = self._build_oscilloscope()
            new_tab_widget.addTab(oscilloscope_widget, "📈 Oscilloscope")
        
        if "Parameters" in tab_types:
            param_panel = ParameterPanelWidget(self)
            new_tab_widget.addTab(param_panel, "⚙️ Parameters")
            # Update reference to the main parameter panel
            self.param_panel = param_panel
            # Connect signals
            param_panel.request_load_parameters.connect(self._start_parameter_loading)
            param_panel.request_set_parameter.connect(self._handle_parameter_set)
            # NOTE: Don't auto-load parameters - user clicks "Load Parameters" button
        
        if "Commands" in tab_types:
            commands_widget = self._build_commands_tab()
            new_tab_widget.addTab(commands_widget, "🎮 Commands")
        
        self.main_splitter.addWidget(new_tab_widget)
        self.tab_widget = new_tab_widget
        
        logger.info("Merged all tabs into single view")
        self.statusBar().showMessage("All tabs merged - fresh instances created", 2000)
    
    def _build_control_panel(self, host: str, port: int) -> QtWidgets.QWidget:
        """Build control panel with connection and settings."""
        panel = QtWidgets.QGroupBox("Control Panel")
        layout = QtWidgets.QHBoxLayout(panel)
        
        # Connection controls
        layout.addWidget(QtWidgets.QLabel("Host:"))
        self.host_edit = QtWidgets.QLineEdit(host)
        self.host_edit.setMaximumWidth(120)
        layout.addWidget(self.host_edit)
        
        layout.addWidget(QtWidgets.QLabel("Port:"))
        self.port_spin = QtWidgets.QSpinBox()
        self.port_spin.setRange(1, 65535)
        self.port_spin.setValue(port)
        self.port_spin.setMaximumWidth(70)
        layout.addWidget(self.port_spin)
        
        self.connect_button = QtWidgets.QPushButton("Connect")
        self.connect_button.clicked.connect(self._on_connect_clicked)
        layout.addWidget(self.connect_button)
        
        self.disconnect_button = QtWidgets.QPushButton("Disconnect")
        self.disconnect_button.clicked.connect(self._on_disconnect_clicked)
        self.disconnect_button.setEnabled(False)
        layout.addWidget(self.disconnect_button)
        
        # Separator
        separator1 = QtWidgets.QFrame()
        separator1.setFrameShape(QtWidgets.QFrame.VLine)
        separator1.setFrameShadow(QtWidgets.QFrame.Sunken)
        layout.addWidget(separator1)
        
        # Telemetry/Oscilloscope controls
        layout.addWidget(QtWidgets.QLabel("Telemetry:"))
        self.start_telemetry_button = QtWidgets.QPushButton("▶ Start")
        self.start_telemetry_button.clicked.connect(self._on_start_telemetry)
        self.start_telemetry_button.setEnabled(False)
        self.start_telemetry_button.setStyleSheet("""
            QPushButton {
                background-color: #10b981;
                color: white;
                font-weight: bold;
                border-radius: 6px;
                padding: 8px 16px;
            }
            QPushButton:hover {
                background-color: #059669;
            }
            QPushButton:disabled {
                background-color: #d1d5db;
            }
        """)
        layout.addWidget(self.start_telemetry_button)
        
        self.stop_telemetry_button = QtWidgets.QPushButton("⏸ Stop")
        self.stop_telemetry_button.clicked.connect(self._on_stop_telemetry)
        self.stop_telemetry_button.setEnabled(False)
        self.stop_telemetry_button.setStyleSheet("""
            QPushButton {
                background-color: #ef4444;
                color: white;
                font-weight: bold;
                border-radius: 6px;
                padding: 8px 16px;
            }
            QPushButton:hover {
                background-color: #dc2626;
            }
            QPushButton:disabled {
                background-color: #d1d5db;
            }
        """)
        layout.addWidget(self.stop_telemetry_button)
        
        # Telemetry status indicator
        self.telemetry_status_label = QtWidgets.QLabel("●")
        self.telemetry_status_label.setStyleSheet("color: #9ca3af; font-size: 16px; font-weight: bold;")
        self.telemetry_status_label.setToolTip("Telemetry status: Gray=Idle, Green=Active, Red=Stalled")
        layout.addWidget(self.telemetry_status_label)
        
        # Separator
        separator2 = QtWidgets.QFrame()
        separator2.setFrameShape(QtWidgets.QFrame.VLine)
        separator2.setFrameShadow(QtWidgets.QFrame.Sunken)
        layout.addWidget(separator2)
        
        # Time/sample window control
        self.window_label = QtWidgets.QLabel("Window (samples):")
        layout.addWidget(self.window_label)
        self.window_spin = QtWidgets.QDoubleSpinBox()
        # Both modes now use samples
        self.window_spin.setDecimals(0)
        self.window_spin.setRange(10, 20000)
        if self.simple_mode:
            self.window_spin.setValue(float(self.simple_sample_window))
        else:
            self.window_spin.setValue(float(self.sample_window))
        self.window_spin.setSingleStep(100)
        self.window_spin.setMaximumWidth(90)
        self.window_spin.valueChanged.connect(self._on_window_changed)
        layout.addWidget(self.window_spin)
        
        # Clear button
        self.clear_button = QtWidgets.QPushButton("Clear Data")
        self.clear_button.clicked.connect(self._on_clear_clicked)
        layout.addWidget(self.clear_button)
        
        # Separator
        separator3 = QtWidgets.QFrame()
        separator3.setFrameShape(QtWidgets.QFrame.VLine)
        separator3.setFrameShadow(QtWidgets.QFrame.Sunken)
        layout.addWidget(separator3)
        
        # Statistics
        layout.addWidget(QtWidgets.QLabel("Packets:"))
        self.packets_label = QtWidgets.QLabel("0")
        layout.addWidget(self.packets_label)
        
        layout.addWidget(QtWidgets.QLabel("Loss:"))
        self.loss_label = QtWidgets.QLabel("0.0%")
        layout.addWidget(self.loss_label)
        
        layout.addStretch()
        
        return panel
    
    def _build_dashboard(self) -> QtWidgets.QWidget:
        """Build dashboard panel with digital signal displays."""
        dashboard = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(dashboard)
        layout.setContentsMargins(15, 15, 15, 15)
        
        # Title
        title = QtWidgets.QLabel("Real-Time Telemetry Monitor")
        title.setStyleSheet("""
            QLabel {
                font-size: 16px;
                font-weight: 600;
                color: #1f2937;
                padding: 10px;
                background-color: #f0f9ff;
                border-left: 4px solid #3b82f6;
                border-radius: 6px;
            }
        """)
        title.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)
        layout.addWidget(title)
        
        # Scroll area for signal groups
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        
        signals_widget = QtWidgets.QWidget()
        signals_layout = QtWidgets.QVBoxLayout(signals_widget)
        
        # Create signal group displays
        self.signal_group_widgets = {}
        self._create_signal_group_displays(signals_layout)
        
        signals_layout.addStretch()
        scroll.setWidget(signals_widget)
        layout.addWidget(scroll)
        
        return dashboard
    
    def _create_signal_group_displays(self, parent_layout: QtWidgets.QVBoxLayout) -> None:
        """Create signal display widgets for all groups."""
        # Measurements group (compact)
        meas_widget = SignalGroupWidget('measurements', 'Grid & DC Measurements', self)
        meas_widget.add_signal_display('grid_voltage_magnitude_pu', 'V Mag', 'pu')
        meas_widget.add_signal_display('grid_current_magnitude_pu', 'I Grid Mag', 'pu')
        meas_widget.add_signal_display('dc_link_current_pu', 'I DC', 'pu')
        meas_widget.add_signal_display('is_commuting', 'Commuting', '')
        self.signal_group_widgets['measurements'] = meas_widget
        parent_layout.addWidget(meas_widget)
        
        # Observer group
        obs_widget = SignalGroupWidget('observer', 'Observer', self)
        obs_widget.add_signal_display('estimated_voltage_magnitude_pu', 'V Est', 'pu')
        obs_widget.add_signal_display('estimated_frequency_pu', 'f Est', 'pu')
        obs_widget.add_signal_display('estimated_angle_rad', 'θ Est', 'rad')
        self.signal_group_widgets['observer'] = obs_widget
        parent_layout.addWidget(obs_widget)
        
        # Modulator group
        mod_widget = SignalGroupWidget('modulator', 'Modulator', self)
        mod_widget.add_signal_display('is_enabled', 'Enabled', '')
        mod_widget.add_signal_display('firing_angle_input_pu', 'α In', 'pu')
        mod_widget.add_signal_display('firing_angle_output_pu', 'α Out', 'pu')
        self.signal_group_widgets['modulator'] = mod_widget
        parent_layout.addWidget(mod_widget)
        
        # Controller group
        ctrl_widget = SignalGroupWidget('controller', 'Controller', self)
        ctrl_widget.add_signal_display('dc_link_current_reference', 'I Ref', 'pu')
        ctrl_widget.add_signal_display('dc_link_current_measured', 'I Meas', 'pu')
        ctrl_widget.add_signal_display('current_error_pu', 'Error', 'pu')
        ctrl_widget.add_signal_display('controller_saturated', 'Saturated', '')
        self.signal_group_widgets['controller'] = ctrl_widget
        parent_layout.addWidget(ctrl_widget)
        
        # State machine group
        state_widget = SignalGroupWidget('state_machine', 'State Machine', self)
        state_widget.add_signal_display('current_state', 'State', '', enum_map=self.sfc_state_map)
        state_widget.add_signal_display('pulse_releases', 'Pulses', '')
        state_widget.add_signal_display('auxiliaries_are_on', 'Aux', '')
        state_widget.add_signal_display('breakers_are_on', 'Breakers', '')
        self.signal_group_widgets['state_machine'] = state_widget
        parent_layout.addWidget(state_widget)
    
    def _build_oscilloscope(self) -> QtWidgets.QWidget:
        """Build oscilloscope panel with real-time plots.

        In simple_mode we only allocate a single plot and a single signal to isolate
        possible UI freeze root causes (e.g., mass curve updates or legend overhead).
        """
        # Use pyqtgraph's GraphicsLayoutWidget
        self.graphics_widget = pg.GraphicsLayoutWidget()
        self.graphics_widget.setBackground('w')
        
        # Configure pyqtgraph for better performance
        pg.setConfigOptions(
            antialias=True,
            useOpenGL=False,  # OpenGL can cause hangs on some systems
            enableExperimental=False
        )
        
        if self.simple_mode:
            # Single plot & single curve for diagnostic isolation (sample index x-axis)
            self.plot1 = self.graphics_widget.addPlot(row=0, col=0, title="Simple Mode: grid_voltage_magnitude_pu")
            self.plot1.setLabel('left', 'Voltage', units='pu')
            self.plot1.setLabel('bottom', 'Sample')
            self.plot1.showGrid(x=True, y=True, alpha=0.3)
            self.plot1.enableAutoRange(x=False, y=True)
            self.plot1.setXRange(0.0, float(self.simple_sample_window), padding=0.02)
            self.simple_buffer = SampleBasedBuffer(capacity=self.simple_sample_window)
            self.simple_curve = self.plot1.plot(pen=pg.mkPen(color=(0, 120, 255), width=2), name='Vmag')
            self.plot_groups = {}  # Keep attribute for compatibility with other code paths
            return self.graphics_widget
        else:
            # Full mode now also uses sample-based x-axis
            # Subplot 1: Grid Measurements
            self.plot1 = self.graphics_widget.addPlot(row=0, col=0, title="Grid Measurements")
            self.plot1.setLabel('left', 'Voltage / Current', units='pu')
            self.plot1.setLabel('bottom', 'Sample')
            self.plot1.addLegend()
            self.plot1.showGrid(x=True, y=True, alpha=0.3)
            self.plot1.enableAutoRange(x=False, y=True)
            self.plot1.setXRange(0.0, float(self.sample_window), padding=0.02)

            self.plot_groups: Dict[str, SignalPlotGroup] = {}
            measurements_group = SignalPlotGroup('measurements', self.plot1, capacity=self.sample_window)
            measurements_group.add_signal('grid_voltage_magnitude_pu', 'V mag')
            measurements_group.add_signal('grid_voltage_a_pu', 'Va')
            measurements_group.add_signal('grid_voltage_b_pu', 'Vb')
            measurements_group.add_signal('grid_voltage_c_pu', 'Vc')
            measurements_group.add_signal('grid_current_magnitude_pu', 'I mag')
            measurements_group.add_signal('grid_current_a_pu', 'Ia')
            measurements_group.add_signal('grid_current_b_pu', 'Ib')
            measurements_group.add_signal('grid_current_c_pu', 'Ic')
            measurements_group.add_signal('dc_link_current_pu', 'Idc')
            self.plot_groups['measurements'] = measurements_group

            # Subplot 2: Observer
            self.plot2 = self.graphics_widget.addPlot(row=1, col=0, title="Grid Observer")
            self.plot2.setLabel('left', 'Magnitude / Frequency', units='pu')
            self.plot2.setLabel('bottom', 'Sample')
            self.plot2.addLegend()
            self.plot2.showGrid(x=True, y=True, alpha=0.3)
            self.plot2.enableAutoRange(x=False, y=True)

            observer_group = SignalPlotGroup('observer', self.plot2, capacity=self.sample_window)
            observer_group.add_signal('estimated_voltage_magnitude_pu', 'V est')
            observer_group.add_signal('estimated_frequency_pu', 'f est')
            self.plot_groups['observer'] = observer_group

            # Subplot 3: DC Link Controller
            self.plot3 = self.graphics_widget.addPlot(row=2, col=0, title="DC Link Current Controller")
            self.plot3.setLabel('left', 'Current / Error', units='pu')
            self.plot3.setLabel('bottom', 'Sample')
            self.plot3.addLegend()
            self.plot3.showGrid(x=True, y=True, alpha=0.3)
            self.plot3.enableAutoRange(x=False, y=True)

            controller_group = SignalPlotGroup('controller', self.plot3, capacity=self.sample_window)
            controller_group.add_signal('dc_link_current_reference', 'I ref')
            controller_group.add_signal('dc_link_current_measured', 'I meas')
            controller_group.add_signal('current_error_pu', 'Error')
            controller_group.add_signal('controller_output_pu', 'Output')
            self.plot_groups['controller'] = controller_group

            # Link x-axes
            self.plot2.setXLink(self.plot1)
            self.plot3.setXLink(self.plot1)

            return self.graphics_widget
    
    def _build_commands_tab(self) -> QtWidgets.QWidget:
        """Build commands tab with control checkboxes and test configurations."""
        commands_widget = QtWidgets.QWidget()
        main_layout = QtWidgets.QVBoxLayout(commands_widget)
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(15)
        
        # Rectifier Operation Section
        rectifier_group = QtWidgets.QGroupBox("Rectifier Operation")
        rectifier_group.setStyleSheet("QGroupBox { font-weight: bold; }")
        rectifier_layout = QtWidgets.QVBoxLayout()
        rectifier_layout.setSpacing(10)
        
        # Rectifier enable checkbox
        self.rectifier_enable_check = QtWidgets.QCheckBox("Enable Rectifier Operation")
        self.rectifier_enable_check.setStyleSheet("QCheckBox { font-size: 12px; }")
        self.rectifier_enable_check.stateChanged.connect(self._on_rectifier_enable_changed)
        rectifier_layout.addWidget(self.rectifier_enable_check)
        
        # DC-Link Current Reference
        ref_layout = QtWidgets.QHBoxLayout()
        ref_label = QtWidgets.QLabel("DC-Link Current Reference (pu):")
        ref_label.setMinimumWidth(220)
        self.dc_current_ref_spin = QtWidgets.QDoubleSpinBox()
        self.dc_current_ref_spin.setRange(0.0, 2.0)
        self.dc_current_ref_spin.setValue(0.0)
        self.dc_current_ref_spin.setSingleStep(0.05)
        self.dc_current_ref_spin.setDecimals(3)
        self.dc_current_ref_spin.setMinimumWidth(120)
        self.dc_current_ref_spin.setStyleSheet("QDoubleSpinBox { font-size: 12px; }")
        self.dc_current_ref_spin.valueChanged.connect(self._on_dc_current_ref_changed)
        
        apply_ref_button = QtWidgets.QPushButton("Apply")
        apply_ref_button.setMaximumWidth(80)
        apply_ref_button.clicked.connect(self._on_apply_dc_current_ref)
        
        ref_layout.addWidget(ref_label)
        ref_layout.addWidget(self.dc_current_ref_spin)
        ref_layout.addWidget(apply_ref_button)
        ref_layout.addStretch()
        rectifier_layout.addLayout(ref_layout)
        
        rectifier_group.setLayout(rectifier_layout)
        main_layout.addWidget(rectifier_group)
        
        # Testing Procedure Section
        testing_group = QtWidgets.QGroupBox("Testing Procedure")
        testing_group.setStyleSheet("QGroupBox { font-weight: bold; }")
        testing_layout = QtWidgets.QVBoxLayout()
        testing_layout.setSpacing(10)
        
        # Test configuration dropdown
        test_layout = QtWidgets.QHBoxLayout()
        test_label = QtWidgets.QLabel("Configure for test:")
        test_label.setMinimumWidth(220)
        self.test_config_combo = QtWidgets.QComboBox()
        self.test_config_combo.setMinimumWidth(300)
        self.test_config_combo.setStyleSheet("QComboBox { font-size: 12px; }")
        
        # Add test configurations
        self.test_config_combo.addItem("-- Select Test --")
        self.test_config_combo.addItem("Thyristor Pulsation Test")
        self.test_config_combo.addItem("Current Regulation Test")
        self.test_config_combo.addItem("Breaker Commands Test")
        self.test_config_combo.addItem("Grid Observer Validation Test")
        self.test_config_combo.addItem("Phase-Locked Loop Test")
        self.test_config_combo.addItem("Commutation Analysis Test")
        self.test_config_combo.addItem("Full System Integration Test")
        
        self.test_config_combo.currentIndexChanged.connect(self._on_test_config_changed)
        
        apply_test_button = QtWidgets.QPushButton("Configure Test")
        apply_test_button.setMaximumWidth(120)
        apply_test_button.clicked.connect(self._on_apply_test_config)
        
        test_layout.addWidget(test_label)
        test_layout.addWidget(self.test_config_combo)
        test_layout.addWidget(apply_test_button)
        test_layout.addStretch()
        testing_layout.addLayout(test_layout)
        
        # Test description area
        self.test_description_text = QtWidgets.QTextEdit()
        self.test_description_text.setReadOnly(True)
        self.test_description_text.setMaximumHeight(150)
        self.test_description_text.setStyleSheet("""
            QTextEdit {
                background-color: #f8f9fa;
                border: 1px solid #dee2e6;
                border-radius: 4px;
                padding: 8px;
                font-size: 11px;
            }
        """)
        self.test_description_text.setPlaceholderText("Test description will appear here...")
        testing_layout.addWidget(self.test_description_text)
        
        testing_group.setLayout(testing_layout)
        main_layout.addWidget(testing_group)
        
        # Status section
        status_group = QtWidgets.QGroupBox("Command Status")
        status_group.setStyleSheet("QGroupBox { font-weight: bold; }")
        status_layout = QtWidgets.QVBoxLayout()
        
        self.command_status_label = QtWidgets.QLabel("Ready - No commands active")
        self.command_status_label.setStyleSheet("""
            QLabel {
                padding: 10px;
                background-color: #e3f2fd;
                border-left: 4px solid #2196f3;
                border-radius: 4px;
                font-size: 11px;
            }
        """)
        status_layout.addWidget(self.command_status_label)
        
        status_group.setLayout(status_layout)
        main_layout.addWidget(status_group)
        
        main_layout.addStretch()
        
        return commands_widget
    
    def _on_rectifier_enable_changed(self, state: int) -> None:
        """Handle rectifier enable checkbox state change."""
        enabled = state == QtCore.Qt.Checked
        status = "enabled" if enabled else "disabled"
        self.command_status_label.setText(f"Rectifier operation {status}")
        logger.info(f"Rectifier operation {status}")
        # TODO: Send command to controller to enable/disable rectifier
    
    def _on_dc_current_ref_changed(self, value: float) -> None:
        """Handle DC current reference value change."""
        logger.debug(f"DC current reference changed to {value:.3f} pu")
    
    def _on_apply_dc_current_ref(self) -> None:
        """Apply DC current reference to controller."""
        value = self.dc_current_ref_spin.value()
        self.command_status_label.setText(f"DC-Link current reference set to {value:.3f} pu")
        logger.info(f"Applying DC current reference: {value:.3f} pu")
        # TODO: Send command to controller to set DC current reference
    
    def _on_test_config_changed(self, index: int) -> None:
        """Handle test configuration selection change."""
        if index == 0:
            self.test_description_text.clear()
            return
        
        test_name = self.test_config_combo.currentText()
        
        # Test descriptions
        descriptions = {
            "Thyristor Pulsation Test": """
<b>Thyristor Pulsation Test</b><br><br>
<b>Objective:</b> Verify thyristor firing and gate pulse generation.<br>
<b>Procedure:</b>
<ul>
<li>Enable single thyristor firing</li>
<li>Observe gate pulse patterns</li>
<li>Verify firing angle timing</li>
<li>Check for proper sequencing</li>
</ul>
<b>Expected Result:</b> Clean, consistent gate pulses at correct intervals.
            """,
            "Current Regulation Test": """
<b>Current Regulation Test</b><br><br>
<b>Objective:</b> Validate DC-link current controller performance.<br>
<b>Procedure:</b>
<ul>
<li>Set DC current reference (0.1-0.5 pu)</li>
<li>Enable rectifier operation</li>
<li>Observe current tracking</li>
<li>Verify settling time and overshoot</li>
</ul>
<b>Expected Result:</b> DC current follows reference with <5% steady-state error.
            """,
            "Breaker Commands Test": """
<b>Breaker Commands Test</b><br><br>
<b>Objective:</b> Test circuit breaker control and interlocks.<br>
<b>Procedure:</b>
<ul>
<li>Command breaker close</li>
<li>Verify status feedback</li>
<li>Test interlock conditions</li>
<li>Command breaker open</li>
</ul>
<b>Expected Result:</b> Breaker responds correctly with proper status indication.
            """,
            "Grid Observer Validation Test": """
<b>Grid Observer Validation Test</b><br><br>
<b>Objective:</b> Verify grid voltage and frequency estimation.<br>
<b>Procedure:</b>
<ul>
<li>Monitor estimated voltage magnitude</li>
<li>Monitor estimated frequency</li>
<li>Compare with actual measurements</li>
<li>Verify phase angle tracking</li>
</ul>
<b>Expected Result:</b> Estimated values match actual within 1% error.
            """,
            "Phase-Locked Loop Test": """
<b>Phase-Locked Loop Test</b><br><br>
<b>Objective:</b> Validate PLL lock and tracking performance.<br>
<b>Procedure:</b>
<ul>
<li>Observe PLL lock indicator</li>
<li>Monitor phase error</li>
<li>Test frequency step response</li>
<li>Verify lock time</li>
</ul>
<b>Expected Result:</b> PLL locks within 100ms with <1° steady-state phase error.
            """,
            "Commutation Analysis Test": """
<b>Commutation Analysis Test</b><br><br>
<b>Objective:</b> Analyze commutation process and overlap angle.<br>
<b>Procedure:</b>
<ul>
<li>Enable commutation monitoring</li>
<li>Observe overlap angle</li>
<li>Monitor commutation failures</li>
<li>Verify recovery behavior</li>
</ul>
<b>Expected Result:</b> Clean commutation with predictable overlap angle.
            """,
            "Full System Integration Test": """
<b>Full System Integration Test</b><br><br>
<b>Objective:</b> Comprehensive system validation under normal operation.<br>
<b>Procedure:</b>
<ul>
<li>Start system from idle state</li>
<li>Ramp DC current reference</li>
<li>Verify all subsystems</li>
<li>Test protection responses</li>
<li>Execute orderly shutdown</li>
</ul>
<b>Expected Result:</b> All subsystems operate correctly without faults.
            """
        }
        
        description = descriptions.get(test_name, "No description available.")
        self.test_description_text.setHtml(description)
    
    def _on_apply_test_config(self) -> None:
        """Apply selected test configuration."""
        index = self.test_config_combo.currentIndex()
        if index == 0:
            QtWidgets.QMessageBox.warning(self, "No Test Selected",
                                         "Please select a test configuration.")
            return
        
        test_name = self.test_config_combo.currentText()
        self.command_status_label.setText(f"Test configured: {test_name}")
        logger.info(f"Applying test configuration: {test_name}")
        # TODO: Send test configuration to controller
        
        QtWidgets.QMessageBox.information(self, "Test Configured",
                                         f"Test configuration '{test_name}' applied.\n\n"
                                         f"Monitor the telemetry and oscilloscope tabs for results.")
    
    def _apply_modern_style(self) -> None:
        """Apply modern QSS stylesheet to the application."""
        modern_style = """
        /* Main Window */
        QMainWindow {
            background-color: #f5f7fa;
        }
        
        /* Tab Widget */
        QTabWidget::pane {
            border: 1px solid #d1d5db;
            border-radius: 8px;
            background-color: white;
            top: -1px;
        }
        
        QTabBar::tab {
            background-color: #e5e7eb;
            color: #374151;
            padding: 10px 20px;
            margin-right: 4px;
            border-top-left-radius: 8px;
            border-top-right-radius: 8px;
            font-size: 13px;
            font-weight: 500;
            min-width: 120px;
        }
        
        QTabBar::tab:selected {
            background-color: white;
            color: #1f2937;
            font-weight: 600;
            border-bottom: 3px solid #3b82f6;
        }
        
        QTabBar::tab:hover:!selected {
            background-color: #f3f4f6;
        }
        
        /* Group Boxes */
        QGroupBox {
            background-color: white;
            border: 1px solid #e5e7eb;
            border-radius: 8px;
            margin-top: 12px;
            padding-top: 12px;
            font-size: 13px;
            font-weight: 600;
            color: #1f2937;
        }
        
        QGroupBox::title {
            subcontrol-origin: margin;
            subcontrol-position: top left;
            padding: 4px 12px;
            background-color: white;
            border-radius: 4px;
            color: #3b82f6;
        }
        
        /* Buttons */
        QPushButton {
            background-color: #3b82f6;
            color: white;
            border: none;
            border-radius: 6px;
            padding: 8px 16px;
            font-size: 12px;
            font-weight: 500;
            min-height: 32px;
        }
        
        QPushButton:hover {
            background-color: #2563eb;
        }
        
        QPushButton:pressed {
            background-color: #1d4ed8;
        }
        
        QPushButton:disabled {
            background-color: #9ca3af;
            color: #d1d5db;
        }
        
        /* Input Fields */
        QLineEdit, QSpinBox, QDoubleSpinBox {
            background-color: white;
            border: 1px solid #d1d5db;
            border-radius: 6px;
            padding: 6px 10px;
            font-size: 12px;
            color: #1f2937;
        }
        
        QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus {
            border: 2px solid #3b82f6;
            outline: none;
        }
        
        /* ComboBox */
        QComboBox {
            background-color: white;
            border: 1px solid #d1d5db;
            border-radius: 6px;
            padding: 6px 10px;
            font-size: 12px;
            color: #1f2937;
            min-height: 28px;
        }
        
        QComboBox:hover {
            border: 1px solid #9ca3af;
        }
        
        QComboBox:focus {
            border: 2px solid #3b82f6;
        }
        
        QComboBox::drop-down {
            border: none;
            width: 30px;
        }
        
        QComboBox::down-arrow {
            image: none;
            border-left: 4px solid transparent;
            border-right: 4px solid transparent;
            border-top: 6px solid #6b7280;
            margin-right: 8px;
        }
        
        QComboBox QAbstractItemView {
            background-color: white;
            border: 1px solid #d1d5db;
            border-radius: 6px;
            selection-background-color: #3b82f6;
            selection-color: white;
            padding: 4px;
        }
        
        /* CheckBox */
        QCheckBox {
            spacing: 8px;
            color: #1f2937;
            font-size: 12px;
        }
        
        QCheckBox::indicator {
            width: 18px;
            height: 18px;
            border: 2px solid #d1d5db;
            border-radius: 4px;
            background-color: white;
        }
        
        QCheckBox::indicator:hover {
            border-color: #3b82f6;
        }
        
        QCheckBox::indicator:checked {
            background-color: #3b82f6;
            border-color: #3b82f6;
            image: none;
        }
        
        /* Labels */
        QLabel {
            color: #374151;
            font-size: 12px;
        }
        
        /* Status Bar */
        QStatusBar {
            background-color: #f9fafb;
            border-top: 1px solid #e5e7eb;
            color: #6b7280;
            font-size: 11px;
        }
        
        /* Scroll Area */
        QScrollArea {
            border: none;
            background-color: transparent;
        }
        
        QScrollBar:vertical {
            background-color: #f3f4f6;
            width: 12px;
            border-radius: 6px;
        }
        
        QScrollBar::handle:vertical {
            background-color: #d1d5db;
            border-radius: 6px;
            min-height: 30px;
        }
        
        QScrollBar::handle:vertical:hover {
            background-color: #9ca3af;
        }
        
        QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
            height: 0px;
        }
        
        /* Text Edit */
        QTextEdit {
            background-color: #f9fafb;
            border: 1px solid #e5e7eb;
            border-radius: 6px;
            padding: 8px;
            font-size: 11px;
            color: #374151;
        }
        
        /* Splitter */
        QSplitter::handle {
            background-color: #d1d5db;
            margin: 1px 0px;
        }
        
        QSplitter::handle:hover {
            background-color: #3b82f6;
        }
        
        QSplitter::handle:horizontal {
            width: 8px;
        }
        
        QSplitter::handle:vertical {
            height: 8px;
        }
        """
        
        self.setStyleSheet(modern_style)
    
    def _on_connect_clicked(self) -> None:
        """Handle connect button click."""
        host = self.host_edit.text().strip()
        port = self.port_spin.value()
        
        # Update client host/port
        self.network_client.host = host
        self.network_client.port = port
        
        if self.network_client.connect():
            self.connect_button.setEnabled(False)
            self.disconnect_button.setEnabled(True)
            self.start_telemetry_button.setEnabled(True)
            self.statusBar().showMessage(f"Connected to {host}:{port} - Sending STOP to sync state...")
            logger.info(f"Connected to {host}:{port}")
            
            # CRITICAL FIX: Send STOP immediately on connect to sync server state
            # Server may be streaming from previous session - force it to stop
            QtCore.QTimer.singleShot(100, self._sync_server_state)
        else:
            QtWidgets.QMessageBox.critical(self, "Connection Failed",
                                          f"Unable to connect to {host}:{port}")
    
    def _on_disconnect_clicked(self) -> None:
        """Handle disconnect button click."""
        # Politely stop telemetry before closing connection
        if self.network_client.connected:
            self.network_client.stop_telemetry()
        self.network_client.disconnect()
        
        # Reset UI state
        self.telemetry_streaming = False
        self.connect_button.setEnabled(True)
        self.disconnect_button.setEnabled(False)
        self.start_telemetry_button.setEnabled(False)
        self.stop_telemetry_button.setEnabled(False)
        self.telemetry_status_label.setStyleSheet("color: #9ca3af; font-size: 16px; font-weight: bold;")
        self.statusBar().showMessage("Disconnected")
        logger.info("Disconnected")
    
    def _sync_server_state(self) -> None:
        """Synchronize server telemetry state on connect.
        
        CRITICAL: Server may be streaming from previous session.
        Send STOP to ensure clean state, then let user manually START.
        """
        logger.info("Syncing server state: sending STOP command")
        
        # Send STOP to ensure server is in known state
        if self.network_client.stop_telemetry():
            self.telemetry_streaming = False
            self.start_telemetry_button.setEnabled(True)
            self.stop_telemetry_button.setEnabled(False)
            self.telemetry_status_label.setStyleSheet("color: #9ca3af; font-size: 16px; font-weight: bold;")
            
            # Clear any stale data from previous session
            if hasattr(self, 'plot_groups') and self.plot_groups:
                for group in self.plot_groups.values():
                    group.clear_all()
                self.start_time = None
                logger.info("Cleared oscilloscope data during sync")
            
            # Clear packet queue
            self._packet_queue.clear()
            
            self.statusBar().showMessage("Connected and synchronized - Click 'Start' to begin telemetry")
            logger.info("Server state synchronized - ready for user to start telemetry")
        else:
            self.statusBar().showMessage("Connected but failed to sync server state")
            logger.warning("Failed to send STOP during state sync")
    
    def _on_start_telemetry(self) -> None:
        """Start telemetry streaming and oscilloscope updates."""
        # Reset sample index
        self.global_sample_index = 0
        
        # Clear oscilloscope data before starting to avoid stale/corrupted data
        if hasattr(self, 'plot_groups') and self.plot_groups:
            for group in self.plot_groups.values():
                group.clear_all()
            self.start_time = None
            logger.info("Cleared oscilloscope data before START")
        if self.simple_mode and hasattr(self, 'simple_buffer'):
            self.simple_buffer.clear()
            if hasattr(self, 'plot1'):
                self.plot1.setXRange(0.0, float(self.simple_sample_window), padding=0.02)
            logger.info("Cleared simple-mode buffer before START")
        
        if self.network_client.start_telemetry():
            self.telemetry_streaming = True
            self.last_packet_time = 0.0
            self.start_telemetry_button.setEnabled(False)
            self.stop_telemetry_button.setEnabled(True)
            self.statusBar().showMessage("Telemetry START command sent - waiting for data...")
            logger.info("Telemetry START command sent")
        else:
            QtWidgets.QMessageBox.warning(self, "Command Failed", 
                                         "Failed to send START command. Check connection.")
    
    def _on_stop_telemetry(self) -> None:
        """Stop telemetry streaming and oscilloscope updates."""
        # Reset sample index
        self.global_sample_index = 0
        
        if self.network_client.stop_telemetry():
            self.telemetry_streaming = False
            self.start_telemetry_button.setEnabled(True)
            self.stop_telemetry_button.setEnabled(False)
            self.telemetry_status_label.setStyleSheet("color: #9ca3af; font-size: 16px; font-weight: bold;")
            
            # Clear oscilloscope data after stopping to avoid displaying stale data
            if hasattr(self, 'plot_groups') and self.plot_groups:
                for group in self.plot_groups.values():
                    group.clear_all()
                self.start_time = None
                logger.info("Cleared oscilloscope data after STOP")
            if self.simple_mode and hasattr(self, 'simple_buffer'):
                self.simple_buffer.clear()
                if hasattr(self, 'plot1'):
                    self.plot1.setXRange(0.0, float(self.simple_sample_window), padding=0.02)
                logger.info("Cleared simple-mode buffer after STOP")
            
            self.statusBar().showMessage("Telemetry streaming stopped")
            logger.info("Telemetry STOP command sent")
        else:
            QtWidgets.QMessageBox.warning(self, "Command Failed",
                                         "Failed to send STOP command. Check connection.")
    
    def _on_connection_changed(self, connected: bool) -> None:
        """Handle connection status change from network client."""
        if not connected:
            # Connection lost
            self.connect_button.setEnabled(True)
            self.disconnect_button.setEnabled(False)
            self.start_telemetry_button.setEnabled(False)
            self.stop_telemetry_button.setEnabled(False)
            self.statusBar().showMessage("Connection lost")
    
    def _on_network_error(self, error_msg: str) -> None:
        """Handle network error from unified client."""
        logger.error(f"Network error: {error_msg}")
        QtWidgets.QMessageBox.warning(self, "Network Error", error_msg)
    
    def _start_parameter_loading(self) -> None:
        """Start loading parameters from controller."""
        if not self.network_client.connected:
            self.param_panel._on_load_error("Not connected to controller")
            return
        
        logger.info("Starting parameter discovery")
        self.param_panel._on_load_progress("Requesting parameter count...")
        
        # Clear oscilloscope data before parameter loading to prevent timing issues
        if hasattr(self, 'plot_groups') and self.plot_groups:
            for group in self.plot_groups.values():
                group.clear_all()
            self.start_time = None
            logger.info("Cleared oscilloscope data before parameter loading")
        
        # Send COUNT request
        if self.network_client.send_parameter_count_request():
            # Response will come via signal
            pass
        else:
            self.param_panel._on_load_error("Failed to send COUNT request")
    
    def _handle_parameter_set(self, component: str, name: str, value: float) -> None:
        """Handle parameter SET request from parameter panel."""
        if not self.network_client.connected:
            self.param_panel.on_parameter_set_complete(component, name, False, "Not connected")
            return
        
        logger.info(f"Setting parameter {component}.{name} = {value}")
        
        # Send SET request
        if self.network_client.send_parameter_set_request(component, name, value):
            # Response will come via signal
            pass
        else:
            self.param_panel.on_parameter_set_complete(component, name, False, "Failed to send request")
    
    def _on_parameter_response(self, response_data: dict) -> None:
        """Handle parameter response from network client."""
        response_type = response_data.get('type')
        
        if response_type == 'count_response':
            total = response_data['total_parameters']
            logger.info(f"Controller has {total} parameters")
            self.param_panel._on_load_progress(f"Loading {total} parameters...")
            
            # Start loading all parameters by index
            self._pending_param_requests = []
            self._parameter_cache = {}
            for i in range(total):
                self._pending_param_requests.append(i)
            
            # Start loading first batch
            self._load_next_parameter()
        
        elif response_type == 'get_by_index_response':
            index = response_data.get('index')
            metadata = response_data.get('metadata')
            
            if metadata:  # SUCCESS
                # Store parameter
                key = f"{metadata.component}.{metadata.name}"
                self._parameter_cache[key] = metadata
                
                logger.debug(f"Loaded parameter {index}: {metadata.component}.{metadata.name}")
                
                # Load next parameter
                self._load_next_parameter()
            else:
                logger.warning(f"Failed to load parameter at index {index}")
                # Continue with next parameter
                self._load_next_parameter()
        
        elif response_type == 'set_response':
            component = response_data['component']
            name = response_data['name']
            status = response_data['status']
            
            if status == 0:  # SUCCESS
                self.param_panel.on_parameter_set_complete(component, name, True)
            else:
                error_msg = response_data.get('error_message', 'Unknown error')
                self.param_panel.on_parameter_set_complete(component, name, False, error_msg)
    
    def _load_next_parameter(self) -> None:
        """Load next parameter from pending queue."""
        if not self._pending_param_requests:
            # All parameters loaded
            parameters = list(self._parameter_cache.values())
            logger.info(f"Parameter loading complete: {len(parameters)} parameters")
            self.param_panel.display_parameters(parameters)
            
            # Clear oscilloscope data to prevent timing corruption
            # Parameter loading disrupts telemetry timing, causing data to overlap on plots
            if hasattr(self, 'plot_groups') and self.plot_groups:
                logger.info("Clearing oscilloscope data after parameter loading")
                for group in self.plot_groups.values():
                    group.clear_all()
                self.start_time = None
            return
        
        # Get next index
        index = self._pending_param_requests.pop(0)
        
        # Send request
        if not self.network_client.send_parameter_get_by_index_request(index):
            logger.error(f"Failed to send GET_BY_INDEX request for index {index}")
            # Try next parameter
            self._load_next_parameter()
        
        # Process events periodically during parameter loading to keep UI responsive
        if index % 10 == 0:
            QtCore.QCoreApplication.processEvents(QtCore.QEventLoop.AllEvents, 5)
    
    def _on_window_changed(self, value: float) -> None:
        """Handle window change (now sample-based for all modes)."""
        new_samples = max(10, int(value))
        
        if self.simple_mode:
            if new_samples != self.simple_sample_window:
                self.simple_sample_window = new_samples
                if hasattr(self, 'simple_buffer'):
                    self.simple_buffer.resize(self.simple_sample_window)
                # Adjust axis range based on current data
                if hasattr(self, 'simple_buffer') and hasattr(self, 'plot1'):
                    inds, _ = self.simple_buffer.get_data()
                    if inds.size:
                        latest = inds[-1]
                        start = max(0.0, latest - self.simple_sample_window)
                        self.plot1.setXRange(start, latest, padding=0.02)
                    else:
                        self.plot1.setXRange(0.0, float(self.simple_sample_window), padding=0.02)
                logger.info(f"Simple-mode sample window changed to {self.simple_sample_window}")
        else:
            if new_samples != self.sample_window:
                self.sample_window = new_samples
                if hasattr(self, 'plot_groups') and self.plot_groups:
                    for group in self.plot_groups.values():
                        group.resize_buffers(self.sample_window)
                # Adjust axis range based on current data
                if hasattr(self, 'plot_groups') and hasattr(self, 'plot1'):
                    latest_idx = None
                    for group in self.plot_groups.values():
                        idx = group.get_latest_sample_index()
                        if idx is not None:
                            latest_idx = idx if latest_idx is None else max(latest_idx, idx)
                    if latest_idx is not None:
                        start = max(0.0, latest_idx - self.sample_window)
                        self.plot1.setXRange(start, latest_idx, padding=0.02)
                    else:
                        self.plot1.setXRange(0.0, float(self.sample_window), padding=0.02)
                logger.info(f"Full-mode sample window changed to {self.sample_window}")
    
    def _on_clear_clicked(self) -> None:
        """Handle clear data button click."""
        # Reset sample index
        self.global_sample_index = 0
        
        if hasattr(self, 'plot_groups') and self.plot_groups:
            for group in self.plot_groups.values():
                group.clear_all()
            self.start_time = None
            if hasattr(self, 'plot1'):
                self.plot1.setXRange(0.0, float(self.sample_window), padding=0.02)
            logger.info("Data cleared")
        else:
            logger.warning("No oscilloscope data to clear (plot_groups not initialized)")
        if self.simple_mode and hasattr(self, 'simple_buffer'):
            self.simple_buffer.clear()
            if hasattr(self, 'plot1'):
                self.plot1.setXRange(0.0, float(self.simple_sample_window), padding=0.02)
            logger.info("Simple-mode data cleared")
    
    def _on_packet_received(self, packet: GridSideTelemetryPacket) -> None:
        """Callback when telemetry packet received - apply decimation and queue for processing."""
        try:
            # Update last packet time for watchdog
            import time
            self.last_packet_time = time.time()
            
            # Log first packet
            if not hasattr(self, '_first_packet_logged'):
                logger.info(f"FIRST PACKET RECEIVED (seq={packet.header.sequence_number}, timestamp={packet.header.timestamp_ms})")
                self._first_packet_logged = True
            
            # Apply decimation filter
            self.packet_counter += 1
            if self.packet_counter % self.decimate_factor != 0:
                return  # Skip this packet
            
            # Log accepted packets periodically
            if self.verbose and (self.packet_counter // self.decimate_factor) % 50 == 1:
                logger.debug(f"Accepted packet {self.packet_counter // self.decimate_factor} (raw count: {self.packet_counter})")
            
            # Queue packet for deferred processing to avoid blocking network thread
            self._packet_queue.append(packet)
            
            # More balanced queue management - only drop when seriously overloaded
            queue_size = len(self._packet_queue)
            if queue_size > 100:
                # Severe overflow - keep newest 50 packets
                dropped = queue_size - 50
                self._packet_queue = self._packet_queue[-50:]
                logger.error(f"SEVERE QUEUE OVERFLOW: Dropped {dropped} packets, kept 50 newest (decimate={self.decimate_factor})")
            elif queue_size > 60:
                # Moderate overflow - drop oldest quarter to catch up gradually
                drop_count = queue_size // 4
                self._packet_queue = self._packet_queue[drop_count:]
                logger.warning(f"Queue overflow: Dropped {drop_count} oldest packets (queue was {queue_size})")
            elif self.verbose and queue_size > 30:
                logger.debug(f"Packet queue growing: {queue_size} packets pending")
                
        except Exception as e:
            logger.error(f"Error in _on_packet_received: {e}", exc_info=True)
    
    def _process_packet_queue(self) -> None:
        """Process queued telemetry packets in batches with error handling."""
        # Heartbeat logging
        self._packet_timer_fire_count += 1
        if self.verbose and self._packet_timer_fire_count % 500 == 0:
            logger.debug(f"Packet timer alive: {self._packet_timer_fire_count} fires, queue size: {len(self._packet_queue)}")
        
        if not self._packet_queue:
            return
        
        try:
            t0 = perf_counter()
            queue_size_before = len(self._packet_queue)
            
            # Process up to 10 packets per cycle for better throughput
            batch_size = min(10, queue_size_before)
            packets_to_process = self._packet_queue[:batch_size]
            self._packet_queue = self._packet_queue[batch_size:]

            for i, packet in enumerate(packets_to_process):
                try:
                    self._process_telemetry_packet(packet)
                except Exception as e:
                    logger.error(f"Error processing packet {i+1}/{batch_size}: {e}", exc_info=True)

            queue_size_after = len(self._packet_queue)
            
            # Log queue health
            if queue_size_after > 80:
                logger.warning(f"Packet queue severely backed up: {queue_size_after} packets pending (processed {batch_size})")
            elif self.verbose and queue_size_after > 20:
                logger.debug(f"Packet queue: {queue_size_after} pending (processed {batch_size})")
            
            processing_time = perf_counter() - t0
            self._record_packet_batch_time(processing_time)
            
            if self.verbose and processing_time > 0.015:  # > 15ms is slow
                logger.warning(f"Slow packet batch: {processing_time*1000:.1f}ms for {batch_size} packets")
                
        except Exception as e:
            logger.error(f"Critical error in _process_packet_queue: {e}", exc_info=True)
    
    def _process_telemetry_packet(self, packet: GridSideTelemetryPacket) -> None:
        """Process a single telemetry packet with comprehensive error handling and logging."""
        try:
            # Use sample index for ALL modes (not time)
            sample_idx = float(self.global_sample_index)
            
            # Update measurements
            if packet.measurements:
                try:
                    meas = packet.measurements
                    self.latest_values['measurements']['grid_voltage_magnitude_pu'] = meas.grid_voltage_magnitude_pu
                    self.latest_values['measurements']['grid_current_magnitude_pu'] = meas.grid_current_magnitude_pu
                    self.latest_values['measurements']['dc_link_current_pu'] = meas.dc_link_current_pu
                    self.latest_values['measurements']['is_commuting'] = float(meas.is_commuting)

                    if self.simple_mode:
                        # Only one signal in simple diagnostic mode
                        if hasattr(self, 'simple_buffer'):
                            self.simple_buffer.append(sample_idx, meas.grid_voltage_magnitude_pu)
                    else:
                        # Full mode - all measurement signals
                        if hasattr(self, 'plot_groups') and 'measurements' in self.plot_groups:
                            pgm = self.plot_groups['measurements']
                            pgm.update_signal('grid_voltage_magnitude_pu', sample_idx, meas.grid_voltage_magnitude_pu)
                            pgm.update_signal('grid_voltage_a_pu', sample_idx, meas.grid_voltage_a_pu)
                            pgm.update_signal('grid_voltage_b_pu', sample_idx, meas.grid_voltage_b_pu)
                            pgm.update_signal('grid_voltage_c_pu', sample_idx, meas.grid_voltage_c_pu)
                            pgm.update_signal('grid_current_magnitude_pu', sample_idx, meas.grid_current_magnitude_pu)
                            pgm.update_signal('grid_current_a_pu', sample_idx, meas.grid_current_a_pu)
                            pgm.update_signal('grid_current_b_pu', sample_idx, meas.grid_current_b_pu)
                            pgm.update_signal('grid_current_c_pu', sample_idx, meas.grid_current_c_pu)
                            pgm.update_signal('dc_link_current_pu', sample_idx, meas.dc_link_current_pu)
                except Exception as e:
                    logger.error(f"Error processing measurements at sample {sample_idx}: {e}", exc_info=True)
            
            # Update observer (skip in simple mode for performance)
            if not self.simple_mode and packet.observer:
                try:
                    obs = packet.observer
                    self.latest_values['observer']['estimated_voltage_magnitude_pu'] = obs.estimated_voltage_magnitude_pu
                    self.latest_values['observer']['estimated_frequency_pu'] = obs.estimated_frequency_pu
                    self.latest_values['observer']['estimated_angle_rad'] = obs.estimated_angle_rad
                    if hasattr(self, 'plot_groups') and 'observer' in self.plot_groups:
                        og = self.plot_groups['observer']
                        og.update_signal('estimated_voltage_magnitude_pu', sample_idx, obs.estimated_voltage_magnitude_pu)
                        og.update_signal('estimated_frequency_pu', sample_idx, obs.estimated_frequency_pu)
                except Exception as e:
                    logger.error(f"Error processing observer at sample {sample_idx}: {e}", exc_info=True)
            
            # Update modulator
            if packet.modulator:
                try:
                    mod = packet.modulator
                    self.latest_values['modulator']['is_enabled'] = float(mod.is_enabled)
                    self.latest_values['modulator']['firing_angle_input_pu'] = mod.firing_angle_input_pu
                    self.latest_values['modulator']['firing_angle_output_pu'] = mod.firing_angle_output_pu
                except Exception as e:
                    logger.error(f"Error processing modulator at sample {sample_idx}: {e}", exc_info=True)
            
            # Update controller (skip in simple mode for performance)
            if not self.simple_mode and packet.controller:
                try:
                    ctrl = packet.controller
                    self.latest_values['controller']['dc_link_current_reference'] = ctrl.dc_link_current_reference
                    self.latest_values['controller']['dc_link_current_measured'] = ctrl.dc_link_current_measured
                    self.latest_values['controller']['current_error_pu'] = ctrl.current_error_pu
                    self.latest_values['controller']['controller_saturated'] = float(ctrl.controller_saturated)
                    if hasattr(self, 'plot_groups') and 'controller' in self.plot_groups:
                        cg = self.plot_groups['controller']
                        cg.update_signal('dc_link_current_reference', sample_idx, ctrl.dc_link_current_reference)
                        cg.update_signal('dc_link_current_measured', sample_idx, ctrl.dc_link_current_measured)
                        cg.update_signal('current_error_pu', sample_idx, ctrl.current_error_pu)
                        cg.update_signal('controller_output_pu', sample_idx, ctrl.controller_output_pu)
                except Exception as e:
                    logger.error(f"Error processing controller at sample {sample_idx}: {e}", exc_info=True)
            
            # Update state machine
            if packet.state_machine:
                try:
                    state = packet.state_machine
                    self.latest_values['state_machine']['current_state'] = float(state.current_state)
                    self.latest_values['state_machine']['pulse_releases'] = float(state.pulse_releases)
                    self.latest_values['state_machine']['auxiliaries_are_on'] = float(state.auxiliaries_are_on)
                    self.latest_values['state_machine']['breakers_are_on'] = float(state.breakers_are_on)
                except Exception as e:
                    logger.error(f"Error processing state machine at sample {sample_idx}: {e}", exc_info=True)
            
            # Increment sample counter for next packet
            self.global_sample_index += 1
            
        except Exception as e:
            logger.error(f"Critical error in _process_telemetry_packet: {e}", exc_info=True)
    
    def _update_oscilloscope(self) -> None:
        """Update oscilloscope plots with comprehensive error handling and logging."""
        t0 = perf_counter()
        
        try:
            if self.simple_mode:
                # Simple mode: single signal plot
                if not hasattr(self, 'simple_buffer'):
                    logger.warning("Simple mode but simple_buffer not initialized")
                    return
                
                try:
                    indices, values = self.simple_buffer.get_data()
                    if len(indices) > 0 and len(values) > 0:
                        if len(indices) != len(values):
                            logger.error(f"Array length mismatch: indices={len(indices)}, values={len(values)}")
                            return
                        
                        self.simple_curve.setData(indices, values, skipFiniteCheck=True, copy=False)
                        latest_idx = indices[-1]
                        start = max(0.0, latest_idx - self.simple_sample_window)
                        self.plot1.setXRange(start, latest_idx, padding=0.02)
                    elif self.verbose and len(indices) == 0:
                        logger.debug("Simple mode: no data yet")
                except Exception as e:
                    logger.error(f"Simple mode plot update failed: {e}", exc_info=True)
                
                self._record_plot_update_time(perf_counter() - t0)
                return

            # Full mode: multiple signal groups
            if not hasattr(self, 'plot_groups') or not self.plot_groups:
                logger.warning("Full mode but plot_groups not initialized")
                return

            try:
                latest_sample_idx = None
                total_updated = 0
                
                for group_name, group in self.plot_groups.items():
                    try:
                        updated_count = group.refresh_plots()
                        total_updated += updated_count
                        
                        group_latest = group.get_latest_sample_index()
                        if group_latest is not None:
                            if latest_sample_idx is None or group_latest > latest_sample_idx:
                                latest_sample_idx = group_latest
                    except Exception as e:
                        logger.error(f"Error refreshing plot group '{group_name}': {e}", exc_info=True)

                # Update x-axis range if we have data
                if latest_sample_idx is not None and hasattr(self, 'plot1'):
                    try:
                        start = max(0.0, latest_sample_idx - self.sample_window)
                        self.plot1.setXRange(start, latest_sample_idx, padding=0.02)
                    except Exception as e:
                        logger.error(f"Error setting x-axis range: {e}", exc_info=True)
                
                if self.verbose and total_updated > 0:
                    logger.debug(f"Updated {total_updated} signals, latest sample: {latest_sample_idx}")
                    
            except Exception as e:
                logger.error(f"Full mode plot update failed: {e}", exc_info=True)
            
            self._record_plot_update_time(perf_counter() - t0)
            
        except Exception as e:
            logger.error(f"Critical error in _update_oscilloscope: {e}", exc_info=True)
    
    def _update_dashboard(self) -> None:
        """Update dashboard displays (less frequent than oscilloscope)."""
        # Update dashboard displays
        if hasattr(self, 'signal_group_widgets'):
            for group_name, group_widget in self.signal_group_widgets.items():
                if group_name in self.latest_values:
                    group_widget.update_all_signals(self.latest_values[group_name])
    
    def _update_statistics(self) -> None:
        """Update statistics display."""
        stats = self.network_client.get_statistics()
        self.packets_label.setText(str(stats['packets_received']))
        self.loss_label.setText(f"{stats['loss_rate_percent']:.2f}%")
    
    def _check_telemetry_health(self) -> None:
        """Watchdog to check if telemetry is active and not stalled."""
        if not self.telemetry_streaming:
            # Not expecting telemetry - gray indicator
            self.telemetry_status_label.setStyleSheet("color: #9ca3af; font-size: 16px; font-weight: bold;")
            return
        
        import time
        current_time = time.time()
        
        if self.last_packet_time == 0.0:
            # Waiting for first packet - yellow indicator
            self.telemetry_status_label.setStyleSheet("color: #f59e0b; font-size: 16px; font-weight: bold;")
            self.telemetry_status_label.setToolTip("Telemetry: Waiting for data...")
        elif (current_time - self.last_packet_time) > 2.0:
            # No packets for 2 seconds - red indicator (stalled)
            self.telemetry_status_label.setStyleSheet("color: #ef4444; font-size: 16px; font-weight: bold;")
            self.telemetry_status_label.setToolTip(f"Telemetry: STALLED! Last packet {current_time - self.last_packet_time:.1f}s ago")
            logger.warning(f"Telemetry appears stalled - no packets for {current_time - self.last_packet_time:.1f}s")
        else:
            # Receiving packets normally - green indicator
            self.telemetry_status_label.setStyleSheet("color: #10b981; font-size: 16px; font-weight: bold;")
            self.telemetry_status_label.setToolTip("Telemetry: Active and receiving data")
    
    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        """Handle window close event."""
        logger.info("Window closing - stopping timers and disconnecting...")
        
        # Stop all timers first to prevent further processing
        if hasattr(self, 'plot_update_timer'):
            self.plot_update_timer.stop()
        if hasattr(self, 'dashboard_update_timer'):
            self.dashboard_update_timer.stop()
        if hasattr(self, 'stats_timer'):
            self.stats_timer.stop()
        if hasattr(self, 'watchdog_timer'):
            self.watchdog_timer.stop()
        if hasattr(self, '_packet_process_timer'):
            self._packet_process_timer.stop()
        
        # Clear packet queue to prevent processing during shutdown
        if hasattr(self, '_packet_queue'):
            self._packet_queue.clear()
        
        # Disconnect from network (non-blocking)
        if hasattr(self, 'network_client') and self.network_client.connected:
            try:
                self.network_client.stop_telemetry()
                # Set a short timeout for disconnect
                self.network_client.disconnect()
            except Exception as e:
                logger.warning(f"Error during disconnect: {e}")
        
        logger.info("Telemetry monitor closed")
        event.accept()


def parse_args() -> argparse.Namespace:
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="SFC Monitor - Combined Telemetry & Parameter Configuration"
    )
    parser.add_argument('--host', default='192.168.2.100', help='Controller IP address')
    parser.add_argument('--port', type=int, default=7000, help='Controller TCP port')
    parser.add_argument('--window', type=float, default=DEFAULT_TIME_WINDOW_SEC,
                       help='Oscilloscope time window in seconds')
    parser.add_argument('--verbose', action='store_true', help='Enable verbose logging')
    parser.add_argument('--simple', action='store_true', help='Enable simple diagnostic mode (single plotted signal)')
    parser.add_argument('--simple-samples', type=int, default=1500, help='Visible sample window in simple mode')
    parser.add_argument('--profile', action='store_true', help='Enable performance instrumentation logging')
    parser.add_argument('--decimate', type=int, default=1, 
                       help='Process only 1 out of N packets (reduces CPU load for high-rate targets)')
    return parser.parse_args()


def main() -> int:
    """Main application entry point."""
    args = parse_args()
    
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle('Fusion')
    
    window = TelemetryMonitorWindow(
        host=args.host,
        port=args.port,
        time_window_sec=args.window,
        verbose=args.verbose,
        simple_mode=args.simple,
        profile=args.profile,
        simple_sample_window=args.simple_samples,
        decimate=args.decimate
    )
    window.show()
    
    logger.info("Application started")
    return app.exec_()


if __name__ == '__main__':
    sys.exit(main())
