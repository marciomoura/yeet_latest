#!/usr/bin/env python3
"""Minimal oscilloscope with telemetry and parameter configuration.

Architecture:
- Left panel: Parameter list with load button and double-click to edit
- Center: Four subplots (voltage, currents, observer, DC link control)
- Right panel: Telemetry dashboard
- Python deque for data (automatic FIFO, no ring buffer math)
- Direct packet processing (no queues, no deferred work)
- Two timers: 100ms for plots, 200ms for dashboard
- Optional data persistence for offline analysis and export
"""

import sys
import logging
import time
import traceback
import json
from collections import deque
from pathlib import Path
from typing import Optional, List, Dict

import numpy as np
from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg

from monitor_client.core.unified_network_client import UnifiedNetworkClient
from monitor_client.core.telemetry_protocol import GridSideTelemetryPacket, MachineSideTelemetryPacket
from monitor_client.core.parameter_protocol import ParameterMetadata
from monitor_client.core.data_persistence import DataPersistence, TelemetrySample
from monitor_client.ui.oscilloscope_widget import OscilloscopeWidget
from monitor_client.ui.telemetry_panel import GridSideTelemetryPanel, MachineSideTelemetryPanel
from monitor_client.config import get_machine_side_plots, get_grid_side_plots, get_default_visible_groups
from monitor_client.utils.telemetry_buffer_mapper import TelemetryBufferMapper

logger = logging.getLogger(__name__)


class ParameterEditDialog(QtWidgets.QDialog):
    """Dialog for editing a single parameter value."""
    
    def __init__(self, metadata: ParameterMetadata, parent=None):
        super().__init__(parent)
        
        self.metadata = metadata
        self.new_value = metadata.current_value
        
        self.setWindowTitle(f"Edit Parameter: {metadata.display_name}")
        self.setModal(True)
        self.resize(500, 300)
        
        layout = QtWidgets.QVBoxLayout(self)
        
        # Info section
        info_group = QtWidgets.QGroupBox("Parameter Information")
        info_layout = QtWidgets.QFormLayout(info_group)
        info_layout.addRow("Component:", QtWidgets.QLabel(metadata.component))
        info_layout.addRow("Name:", QtWidgets.QLabel(metadata.name))
        info_layout.addRow("Display Name:", QtWidgets.QLabel(metadata.display_name))
        info_layout.addRow("Unit:", QtWidgets.QLabel(metadata.unit))
        info_layout.addRow("Type:", QtWidgets.QLabel(str(metadata.type.name)))
        info_layout.addRow("Range:", QtWidgets.QLabel(f"[{metadata.min_value:.6f}, {metadata.max_value:.6f}]"))
        info_layout.addRow("Default:", QtWidgets.QLabel(f"{metadata.default_value:.6f}"))
        layout.addWidget(info_group)
        
        # Description
        desc_label = QtWidgets.QLabel(metadata.description)
        desc_label.setWordWrap(True)
        desc_label.setStyleSheet("color: #888; font-style: italic; padding: 5px;")
        layout.addWidget(desc_label)
        
        # Value editor
        value_group = QtWidgets.QGroupBox("Value")
        value_layout = QtWidgets.QHBoxLayout(value_group)
        
        value_layout.addWidget(QtWidgets.QLabel("Current:"))
        self.current_label = QtWidgets.QLabel(f"{metadata.current_value:.6f}")
        self.current_label.setStyleSheet("font-weight: bold;")
        value_layout.addWidget(self.current_label)
        
        value_layout.addWidget(QtWidgets.QLabel("  →  New:"))
        self.value_spinbox = QtWidgets.QDoubleSpinBox()
        self.value_spinbox.setDecimals(6)
        self.value_spinbox.setRange(metadata.min_value, metadata.max_value)
        self.value_spinbox.setValue(metadata.current_value)
        self.value_spinbox.setSingleStep((metadata.max_value - metadata.min_value) / 100.0)
        self.value_spinbox.valueChanged.connect(self._on_value_changed)
        value_layout.addWidget(self.value_spinbox)
        
        layout.addWidget(value_group)
        
        # Buttons
        button_layout = QtWidgets.QHBoxLayout()
        button_layout.addStretch()
        
        cancel_btn = QtWidgets.QPushButton("Cancel")
        cancel_btn.clicked.connect(self.reject)
        button_layout.addWidget(cancel_btn)
        
        self.apply_btn = QtWidgets.QPushButton("Apply")
        self.apply_btn.setDefault(True)
        self.apply_btn.clicked.connect(self.accept)
        button_layout.addWidget(self.apply_btn)
        
        layout.addLayout(button_layout)
    
    def _on_value_changed(self, value: float):
        """Handle value change."""
        self.new_value = value
    
    def get_new_value(self) -> float:
        """Get the new value entered by user."""
        return self.new_value


class TelemetryMonitorWindow(QtWidgets.QMainWindow):
    """Minimal telemetry monitor with parameters, plots, and dashboard."""
    
    def __init__(self, host: str, port: int, target: str = 'grid', verbose: bool = False, decimate: int = 0, window_size: int = 2):
        super().__init__()
        
        logger.info(f"=== Monitor with Parameters (target={target}, verbose={verbose}, decimate={decimate}, window={window_size}) ===")
        
        self.target_side = target  # 'grid' or 'machine'
        self.setWindowTitle(f"SFC Monitor - Parameters + Telemetry [{target.upper()}]")
        self.resize(1800, 1000)
        
        # Apply dark theme
        self.setStyleSheet("""
            QMainWindow { background-color: #2b2b2b; }
            QWidget { background-color: #2b2b2b; color: #ffffff; }
            QPushButton { 
                background-color: #3c3c3c; 
                color: #ffffff; 
                border: 1px solid #555;
                padding: 5px 15px;
                border-radius: 3px;
            }
            QPushButton:hover { background-color: #4a4a4a; }
            QPushButton:pressed { background-color: #2a2a2a; }
            QPushButton:disabled { background-color: #2b2b2b; color: #666666; }
            QLabel { color: #ffffff; background: transparent; }
            QScrollArea { border: none; background-color: #1e1e1e; }
            QTreeWidget { background-color: #1e1e1e; color: #ffffff; border: 1px solid #444; }
            QTreeWidget::item:hover { background-color: #3c3c3c; }
            QTreeWidget::item:selected { background-color: #094771; }
            QHeaderView::section { background-color: #2b2b2b; color: #ffffff; border: 1px solid #444; padding: 4px; }
            QSpinBox, QDoubleSpinBox { background-color: #1e1e1e; color: #ffffff; border: 1px solid #444; padding: 2px; }
        """)
        
        self.verbose = verbose
        self.decimate_factor = max(1, decimate)
        self.decimation_mode = "Skip"  # "Skip" or "Average"
        self.packet_counter = 0
        self.sample_index = 0
        self.window_size = window_size
        self.layout_mode = "1xN"  # "1xN" for vertical stack or "2x2" for grid
        
        # Data persistence (optional - disabled by default for performance)
        self.data_persistence = DataPersistence(enable_persistence=False)
        self.enable_persistence = False
        
        # Averaging accumulators for decimation
        self.avg_voltage = 0.0
        self.avg_current_a = 0.0
        self.avg_current_b = 0.0
        self.avg_current_c = 0.0
        self.avg_observer_voltage = 0.0
        self.avg_observer_freq = 0.0
        self.avg_dclink_ref = 0.0
        self.avg_dclink_measured = 0.0
        # New accumulators for expanded plots
        self.avg_dclink_current = 0.0
        self.avg_grid_voltage_a = 0.0
        self.avg_grid_voltage_b = 0.0
        self.avg_grid_voltage_c = 0.0
        self.avg_grid_voltage_mag = 0.0
        self.avg_estimated_voltage_mag = 0.0
        self.avg_estimated_angle = 0.0
        self.avg_estimated_freq = 0.0
        self.avg_current_error = 0.0
        self.avg_controller_saturated = 0.0
        self.avg_firing_angle_input = 0.0
        self.avg_firing_angle_output = 0.0
        self.avg_operating_mode = 0.0
        self.avg_request_extinguish_dc = 0.0
        self.avg_request_force_commutation = 0.0
        self.avg_modulator_enabled = 0.0
        self.avg_force_commutation_ack = 0.0
        self.avg_is_requesting_pi_reset = 0.0
        self.avg_modulator_state = 0.0
        self.avg_firing_pulse_1 = 0.0
        self.avg_firing_pulse_2 = 0.0
        self.avg_firing_pulse_3 = 0.0
        self.avg_firing_pulse_4 = 0.0
        self.avg_firing_pulse_5 = 0.0
        self.avg_firing_pulse_6 = 0.0
        self.avg_count = 0  # Counter for averaging
        
        # Create oscilloscope widget (manages all plots and buffers)
        # Uses plot definitions from plot_definitions.py based on target side
        self.oscilloscope = OscilloscopeWidget(window_size=window_size, layout_mode=self.layout_mode)
        self.oscilloscope.create_plots_from_config(self.target_side)
        
        # Create automatic buffer mapper
        self.buffer_mapper = TelemetryBufferMapper(target=self.target_side)
        logger.info(f"Initialized buffer mapper for {self.target_side} with {len(self.buffer_mapper.mappings)} buffers")
        
        # Latest telemetry for dashboard
        self.latest_packet = None
        
        # Parameters
        self.parameters: List[ParameterMetadata] = []
        self.param_loading = False
        self.was_streaming_before_param_load = False
        self.pending_parameter_indices: List[int] = []  # Queue of indices to request
        self.param_request_timer = QtCore.QTimer(self)
        self.param_request_timer.timeout.connect(self._send_next_parameter_request)
        self.param_request_timer.setInterval(20)  # 50ms between requests (20 requests/sec)
        
        # Build UI
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        main_layout = QtWidgets.QVBoxLayout(central)
        
        # Connection controls
        conn_layout = QtWidgets.QHBoxLayout()
        self.connect_btn = QtWidgets.QPushButton("Connect")
        self.disconnect_btn = QtWidgets.QPushButton("Disconnect")
        self.disconnect_btn.setEnabled(False)
        self.connect_btn.clicked.connect(self._on_connect)
        self.disconnect_btn.clicked.connect(self._on_disconnect)
        conn_layout.addWidget(QtWidgets.QLabel(f"Target: {host}:{port}"))
        conn_layout.addWidget(self.connect_btn)
        conn_layout.addWidget(self.disconnect_btn)
        conn_layout.addStretch()
        main_layout.addLayout(conn_layout)
        
        # Telemetry controls
        telem_layout = QtWidgets.QHBoxLayout()
        self.start_btn = QtWidgets.QPushButton("Start")
        self.stop_btn = QtWidgets.QPushButton("Stop")
        self.clear_btn = QtWidgets.QPushButton("Clear Plots")
        self.start_btn.setEnabled(False)
        self.stop_btn.setEnabled(False)
        self.clear_btn.setEnabled(True)
        self.start_btn.clicked.connect(self._on_start)
        self.stop_btn.clicked.connect(self._on_stop)
        self.clear_btn.clicked.connect(self._on_clear)
        telem_layout.addWidget(self.start_btn)
        telem_layout.addWidget(self.stop_btn)
        telem_layout.addWidget(self.clear_btn)
        
        # Data persistence controls
        telem_layout.addWidget(QtWidgets.QLabel("  |  Data:"))
        self.persist_checkbox = QtWidgets.QCheckBox("Persist All Data")
        self.persist_checkbox.setChecked(False)
        self.persist_checkbox.setToolTip("Keep all samples in memory for export (uses more RAM)")
        self.persist_checkbox.stateChanged.connect(self._on_toggle_persistence)
        telem_layout.addWidget(self.persist_checkbox)
        
        self.export_btn = QtWidgets.QPushButton("Export CSV")
        self.export_btn.setToolTip("Save data to CSV file for offline analysis")
        self.export_btn.clicked.connect(self._on_export_data)
        telem_layout.addWidget(self.export_btn)
        
        # Panel toggle buttons
        telem_layout.addWidget(QtWidgets.QLabel("  |  Panels:"))
        
        self.toggle_params_btn = QtWidgets.QPushButton("Hide Params")
        self.toggle_params_btn.clicked.connect(self._on_toggle_params_only)
        self.toggle_params_btn.setToolTip("Toggle parameters panel")
        telem_layout.addWidget(self.toggle_params_btn)
        
        self.toggle_telemetry_btn = QtWidgets.QPushButton("Hide Telemetry")
        self.toggle_telemetry_btn.clicked.connect(self._on_toggle_telemetry_only)
        self.toggle_telemetry_btn.setToolTip("Toggle telemetry panel")
        telem_layout.addWidget(self.toggle_telemetry_btn)
        
        self.toggle_all_panels_btn = QtWidgets.QPushButton("Hide All")
        self.toggle_all_panels_btn.clicked.connect(self._on_toggle_all_panels)
        self.toggle_all_panels_btn.setToolTip("Toggle both panels simultaneously")
        telem_layout.addWidget(self.toggle_all_panels_btn)
        
        telem_layout.addWidget(QtWidgets.QLabel("  |  Window Size:"))
        self.window_size_spinbox = QtWidgets.QSpinBox()
        self.window_size_spinbox.setRange(10, 50000)
        self.window_size_spinbox.setSingleStep(100)
        self.window_size_spinbox.setValue(window_size)
        self.window_size_spinbox.setToolTip("Number of samples to display on X-axis")
        self.window_size_spinbox.valueChanged.connect(self._on_window_size_changed)
        telem_layout.addWidget(self.window_size_spinbox)
        telem_layout.addWidget(QtWidgets.QLabel("samples"))
        
        # Decimation control
        telem_layout.addWidget(QtWidgets.QLabel("  |  Decimation:"))
        self.decimation_spinbox = QtWidgets.QSpinBox()
        self.decimation_spinbox.setRange(1, 100)
        self.decimation_spinbox.setSingleStep(1)
        self.decimation_spinbox.setValue(decimate)
        self.decimation_spinbox.setToolTip("Process 1 out of N packets (skip or average)")
        self.decimation_spinbox.valueChanged.connect(self._on_decimation_changed)
        telem_layout.addWidget(self.decimation_spinbox)
        
        # Decimation mode selector
        self.decimation_mode_combo = QtWidgets.QComboBox()
        self.decimation_mode_combo.addItem("Skip")
        self.decimation_mode_combo.addItem("Average")
        self.decimation_mode_combo.setToolTip("Decimation mode: Skip packets or Average them")
        self.decimation_mode_combo.currentTextChanged.connect(self._on_decimation_mode_changed)
        telem_layout.addWidget(self.decimation_mode_combo)
        
        # Layout mode toggle
        telem_layout.addWidget(QtWidgets.QLabel("  |  Layout:"))
        self.layout_mode_btn = QtWidgets.QPushButton("1xN (Vertical)")
        self.layout_mode_btn.setMaximumWidth(120)
        self.layout_mode_btn.setToolTip("Toggle between 1xN vertical stack and 2x2 grid layout")
        self.layout_mode_btn.clicked.connect(self._on_toggle_layout_mode)
        telem_layout.addWidget(self.layout_mode_btn)
        
        telem_layout.addStretch()
        self.status_label = QtWidgets.QLabel("Disconnected")
        telem_layout.addWidget(self.status_label)
        main_layout.addLayout(telem_layout)
        
        # Oscilloscope view configuration
        view_layout = QtWidgets.QHBoxLayout()
        view_layout.addWidget(QtWidgets.QLabel("Oscilloscope Views:"))
        view_layout.addStretch()
        main_layout.addLayout(view_layout)
        
        # Main content: parameters (left), plots (center), telemetry (right)
        content_layout = QtWidgets.QHBoxLayout()
        
        # Left: Parameter panel (20% width)
        self.param_panel = self._build_parameter_panel()
        content_layout.addWidget(self.param_panel, 20)
        
        # Center: Four plots in 2x2 grid (60% width)
        plots_widget = self._build_plots()
        content_layout.addWidget(plots_widget, 60)
        
        # Right: Telemetry dashboard (20% width) - choose based on target
        if target == 'grid':
            self.telemetry_panel = GridSideTelemetryPanel(parent=self)
        else:
            self.telemetry_panel = MachineSideTelemetryPanel(parent=self)
        
        content_layout.addWidget(self.telemetry_panel, 20)
        
        # Store layout reference for toggling
        self.content_layout = content_layout
        self.panels_visible = True
        self.params_visible = True
        self.telemetry_visible = True
        
        main_layout.addLayout(content_layout, 1)
        
        # Network client
        self.network_client = UnifiedNetworkClient(host, port, verbose=verbose, parent=self)
        self.network_client.connected_changed.connect(self._on_connection_changed)
        self.network_client.telemetry_packet_received.connect(self._on_packet)
        self.network_client.parameter_response_received.connect(self._on_parameter_response)
        
        # Plot update timer (100ms)
        self.plot_timer = QtCore.QTimer(self)
        self.plot_timer.timeout.connect(self._update_plot)
        self.plot_timer.start(100)
        
        # Dashboard update timer (200ms - slower to reduce overhead)
        self.dashboard_timer = QtCore.QTimer(self)
        self.dashboard_timer.timeout.connect(self._update_dashboard)
        self.dashboard_timer.start(200)
    
    def eventFilter(self, obj, event):
        """Handle keyboard events for parameter search."""
        if obj == self.param_tree and event.type() == QtCore.QEvent.KeyPress:
            # Ctrl+F to focus search box
            if event.key() == QtCore.Qt.Key_F and event.modifiers() == QtCore.Qt.ControlModifier:
                self.param_search_box.setFocus()
                self.param_search_box.selectAll()
                return True
        
        if obj == self.param_search_box and event.type() == QtCore.QEvent.KeyPress:
            # Esc to clear search and return focus to tree
            if event.key() == QtCore.Qt.Key_Escape:
                self.param_search_box.clear()
                self.param_tree.setFocus()
                return True
        
        return super().eventFilter(obj, event)
    
    def _on_parameter_search_changed(self, search_text: str):
        """Filter parameter groups based on search text.
        
        Shows only groups (components) whose names contain the search text.
        All children of matching groups are shown.
        """
        search_text = search_text.lower().strip()
        
        # If search is empty, show all groups
        if not search_text:
            for i in range(self.param_tree.topLevelItemCount()):
                self.param_tree.topLevelItem(i).setHidden(False)
            return
        
        # Filter groups by name
        visible_count = 0
        for i in range(self.param_tree.topLevelItemCount()):
            group_item = self.param_tree.topLevelItem(i)
            group_name = group_item.text(0).lower()
            
            # Check if group name contains search text
            if search_text in group_name:
                group_item.setHidden(False)
                group_item.setExpanded(True)  # Expand matching groups
                visible_count += 1
            else:
                group_item.setHidden(True)
        
        # Update status to show filter results
        if visible_count == 0:
            self.param_status_label.setText(f"No groups match '{search_text}'")
        elif visible_count == 1:
            self.param_status_label.setText(f"1 group matches")
        else:
            self.param_status_label.setText(f"{visible_count} groups match")
    
    def _build_parameter_panel(self) -> QtWidgets.QWidget:
        """Build parameter configuration panel."""
        panel = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        
        # Header
        header = QtWidgets.QLabel("PARAMETERS")
        header.setStyleSheet("color: #00ffff; font-size: 14pt; font-weight: bold; padding: 5px;")
        layout.addWidget(header)
        
        # Search box for filtering parameter groups
        search_layout = QtWidgets.QHBoxLayout()
        search_label = QtWidgets.QLabel("🔍")
        search_label.setStyleSheet("font-size: 14pt;")
        search_layout.addWidget(search_label)
        
        self.param_search_box = QtWidgets.QLineEdit()
        self.param_search_box.setPlaceholderText("Search groups (Ctrl+F)...")
        self.param_search_box.setToolTip("Filter parameter groups by name (Ctrl+F to focus)")
        self.param_search_box.textChanged.connect(self._on_parameter_search_changed)
        self.param_search_box.setClearButtonEnabled(True)
        search_layout.addWidget(self.param_search_box)
        
        self.clear_search_btn = QtWidgets.QPushButton("✕")
        self.clear_search_btn.setMaximumWidth(30)
        self.clear_search_btn.setToolTip("Clear search (Esc)")
        self.clear_search_btn.clicked.connect(lambda: self.param_search_box.clear())
        search_layout.addWidget(self.clear_search_btn)
        
        layout.addLayout(search_layout)
        
        # Button row 1: Load from controller
        self.load_params_btn = QtWidgets.QPushButton("Load from Controller")
        self.load_params_btn.setEnabled(False)
        self.load_params_btn.setToolTip("Load all parameter definitions from the controller")
        self.load_params_btn.clicked.connect(self._on_load_parameters)
        layout.addWidget(self.load_params_btn)
        
        # Button row 2: Refresh values
        self.refresh_params_btn = QtWidgets.QPushButton("Refresh Values")
        self.refresh_params_btn.setEnabled(False)
        self.refresh_params_btn.setToolTip("Reload current parameter values from controller")
        self.refresh_params_btn.clicked.connect(self._on_refresh_parameters)
        layout.addWidget(self.refresh_params_btn)
        
        # Button row 3: Save/Load to file
        file_btn_layout = QtWidgets.QHBoxLayout()
        
        self.save_params_btn = QtWidgets.QPushButton("Save to File")
        self.save_params_btn.setEnabled(False)
        self.save_params_btn.setToolTip("Save current parameter values to JSON file")
        self.save_params_btn.clicked.connect(self._on_save_parameters_to_file)
        file_btn_layout.addWidget(self.save_params_btn)
        
        self.load_file_params_btn = QtWidgets.QPushButton("Load from File")
        self.load_file_params_btn.setEnabled(False)
        self.load_file_params_btn.setToolTip("Load parameter values from JSON file and apply to controller")
        self.load_file_params_btn.clicked.connect(self._on_load_parameters_from_file)
        file_btn_layout.addWidget(self.load_file_params_btn)
        
        layout.addLayout(file_btn_layout)
        
        # Parameter tree
        self.param_tree = QtWidgets.QTreeWidget()
        self.param_tree.setHeaderLabels(["Parameter", "Value"])
        self.param_tree.setColumnWidth(0, 200)
        self.param_tree.itemDoubleClicked.connect(self._on_parameter_double_clicked)
        self.param_tree.installEventFilter(self)  # Install event filter for keyboard shortcuts
        layout.addWidget(self.param_tree)
        
        # Status
        self.param_status_label = QtWidgets.QLabel("Not loaded")
        self.param_status_label.setStyleSheet("color: #888; font-size: 9pt; padding: 5px;")
        layout.addWidget(self.param_status_label)
        
        return panel
    
    def _build_plots(self) -> QtWidgets.QWidget:
        """Build plots using the OscilloscopeWidget for grid-side and machine-side.
        
        The oscilloscope widget handles all plot creation, data buffering,
        and rendering. Grid-side shows grid-specific plots, machine-side shows
        machine-specific plots (measurements, observer, speed control, torque control).
        """
        container_widget = QtWidgets.QWidget()
        container_layout = QtWidgets.QVBoxLayout(container_widget)
        container_layout.setContentsMargins(0, 0, 0, 0)
        
        # Configure mouse event handler for window size adjustment
        self.oscilloscope.configure_wheel_handler(self._create_wheel_handler)
        self.oscilloscope.disable_mouse_interactions()
        container_layout.addWidget(self.oscilloscope)
        
        return container_widget
    
    def _create_wheel_handler(self, viewbox, event):
        """Handle wheel event to adjust window size.
        
        Args:
            viewbox: The viewbox where the wheel event occurred (ignored, just for compatibility)
            event: The wheel event
        """
        # Get wheel delta
        delta = event.angleDelta().y() if hasattr(event, 'angleDelta') else (event.delta() if hasattr(event, 'delta') else 0)
        
        # Adjust window size
        current_size = self.window_size_spinbox.value()
        step = 100
        
        if delta > 0:
            # Zoom in - decrease window size
            new_size = max(self.window_size_spinbox.minimum(), current_size - step)
        else:
            # Zoom out - increase window size
            new_size = min(self.window_size_spinbox.maximum(), current_size + step)
        
        self.window_size_spinbox.setValue(new_size)
        event.accept()  # Accept the event so it doesn't propagate
    
    
    
    # Connection handlers
    def _on_connect(self):
        """Connect to controller."""
        if self.network_client.connect():
            logger.info("Connected")
        else:
            QtWidgets.QMessageBox.warning(self, "Error", "Connection failed")
    
    def _on_disconnect(self):
        """Disconnect from controller."""
        self.network_client.disconnect()
        logger.info("Disconnected")
    
    def _on_connection_changed(self, connected: bool):
        """Handle connection status change."""
        self.connect_btn.setEnabled(not connected)
        self.disconnect_btn.setEnabled(connected)
        self.start_btn.setEnabled(connected)
        self.stop_btn.setEnabled(False)
        self.load_params_btn.setEnabled(connected)
        
        # Enable refresh only if connected and parameters are loaded
        self.refresh_params_btn.setEnabled(connected and len(self.parameters) > 0)
        
        # Enable save/load file buttons only if parameters are loaded
        self.save_params_btn.setEnabled(len(self.parameters) > 0)
        
        # Enable load from file only if connected (so we can apply them)
        self.load_file_params_btn.setEnabled(connected)
        
        if connected:
            self.status_label.setText("Connected - Ready")
        else:
            self.status_label.setText("Disconnected")
            # Keep parameters persisted when disconnected
            # Only clear on window destruction or explicit reload
            if self.parameters:
                self.param_status_label.setText(f"Disconnected ({len(self.parameters)} params cached)")
            else:
                self.param_status_label.setText("Not loaded")
    
    # Telemetry handlers
    def _on_start(self):
        """Start telemetry."""
        if self.oscilloscope is not None:
            self.oscilloscope.clear_buffers()
        self.sample_index = 0
        self.packet_counter = 0
        # Reset averaging accumulators
        self.avg_voltage = 0.0
        self.avg_current_a = 0.0
        self.avg_current_b = 0.0
        self.avg_current_c = 0.0
        self.avg_observer_voltage = 0.0
        self.avg_observer_freq = 0.0
        self.avg_dclink_ref = 0.0
        self.avg_dclink_measured = 0.0
        self.avg_count = 0
        if self.network_client.start_telemetry():
            self.start_btn.setEnabled(False)
            self.stop_btn.setEnabled(True)
            self.status_label.setText("Streaming...")
            logger.info("Telemetry started")
        else:
            QtWidgets.QMessageBox.warning(self, "Error", "Failed to start telemetry")
    
    def _on_stop(self):
        """Stop telemetry."""
        if self.network_client.stop_telemetry():
            self.start_btn.setEnabled(True)
            self.stop_btn.setEnabled(False)
            self.status_label.setText("Connected - Stopped")
            logger.info("Telemetry stopped")
    
    def _on_clear(self):
        """Clear buffer and reset."""
        # Clear oscilloscope buffers (grid-side only)
        if self.oscilloscope is not None:
            self.oscilloscope.clear_buffers()
        self.sample_index = 0
        self.packet_counter = 0
        # Clear persistent data if enabled
        if self.enable_persistence:
            self.data_persistence.clear()
        logger.info("Cleared")
    
    def _on_toggle_persistence(self, state):
        """Handle persistence toggle checkbox."""
        self.enable_persistence = (state == QtCore.Qt.Checked)
        if self.enable_persistence:
            logger.info("Data persistence enabled - all samples will be kept in memory")
        else:
            logger.info("Data persistence disabled - using rolling 10k-sample window")
    
    def _on_export_data(self):
        """Handle export data button - open file dialog and export."""
        from pathlib import Path
        from PyQt5.QtWidgets import QFileDialog
        from datetime import datetime
        
        # Create default filename with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        default_name = f"telemetry_export_{timestamp}"
        
        # Open file dialog for CSV export
        file_dialog = QFileDialog(self)
        file_dialog.setDefaultSuffix("csv")
        file_dialog.setAcceptMode(QFileDialog.AcceptSave)
        filepath, _ = file_dialog.getSaveFileName(
            self,
            "Export Telemetry Data",
            str(Path.home() / f"{default_name}.csv"),
            "CSV Files (*.csv);;All Files (*)"
        )
        
        if filepath:
            try:
                # Prepare data buffers dict
                data_buffers = {
                    'timestamp_ms': list(self.observer_buffer['timestamp_ms']) if 'timestamp_ms' in self.observer_buffer else [],
                    'voltage_magnitude': list(self.voltage_buffer) if self.voltage_buffer else [],
                    'current_magnitude': list(self.current_buffer) if self.current_buffer else [],
                    'dc_link_current': list(self.dclink_buffer) if self.dclink_buffer else [],
                    'observer_voltage': list(self.observer_buffer) if isinstance(self.observer_buffer, deque) else list(self.observer_buffer.values())[0] if self.observer_buffer else [],
                    'observer_frequency': list(self.observer_buffer) if isinstance(self.observer_buffer, deque) else [],
                    'firing_angle': list(self.firing_angle_buffer) if self.firing_angle_buffer else [],
                }
                
                # Export using data persistence module
                self.data_persistence.export_csv(filepath, data_buffers)
                logger.info(f"Data exported to: {filepath}")
                
                # Also export metadata
                metadata_path = str(Path(filepath).with_suffix('.json'))
                self.data_persistence.export_json(metadata_path)
                logger.info(f"Metadata exported to: {metadata_path}")
                
                # Show confirmation dialog
                from PyQt5.QtWidgets import QMessageBox
                QMessageBox.information(
                    self,
                    "Export Successful",
                    f"Data exported to:\n{filepath}\n\nMetadata saved to:\n{metadata_path}"
                )
            except Exception as e:
                logger.error(f"Failed to export data: {e}")
                from PyQt5.QtWidgets import QMessageBox
                QMessageBox.critical(self, "Export Failed", f"Error: {e}")
    
    
    def _on_window_size_changed(self, value: int):
        """Handle window size change at runtime."""
        self.window_size = value
        if self.oscilloscope is not None:
            self.oscilloscope.set_window_size(value)
        logger.info(f"Window size changed to {value} samples")
    
    def _on_decimation_changed(self, value: int):
        """Handle decimation factor change at runtime."""
        self.decimate_factor = max(1, value)
        # Reset averaging accumulators
        self.avg_count = 0
        logger.info(f"Decimation factor changed to {self.decimate_factor} ({self.decimation_mode} mode)")
    
    def _on_decimation_mode_changed(self, mode: str):
        """Handle decimation mode change at runtime."""
        self.decimation_mode = mode
        # Reset averaging accumulators when switching modes
        self.avg_count = 0
        self.avg_voltage = 0.0
        self.avg_current_a = 0.0
        self.avg_current_b = 0.0
        self.avg_current_c = 0.0
        self.avg_observer_voltage = 0.0
        self.avg_observer_freq = 0.0
        self.avg_dclink_ref = 0.0
        self.avg_dclink_measured = 0.0
        logger.info(f"Decimation mode changed to {mode} (factor: {self.decimate_factor})")
    
    def _on_toggle_layout_mode(self):
        """Toggle between 1xN and 2x2 layout modes."""
        if self.layout_mode == "1xN":
            self.layout_mode = "2x2"
            self.layout_mode_btn.setText("2x2 (Grid)")
            logger.info("Switched to 2x2 grid layout")
        else:
            self.layout_mode = "1xN"
            self.layout_mode_btn.setText("1xN (Vertical)")
            logger.info("Switched to 1xN vertical layout")
        
        # Update oscilloscope layout (grid-side only)
        if self.oscilloscope is not None:
            self.oscilloscope.set_layout_mode(self.layout_mode)
    
    def _on_toggle_params_only(self):
        """Toggle visibility of parameters panel only."""
        self.params_visible = not self.params_visible
        
        if self.params_visible:
            self.param_panel.show()
            self.toggle_params_btn.setText("Hide Params")
            logger.info("Parameters panel shown")
        else:
            self.param_panel.hide()
            self.toggle_params_btn.setText("Show Params")
            logger.info("Parameters panel hidden")
        
        # Update "Hide All" button text based on visibility state
        self._update_all_panels_button_text()
    
    def _on_toggle_telemetry_only(self):
        """Toggle visibility of telemetry panel only."""
        self.telemetry_visible = not self.telemetry_visible
        
        if self.telemetry_visible:
            self.telemetry_panel.show()
            self.toggle_telemetry_btn.setText("Hide Telemetry")
            logger.info("Telemetry panel shown")
        else:
            self.telemetry_panel.hide()
            self.toggle_telemetry_btn.setText("Show Telemetry")
            logger.info("Telemetry panel hidden")
        
        # Update "Hide All" button text based on visibility state
        self._update_all_panels_button_text()
    
    def _on_toggle_all_panels(self):
        """Toggle visibility of both panels simultaneously."""
        # If both are visible, hide both. Otherwise, show both.
        should_show = not (self.params_visible and self.telemetry_visible)
        
        self.params_visible = should_show
        self.telemetry_visible = should_show
        
        if should_show:
            self.param_panel.show()
            self.telemetry_panel.show()
            self.toggle_params_btn.setText("Hide Params")
            self.toggle_telemetry_btn.setText("Hide Telemetry")
            self.toggle_all_panels_btn.setText("Hide All")
            logger.info("All panels shown")
        else:
            self.param_panel.hide()
            self.telemetry_panel.hide()
            self.toggle_params_btn.setText("Show Params")
            self.toggle_telemetry_btn.setText("Show Telemetry")
            self.toggle_all_panels_btn.setText("Show All")
            logger.info("All panels hidden - maximizing plot area")
    
    def _update_all_panels_button_text(self):
        """Update the 'Hide All' button text based on current panel visibility."""
        if self.params_visible and self.telemetry_visible:
            self.toggle_all_panels_btn.setText("Hide All")
        elif not self.params_visible and not self.telemetry_visible:
            self.toggle_all_panels_btn.setText("Show All")
        else:
            # Mixed state - show partial indicator
            self.toggle_all_panels_btn.setText("Show All" if self.params_visible or self.telemetry_visible else "Hide All")
    
    def _on_toggle_panels(self):
        """Toggle visibility of parameters and telemetry panels."""
        self.panels_visible = not self.panels_visible
        
        if self.panels_visible:
            # Show both panels
            self.param_panel.show()
            self.telemetry_panel.show()
            self.minimize_panels_btn.setText("Hide Panels")
            logger.info("Panels shown")
        else:
            # Hide both panels to maximize plot area
            self.param_panel.hide()
            self.telemetry_panel.hide()
            self.minimize_panels_btn.setText("Show Panels")
            logger.info("Panels hidden - maximizing plot area")
    
    def _on_packet(self, packet):
        """Handle incoming telemetry packet (grid or machine side)."""
        try:
            # Dispatch based on packet type
            if isinstance(packet, MachineSideTelemetryPacket):
                self._on_machine_packet(packet)
            else:
                self._on_grid_packet(packet)
        except Exception as e:
            logger.error(f"Packet error: {e}")
            logger.error(traceback.format_exc())
    
    def _on_grid_packet(self, packet: GridSideTelemetryPacket):
        """Handle grid-side telemetry packet."""
        try:
            self.packet_counter += 1
            
            # Store for dashboard (always update)
            self.latest_packet = packet
            
            # Only process oscilloscope if it exists (grid-side only)
            if self.oscilloscope is None:
                return
            
            if self.decimation_mode == "Skip":
                # Skip mode: only process every Nth packet
                if self.packet_counter % self.decimate_factor != 0:
                    return
                
                # Automatically map all signals to buffers using plot definitions
                # This replaces all the manual buffer packing above
                for buffer_name in self.buffer_mapper.get_all_buffer_names():
                    buffer_tuple = self.buffer_mapper.map_signals_to_buffer(
                        buffer_name, self.sample_index, packet.signals
                    )
                    self.oscilloscope.add_sample_to_buffer(buffer_name, buffer_tuple)
                
                # Store to persistent data if enabled
                if self.enable_persistence:
                    sample = TelemetrySample(
                        timestamp_ms=packet.header.timestamp_ms,
                        sequence_number=packet.header.sequence_number,
                        signal_count=packet.header.signal_count,
                        signals=packet.signals,
                    )
                    self.data_persistence.add_sample(sample)
                
                self.sample_index += 1
            
            else:  # Average mode
                # Accumulate values for averaging
                if packet.measurements:
                    meas = packet.measurements
                    self.avg_voltage += meas.grid_voltage_magnitude_pu
                    self.avg_current_a += meas.grid_current_a_pu
                    self.avg_current_b += meas.grid_current_b_pu
                    self.avg_current_c += meas.grid_current_c_pu
                    self.avg_dclink_current += meas.dc_link_current_pu
                    self.avg_grid_voltage_a += meas.grid_voltage_a_pu
                    self.avg_grid_voltage_b += meas.grid_voltage_b_pu
                    self.avg_grid_voltage_c += meas.grid_voltage_c_pu
                    self.avg_grid_voltage_mag += meas.grid_voltage_magnitude_pu
                
                if packet.observer:
                    obs = packet.observer
                    self.avg_observer_voltage += obs.estimated_voltage_magnitude_pu
                    self.avg_observer_freq += obs.estimated_frequency_pu
                    self.avg_estimated_voltage_mag += obs.estimated_voltage_magnitude_pu
                    self.avg_estimated_angle += obs.estimated_angle_rad
                    self.avg_estimated_freq += obs.estimated_frequency_hz
                
                if packet.controller:
                    ctrl = packet.controller
                    self.avg_dclink_ref += ctrl.dc_link_current_reference
                    self.avg_dclink_measured += ctrl.dc_link_current_measured
                    self.avg_current_error += ctrl.current_error_pu
                    self.avg_controller_saturated += 1.0 if ctrl.controller_saturated else 0.0
                
                if packet.modulator:
                    mod = packet.modulator
                    self.avg_firing_angle_input += mod.firing_angle_input_pu
                    self.avg_firing_angle_output += mod.firing_angle_output_pu
                    self.avg_operating_mode += float(mod.operating_mode)
                    self.avg_request_extinguish_dc += 1.0 if mod.request_extinguish_dc else 0.0
                    self.avg_request_force_commutation += 1.0 if mod.request_force_commutation else 0.0
                    self.avg_modulator_enabled += 1.0 if mod.is_enabled else 0.0
                    self.avg_force_commutation_ack += 1.0 if mod.force_commutation_ack else 0.0
                    self.avg_is_requesting_pi_reset += 1.0 if mod.is_requesting_pi_reset else 0.0
                    self.avg_modulator_state += float(mod.modulator_state)
                
                if packet.pulse_generator:
                    pulses = packet.pulse_generator
                    self.avg_firing_pulse_1 += 1.0 if pulses.firing_pulse_1 else 0.0
                    self.avg_firing_pulse_2 += 1.0 if pulses.firing_pulse_2 else 0.0
                    self.avg_firing_pulse_3 += 1.0 if pulses.firing_pulse_3 else 0.0
                    self.avg_firing_pulse_4 += 1.0 if pulses.firing_pulse_4 else 0.0
                    self.avg_firing_pulse_5 += 1.0 if pulses.firing_pulse_5 else 0.0
                    self.avg_firing_pulse_6 += 1.0 if pulses.firing_pulse_6 else 0.0
                
                self.avg_count += 1
                
                # Check if we've accumulated enough samples
                if self.avg_count >= self.decimate_factor:
                    # Store averaged values
                    self.oscilloscope.add_sample_to_buffer('measurements_voltage_and_current', (self.sample_index, 
                                              self.avg_voltage / self.avg_count,
                                              self.avg_current_a / self.avg_count,
                                              self.avg_current_b / self.avg_count,
                                              self.avg_current_c / self.avg_count))
                    self.oscilloscope.add_sample_to_buffer('observer_voltage_freq', (self.sample_index,
                                               self.avg_observer_voltage / self.avg_count,
                                               self.avg_observer_freq / self.avg_count,
                                               self.avg_estimated_angle / self.avg_count))
                    self.oscilloscope.add_sample_to_buffer('controller_dclink', (self.sample_index,
                                             self.avg_dclink_ref / self.avg_count,
                                             self.avg_dclink_measured / self.avg_count,
                                             self.avg_current_error / self.avg_count))
                    
                    # New: Store averaged values in new buffers
                    self.oscilloscope.add_sample_to_buffer('current_measurements_abc_dclink', (self.sample_index,
                                                           self.avg_current_a / self.avg_count,
                                                           self.avg_current_b / self.avg_count,
                                                           self.avg_current_c / self.avg_count,
                                                           self.avg_dclink_current / self.avg_count))
                    self.oscilloscope.add_sample_to_buffer('observer_voltage_analysis_voltages', (self.sample_index,
                                                                 self.avg_grid_voltage_a / self.avg_count,
                                                                 self.avg_grid_voltage_b / self.avg_count,
                                                                 self.avg_grid_voltage_c / self.avg_count,
                                                                 self.avg_grid_voltage_mag / self.avg_count,
                                                                 self.avg_estimated_voltage_mag / self.avg_count))
                    self.oscilloscope.add_sample_to_buffer('observer_voltage_analysis_freq_angle', (self.sample_index,
                                                                 self.avg_estimated_freq / self.avg_count,
                                                                 self.avg_estimated_angle / self.avg_count,
                                                                 self.avg_observer_freq / self.avg_count))
                    self.oscilloscope.add_sample_to_buffer('operation_mode_monitor_controller_status', (self.sample_index,
                                                               self.avg_current_error / self.avg_count,
                                                               1.0,  # controller_output
                                                               self.avg_controller_saturated / self.avg_count))
                    self.oscilloscope.add_sample_to_buffer('modulator_firing_angle_control', (self.sample_index,
                                                   self.avg_firing_angle_input / self.avg_count,
                                                   self.avg_firing_angle_output / self.avg_count,
                                                   self.avg_operating_mode / self.avg_count,
                                                   self.avg_request_extinguish_dc / self.avg_count,
                                                   self.avg_request_force_commutation / self.avg_count,
                                                   self.avg_modulator_enabled / self.avg_count,
                                                   self.avg_force_commutation_ack / self.avg_count,
                                                   self.avg_is_requesting_pi_reset / self.avg_count,
                                                   self.avg_modulator_state / self.avg_count))
                    self.oscilloscope.add_sample_to_buffer('pulse_generator_firing_pulses', (self.sample_index,
                                                   self.avg_firing_pulse_1 / self.avg_count,
                                                   self.avg_firing_pulse_2 / self.avg_count,
                                                   self.avg_firing_pulse_3 / self.avg_count,
                                                   self.avg_firing_pulse_4 / self.avg_count,
                                                   self.avg_firing_pulse_5 / self.avg_count,
                                                   self.avg_firing_pulse_6 / self.avg_count))
                    
                    # Store to persistent data if enabled (averaged packet)
                    if self.enable_persistence:
                        averaged_signals = [0.0] * 256
                        averaged_signals[0] = self.avg_grid_voltage_mag / self.avg_count
                        averaged_signals[1] = self.avg_grid_voltage_a / self.avg_count
                        averaged_signals[2] = self.avg_grid_voltage_b / self.avg_count
                        averaged_signals[3] = self.avg_grid_voltage_c / self.avg_count
                        averaged_signals[8] = self.avg_dclink_current / self.avg_count
                        averaged_signals[13] = self.avg_estimated_voltage_mag / self.avg_count
                        averaged_signals[17] = self.avg_estimated_freq / self.avg_count
                        averaged_signals[20] = self.avg_firing_angle_output / self.avg_count
                        
                        sample = TelemetrySample(
                            timestamp_ms=packet.header.timestamp_ms,
                            sequence_number=packet.header.sequence_number,
                            signal_count=self.avg_count,
                            signals=averaged_signals,
                        )
                        self.data_persistence.add_sample(sample)
                    
                    # Reset accumulators
                    self.avg_voltage = 0.0
                    self.avg_current_a = 0.0
                    self.avg_current_b = 0.0
                    self.avg_current_c = 0.0
                    self.avg_dclink_current = 0.0
                    self.avg_observer_voltage = 0.0
                    self.avg_observer_freq = 0.0
                    self.avg_dclink_ref = 0.0
                    self.avg_dclink_measured = 0.0
                    self.avg_grid_voltage_a = 0.0
                    self.avg_grid_voltage_b = 0.0
                    self.avg_grid_voltage_c = 0.0
                    self.avg_grid_voltage_mag = 0.0
                    self.avg_estimated_voltage_mag = 0.0
                    self.avg_estimated_angle = 0.0
                    self.avg_estimated_freq = 0.0
                    self.avg_current_error = 0.0
                    self.avg_controller_saturated = 0.0
                    self.avg_firing_angle_input = 0.0
                    self.avg_firing_angle_output = 0.0
                    self.avg_operating_mode = 0.0
                    self.avg_request_extinguish_dc = 0.0
                    self.avg_request_force_commutation = 0.0
                    self.avg_modulator_enabled = 0.0
                    self.avg_force_commutation_ack = 0.0
                    self.avg_is_requesting_pi_reset = 0.0
                    self.avg_modulator_state = 0.0
                    self.avg_firing_pulse_1 = 0.0
                    self.avg_firing_pulse_2 = 0.0
                    self.avg_firing_pulse_3 = 0.0
                    self.avg_firing_pulse_4 = 0.0
                    self.avg_firing_pulse_5 = 0.0
                    self.avg_firing_pulse_6 = 0.0
                    self.avg_count = 0
                    
                    self.sample_index += 1
        
        except Exception as e:
            logger.error(f"Packet error: {e}", exc_info=True)
    
    def _on_machine_packet(self, packet: MachineSideTelemetryPacket):
        """Handle machine-side telemetry packet using plot definitions.
        
        Extract measurements, observer data, and controller states from the packet
        and populate oscilloscope buffers for real-time visualization.
        Uses signal indices from plot_definitions.py to extract data directly.
        """
        try:
            self.packet_counter += 1
            
            # Store for dashboard (always update)
            self.latest_packet = packet
            
            # Persist data if enabled
            if self.enable_persistence:
                sample = TelemetrySample(
                    timestamp_ms=packet.header.timestamp_ms,
                    sequence_number=packet.header.sequence_number,
                    signal_count=packet.header.signal_count,
                    signals=packet.signals,
                )
                self.data_persistence.add_sample(sample)
            
            # Only update oscilloscope on decimated samples
            if self.packet_counter % self.decimate_factor == 0 and self.oscilloscope is not None:
                # Get plot definitions
                plot_definitions = get_machine_side_plots()
                
                # Extract and add data for each plot group
                for group_name, plots_dict in plot_definitions.items():
                    for plot_name, plot_config in plots_dict.items():
                        # Build data tuple from signal indices
                        buffer_key = f"{group_name}_{plot_name}"
                        data_values = [self.sample_index]
                        
                        for signal in plot_config['signals']:
                            signal_index = signal['index']
                            data_values.append(packet.get_signal(signal_index))
                        
                        # Add to oscilloscope buffer
                        self.oscilloscope.add_sample_to_buffer(buffer_key, tuple(data_values))
                
                self.sample_index += 1
        
        except Exception as e:
            logger.error(f"Machine packet error: {e}", exc_info=True)
    
    # Parameter handlers
    def _on_load_parameters(self):
        """Load parameters from controller."""
        if self.param_loading:
            logger.warning("Parameter load already in progress")
            QtWidgets.QMessageBox.information(self, "Please Wait", "Parameter loading already in progress...")
            return
        
        if not self.network_client.connected:
            QtWidgets.QMessageBox.warning(self, "Error", "Not connected to controller")
            return
        
        # Stop telemetry if running to avoid interference
        self.was_streaming_before_param_load = False
        if self.stop_btn.isEnabled():
            self.was_streaming_before_param_load = True
            self.network_client.stop_telemetry()
            logger.info("Stopped telemetry before loading parameters")
        
        self.param_loading = True
        self.parameters.clear()
        self.param_tree.clear()
        self.param_status_label.setText("Requesting parameter count...")
        self.load_params_btn.setEnabled(False)
        
        # Small delay to ensure stop command is processed
        QtCore.QTimer.singleShot(100, self._send_parameter_count_request)
    
    def _send_parameter_count_request(self):
        """Send parameter count request after brief delay."""
        # Disable packet timeout during parameter loading (can take >5 seconds for many parameters)
        self.network_client.set_packet_timeout_enabled(False)
        
        # Request parameter count
        logger.info("Sending parameter count request...")
        if not self.network_client.send_parameter_count_request():
            QtWidgets.QMessageBox.warning(self, "Error", "Failed to send parameter count request")
            self.param_loading = False
            self.load_params_btn.setEnabled(True)
            self.param_status_label.setText("Failed to request parameters")
            
            # Re-enable timeout
            self.network_client.set_packet_timeout_enabled(True)
            
            # Restart telemetry if it was running
            if self.was_streaming_before_param_load:
                self.network_client.start_telemetry()
            return
        
        # Set a timeout to re-enable button if no response
        QtCore.QTimer.singleShot(30000, self._check_parameter_load_timeout)  # 30 seconds for many parameters
        
        logger.info("Parameter count request sent, waiting for response...")
    
    def _send_next_parameter_request(self):
        """Send next parameter request from queue (called by timer)."""
        if not self.pending_parameter_indices:
            # All requests sent, stop timer
            self.param_request_timer.stop()
            logger.info("All parameter requests sent, waiting for responses...")
            return
        
        # Send next request
        index = self.pending_parameter_indices.pop(0)
        if not self.network_client.send_parameter_get_by_index_request(index):
            logger.error(f"Failed to send parameter request for index {index}")
            # Continue anyway - might recover
        
        # Update status
        remaining = len(self.pending_parameter_indices)
        if remaining > 0:
            self.param_status_label.setText(f"Requesting parameters... ({remaining} remaining)")
    
    def _check_parameter_load_timeout(self):
        """Check if parameter loading timed out."""
        if self.param_loading and len(self.parameters) == 0:
            logger.warning("Parameter load timed out - no response from controller")
            self.param_loading = False
            self.load_params_btn.setEnabled(True)
            self.param_status_label.setText("Load timeout - controller may not support parameters")
            
            # Stop request timer
            self.param_request_timer.stop()
            self.pending_parameter_indices.clear()
            
            # Re-enable packet timeout
            self.network_client.set_packet_timeout_enabled(True)
            
            # Restart telemetry if it was running
            if self.was_streaming_before_param_load:
                self.network_client.start_telemetry()
                self.was_streaming_before_param_load = False
            
            QtWidgets.QMessageBox.warning(
                self, 
                "Parameter Load Timeout", 
                "No response from controller.\n\n"
                "Possible causes:\n"
                "- Controller firmware doesn't support parameter configuration\n"
                "- Parameter system not initialized in firmware\n"
                "- Network communication issue\n\n"
                "Check controller logs for errors."
            )
    
    def _on_parameter_response(self, response: dict):
        """Handle parameter response from controller."""
        try:
            response_type = response.get('type')
            
            if response_type == 'count_response':
                # Received parameter count, now request each by index
                total_count = response.get('total_parameters', 0)
                logger.info(f"Received parameter count: {total_count}")
                self.param_status_label.setText(f"Loading {total_count} parameters...")
                
                # Queue all indices for gradual requesting (prevent server overload)
                self.pending_parameter_indices = list(range(total_count))
                
                # Start timer to send requests gradually
                if len(self.pending_parameter_indices) > 0:
                    self.param_request_timer.start()
                    logger.info(f"Starting gradual parameter load (50ms between requests)")
                else:
                    # No parameters - finish immediately
                    self.param_loading = False
                    self.load_params_btn.setEnabled(True)
                    self.network_client.set_packet_timeout_enabled(True)
                    self.param_status_label.setText("No parameters available")
            
            elif response_type == 'get_by_index_response':
                # Received parameter metadata
                metadata: ParameterMetadata = response.get('metadata')
                if metadata:
                    self.parameters.append(metadata)
                    self._add_parameter_to_tree(metadata)
                    
                    # Update status
                    self.param_status_label.setText(f"Loaded {len(self.parameters)}/{metadata.total_parameters} parameters")
                    
                    # Check if done
                    if len(self.parameters) >= metadata.total_parameters:
                        self.param_loading = False
                        self.load_params_btn.setEnabled(True)
                        self.refresh_params_btn.setEnabled(True)  # Enable refresh after load
                        self.save_params_btn.setEnabled(True)  # Enable save after load
                        
                        # Update status and apply any active search filter
                        group_count = self.param_tree.topLevelItemCount()
                        if self.param_search_box.text().strip():
                            # Re-apply filter after loading
                            self._on_parameter_search_changed(self.param_search_box.text())
                        else:
                            self.param_status_label.setText(f"Loaded {len(self.parameters)} parameters ({group_count} groups)")
                        
                        logger.info(f"All parameters loaded: {len(self.parameters)} parameters in {group_count} groups")
                        
                        # Stop request timer and clear queue
                        self.param_request_timer.stop()
                        self.pending_parameter_indices.clear()
                        
                        # Re-enable packet timeout
                        self.network_client.set_packet_timeout_enabled(True)
                        
                        # Restart telemetry if it was running before
                        if self.was_streaming_before_param_load:
                            logger.info("Restarting telemetry after parameter load")
                            self.network_client.start_telemetry()
                            self.was_streaming_before_param_load = False
            
            elif response_type == 'set_response':
                # Parameter set response
                status = response.get('status')
                component = response.get('component')
                name = response.get('name')
                new_value = response.get('new_value')
                
                if status.name == 'SUCCESS':
                    logger.info(f"Parameter set: {component}.{name} = {new_value}")
                    # Update tree
                    self._update_parameter_in_tree(component, name, new_value)
                else:
                    error_msg = response.get('error_message', 'Unknown error')
                    QtWidgets.QMessageBox.warning(self, "Parameter Set Failed", f"{component}.{name}: {error_msg}")
        
        except Exception as e:
            logger.error(f"Parameter response error: {e}", exc_info=True)
    
    def _add_parameter_to_tree(self, metadata: ParameterMetadata):
        """Add parameter to tree widget."""
        # Find or create component node
        component_item = None
        for i in range(self.param_tree.topLevelItemCount()):
            item = self.param_tree.topLevelItem(i)
            if item.text(0) == metadata.component:
                component_item = item
                break
        
        if not component_item:
            component_item = QtWidgets.QTreeWidgetItem([metadata.component, ""])
            component_item.setFont(0, QtGui.QFont("Arial", 10, QtGui.QFont.Bold))
            self.param_tree.addTopLevelItem(component_item)
        
        # Add parameter as child
        param_item = QtWidgets.QTreeWidgetItem([metadata.display_name, f"{metadata.current_value:.6f} {metadata.unit}"])
        param_item.setData(0, QtCore.Qt.UserRole, metadata)  # Store metadata
        component_item.addChild(param_item)
        component_item.setExpanded(True)
    
    def _update_parameter_in_tree(self, component: str, name: str, new_value: float):
        """Update parameter value in tree."""
        # Find parameter in tree
        for i in range(self.param_tree.topLevelItemCount()):
            comp_item = self.param_tree.topLevelItem(i)
            if comp_item.text(0) == component:
                for j in range(comp_item.childCount()):
                    param_item = comp_item.child(j)
                    metadata: ParameterMetadata = param_item.data(0, QtCore.Qt.UserRole)
                    if metadata and metadata.name == name:
                        # Update metadata
                        metadata.current_value = new_value
                        # Update display
                        param_item.setText(1, f"{new_value:.6f} {metadata.unit}")
                        logger.info(f"Updated tree: {component}.{name} = {new_value}")
                        return
    
    def _on_parameter_double_clicked(self, item: QtWidgets.QTreeWidgetItem, column: int):
        """Handle double-click on parameter item."""
        metadata: ParameterMetadata = item.data(0, QtCore.Qt.UserRole)
        if not metadata:
            return  # Clicked on component node
        
        # Open edit dialog
        dialog = ParameterEditDialog(metadata, self)
        if dialog.exec_() == QtWidgets.QDialog.Accepted:
            new_value = dialog.get_new_value()
            logger.info(f"User wants to set {metadata.full_name} = {new_value}")
            
            # Send set request
            if not self.network_client.send_parameter_set_request(metadata.component, metadata.name, new_value):
                QtWidgets.QMessageBox.warning(self, "Error", "Failed to send parameter set request")
    
    def _on_save_parameters_to_file(self):
        """Save current parameter values to JSON file."""
        if not self.parameters:
            QtWidgets.QMessageBox.warning(self, "No Parameters", "No parameters loaded to save")
            return
        
        # Open file dialog
        filename, _ = QtWidgets.QFileDialog.getSaveFileName(
            self,
            "Save Parameters to JSON",
            f"parameters_{self.target_side}_{time.strftime('%Y%m%d_%H%M%S')}.json",
            "JSON Files (*.json);;All Files (*)"
        )
        
        if not filename:
            return  # User cancelled
        
        try:
            # Build parameter dictionary
            param_dict = {
                "metadata": {
                    "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
                    "target": self.target_side,
                    "total_parameters": len(self.parameters)
                },
                "parameters": {}
            }
            
            for param in self.parameters:
                key = f"{param.component}.{param.name}"
                param_dict["parameters"][key] = {
                    "component": param.component,
                    "name": param.name,
                    "display_name": param.display_name,
                    "value": param.current_value,
                    "unit": param.unit,
                    "description": param.description,
                    "min": param.min_value,
                    "max": param.max_value,
                    "default": param.default_value
                }
            
            # Save to file
            with open(filename, 'w') as f:
                json.dump(param_dict, f, indent=2)
            
            logger.info(f"Saved {len(self.parameters)} parameters to {filename}")
            QtWidgets.QMessageBox.information(
                self, 
                "Parameters Saved", 
                f"Successfully saved {len(self.parameters)} parameters to:\n{filename}"
            )
        
        except Exception as e:
            error_msg = f"Failed to save parameters: {e}"
            logger.error(error_msg, exc_info=True)
            QtWidgets.QMessageBox.critical(self, "Save Failed", error_msg)
    
    def _on_load_parameters_from_file(self):
        """Load parameter values from JSON file and apply to controller."""
        if not self.network_client.connected:
            QtWidgets.QMessageBox.warning(self, "Not Connected", "Must be connected to controller to load parameters")
            return
        
        # Open file dialog
        filename, _ = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "Load Parameters from JSON",
            "",
            "JSON Files (*.json);;All Files (*)"
        )
        
        if not filename:
            return  # User cancelled
        
        try:
            # Load JSON file
            with open(filename, 'r') as f:
                param_dict = json.load(f)
            
            parameters = param_dict.get("parameters", {})
            if not parameters:
                QtWidgets.QMessageBox.warning(self, "Invalid File", "No parameters found in JSON file")
                return
            
            # Confirm with user
            reply = QtWidgets.QMessageBox.question(
                self,
                "Confirm Load",
                f"Load {len(parameters)} parameters from file and apply to controller?\n\n"
                f"File: {filename}\n"
                f"Target: {param_dict.get('metadata', {}).get('target', 'unknown')}\n"
                f"Timestamp: {param_dict.get('metadata', {}).get('timestamp', 'unknown')}\n\n"
                f"This will change controller parameters!",
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No
            )
            
            if reply != QtWidgets.QMessageBox.Yes:
                return
            
            # Apply parameters one by one
            success_count = 0
            failed_params = []
            
            # Create progress dialog
            progress = QtWidgets.QProgressDialog(
                "Loading parameters from file...", 
                "Cancel", 
                0, 
                len(parameters), 
                self
            )
            progress.setWindowModality(QtCore.Qt.WindowModal)
            progress.setMinimumDuration(0)
            
            for idx, (key, param_data) in enumerate(parameters.items()):
                if progress.wasCanceled():
                    break
                
                progress.setValue(idx)
                progress.setLabelText(f"Setting parameter {idx+1}/{len(parameters)}: {key}")
                
                component = param_data.get("component")
                name = param_data.get("name")
                value = param_data.get("value")
                
                if component and name and value is not None:
                    # Send parameter set request
                    if self.network_client.send_parameter_set_request(component, name, value):
                        success_count += 1
                        # Small delay to avoid overwhelming controller
                        QtCore.QThread.msleep(10)
                    else:
                        failed_params.append(key)
                        logger.warning(f"Failed to set parameter: {key}")
            
            progress.setValue(len(parameters))
            
            # Show results
            if failed_params:
                QtWidgets.QMessageBox.warning(
                    self,
                    "Partial Success",
                    f"Loaded {success_count}/{len(parameters)} parameters.\n\n"
                    f"Failed to set {len(failed_params)} parameters:\n" + 
                    "\n".join(failed_params[:10]) +
                    (f"\n... and {len(failed_params)-10} more" if len(failed_params) > 10 else "")
                )
            else:
                QtWidgets.QMessageBox.information(
                    self,
                    "Load Complete",
                    f"Successfully loaded {success_count} parameters from file!"
                )
            
            logger.info(f"Loaded {success_count}/{len(parameters)} parameters from {filename}")
        
        except json.JSONDecodeError as e:
            error_msg = f"Invalid JSON file: {e}"
            logger.error(error_msg)
            QtWidgets.QMessageBox.critical(self, "Load Failed", error_msg)
        except Exception as e:
            error_msg = f"Failed to load parameters: {e}"
            logger.error(error_msg, exc_info=True)
            QtWidgets.QMessageBox.critical(self, "Load Failed", error_msg)
    
    def _on_refresh_parameters(self):
        """Refresh parameter values from controller."""
        if not self.network_client.connected:
            QtWidgets.QMessageBox.warning(self, "Not Connected", "Must be connected to controller to refresh parameters")
            return
        
        if not self.parameters:
            QtWidgets.QMessageBox.information(self, "No Parameters", "No parameters loaded yet. Use 'Load from Controller' first.")
            return
        
        # Confirm with user
        reply = QtWidgets.QMessageBox.question(
            self,
            "Confirm Refresh",
            f"Reload current values for all {len(self.parameters)} parameters from controller?",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No
        )
        
        if reply != QtWidgets.QMessageBox.Yes:
            return
        
        # Simply reload all parameters using existing mechanism
        # Clear current list and reload from controller
        self._on_load_parameters()
    
    # Plot updates
    def _update_plot(self):
        """Update plots from buffers.
        
        Delegates to the OscilloscopeWidget which handles all plot rendering.
        Works for both grid-side and machine-side oscilloscope displays.
        """
        if self.oscilloscope is None:
            return
        
        try:
            self.oscilloscope.update_plots()
        except Exception as e:
            logger.error(f"Plot error: {e}", exc_info=True)
    
    def _update_dashboard(self):
        """Update telemetry dashboard using appropriate handler based on packet type."""
        if not self.latest_packet:
            return
        
        try:
            packet = self.latest_packet
            
            # Dispatch to appropriate handler based on packet type
            if isinstance(packet, GridSideTelemetryPacket):
                self._update_grid_dashboard(packet)
            elif isinstance(packet, MachineSideTelemetryPacket):
                self._update_machine_dashboard(packet)
        
        except Exception as e:
            logger.error(f"Dashboard error: {e}", exc_info=True)
    
    def _update_grid_dashboard(self, packet: GridSideTelemetryPacket):
        """Update grid-side telemetry dashboard using signal definitions.
        
        Args:
            packet: Grid-side telemetry packet
        """
        from ..config.grid_side_telemetry_definitions import get_grid_side_telemetry
        
        definitions = get_grid_side_telemetry()
        
        # Iterate through each section in definitions
        for section_key, section_data in definitions.items():
            section_title = section_data.get('title', section_key.replace('_', ' ').title())
            for field in section_data['fields']:
                signal_name = field['name']
                signal_index = field['index']
                signal_type = field.get('type', 'float')
                signal_format = field.get('format', '.4f')  # Use format from definition
                enum_map = field.get('enum_map', None)
                
                # Get the value from packet by index based on type
                if signal_type == 'bool':
                    value = packet.get_bool_signal(signal_index)
                else:
                    value = packet.get_signal(signal_index)
                
                # Handle enum type: map integer value to string
                if signal_type == 'enum' and enum_map is not None:
                    # Convert float to int for enum lookup
                    int_value = int(value)
                    display_value = enum_map.get(int_value, f"UNKNOWN({int_value})")
                    self.telemetry_panel.update_signal(signal_name, display_value, section=section_title)
                # Update the signal with format from definition
                elif signal_format == 'bool':
                    self.telemetry_panel.update_signal(signal_name, value, section=section_title)
                else:
                    self.telemetry_panel.update_signal(signal_name, value, signal_format, section=section_title)
    
    def _update_machine_dashboard(self, packet: MachineSideTelemetryPacket):
        """Update machine-side telemetry dashboard using signal definitions.
        
        Args:
            packet: Machine-side telemetry packet
        """
        from ..config.machine_side_telemetry_definitions import get_machine_side_telemetry
        
        definitions = get_machine_side_telemetry()
        
        # Iterate through each section in definitions
        for section_key, section_data in definitions.items():
            section_title = section_data.get('title', section_key.replace('_', ' ').title())
            for field in section_data['fields']:
                signal_name = field['name']
                signal_index = field['index']
                signal_type = field.get('type', 'float')
                signal_format = field.get('format', '.4f')  # Use format from definition
                enum_map = field.get('enum_map', None)
                
                # Get the value from packet by index based on type
                if signal_type == 'bool':
                    value = packet.get_bool_signal(signal_index)
                else:
                    value = packet.get_signal(signal_index)
                
                # Handle enum type: map integer value to string
                if signal_type == 'enum' and enum_map is not None:
                    # Convert float to int for enum lookup
                    int_value = int(value)
                    display_value = enum_map.get(int_value, f"UNKNOWN({int_value})")
                    self.telemetry_panel.update_signal(signal_name, display_value, section=section_title)
                # Update the signal with format from definition
                elif signal_format == 'bool':
                    self.telemetry_panel.update_signal(signal_name, value, section=section_title)
                else:
                    self.telemetry_panel.update_signal(signal_name, value, signal_format, section=section_title)
        
        # Pulse Generator
        # if packet.pulse_generator:
        #     pg = packet.pulse_generator
        #     self.telemetry_panel.update_signal('pulse_p1', pg.firing_pulse_1)
        #     self.telemetry_panel.update_signal('pulse_p2', pg.firing_pulse_2)
        #     self.telemetry_panel.update_signal('pulse_p3', pg.firing_pulse_3)
        #     self.telemetry_panel.update_signal('pulse_p4', pg.firing_pulse_4)
        #     self.telemetry_panel.update_signal('pulse_p5', pg.firing_pulse_5)


def main():
    """Entry point."""
    import argparse
    parser = argparse.ArgumentParser(description="SFC Monitor with Parameters")
    parser.add_argument('--host', default='192.168.2.100', help='MCU IP address')
    parser.add_argument('--port', type=int, default=None, help='MCU telemetry port (auto-determined by --target if not specified)')
    parser.add_argument('--target', choices=['grid', 'machine'], default='grid', help='Target side: grid (port 7000) or machine (port 7001) [default: grid]')
    parser.add_argument('--verbose', action='store_true')
    parser.add_argument('--decimate', type=int, default=1, help='Process 1 out of N packets')
    parser.add_argument('--window', type=int, default=1000, help='Number of samples to display on X-axis')
    args = parser.parse_args()
    
    # Auto-determine port based on target if not explicitly specified
    if args.port is None:
        args.port = 7000 if args.target == 'grid' else 7001
    
    app = QtWidgets.QApplication(sys.argv)
    window = TelemetryMonitorWindow(args.host, args.port, args.target, args.verbose, args.decimate, args.window)
    window.show()
    return app.exec_()


if __name__ == '__main__':
    sys.exit(main())
