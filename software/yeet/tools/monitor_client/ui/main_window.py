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
from monitor_client.config import get_machine_side_plots, get_default_visible_groups

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
        self.avg_modulator_enabled = 0.0
        self.avg_count = 0  # Counter for averaging
        
        # Create oscilloscope widget (manages all plots and buffers)
        # Note: Grid-side and machine-side have different plot sets
        self.oscilloscope = OscilloscopeWidget(window_size=window_size, layout_mode=self.layout_mode)
        
        if self.target_side == 'grid':
            self.oscilloscope.create_standard_plots()
        else:
            # For machine-side, create machine-specific plots
            self._create_machine_side_plots()
        
        # Latest telemetry for dashboard
        self.latest_packet = None
        
        # Parameters
        self.parameters: List[ParameterMetadata] = []
        self.param_loading = False
        self.was_streaming_before_param_load = False
        self.pending_parameter_indices: List[int] = []  # Queue of indices to request
        self.param_request_timer = QtCore.QTimer(self)
        self.param_request_timer.timeout.connect(self._send_next_parameter_request)
        self.param_request_timer.setInterval(50)  # 50ms between requests (20 requests/sec)
        
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
        self.window_size_spinbox.setRange(10, 10000)
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
    
    def _build_parameter_panel(self) -> QtWidgets.QWidget:
        """Build parameter configuration panel."""
        panel = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        
        # Header
        header = QtWidgets.QLabel("PARAMETERS")
        header.setStyleSheet("color: #00ffff; font-size: 14pt; font-weight: bold; padding: 5px;")
        layout.addWidget(header)
        
        # Load button
        self.load_params_btn = QtWidgets.QPushButton("Load Parameters")
        self.load_params_btn.setEnabled(False)
        self.load_params_btn.clicked.connect(self._on_load_parameters)
        layout.addWidget(self.load_params_btn)
        
        # Parameter tree
        self.param_tree = QtWidgets.QTreeWidget()
        self.param_tree.setHeaderLabels(["Parameter", "Value"])
        self.param_tree.setColumnWidth(0, 200)
        self.param_tree.itemDoubleClicked.connect(self._on_parameter_double_clicked)
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
    
    def _create_machine_side_plots(self):
        """Create machine-side oscilloscope plots for monitoring key components.
        
        Machine plots include:
        - Measurements: Voltage (mag, A, B, C), speed, commutation
        - Observer: Estimated voltage, frequency, angle
        - Speed Controller: Torque ref, measured speed, error, PI output
        - Torque Controller: Current refs, references, saturation
        - Speed Generator: Speed reference, torque limits
        - Modulator: Firing angles, enabled state
        - Excitation: Voltage setpoint, field state, running
        """
        # Clear any existing state from grid-side plots
        self.oscilloscope.plot_groups.clear()
        self.oscilloscope.plot_curves.clear()
        self.oscilloscope.buffers.clear()
        self.oscilloscope.view_configs.clear()
        self.oscilloscope.visible_views_order = []
        self.oscilloscope._plot_layout_map.clear()
        
        # Recreate the plots_container to avoid pyqtgraph internal state corruption
        old_container = self.oscilloscope.plots_container
        self.oscilloscope.plots_container = pg.GraphicsLayoutWidget()
        self.oscilloscope.plots_container.setBackground('#1e1e1e')
        
        # Replace the widget in the layout
        layout = self.oscilloscope.layout()
        layout.replaceWidget(old_container, self.oscilloscope.plots_container)
        old_container.deleteLater()
        
        # Reset view buttons
        for button in list(self.oscilloscope.view_buttons.values()):
            self.oscilloscope.view_button_layout.removeWidget(button)
            button.deleteLater()
        self.oscilloscope.view_buttons.clear()
        
        # Clear all items from view button layout
        while self.oscilloscope.view_button_layout.count():
            item = self.oscilloscope.view_button_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()
        
        # Load plot definitions from configuration
        plot_definitions = get_machine_side_plots()
        default_visible = get_default_visible_groups('machine')
        
        # Create plots from definitions (following grid-side pattern)
        self.oscilloscope.visible_views_order = []
        for group_name, group_plots in plot_definitions.items():
            self.oscilloscope.plot_groups[group_name] = {}
            self.oscilloscope.view_configs[group_name] = {
                'visible': group_name in default_visible
            }
            
            if self.oscilloscope.view_configs[group_name]['visible']:
                self.oscilloscope.visible_views_order.append(group_name)
            
            for plot_name, plot_config in group_plots.items():
                self.oscilloscope._create_plot(group_name, plot_name, plot_config)
        
        # Create view toggle buttons for each defined group
        self.oscilloscope._create_view_buttons()
        
        # Initial layout organization - defer this to avoid layout state issues
        # The layout will be properly organized on first data update
        # This avoids the "Cell already taken" error from pyqtgraph
        QtCore.QTimer.singleShot(0, self.oscilloscope._reorganize_subplot_layout)
    
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
        
        if connected:
            self.status_label.setText("Connected - Ready")
        else:
            self.status_label.setText("Disconnected")
            self.param_status_label.setText("Not loaded")
            self.param_tree.clear()
            self.parameters.clear()
    
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
                
                # Extract values and store with sample index
                if packet.measurements:
                    meas = packet.measurements
                    self.oscilloscope.add_sample_to_buffer('measurements_voltage_and_current', 
                                                           (self.sample_index, meas.grid_voltage_magnitude_pu,
                                                            meas.grid_current_a_pu, meas.grid_current_b_pu, meas.grid_current_c_pu))
                    self.oscilloscope.add_sample_to_buffer('current_measurements_abc_dclink', (self.sample_index, 
                                                           meas.grid_current_a_pu, meas.grid_current_b_pu, 
                                                           meas.grid_current_c_pu, meas.dc_link_current_pu))
                    
                    # Store to persistent data if enabled
                    if self.enable_persistence:
                        sample = TelemetrySample(
                            timestamp_ms=packet.header.timestamp_ms,
                            sequence_number=packet.header.sequence_number,
                            signal_count=packet.header.signal_count,
                            signals=packet.signals,
                        )
                        self.data_persistence.add_sample(sample)
                
                if packet.observer:
                    obs = packet.observer
                    self.oscilloscope.add_sample_to_buffer('observer_voltage_freq', (self.sample_index, obs.estimated_voltage_magnitude_pu, obs.estimated_frequency_pu))
                
                if packet.controller:
                    ctrl = packet.controller
                    self.oscilloscope.add_sample_to_buffer('current_controller_dclink', (self.sample_index, ctrl.dc_link_current_reference, ctrl.dc_link_current_measured))
                
                # New: Extract all signals for expanded plots
                if packet.measurements and packet.observer:
                    meas = packet.measurements
                    obs = packet.observer
                    self.oscilloscope.add_sample_to_buffer('observer_voltage_analysis_voltages', (self.sample_index,
                                                                 meas.grid_voltage_a_pu, meas.grid_voltage_b_pu, meas.grid_voltage_c_pu,
                                                                 meas.grid_voltage_magnitude_pu, obs.estimated_voltage_magnitude_pu))
                    self.oscilloscope.add_sample_to_buffer('observer_voltage_analysis_freq_angle', (self.sample_index,
                                                                 obs.estimated_frequency_hz, obs.estimated_angle_rad,
                                                                 obs.estimated_frequency_pu))
                
                if packet.controller:
                    ctrl = packet.controller
                    self.oscilloscope.add_sample_to_buffer('operation_mode_monitor_controller_status', (self.sample_index,
                                                               ctrl.current_error_pu,
                                                               ctrl.controller_output_pu,
                                                               1.0 if ctrl.controller_saturated else 0.0))
                
                if packet.modulator:
                    mod = packet.modulator
                    self.oscilloscope.add_sample_to_buffer('modulator_firing_angle_control', (self.sample_index,
                                                   mod.firing_angle_input_pu,
                                                   mod.firing_angle_output_pu,
                                                   1.0 if mod.is_enabled else 0.0))
                
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
                    self.avg_modulator_enabled += 1.0 if mod.is_enabled else 0.0
                
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
                                               self.avg_observer_freq / self.avg_count))
                    self.oscilloscope.add_sample_to_buffer('current_controller_dclink', (self.sample_index,
                                             self.avg_dclink_ref / self.avg_count,
                                             self.avg_dclink_measured / self.avg_count))
                    
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
                                                   self.avg_modulator_enabled / self.avg_count))
                    
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
                    self.avg_modulator_enabled = 0.0
                    self.avg_count = 0
                    
                    self.sample_index += 1
        
        except Exception as e:
            logger.error(f"Packet error: {e}", exc_info=True)
    
    def _on_machine_packet(self, packet: MachineSideTelemetryPacket):
        """Handle machine-side telemetry packet.
        
        Extract machine measurements, observer data, and controller states
        and populate oscilloscope buffers for real-time visualization.
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
                # Extract machine-side data
                meas = packet.measurements
                obs = packet.observer
                mod = packet.modulator
                ctrl = packet.controller  # This is MachineControllerData
                torque = packet.torque_controller
                speed = packet.speed_controller
                gen = packet.speed_generator
                exc = packet.excitation_controller
                
                # Machine Measurements - All in one plot
                self.oscilloscope.add_sample_to_buffer(
                    'measurements_all_measurements',
                    (self.sample_index,
                     meas.machine_voltage_magnitude_filtered_pu,
                     meas.machine_voltage_a_pu,
                     meas.machine_voltage_b_pu,
                     meas.machine_voltage_c_pu,
                     meas.machine_side_speed,
                     meas.machine_voltage_magnitude,
                     float(meas.is_commuting))
                )
                
                # Observer - All in one plot
                self.oscilloscope.add_sample_to_buffer(
                    'observer_all_observer',
                    (self.sample_index,
                     obs.estimated_voltage_magnitude_pu,
                     obs.estimated_frequency_pu,
                     obs.estimated_frequency_hz,
                     obs.estimated_angle_rad,
                     obs.estimated_voltage_alpha_pu)
                )
                
                # Speed Controller - All in one plot
                self.oscilloscope.add_sample_to_buffer(
                    'speed_controller_all_speed_control',
                    (self.sample_index,
                     speed.speed_torque_reference,
                     speed.speed_measured_speed_pu,
                     speed.speed_error,
                     speed.speed_pi_output,
                     float(speed.speed_is_saturated))
                )
                
                # Torque Controller - All in one plot
                self.oscilloscope.add_sample_to_buffer(
                    'torque_controller_all_torque_control',
                    (self.sample_index,
                     torque.torque_current_reference_pu,
                     torque.torque_base_reference,
                     torque.torque_total_reference,
                     torque.torque_limited_reference,
                     float(torque.torque_is_saturated))
                )
                
                # Speed Generator - All in one plot
                self.oscilloscope.add_sample_to_buffer(
                    'speed_generator_all_speed_gen',
                    (self.sample_index,
                     gen.generator_speed_reference_pu,
                     gen.generator_torque_limit_max_pu,
                     gen.generator_torque_limit_min_pu,
                     float(gen.generator_is_profile_complete),
                     float(gen.generator_is_breakaway_boost_active))
                )
                
                # Modulator - All in one plot
                self.oscilloscope.add_sample_to_buffer(
                    'modulator_all_modulator',
                    (self.sample_index,
                     mod.firing_angle_input_pu,
                     mod.firing_angle_output_pu,
                     float(mod.is_enabled),
                     float(mod.force_commutation_ack),
                     0.0)  # rotor_sector not available in current packet structure
                )
                
                # Excitation - All in one plot
                self.oscilloscope.add_sample_to_buffer(
                    'excitation_all_excitation',
                    (self.sample_index,
                     exc.excitation_voltage_setpoint_pu,
                     float(exc.excitation_enable),
                     float(exc.excitation_is_field_established),
                     float(exc.excitation_is_running))
                )
                
                # Trigger plot update
                self.oscilloscope.update_plots()
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
                        self.param_status_label.setText(f"Loaded {len(self.parameters)} parameters")
                        logger.info(f"All parameters loaded: {len(self.parameters)}")
                        
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
    
    # Plot updates
    def _update_plot(self):
        """Update plots from buffers.
        
        Delegates to the OscilloscopeWidget which handles all plot rendering.
        Only updates for grid-side (machine-side doesn't have oscilloscope).
        """
        if self.oscilloscope is None:
            return  # Machine-side: no oscilloscope
        
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
        """Update grid-side telemetry dashboard.
        
        Args:
            packet: Grid-side telemetry packet
        """
        # Measurements
        if packet.measurements:
            m = packet.measurements
            self.telemetry_panel.update_signal('grid_voltage_magnitude_pu', m.grid_voltage_magnitude_pu, '.4f')
            self.telemetry_panel.update_signal('grid_voltage_a_pu', m.grid_voltage_a_pu, '.4f')
            self.telemetry_panel.update_signal('grid_voltage_b_pu', m.grid_voltage_b_pu, '.4f')
            self.telemetry_panel.update_signal('grid_voltage_c_pu', m.grid_voltage_c_pu, '.4f')
            self.telemetry_panel.update_signal('grid_current_magnitude_pu', m.grid_current_magnitude_pu, '.4f')
            self.telemetry_panel.update_signal('grid_current_a_pu', m.grid_current_a_pu, '.4f')
            self.telemetry_panel.update_signal('grid_current_b_pu', m.grid_current_b_pu, '.4f')
            self.telemetry_panel.update_signal('grid_current_c_pu', m.grid_current_c_pu, '.4f')
            self.telemetry_panel.update_signal('dc_link_current_pu', m.dc_link_current_pu, '.4f')
            self.telemetry_panel.update_signal('is_commuting', m.is_commuting)
        
        # Observer
        if packet.observer:
            o = packet.observer
            self.telemetry_panel.update_signal('estimated_voltage_magnitude_pu', o.estimated_voltage_magnitude_pu, '.4f')
            self.telemetry_panel.update_signal('estimated_frequency_pu', o.estimated_frequency_pu, '.4f')
            self.telemetry_panel.update_signal('estimated_angle_rad', o.estimated_angle_rad, '.4f')
        
        # Controller
        if packet.controller:
            c = packet.controller
            self.telemetry_panel.update_signal('dc_link_current_reference', c.dc_link_current_reference, '.4f')
            self.telemetry_panel.update_signal('dc_link_current_measured', c.dc_link_current_measured, '.4f')
            self.telemetry_panel.update_signal('current_error_pu', c.current_error_pu, '.4f')
            self.telemetry_panel.update_signal('controller_output_pu', c.controller_output_pu, '.4f')
            self.telemetry_panel.update_signal('operating_mode', "Forced" if c.operating_mode else "Natural")
            self.telemetry_panel.update_signal('controller_saturated', c.controller_saturated)
            self.telemetry_panel.update_signal('freeze_integrator', c.freeze_integrator)
        
        # Modulator
        if packet.modulator:
            mod = packet.modulator
            self.telemetry_panel.update_signal('is_enabled', mod.is_enabled)
            self.telemetry_panel.update_signal('firing_angle_input_pu', mod.firing_angle_input_pu, '.4f')
            self.telemetry_panel.update_signal('firing_angle_output_pu', mod.firing_angle_output_pu, '.4f')
            self.telemetry_panel.update_signal('force_commutation_ack', mod.force_commutation_ack)
        
        # State Machine
        if packet.state_machine:
            sm = packet.state_machine
            state_names = ["IDLE", "AUXILIARIES_ON", "BREAKERS_OFF", "BREAKERS_ON", "OPERATION_OFF", "OPERATION_ON", "SHUTDOWN_REQUESTED"]
            state_text = state_names[sm.current_state] if 0 <= sm.current_state < len(state_names) else f"UNKNOWN({sm.current_state})"
            self.telemetry_panel.update_signal('current_state', state_text)
            self.telemetry_panel.update_signal('pulse_releases', sm.pulse_releases)
            self.telemetry_panel.update_signal('auxiliaries_are_on', sm.auxiliaries_are_on)
            self.telemetry_panel.update_signal('breakers_are_on', sm.breakers_are_on)
            self.telemetry_panel.update_signal('start_command', sm.start_command)
            self.telemetry_panel.update_signal('stop_command', sm.stop_command)
            self.telemetry_panel.update_signal('is_current_extinguished', sm.is_current_extinguished)
            self.telemetry_panel.update_signal('is_shutdown_requested', sm.is_shutdown_requested)
    
    def _update_machine_dashboard(self, packet: MachineSideTelemetryPacket):
        """Update machine-side telemetry dashboard.
        
        Args:
            packet: Machine-side telemetry packet
        """
        # Machine Measurements
        if packet.measurements:
            m = packet.measurements
            self.telemetry_panel.update_signal('machine_voltage_magnitude_filtered_pu', m.machine_voltage_magnitude_filtered_pu, '.4f')
            self.telemetry_panel.update_signal('machine_voltage_a_pu', m.machine_voltage_a_pu, '.4f')
            self.telemetry_panel.update_signal('machine_voltage_b_pu', m.machine_voltage_b_pu, '.4f')
            self.telemetry_panel.update_signal('machine_voltage_c_pu', m.machine_voltage_c_pu, '.4f')
            self.telemetry_panel.update_signal('machine_side_speed', m.machine_side_speed, '.4f')
            self.telemetry_panel.update_signal('machine_voltage_magnitude', m.machine_voltage_magnitude, '.4f')
            self.telemetry_panel.update_signal('is_commuting', m.is_commuting)
        
        # Observer
        if packet.observer:
            o = packet.observer
            self.telemetry_panel.update_signal('estimated_voltage_magnitude_pu', o.estimated_voltage_magnitude_pu, '.4f')
            self.telemetry_panel.update_signal('estimated_frequency_pu', o.estimated_frequency_pu, '.4f')
            self.telemetry_panel.update_signal('estimated_angle_rad', o.estimated_angle_rad, '.4f')
            self.telemetry_panel.update_signal('estimated_frequency_hz', o.estimated_frequency_hz, '.4f')
            self.telemetry_panel.update_signal('estimated_voltage_alpha_pu', o.estimated_voltage_alpha_pu, '.4f')
        
        # Modulator
        if packet.modulator:
            mod = packet.modulator
            self.telemetry_panel.update_signal('firing_angle_rad', mod.firing_angle_output_pu, '.4f')
            self.telemetry_panel.update_signal('is_enabled', mod.is_enabled)
            self.telemetry_panel.update_signal('release_pulses', mod.is_requesting_pi_reset)
            self.telemetry_panel.update_signal('request_force_commutation', mod.force_commutation_ack)
            self.telemetry_panel.update_signal('current_rotor_sector', mod.firing_angle_input_pu, '.4f')
        
        # Machine Controller
        if packet.controller:
            c = packet.controller
            self.telemetry_panel.update_signal('controller_firing_angle', c.controller_firing_angle, '.4f')
            self.telemetry_panel.update_signal('controller_machine_speed', c.controller_machine_speed, '.4f')
            self.telemetry_panel.update_signal('controller_dc_link_current', c.controller_dc_link_current, '.4f')
        
        # Torque Controller
        if packet.torque_controller:
            tc = packet.torque_controller
            self.telemetry_panel.update_signal('torque_current_reference_pu', tc.torque_current_reference_pu, '.4f')
            self.telemetry_panel.update_signal('torque_base_reference', tc.torque_base_reference, '.4f')
            self.telemetry_panel.update_signal('torque_total_reference', tc.torque_total_reference, '.4f')
            self.telemetry_panel.update_signal('torque_limited_reference', tc.torque_limited_reference, '.4f')
            self.telemetry_panel.update_signal('torque_is_saturated', tc.torque_is_saturated)
        
        # Speed Controller
        if packet.speed_controller:
            sc = packet.speed_controller
            self.telemetry_panel.update_signal('speed_torque_reference', sc.speed_torque_reference, '.4f')
            self.telemetry_panel.update_signal('speed_measured_speed_pu', sc.speed_measured_speed_pu, '.4f')
            self.telemetry_panel.update_signal('speed_error', sc.speed_error, '.4f')
            self.telemetry_panel.update_signal('speed_pi_output', sc.speed_pi_output, '.4f')
            self.telemetry_panel.update_signal('speed_is_saturated', sc.speed_is_saturated)
        
        # Speed Generator
        if packet.speed_generator:
            sg = packet.speed_generator
            self.telemetry_panel.update_signal('generator_speed_reference_pu', sg.generator_speed_reference_pu, '.4f')
            self.telemetry_panel.update_signal('generator_torque_limit_max_pu', sg.generator_torque_limit_max_pu, '.4f')
            self.telemetry_panel.update_signal('generator_torque_limit_min_pu', sg.generator_torque_limit_min_pu, '.4f')
            self.telemetry_panel.update_signal('generator_is_profile_complete', sg.generator_is_profile_complete)
            self.telemetry_panel.update_signal('generator_is_breakaway_boost_active', sg.generator_is_breakaway_boost_active)
        
        # Excitation Controller
        if packet.excitation_controller:
            ec = packet.excitation_controller
            self.telemetry_panel.update_signal('excitation_voltage_setpoint_pu', ec.excitation_voltage_setpoint_pu, '.4f')
            self.telemetry_panel.update_signal('excitation_enable', ec.excitation_enable)
            self.telemetry_panel.update_signal('excitation_is_field_established', ec.excitation_is_field_established)
            self.telemetry_panel.update_signal('excitation_is_running', ec.excitation_is_running)
        
        # Pulse Generator
        if packet.pulse_generator:
            pg = packet.pulse_generator
            self.telemetry_panel.update_signal('pulse_p1', pg.firing_pulse_1)
            self.telemetry_panel.update_signal('pulse_p2', pg.firing_pulse_2)
            self.telemetry_panel.update_signal('pulse_p3', pg.firing_pulse_3)
            self.telemetry_panel.update_signal('pulse_p4', pg.firing_pulse_4)
            self.telemetry_panel.update_signal('pulse_p5', pg.firing_pulse_5)


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
