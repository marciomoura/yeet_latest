#!/usr/bin/env python3
"""Minimal oscilloscope with telemetry and parameter configuration.

Architecture:
- Left panel: Parameter list with load button and double-click to edit
- Center: Four subplots (voltage, currents, observer, DC link control)
- Right panel: Telemetry dashboard
- Python deque for data (automatic FIFO, no ring buffer math)
- Direct packet processing (no queues, no deferred work)
- Two timers: 100ms for plots, 200ms for dashboard
"""

import sys
import logging
import time
from collections import deque
from typing import Optional, List, Dict

import numpy as np
from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg

from monitor_client.core.unified_network_client import UnifiedNetworkClient
from monitor_client.core.telemetry_protocol import GridSideTelemetryPacket
from monitor_client.core.parameter_protocol import ParameterMetadata

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
    
    def __init__(self, host: str, port: int, verbose: bool = False, decimate: int = 1, window_size: int = 2):
        super().__init__()
        
        logger.info(f"=== Monitor with Parameters (verbose={verbose}, decimate={decimate}, window={window_size}) ===")
        
        self.setWindowTitle("SFC Monitor - Parameters + Telemetry")
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
        self.packet_counter = 0
        self.sample_index = 0
        self.start_time = None  # Time when telemetry started (in seconds)
        self.window_size = window_size
        self.layout_mode = "1xN"  # "1xN" for vertical stack or "2x2" for grid
        
        # Simple deque buffers
        self.voltage_buffer = deque(maxlen=2000)
        self.current_buffer = deque(maxlen=2000)
        self.observer_buffer = deque(maxlen=2000)
        self.dclink_buffer = deque(maxlen=2000)
        
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
        self.window_size_spinbox.setRange(1, 300)
        self.window_size_spinbox.setSingleStep(5)
        self.window_size_spinbox.setValue(window_size)
        self.window_size_spinbox.setToolTip("Time window in seconds to display on X-axis")
        self.window_size_spinbox.valueChanged.connect(self._on_window_size_changed)
        telem_layout.addWidget(self.window_size_spinbox)
        telem_layout.addWidget(QtWidgets.QLabel("seconds"))
        
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
        
        # Create view toggle buttons for grid-side components
        # Set visible=True for views you want enabled by default
        self.view_configs = {
            'measurements': {'button': None, 'visible': True},
            'observer': {'button': None, 'visible': True},
            'current_controller': {'button': None, 'visible': True},
            'modulator': {'button': None, 'visible': False},
            'operation_mode_monitor': {'button': None, 'visible': False},
            'state_machine': {'button': None, 'visible': False},
        }
        
        for view_name in self.view_configs.keys():
            btn = QtWidgets.QPushButton(f"{'Hide' if self.view_configs[view_name]['visible'] else 'Show'} {view_name.replace('_', ' ').title()}")
            btn.clicked.connect(lambda checked, v=view_name: self._on_toggle_view(v))
            btn.setToolTip(f"Toggle {view_name} oscilloscope view")
            view_layout.addWidget(btn)
            self.view_configs[view_name]['button'] = btn
        
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
        
        # Right: Telemetry dashboard (20% width)
        self.telemetry_panel = self._build_telemetry_panel()
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
        
        logger.info("Monitor with parameters initialized")
    
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
        """Build plots with dynamic ordering based on user clicks."""
        self.plots_container = QtWidgets.QWidget()
        
        # Track visible views in order of activation
        # Initialize with views that should be visible by default
        self.visible_views_order = [view for view, config in self.view_configs.items() if config['visible']]
        
        # Store plot groups by view type for reference
        self.plot_groups = {
            'measurements': [],
            'observer': [],
            'current_controller': [],
            'modulator': [],
            'operation_mode_monitor': [],
            'state_machine': [],
        }
        
        # Plot 1: Grid Voltage - measurements view
        self.voltage_plot = pg.PlotWidget()
        self.voltage_plot.setBackground('#1e1e1e')
        self.voltage_plot.setLabel('left', 'Grid Voltage (pu)', color='#ffffff')
        self.voltage_plot.setLabel('bottom', 'Time (s)', color='#ffffff')
        self.voltage_plot.showGrid(x=True, y=True, alpha=0.3)
        self.voltage_plot.getAxis('left').setTextPen('#ffffff')
        self.voltage_plot.getAxis('bottom').setTextPen('#ffffff')
        self.voltage_plot.addLegend()
        self.voltage_curve = self.voltage_plot.plot(pen=pg.mkPen(color='g', width=2), name='Voltage')
        self.voltage_plot.setMinimumHeight(200)
        self.voltage_plot.hide()
        self.plot_groups['measurements'].append(self.voltage_plot)
        
        # Plot 2: Grid Phase Currents - measurements view
        self.current_plot = pg.PlotWidget()
        self.current_plot.setBackground('#1e1e1e')
        self.current_plot.setLabel('left', 'Grid Current (pu)', color='#ffffff')
        self.current_plot.setLabel('bottom', 'Time (s)', color='#ffffff')
        self.current_plot.showGrid(x=True, y=True, alpha=0.3)
        self.current_plot.getAxis('left').setTextPen('#ffffff')
        self.current_plot.getAxis('bottom').setTextPen('#ffffff')
        self.current_plot.addLegend()
        self.current_curve_a = self.current_plot.plot(pen=pg.mkPen('r', width=2), name='Phase A')
        self.current_curve_b = self.current_plot.plot(pen=pg.mkPen('g', width=2), name='Phase B')
        self.current_curve_c = self.current_plot.plot(pen=pg.mkPen('b', width=2), name='Phase C')
        self.current_plot.setMinimumHeight(200)
        self.current_plot.hide()
        self.plot_groups['measurements'].append(self.current_plot)
        
        # Plot 3: Observer - observer view
        self.observer_plot = pg.PlotWidget()
        self.observer_plot.setBackground('#1e1e1e')
        self.observer_plot.setLabel('left', 'Observer', color='#ffffff')
        self.observer_plot.setLabel('bottom', 'Time (s)', color='#ffffff')
        self.observer_plot.showGrid(x=True, y=True, alpha=0.3)
        self.observer_plot.getAxis('left').setTextPen('#ffffff')
        self.observer_plot.getAxis('bottom').setTextPen('#ffffff')
        self.observer_plot.addLegend()
        self.observer_voltage_curve = self.observer_plot.plot(pen=pg.mkPen('c', width=2), name='V Est (pu)')
        self.observer_freq_curve = self.observer_plot.plot(pen=pg.mkPen('m', width=2), name='Freq (pu)')
        self.observer_plot.setMinimumHeight(200)
        self.observer_plot.hide()
        self.plot_groups['observer'].append(self.observer_plot)
        
        # Plot 4: DC Link Control - current_controller view
        self.dclink_plot = pg.PlotWidget()
        self.dclink_plot.setBackground('#1e1e1e')
        self.dclink_plot.setLabel('left', 'DC Link Current (pu)', color='#ffffff')
        self.dclink_plot.setLabel('bottom', 'Time (s)', color='#ffffff')
        self.dclink_plot.showGrid(x=True, y=True, alpha=0.3)
        self.dclink_plot.getAxis('left').setTextPen('#ffffff')
        self.dclink_plot.getAxis('bottom').setTextPen('#ffffff')
        self.dclink_plot.addLegend()
        self.dclink_ref_curve = self.dclink_plot.plot(pen=pg.mkPen('y', width=2), name='Reference')
        self.dclink_measured_curve = self.dclink_plot.plot(pen=pg.mkPen('w', width=2), name='Measured')
        self.dclink_plot.setMinimumHeight(200)
        self.dclink_plot.hide()
        self.plot_groups['current_controller'].append(self.dclink_plot)
        
        # Display default views
        self._rebuild_plots_layout()
        
        return self.plots_container
    
    def _build_telemetry_panel(self) -> QtWidgets.QWidget:
        """Build telemetry dashboard panel."""
        panel = QtWidgets.QWidget()
        panel.setMaximumWidth(400)
        
        # Scrollable area
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        
        # Content widget
        content = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(content)
        layout.setSpacing(10)
        
        # Header
        header = QtWidgets.QLabel("TELEMETRY")
        header.setStyleSheet("color: #00ffff; font-size: 14pt; font-weight: bold;")
        layout.addWidget(header)
        
        # Store labels for updates
        self.telem_labels: Dict[str, QtWidgets.QLabel] = {}
        
        # State Machine
        self._add_telemetry_section(layout, "State Machine", [
            "current_state", "pulse_releases", "auxiliaries_are_on", "breakers_are_on",
            "start_command", "stop_command", "is_current_extinguished", "is_shutdown_requested"
        ])

        # Grid Measurements
        self._add_telemetry_section(layout, "Grid Measurements", [
            "grid_voltage_magnitude_pu", "grid_voltage_a_pu", "grid_voltage_b_pu", "grid_voltage_c_pu",
            "grid_current_magnitude_pu", "grid_current_a_pu", "grid_current_b_pu", "grid_current_c_pu",
            "dc_link_current_pu", "is_commuting"
        ])
        
        # Observer
        self._add_telemetry_section(layout, "Observer", [
            "estimated_voltage_magnitude_pu", "estimated_frequency_pu", "estimated_angle_rad"
        ])
        
        # Controller
        self._add_telemetry_section(layout, "Controller", [
            "dc_link_current_reference", "dc_link_current_measured", 
            "current_error_pu", "controller_output_pu",
            "operating_mode", "controller_saturated", "freeze_integrator"
        ])
        
        # Modulator
        self._add_telemetry_section(layout, "Modulator", [
            "is_enabled", "firing_angle_input_pu", "firing_angle_output_pu", "force_commutation_ack"
        ])
        
        layout.addStretch()
        scroll.setWidget(content)
        
        # Wrap in layout
        panel_layout = QtWidgets.QVBoxLayout(panel)
        panel_layout.setContentsMargins(0, 0, 0, 0)
        panel_layout.addWidget(scroll)
        
        return panel
    
    def _add_telemetry_section(self, layout: QtWidgets.QVBoxLayout, title: str, fields: List[str]):
        """Add a telemetry section with fields."""
        section_label = QtWidgets.QLabel(title)
        section_label.setStyleSheet("color: #ffff00; font-weight: bold; font-size: 11pt; margin-top: 10px;")
        layout.addWidget(section_label)
        
        for field in fields:
            field_layout = QtWidgets.QHBoxLayout()
            field_layout.setSpacing(5)
            
            name_label = QtWidgets.QLabel(field + ":")
            name_label.setStyleSheet("color: #d0d0d0; font-size: 9pt;")
            name_label.setMinimumWidth(200)
            field_layout.addWidget(name_label)
            
            value_label = QtWidgets.QLabel("---")
            value_label.setStyleSheet("color: #ffffff; font-family: 'Courier New'; font-size: 9pt;")
            value_label.setAlignment(QtCore.Qt.AlignRight)
            field_layout.addWidget(value_label)
            
            self.telem_labels[field] = value_label
            layout.addLayout(field_layout)
    
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
        self.voltage_buffer.clear()
        self.current_buffer.clear()
        self.observer_buffer.clear()
        self.dclink_buffer.clear()
        self.sample_index = 0
        self.packet_counter = 0
        self.start_time = time.time()  # Record start time
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
        self.voltage_buffer.clear()
        self.current_buffer.clear()
        self.observer_buffer.clear()
        self.dclink_buffer.clear()
        self.sample_index = 0
        self.packet_counter = 0
        logger.info("Cleared")
    
    def _on_window_size_changed(self, value: int):
        """Handle window size change at runtime."""
        self.window_size = value
        logger.info(f"Window size changed to {value} samples")
    
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
        
        # Rebuild layout with new mode
        self._rebuild_plots_layout()
    
    def _on_toggle_view(self, view_name: str):
        """Toggle visibility of plots for a specific view."""
        if view_name not in self.view_configs:
            logger.warning(f"Unknown view: {view_name}")
            return
        
        # Toggle visibility
        self.view_configs[view_name]['visible'] = not self.view_configs[view_name]['visible']
        
        # Update button text
        btn = self.view_configs[view_name]['button']
        display_name = view_name.replace('_', ' ').title()
        
        if self.view_configs[view_name]['visible']:
            btn.setText(f"Hide {display_name}")
            logger.info(f"{display_name} view shown")
            # Add view to visible list in order
            if view_name not in self.visible_views_order:
                self.visible_views_order.append(view_name)
        else:
            btn.setText(f"Show {display_name}")
            logger.info(f"{display_name} view hidden")
            # Remove view from visible list
            if view_name in self.visible_views_order:
                self.visible_views_order.remove(view_name)
        
        # Rebuild layout with plots in order
        self._rebuild_plots_layout()
    
    def _rebuild_plots_layout(self):
        """Rebuild the plots layout based on layout mode (1xN vertical or 2x2 grid)."""
        # Remove and delete old layout properly
        old_layout = self.plots_container.layout()
        if old_layout:
            # Remove all items and widgets
            while old_layout.count() > 0:
                item = old_layout.takeAt(0)
                if item.widget():
                    item.widget().setParent(None)  # Remove widget from layout
            # Delete the old layout
            QtWidgets.QWidget().setLayout(old_layout)  # Transfer ownership to temporary widget
        
        # Create new layout based on mode
        if self.layout_mode == "1xN":
            self._rebuild_plots_1xN_layout()
        else:  # 2x2
            self._rebuild_plots_2x2_layout()
    
    def _rebuild_plots_1xN_layout(self):
        """Rebuild layout as 1xN vertical stack."""
        # Create new vertical layout
        new_layout = QtWidgets.QVBoxLayout()
        new_layout.setSpacing(5)
        new_layout.setContentsMargins(0, 0, 0, 0)
        
        # Add plots for visible views in order
        for view_name in self.visible_views_order:
            for plot in self.plot_groups.get(view_name, []):
                new_layout.addWidget(plot)
                plot.show()
        
        # Add stretch at end to push plots to top
        new_layout.addStretch()
        
        # Set the layout
        self.plots_container.setLayout(new_layout)
    
    def _rebuild_plots_2x2_layout(self):
        """Rebuild layout as 2x2 grid."""
        # Create new grid layout
        new_layout = QtWidgets.QGridLayout()
        new_layout.setSpacing(5)
        new_layout.setContentsMargins(0, 0, 0, 0)
        
        # Collect all plots to display
        all_plots = []
        for view_name in self.visible_views_order:
            for plot in self.plot_groups.get(view_name, []):
                all_plots.append(plot)
        
        # Add plots to grid: row 0, row 1, etc. (2 columns each)
        row = 0
        for i, plot in enumerate(all_plots):
            row = i // 2
            col = i % 2
            new_layout.addWidget(plot, row, col)
            plot.show()
        
        # Add stretch to fill remaining space if there are plots
        if all_plots:
            new_layout.setRowStretch(row + 1, 1)
        
        # Set the layout
        self.plots_container.setLayout(new_layout)
    
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
    
    def _on_packet(self, packet: GridSideTelemetryPacket):
        """Handle incoming telemetry packet."""
        try:
            self.packet_counter += 1
            
            # Decimation
            if self.packet_counter % self.decimate_factor != 0:
                return
            
            # Calculate elapsed time in seconds
            if self.start_time is not None:
                elapsed_time = time.time() - self.start_time
            else:
                elapsed_time = 0.0
            
            # Store for dashboard
            self.latest_packet = packet
            
            # Extract values
            if packet.measurements:
                meas = packet.measurements
                self.voltage_buffer.append((elapsed_time, meas.grid_voltage_magnitude_pu))
                self.current_buffer.append((elapsed_time, meas.grid_current_a_pu, meas.grid_current_b_pu, meas.grid_current_c_pu))
            
            if packet.observer:
                obs = packet.observer
                self.observer_buffer.append((elapsed_time, obs.estimated_voltage_magnitude_pu, obs.estimated_frequency_pu))
            
            if packet.controller:
                ctrl = packet.controller
                self.dclink_buffer.append((elapsed_time, ctrl.dc_link_current_reference, ctrl.dc_link_current_measured))
            
            self.sample_index += 1
        
        except Exception as e:
            logger.error(f"Packet error: {e}", exc_info=True)
    
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
        """Update plots from buffers."""
        try:
            # Voltage
            if len(self.voltage_buffer) > 0:
                x_data = np.array([p[0] for p in self.voltage_buffer])
                y_data = np.array([p[1] for p in self.voltage_buffer])
                self.voltage_curve.setData(x_data, y_data)
                if len(x_data) > 0:
                    latest = x_data[-1]
                    earliest = x_data[0]
                    # Show window_size seconds, but if total data is less than window, show all from earliest
                    x_min = max(earliest, latest - self.window_size)
                    self.voltage_plot.setXRange(x_min, latest, padding=0)
            
            # Currents
            if len(self.current_buffer) > 0:
                x_data = np.array([p[0] for p in self.current_buffer])
                y_data_a = np.array([p[1] for p in self.current_buffer])
                y_data_b = np.array([p[2] for p in self.current_buffer])
                y_data_c = np.array([p[3] for p in self.current_buffer])
                self.current_curve_a.setData(x_data, y_data_a)
                self.current_curve_b.setData(x_data, y_data_b)
                self.current_curve_c.setData(x_data, y_data_c)
                if len(x_data) > 0:
                    latest = x_data[-1]
                    earliest = x_data[0]
                    # Show window_size seconds, but if total data is less than window, show all from earliest
                    x_min = max(earliest, latest - self.window_size)
                    self.current_plot.setXRange(x_min, latest, padding=0)
            
            # Observer
            if len(self.observer_buffer) > 0:
                x_data = np.array([p[0] for p in self.observer_buffer])
                y_voltage = np.array([p[1] for p in self.observer_buffer])
                y_freq = np.array([p[2] for p in self.observer_buffer])
                self.observer_voltage_curve.setData(x_data, y_voltage)
                self.observer_freq_curve.setData(x_data, y_freq)
                if len(x_data) > 0:
                    latest = x_data[-1]
                    earliest = x_data[0]
                    # Show window_size seconds, but if total data is less than window, show all from earliest
                    x_min = max(earliest, latest - self.window_size)
                    self.observer_plot.setXRange(x_min, latest, padding=0)
            
            # DC Link
            if len(self.dclink_buffer) > 0:
                x_data = np.array([p[0] for p in self.dclink_buffer])
                y_ref = np.array([p[1] for p in self.dclink_buffer])
                y_meas = np.array([p[2] for p in self.dclink_buffer])
                self.dclink_ref_curve.setData(x_data, y_ref)
                self.dclink_measured_curve.setData(x_data, y_meas)
                if len(x_data) > 0:
                    latest = x_data[-1]
                    earliest = x_data[0]
                    # Show window_size seconds, but if total data is less than window, show all from earliest
                    x_min = max(earliest, latest - self.window_size)
                    self.dclink_plot.setXRange(x_min, latest, padding=0)
        
        except Exception as e:
            logger.error(f"Plot error: {e}", exc_info=True)
    
    def _update_dashboard(self):
        """Update telemetry dashboard."""
        if not self.latest_packet:
            return
        
        try:
            packet = self.latest_packet
            
            # Measurements
            if packet.measurements:
                m = packet.measurements
                self.telem_labels['grid_voltage_magnitude_pu'].setText(f"{m.grid_voltage_magnitude_pu:.4f}")
                self.telem_labels['grid_voltage_a_pu'].setText(f"{m.grid_voltage_a_pu:.4f}")
                self.telem_labels['grid_voltage_b_pu'].setText(f"{m.grid_voltage_b_pu:.4f}")
                self.telem_labels['grid_voltage_c_pu'].setText(f"{m.grid_voltage_c_pu:.4f}")
                self.telem_labels['grid_current_magnitude_pu'].setText(f"{m.grid_current_magnitude_pu:.4f}")
                self.telem_labels['grid_current_a_pu'].setText(f"{m.grid_current_a_pu:.4f}")
                self.telem_labels['grid_current_b_pu'].setText(f"{m.grid_current_b_pu:.4f}")
                self.telem_labels['grid_current_c_pu'].setText(f"{m.grid_current_c_pu:.4f}")
                self.telem_labels['dc_link_current_pu'].setText(f"{m.dc_link_current_pu:.4f}")
                self.telem_labels['is_commuting'].setText("YES" if m.is_commuting else "NO")
            
            # Observer
            if packet.observer:
                o = packet.observer
                self.telem_labels['estimated_voltage_magnitude_pu'].setText(f"{o.estimated_voltage_magnitude_pu:.4f}")
                self.telem_labels['estimated_frequency_pu'].setText(f"{o.estimated_frequency_pu:.4f}")
                self.telem_labels['estimated_angle_rad'].setText(f"{o.estimated_angle_rad:.4f}")
            
            # Controller
            if packet.controller:
                c = packet.controller
                self.telem_labels['dc_link_current_reference'].setText(f"{c.dc_link_current_reference:.4f}")
                self.telem_labels['dc_link_current_measured'].setText(f"{c.dc_link_current_measured:.4f}")
                self.telem_labels['current_error_pu'].setText(f"{c.current_error_pu:.4f}")
                self.telem_labels['controller_output_pu'].setText(f"{c.controller_output_pu:.4f}")
                self.telem_labels['operating_mode'].setText("Forced" if c.operating_mode else "Natural")
                self.telem_labels['controller_saturated'].setText("YES" if c.controller_saturated else "NO")
                self.telem_labels['freeze_integrator'].setText("YES" if c.freeze_integrator else "NO")
            
            # Modulator
            if packet.modulator:
                mod = packet.modulator
                self.telem_labels['is_enabled'].setText("YES" if mod.is_enabled else "NO")
                self.telem_labels['firing_angle_input_pu'].setText(f"{mod.firing_angle_input_pu:.4f}")
                self.telem_labels['firing_angle_output_pu'].setText(f"{mod.firing_angle_output_pu:.4f}")
                self.telem_labels['force_commutation_ack'].setText("YES" if mod.force_commutation_ack else "NO")
            
            # State Machine
            if packet.state_machine:
                sm = packet.state_machine
                state_names = ["IDLE", "AUXILIARIES_ON", "BREAKERS_OFF", "BREAKERS_ON", "OPERATION_OFF", "OPERATION_ON", "SHUTDOWN_REQUESTED"]
                state_text = state_names[sm.current_state] if 0 <= sm.current_state < len(state_names) else f"UNKNOWN({sm.current_state})"
                self.telem_labels['current_state'].setText(state_text)
                self.telem_labels['pulse_releases'].setText("YES" if sm.pulse_releases else "NO")
                self.telem_labels['auxiliaries_are_on'].setText("YES" if sm.auxiliaries_are_on else "NO")
                self.telem_labels['breakers_are_on'].setText("YES" if sm.breakers_are_on else "NO")
                self.telem_labels['start_command'].setText("YES" if sm.start_command else "NO")
                self.telem_labels['stop_command'].setText("YES" if sm.stop_command else "NO")
                self.telem_labels['is_current_extinguished'].setText("YES" if sm.is_current_extinguished else "NO")
                self.telem_labels['is_shutdown_requested'].setText("YES" if sm.is_shutdown_requested else "NO")
        
        except Exception as e:
            logger.error(f"Dashboard error: {e}", exc_info=True)


def main():
    """Entry point."""
    import argparse
    parser = argparse.ArgumentParser(description="SFC Monitor with Parameters")
    parser.add_argument('--host', default='192.168.2.100')
    parser.add_argument('--port', type=int, default=7000)
    parser.add_argument('--verbose', action='store_true')
    parser.add_argument('--decimate', type=int, default=1, help='Process 1 out of N packets')
    parser.add_argument('--window', type=int, default=2, help='Time window in seconds to display on X-axis')
    args = parser.parse_args()
    
    app = QtWidgets.QApplication(sys.argv)
    window = TelemetryMonitorWindow(args.host, args.port, args.verbose, args.decimate, args.window)
    window.show()
    return app.exec_()


if __name__ == '__main__':
    sys.exit(main())
