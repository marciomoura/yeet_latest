#!/usr/bin/env python3
"""Combined telemetry monitor with dashboard and oscilloscope.

This application provides a comprehensive monitoring interface combining:
    - Digital dashboard (left side): Real-time signal values with state display
    - Oscilloscope (right side): Continuous real-time waveform plots

Features:
    - Three synchronized oscilloscope subplots
    - Digital displays for all signals organized by functional group
    - State machine enum-to-text conversion
    - Auto-scrolling time window with zoom/pan capabilities
    - Connection management and telemetry control

Dependencies:
    pip install PyQt5 pyqtgraph numpy

Usage:
    python telemetry_monitor.py --host localhost --port 7000 --window 5.0

Architecture:
    - Left panel: Signal dashboard with current values
    - Right panel: Real-time oscilloscope plots
    - Shared network client for efficient data handling
"""

import argparse
import logging
import sys
from collections import deque
from typing import Dict, List, Optional

import numpy as np
import pyqtgraph as pg
from PyQt5 import QtCore, QtGui, QtWidgets

from telemetry_network_client import TelemetryNetworkClient
from telemetry_protocol import GridSideTelemetryPacket

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# UI update intervals
UI_UPDATE_INTERVAL_MS = 50      # 20 Hz screen refresh
STATS_UPDATE_INTERVAL_MS = 500  # 2 Hz statistics

# Oscilloscope parameters
DEFAULT_TIME_WINDOW_SEC = 5.0   # Default 5 second window
MAX_POINTS_PER_PLOT = 2000      # Maximum points before downsampling


class CircularTimeSeriesBuffer:
    """Efficient circular buffer for time-series data with auto-downsampling."""
    
    def __init__(self, max_points: int = MAX_POINTS_PER_PLOT):
        self.max_points = max_points
        self.times = deque(maxlen=max_points * 2)
        self.values = deque(maxlen=max_points * 2)
        self.downsample_count = 0
    
    def append(self, time_s: float, value: float) -> None:
        self.times.append(time_s)
        self.values.append(value)
        
        if len(self.times) > self.max_points * 1.5:
            self._downsample()
    
    def _downsample(self) -> None:
        self.times = deque(list(self.times)[::2], maxlen=self.max_points * 2)
        self.values = deque(list(self.values)[::2], maxlen=self.max_points * 2)
        self.downsample_count += 1
    
    def get_data(self, time_window_sec: Optional[float] = None) -> tuple:
        if len(self.times) == 0:
            return np.array([]), np.array([])
        
        times_list = list(self.times)
        values_list = list(self.values)
        
        if time_window_sec is not None and len(times_list) > 0:
            latest_time = times_list[-1]
            cutoff_time = latest_time - time_window_sec
            
            start_idx = 0
            for i, t in enumerate(times_list):
                if t >= cutoff_time:
                    start_idx = i
                    break
            
            times_list = times_list[start_idx:]
            values_list = values_list[start_idx:]
        
        return np.array(times_list), np.array(values_list)
    
    def clear(self) -> None:
        self.times.clear()
        self.values.clear()
        self.downsample_count = 0


class SignalPlotGroup:
    """Group of related signals displayed on a single plot."""
    
    def __init__(self, name: str, plot_widget: pg.PlotWidget):
        self.name = name
        self.plot_widget = plot_widget
        self.curves: Dict[str, pg.PlotDataItem] = {}
        self.buffers: Dict[str, CircularTimeSeriesBuffer] = {}
        self.colors = [
            (255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0),
            (255, 0, 255), (0, 255, 255), (255, 128, 0), (128, 0, 255)
        ]
        self.color_index = 0
    
    def add_signal(self, signal_name: str, display_name: str) -> None:
        color = self.colors[self.color_index % len(self.colors)]
        self.color_index += 1
        
        curve = self.plot_widget.plot(pen=pg.mkPen(color=color, width=2), name=display_name)
        self.curves[signal_name] = curve
        self.buffers[signal_name] = CircularTimeSeriesBuffer()
    
    def update_signal(self, signal_name: str, time_s: float, value: float) -> None:
        if signal_name in self.buffers:
            self.buffers[signal_name].append(time_s, value)
    
    def refresh_plots(self, time_window_sec: float) -> None:
        for signal_name, curve in self.curves.items():
            buffer = self.buffers[signal_name]
            times, values = buffer.get_data(time_window_sec)
            curve.setData(times, values)
    
    def clear_all(self) -> None:
        for buffer in self.buffers.values():
            buffer.clear()


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
        self.layout.setSpacing(3)
        
        self.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                font-size: 12px;
                border: 2px solid #667eea;
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


class TelemetryMonitorWindow(QtWidgets.QMainWindow):
    """Main window with combined dashboard and oscilloscope."""
    
    def __init__(self, host: str, port: int, time_window_sec: float, verbose: bool = False):
        super().__init__()
        
        self.setWindowTitle("SFC Telemetry Monitor - Dashboard & Oscilloscope")
        self.resize(1800, 1000)
        
        self.time_window_sec = time_window_sec
        self.start_time = None
        
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
        
        # Create network client
        self.network_client = TelemetryNetworkClient(host=host, port=port, verbose=verbose)
        self.network_client.set_packet_callback(self._on_packet_received)
        
        # Build UI
        self._build_ui(host, port)
        
        # Setup update timers
        self.plot_update_timer = QtCore.QTimer(self)
        self.plot_update_timer.timeout.connect(self._update_displays)
        self.plot_update_timer.start(UI_UPDATE_INTERVAL_MS)
        
        self.stats_timer = QtCore.QTimer(self)
        self.stats_timer.timeout.connect(self._update_statistics)
        self.stats_timer.start(STATS_UPDATE_INTERVAL_MS)
        
        logger.info("Telemetry monitor initialized")
    
    def _build_ui(self, host: str, port: int) -> None:
        """Build main UI with dashboard on left, oscilloscope on right."""
        central = QtWidgets.QWidget(self)
        self.setCentralWidget(central)
        main_layout = QtWidgets.QVBoxLayout(central)
        
        # Control panel at top
        control_panel = self._build_control_panel(host, port)
        main_layout.addWidget(control_panel)
        
        # Split view: Dashboard (left) and Oscilloscope (right)
        splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        
        # Left side: Dashboard
        dashboard_widget = self._build_dashboard()
        splitter.addWidget(dashboard_widget)
        
        # Right side: Oscilloscope
        oscilloscope_widget = self._build_oscilloscope()
        splitter.addWidget(oscilloscope_widget)
        
        # Set initial sizes (30% dashboard, 70% oscilloscope)
        splitter.setSizes([540, 1260])
        
        main_layout.addWidget(splitter, stretch=1)
        
        self.statusBar().showMessage("Disconnected")
    
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
        
        # Time window control
        layout.addWidget(QtWidgets.QLabel("Window (s):"))
        self.window_spin = QtWidgets.QDoubleSpinBox()
        self.window_spin.setRange(0.1, 60.0)
        self.window_spin.setValue(self.time_window_sec)
        self.window_spin.setSingleStep(0.5)
        self.window_spin.setMaximumWidth(70)
        self.window_spin.valueChanged.connect(self._on_window_changed)
        layout.addWidget(self.window_spin)
        
        # Clear button
        self.clear_button = QtWidgets.QPushButton("Clear Data")
        self.clear_button.clicked.connect(self._on_clear_clicked)
        layout.addWidget(self.clear_button)
        
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
        
        # Title
        title = QtWidgets.QLabel("Digital Dashboard")
        title.setStyleSheet("font-size: 14px; font-weight: bold; color: #667eea;")
        title.setAlignment(QtCore.Qt.AlignCenter)
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
        """Build oscilloscope panel with real-time plots."""
        # Use pyqtgraph's GraphicsLayoutWidget
        self.graphics_widget = pg.GraphicsLayoutWidget()
        self.graphics_widget.setBackground('w')
        
        pg.setConfigOptions(antialias=True)
        
        # Subplot 1: Grid Measurements
        self.plot1 = self.graphics_widget.addPlot(row=0, col=0, title="Grid Measurements")
        self.plot1.setLabel('left', 'Voltage / Current', units='pu')
        self.plot1.setLabel('bottom', 'Time', units='s')
        self.plot1.addLegend()
        self.plot1.showGrid(x=True, y=True, alpha=0.3)
        
        self.plot_groups: Dict[str, SignalPlotGroup] = {}
        measurements_group = SignalPlotGroup('measurements', self.plot1)
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
        self.plot2.setLabel('bottom', 'Time', units='s')
        self.plot2.addLegend()
        self.plot2.showGrid(x=True, y=True, alpha=0.3)
        
        observer_group = SignalPlotGroup('observer', self.plot2)
        observer_group.add_signal('estimated_voltage_magnitude_pu', 'V est')
        observer_group.add_signal('estimated_frequency_pu', 'f est')
        self.plot_groups['observer'] = observer_group
        
        # Subplot 3: DC Link Controller
        self.plot3 = self.graphics_widget.addPlot(row=2, col=0, title="DC Link Current Controller")
        self.plot3.setLabel('left', 'Current / Error', units='pu')
        self.plot3.setLabel('bottom', 'Time', units='s')
        self.plot3.addLegend()
        self.plot3.showGrid(x=True, y=True, alpha=0.3)
        
        controller_group = SignalPlotGroup('controller', self.plot3)
        controller_group.add_signal('dc_link_current_reference', 'I ref')
        controller_group.add_signal('dc_link_current_measured', 'I meas')
        controller_group.add_signal('current_error_pu', 'Error')
        controller_group.add_signal('controller_output_pu', 'Output')
        self.plot_groups['controller'] = controller_group
        
        # Link x-axes
        self.plot2.setXLink(self.plot1)
        self.plot3.setXLink(self.plot1)
        
        return self.graphics_widget
    
    def _on_connect_clicked(self) -> None:
        """Handle connect button click."""
        host = self.host_edit.text().strip()
        port = self.port_spin.value()
        
        self.network_client.host = host
        self.network_client.port = port
        
        if self.network_client.connect():
            self.connect_button.setEnabled(False)
            self.disconnect_button.setEnabled(True)
            self.statusBar().showMessage(f"Connected to {host}:{port}")
            logger.info(f"Connected to {host}:{port}")
            self.network_client.send_command('START')
        else:
            QtWidgets.QMessageBox.critical(self, "Connection Failed",
                                          f"Unable to connect to {host}:{port}")
    
    def _on_disconnect_clicked(self) -> None:
        """Handle disconnect button click."""
        self.network_client.send_command('STOP')
        self.network_client.disconnect()
        self.connect_button.setEnabled(True)
        self.disconnect_button.setEnabled(False)
        self.statusBar().showMessage("Disconnected")
        logger.info("Disconnected")
    
    def _on_window_changed(self, value: float) -> None:
        """Handle time window change."""
        self.time_window_sec = value
    
    def _on_clear_clicked(self) -> None:
        """Handle clear data button click."""
        for group in self.plot_groups.values():
            group.clear_all()
        self.start_time = None
        logger.info("Data cleared")
    
    def _on_packet_received(self, packet: GridSideTelemetryPacket) -> None:
        """Callback when telemetry packet received."""
        if self.start_time is None:
            self.start_time = packet.header.timestamp_ms / 1000.0
        
        time_s = (packet.header.timestamp_ms / 1000.0) - self.start_time
        
        # Update measurements (dashboard + oscilloscope)
        if packet.measurements:
            meas = packet.measurements
            self.latest_values['measurements']['grid_voltage_magnitude_pu'] = meas.grid_voltage_magnitude_pu
            self.latest_values['measurements']['grid_current_magnitude_pu'] = meas.grid_current_magnitude_pu
            self.latest_values['measurements']['dc_link_current_pu'] = meas.dc_link_current_pu
            self.latest_values['measurements']['is_commuting'] = float(meas.is_commuting)
            
            self.plot_groups['measurements'].update_signal('grid_voltage_magnitude_pu', time_s, meas.grid_voltage_magnitude_pu)
            self.plot_groups['measurements'].update_signal('grid_voltage_a_pu', time_s, meas.grid_voltage_a_pu)
            self.plot_groups['measurements'].update_signal('grid_voltage_b_pu', time_s, meas.grid_voltage_b_pu)
            self.plot_groups['measurements'].update_signal('grid_voltage_c_pu', time_s, meas.grid_voltage_c_pu)
            self.plot_groups['measurements'].update_signal('grid_current_magnitude_pu', time_s, meas.grid_current_magnitude_pu)
            self.plot_groups['measurements'].update_signal('grid_current_a_pu', time_s, meas.grid_current_a_pu)
            self.plot_groups['measurements'].update_signal('grid_current_b_pu', time_s, meas.grid_current_b_pu)
            self.plot_groups['measurements'].update_signal('grid_current_c_pu', time_s, meas.grid_current_c_pu)
            self.plot_groups['measurements'].update_signal('dc_link_current_pu', time_s, meas.dc_link_current_pu)
        
        # Update observer
        if packet.observer:
            obs = packet.observer
            self.latest_values['observer']['estimated_voltage_magnitude_pu'] = obs.estimated_voltage_magnitude_pu
            self.latest_values['observer']['estimated_frequency_pu'] = obs.estimated_frequency_pu
            self.latest_values['observer']['estimated_angle_rad'] = obs.estimated_angle_rad
            
            self.plot_groups['observer'].update_signal('estimated_voltage_magnitude_pu', time_s, obs.estimated_voltage_magnitude_pu)
            self.plot_groups['observer'].update_signal('estimated_frequency_pu', time_s, obs.estimated_frequency_pu)
        
        # Update modulator
        if packet.modulator:
            mod = packet.modulator
            self.latest_values['modulator']['is_enabled'] = float(mod.is_enabled)
            self.latest_values['modulator']['firing_angle_input_pu'] = mod.firing_angle_input_pu
            self.latest_values['modulator']['firing_angle_output_pu'] = mod.firing_angle_output_pu
        
        # Update controller
        if packet.controller:
            ctrl = packet.controller
            self.latest_values['controller']['dc_link_current_reference'] = ctrl.dc_link_current_reference
            self.latest_values['controller']['dc_link_current_measured'] = ctrl.dc_link_current_measured
            self.latest_values['controller']['current_error_pu'] = ctrl.current_error_pu
            self.latest_values['controller']['controller_saturated'] = float(ctrl.controller_saturated)
            
            self.plot_groups['controller'].update_signal('dc_link_current_reference', time_s, ctrl.dc_link_current_reference)
            self.plot_groups['controller'].update_signal('dc_link_current_measured', time_s, ctrl.dc_link_current_measured)
            self.plot_groups['controller'].update_signal('current_error_pu', time_s, ctrl.current_error_pu)
            self.plot_groups['controller'].update_signal('controller_output_pu', time_s, ctrl.controller_output_pu)
        
        # Update state machine
        if packet.state_machine:
            state = packet.state_machine
            self.latest_values['state_machine']['current_state'] = float(state.current_state)
            self.latest_values['state_machine']['pulse_releases'] = float(state.pulse_releases)
            self.latest_values['state_machine']['auxiliaries_are_on'] = float(state.auxiliaries_are_on)
            self.latest_values['state_machine']['breakers_are_on'] = float(state.breakers_are_on)
    
    def _update_displays(self) -> None:
        """Update both dashboard and oscilloscope."""
        # Update oscilloscope plots
        for group in self.plot_groups.values():
            group.refresh_plots(self.time_window_sec)
        
        # Update dashboard displays
        for group_name, group_widget in self.signal_group_widgets.items():
            if group_name in self.latest_values:
                group_widget.update_all_signals(self.latest_values[group_name])
    
    def _update_statistics(self) -> None:
        """Update statistics display."""
        stats = self.network_client.get_statistics()
        self.packets_label.setText(str(stats['packets_received']))
        self.loss_label.setText(f"{stats['loss_rate_percent']:.2f}%")
    
    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        """Handle window close event."""
        if self.network_client.connected:
            self.network_client.send_command('STOP')
            self.network_client.disconnect()
        logger.info("Telemetry monitor closed")
        event.accept()


def parse_args() -> argparse.Namespace:
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="SFC Telemetry Monitor - Combined Dashboard & Oscilloscope"
    )
    parser.add_argument('--host', default='localhost', help='Controller IP address')
    parser.add_argument('--port', type=int, default=7000, help='Controller TCP port')
    parser.add_argument('--window', type=float, default=DEFAULT_TIME_WINDOW_SEC,
                       help='Oscilloscope time window in seconds')
    parser.add_argument('--verbose', action='store_true', help='Enable verbose logging')
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
        verbose=args.verbose
    )
    window.show()
    
    logger.info("Application started")
    return app.exec_()


if __name__ == '__main__':
    sys.exit(main())
