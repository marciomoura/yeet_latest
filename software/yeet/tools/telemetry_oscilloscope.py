#!/usr/bin/env python3
"""Real-time oscilloscope-style telemetry viewer for SFC control system.

This application provides a continuous monitoring oscilloscope display with:
    - Three synchronized subplots for different signal groups
    - Auto-scrolling time window (continuous monitoring)
    - Configurable downsampling for high-rate data
    - Zoom and pan capabilities
    - Region selection for detailed analysis

Features:
    - Grid Measurements: Voltage (abc + magnitude), current, commutation
    - Observer: Estimated voltage, frequency, angle
    - DC Link Controller: Reference, measured, error, output

Dependencies:
    pip install PyQt5 pyqtgraph numpy

Usage:
    python telemetry_oscilloscope.py --host localhost --port 7000 --window 5.0

Architecture:
    - Uses pyqtgraph for high-performance real-time plotting
    - Circular buffers for efficient data management
    - Configurable time window (default 5 seconds)
    - Auto-downsampling when buffer exceeds capacity
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

# UI update and data parameters
UI_UPDATE_INTERVAL_MS = 50      # 20 Hz screen refresh
STATS_UPDATE_INTERVAL_MS = 500  # 2 Hz statistics
DEFAULT_TIME_WINDOW_SEC = 5.0   # Default 5 second window
MAX_POINTS_PER_PLOT = 2000      # Maximum points before downsampling


class CircularTimeSeriesBuffer:
    """Efficient circular buffer for time-series data with auto-downsampling.
    
    Maintains a fixed-size buffer of (time, value) pairs. When capacity
    is exceeded, automatically downsamples by factor of 2.
    """
    
    def __init__(self, max_points: int = MAX_POINTS_PER_PLOT):
        """Initialize circular buffer.
        
        Args:
            max_points: Maximum number of points to store
        """
        self.max_points = max_points
        self.times = deque(maxlen=max_points * 2)  # Allow 2x before downsample
        self.values = deque(maxlen=max_points * 2)
        self.downsample_count = 0
    
    def append(self, time_s: float, value: float) -> None:
        """Append new data point.
        
        Args:
            time_s: Timestamp in seconds
            value: Signal value
        """
        self.times.append(time_s)
        self.values.append(value)
        
        # Auto-downsample if buffer gets too large
        if len(self.times) > self.max_points * 1.5:
            self._downsample()
    
    def _downsample(self) -> None:
        """Downsample buffer by factor of 2 (keep every other point)."""
        self.times = deque(list(self.times)[::2], maxlen=self.max_points * 2)
        self.values = deque(list(self.values)[::2], maxlen=self.max_points * 2)
        self.downsample_count += 1
        logger.debug(f"Downsampled buffer (count: {self.downsample_count})")
    
    def get_data(self, time_window_sec: Optional[float] = None) -> tuple:
        """Get data for plotting.
        
        Args:
            time_window_sec: If provided, only return last N seconds
            
        Returns:
            Tuple of (times_array, values_array)
        """
        if len(self.times) == 0:
            return np.array([]), np.array([])
        
        times_list = list(self.times)
        values_list = list(self.values)
        
        if time_window_sec is not None and len(times_list) > 0:
            # Find cutoff time
            latest_time = times_list[-1]
            cutoff_time = latest_time - time_window_sec
            
            # Find index where to start
            start_idx = 0
            for i, t in enumerate(times_list):
                if t >= cutoff_time:
                    start_idx = i
                    break
            
            times_list = times_list[start_idx:]
            values_list = values_list[start_idx:]
        
        return np.array(times_list), np.array(values_list)
    
    def clear(self) -> None:
        """Clear all data."""
        self.times.clear()
        self.values.clear()
        self.downsample_count = 0


class SignalPlotGroup:
    """Group of related signals displayed on a single plot."""
    
    def __init__(self, name: str, plot_widget: pg.PlotWidget):
        """Initialize signal plot group.
        
        Args:
            name: Group name
            plot_widget: PyQtGraph plot widget
        """
        self.name = name
        self.plot_widget = plot_widget
        self.curves: Dict[str, pg.PlotDataItem] = {}
        self.buffers: Dict[str, CircularTimeSeriesBuffer] = {}
        self.colors = [
            (255, 0, 0),      # Red
            (0, 255, 0),      # Green
            (0, 0, 255),      # Blue
            (255, 255, 0),    # Yellow
            (255, 0, 255),    # Magenta
            (0, 255, 255),    # Cyan
            (255, 128, 0),    # Orange
            (128, 0, 255),    # Purple
        ]
        self.color_index = 0
    
    def add_signal(self, signal_name: str, display_name: str) -> None:
        """Add signal to plot group.
        
        Args:
            signal_name: Internal signal name
            display_name: Display name for legend
        """
        color = self.colors[self.color_index % len(self.colors)]
        self.color_index += 1
        
        curve = self.plot_widget.plot(
            pen=pg.mkPen(color=color, width=2),
            name=display_name
        )
        self.curves[signal_name] = curve
        self.buffers[signal_name] = CircularTimeSeriesBuffer()
    
    def update_signal(self, signal_name: str, time_s: float, value: float) -> None:
        """Update signal with new data point.
        
        Args:
            signal_name: Signal name
            time_s: Timestamp in seconds
            value: Signal value
        """
        if signal_name in self.buffers:
            self.buffers[signal_name].append(time_s, value)
    
    def refresh_plots(self, time_window_sec: float) -> None:
        """Refresh all plots with latest data.
        
        Args:
            time_window_sec: Time window to display (seconds)
        """
        for signal_name, curve in self.curves.items():
            buffer = self.buffers[signal_name]
            times, values = buffer.get_data(time_window_sec)
            curve.setData(times, values)
    
    def clear_all(self) -> None:
        """Clear all signal data."""
        for buffer in self.buffers.values():
            buffer.clear()


class OscilloscopeWindow(QtWidgets.QMainWindow):
    """Main oscilloscope window with real-time plots."""
    
    def __init__(self, host: str, port: int, time_window_sec: float, verbose: bool = False):
        """Initialize oscilloscope window.
        
        Args:
            host: Controller IP address
            port: Controller TCP port
            time_window_sec: Time window to display (seconds)
            verbose: Enable verbose logging
        """
        super().__init__()
        
        self.setWindowTitle("SFC Telemetry Oscilloscope")
        self.resize(1400, 900)
        
        self.time_window_sec = time_window_sec
        self.start_time = None  # Will be set on first packet
        
        # Create network client
        self.network_client = TelemetryNetworkClient(host=host, port=port, verbose=verbose)
        self.network_client.set_packet_callback(self._on_packet_received)
        
        # Build UI
        self._build_ui(host, port)
        
        # Setup update timers
        self.plot_update_timer = QtCore.QTimer(self)
        self.plot_update_timer.timeout.connect(self._update_plots)
        self.plot_update_timer.start(UI_UPDATE_INTERVAL_MS)
        
        self.stats_timer = QtCore.QTimer(self)
        self.stats_timer.timeout.connect(self._update_statistics)
        self.stats_timer.start(STATS_UPDATE_INTERVAL_MS)
        
        logger.info("Oscilloscope initialized")
    
    def _build_ui(self, host: str, port: int) -> None:
        """Build main UI components."""
        # Central widget and layout
        central = QtWidgets.QWidget(self)
        self.setCentralWidget(central)
        main_layout = QtWidgets.QVBoxLayout(central)
        
        # Control panel
        control_panel = self._build_control_panel(host, port)
        main_layout.addWidget(control_panel)
        
        # Plot area
        self.plot_groups: Dict[str, SignalPlotGroup] = {}
        plot_area = self._build_plot_area()
        main_layout.addWidget(plot_area, stretch=1)
        
        # Status bar
        self.statusBar().showMessage("Disconnected")
    
    def _build_control_panel(self, host: str, port: int) -> QtWidgets.QWidget:
        """Build control panel with connection and settings."""
        panel = QtWidgets.QGroupBox("Control Panel")
        layout = QtWidgets.QHBoxLayout(panel)
        
        # Connection controls
        layout.addWidget(QtWidgets.QLabel("Host:"))
        self.host_edit = QtWidgets.QLineEdit(host)
        self.host_edit.setMaximumWidth(150)
        layout.addWidget(self.host_edit)
        
        layout.addWidget(QtWidgets.QLabel("Port:"))
        self.port_spin = QtWidgets.QSpinBox()
        self.port_spin.setRange(1, 65535)
        self.port_spin.setValue(port)
        self.port_spin.setMaximumWidth(80)
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
        self.window_spin.setMaximumWidth(80)
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
        
        layout.addStretch()
        
        return panel
    
    def _build_plot_area(self) -> QtWidgets.QWidget:
        """Build plot area with three subplots."""
        # Use pyqtgraph's GraphicsLayoutWidget for efficient plotting
        self.graphics_widget = pg.GraphicsLayoutWidget()
        self.graphics_widget.setBackground('w')  # White background
        
        # Configure pyqtgraph for better performance
        pg.setConfigOptions(antialias=True)
        
        # Subplot 1: Grid Measurements
        self.plot1 = self.graphics_widget.addPlot(row=0, col=0, 
                                                   title="Grid Measurements")
        self.plot1.setLabel('left', 'Voltage / Current', units='pu')
        self.plot1.setLabel('bottom', 'Time', units='s')
        self.plot1.addLegend()
        self.plot1.showGrid(x=True, y=True, alpha=0.3)
        
        measurements_group = SignalPlotGroup('measurements', self.plot1)
        measurements_group.add_signal('grid_voltage_magnitude_pu', 'V mag')
        measurements_group.add_signal('grid_voltage_a_pu', 'Va')
        measurements_group.add_signal('grid_voltage_b_pu', 'Vb')
        measurements_group.add_signal('grid_voltage_c_pu', 'Vc')
        measurements_group.add_signal('dc_link_current_pu', 'Idc')
        self.plot_groups['measurements'] = measurements_group
        
        # Subplot 2: Observer
        self.plot2 = self.graphics_widget.addPlot(row=1, col=0, 
                                                   title="Grid Observer")
        self.plot2.setLabel('left', 'Magnitude / Frequency', units='pu')
        self.plot2.setLabel('bottom', 'Time', units='s')
        self.plot2.addLegend()
        self.plot2.showGrid(x=True, y=True, alpha=0.3)
        
        observer_group = SignalPlotGroup('observer', self.plot2)
        observer_group.add_signal('estimated_voltage_magnitude_pu', 'V est')
        observer_group.add_signal('estimated_frequency_pu', 'f est')
        self.plot_groups['observer'] = observer_group
        
        # Subplot 3: DC Link Controller
        self.plot3 = self.graphics_widget.addPlot(row=2, col=0, 
                                                   title="DC Link Current Controller")
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
        
        # Link x-axes for synchronized zooming
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
            
            # Send START command
            self.network_client.send_command('START')
        else:
            QtWidgets.QMessageBox.critical(
                self, "Connection Failed",
                f"Unable to connect to {host}:{port}"
            )
    
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
        logger.info(f"Time window changed to {value:.1f} seconds")
    
    def _on_clear_clicked(self) -> None:
        """Handle clear data button click."""
        for group in self.plot_groups.values():
            group.clear_all()
        self.start_time = None
        logger.info("Data cleared")
    
    def _on_packet_received(self, packet: GridSideTelemetryPacket) -> None:
        """Callback when telemetry packet received.
        
        Args:
            packet: Decoded telemetry packet
        """
        # Set start time on first packet
        if self.start_time is None:
            self.start_time = packet.header.timestamp_ms / 1000.0
        
        # Convert timestamp to relative time in seconds
        time_s = (packet.header.timestamp_ms / 1000.0) - self.start_time
        
        # Update measurements
        if packet.measurements:
            meas = packet.measurements
            self.plot_groups['measurements'].update_signal(
                'grid_voltage_magnitude_pu', time_s, meas.grid_voltage_magnitude_pu)
            self.plot_groups['measurements'].update_signal(
                'grid_voltage_a_pu', time_s, meas.grid_voltage_a_pu)
            self.plot_groups['measurements'].update_signal(
                'grid_voltage_b_pu', time_s, meas.grid_voltage_b_pu)
            self.plot_groups['measurements'].update_signal(
                'grid_voltage_c_pu', time_s, meas.grid_voltage_c_pu)
            self.plot_groups['measurements'].update_signal(
                'dc_link_current_pu', time_s, meas.dc_link_current_pu)
        
        # Update observer
        if packet.observer:
            obs = packet.observer
            self.plot_groups['observer'].update_signal(
                'estimated_voltage_magnitude_pu', time_s, obs.estimated_voltage_magnitude_pu)
            self.plot_groups['observer'].update_signal(
                'estimated_frequency_pu', time_s, obs.estimated_frequency_pu)
        
        # Update controller
        if packet.controller:
            ctrl = packet.controller
            self.plot_groups['controller'].update_signal(
                'dc_link_current_reference', time_s, ctrl.dc_link_current_reference)
            self.plot_groups['controller'].update_signal(
                'dc_link_current_measured', time_s, ctrl.dc_link_current_measured)
            self.plot_groups['controller'].update_signal(
                'current_error_pu', time_s, ctrl.current_error_pu)
            self.plot_groups['controller'].update_signal(
                'controller_output_pu', time_s, ctrl.controller_output_pu)
    
    def _update_plots(self) -> None:
        """Update all plots (called by timer)."""
        for group in self.plot_groups.values():
            group.refresh_plots(self.time_window_sec)
    
    def _update_statistics(self) -> None:
        """Update statistics display."""
        stats = self.network_client.get_statistics()
        self.packets_label.setText(str(stats['packets_received']))
    
    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        """Handle window close event."""
        if self.network_client.connected:
            self.network_client.send_command('STOP')
            self.network_client.disconnect()
        logger.info("Oscilloscope closed")
        event.accept()


def parse_args() -> argparse.Namespace:
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="SFC Telemetry Oscilloscope - Real-time continuous monitoring"
    )
    parser.add_argument(
        '--host',
        default='localhost',
        help='Controller IP address (default: localhost)'
    )
    parser.add_argument(
        '--port',
        type=int,
        default=7000,
        help='Controller TCP port (default: 7000)'
    )
    parser.add_argument(
        '--window',
        type=float,
        default=DEFAULT_TIME_WINDOW_SEC,
        help='Time window in seconds (default: 5.0)'
    )
    parser.add_argument(
        '--verbose',
        action='store_true',
        help='Enable verbose logging for debugging'
    )
    return parser.parse_args()


def main() -> int:
    """Main application entry point."""
    args = parse_args()
    
    app = QtWidgets.QApplication(sys.argv)
    
    # Set application style
    app.setStyle('Fusion')
    
    # Create and show oscilloscope window
    window = OscilloscopeWindow(
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
