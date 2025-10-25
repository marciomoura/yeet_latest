#!/usr/bin/env python3
"""EXTENDED MINIMAL - Oscilloscope + Telemetry Dashboard

Extends the proven stable minimal oscilloscope with a telemetry dashboard
to test if dashboard integration causes stability issues.

Architecture (same as minimal):
- Python deque for oscilloscope data (automatic FIFO, no ring buffer math)
- Direct packet processing (no queues, no deferred work)
- Single 100ms timer for plot updates
- ADDED: Separate 200ms timer for dashboard updates
- ADDED: Right panel with telemetry values
"""

import sys
import logging
from collections import deque
from typing import Optional

import numpy as np
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg

from monitor_client.core.unified_network_client import UnifiedNetworkClient
from monitor_client.core.telemetry_protocol import GridSideTelemetryPacket

logger = logging.getLogger(__name__)


class MinimalBuffer:
    """Ultra-simple buffer using Python deque - automatic FIFO."""
    
    def __init__(self, maxlen: int = 2000):
        self.x_data = deque(maxlen=maxlen)
        self.y_data = deque(maxlen=maxlen)
        self.maxlen = maxlen
    
    def append(self, x: float, y: float):
        """Add data point."""
        self.x_data.append(x)
        self.y_data.append(y)
    
    def clear(self):
        """Clear all data."""
        self.x_data.clear()
        self.y_data.clear()
    
    def get_arrays(self):
        """Convert to numpy arrays for plotting."""
        return np.array(self.x_data), np.array(self.y_data)


class TelemetryMonitorWindow(QtWidgets.QMainWindow):
    """Extended minimal monitor with oscilloscope + dashboard."""
    
    def __init__(self, host: str, port: int, verbose: bool = False, decimate: int = 1):
        super().__init__()
        
        logger.info(f"=== Extended Minimal Monitor (verbose={verbose}, decimate={decimate}) ===")
        
        self.setWindowTitle("Extended Minimal SFC Monitor")
        self.resize(1600, 900)
        
        self.verbose = verbose
        self.decimate_factor = max(1, decimate)
        self.packet_counter = 0
        self.sample_index = 0
        
        # Store connection info
        self.host = host
        self.port = port
        
        # Oscilloscope buffer for three phase currents
        self.osc_buffer = MinimalBuffer(maxlen=2000)
        
        # Latest telemetry values for dashboard
        self.latest_values = {
            'measurements': {
                'grid_voltage_magnitude_pu': 0.0,
                'grid_voltage_a_pu': 0.0,
                'grid_voltage_b_pu': 0.0,
                'grid_voltage_c_pu': 0.0,
                'grid_current_magnitude_pu': 0.0,
                'grid_current_a_pu': 0.0,
                'grid_current_b_pu': 0.0,
                'grid_current_c_pu': 0.0,
                'dc_link_current_pu': 0.0,
            },
            'observer': {
                'estimated_voltage_magnitude_pu': 0.0,
                'estimated_frequency_pu': 0.0,
                'estimated_angle_rad': 0.0,
            },
            'controller': {
                'dc_link_current_reference': 0.0,
                'dc_link_current_measured': 0.0,
                'current_error_pu': 0.0,
            }
        }
        
        # Build UI
        self._build_ui()
        
        # Network client
        self.network_client = UnifiedNetworkClient(host, port)
        self.network_client.connected_changed.connect(self._on_connection_changed)
        self.network_client.telemetry_packet_received.connect(self._on_packet)
        
        # Two INDEPENDENT timers - dashboard disabled for testing
        self.plot_timer = QtCore.QTimer()
        self.plot_timer.timeout.connect(self._update_plot)
        self.plot_timer.start(100)  # 100ms for oscilloscope
        
        # self.dashboard_timer = QtCore.QTimer()
        # self.dashboard_timer.timeout.connect(self._update_dashboard)
        # self.dashboard_timer.start(500)  # 500ms for dashboard (reduced frequency)
        
        logger.info(f"Timers: plot=100ms (dashboard disabled for testing)")
    
    def _build_ui(self):
        """Build UI: oscilloscope left (70%), dashboard right (30%)."""
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        main_layout = QtWidgets.QHBoxLayout(central)
        
        # LEFT: Oscilloscope
        osc_widget = self._build_oscilloscope()
        main_layout.addWidget(osc_widget, 10)  # Full width for now
        
        # RIGHT: Dashboard - TEMPORARILY DISABLED for testing
        # dashboard_widget = self._build_dashboard()
        # main_layout.addWidget(dashboard_widget, 3)  # 30% width
    
    def _build_oscilloscope(self) -> QtWidgets.QWidget:
        """Build oscilloscope panel."""
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(widget)
        
        # Title and connection info at top
        header_layout = QtWidgets.QHBoxLayout()
        title = QtWidgets.QLabel("Oscilloscope - Grid Phase Currents (A, B, C)")
        title.setStyleSheet("font-size: 14pt; font-weight: bold;")
        header_layout.addWidget(title)
        header_layout.addStretch()
        self.connection_info_label = QtWidgets.QLabel(f"Target: {self.host}:{self.port}")
        self.connection_info_label.setStyleSheet("font-size: 10pt; color: gray;")
        header_layout.addWidget(self.connection_info_label)
        layout.addLayout(header_layout)
        
        # Controls - Connection (at top)
        conn_layout = QtWidgets.QHBoxLayout()
        self.connect_btn = QtWidgets.QPushButton("CONNECT")
        self.disconnect_btn = QtWidgets.QPushButton("DISCONNECT")
        self.connect_btn.setEnabled(True)
        self.disconnect_btn.setEnabled(False)
        self.connect_btn.clicked.connect(self._on_connect)
        self.disconnect_btn.clicked.connect(self._on_disconnect)
        conn_layout.addWidget(self.connect_btn)
        conn_layout.addWidget(self.disconnect_btn)
        conn_layout.addStretch()
        layout.addLayout(conn_layout)
        
        # Controls - Telemetry (at top)
        telem_layout = QtWidgets.QHBoxLayout()
        self.start_btn = QtWidgets.QPushButton("START")
        self.stop_btn = QtWidgets.QPushButton("STOP")
        self.clear_btn = QtWidgets.QPushButton("CLEAR")
        self.start_btn.setEnabled(False)
        self.stop_btn.setEnabled(False)
        self.clear_btn.setEnabled(True)
        self.start_btn.clicked.connect(self._on_start)
        self.stop_btn.clicked.connect(self._on_stop)
        self.clear_btn.clicked.connect(self._on_clear)
        telem_layout.addWidget(self.start_btn)
        telem_layout.addWidget(self.stop_btn)
        telem_layout.addWidget(self.clear_btn)
        telem_layout.addStretch()
        self.status_label = QtWidgets.QLabel("Disconnected")
        self.status_label.setStyleSheet("color: red; font-weight: bold;")
        telem_layout.addWidget(self.status_label)
        self.packet_count_label = QtWidgets.QLabel("Packets: 0")
        self.packet_count_label.setStyleSheet("font-family: monospace;")
        telem_layout.addWidget(self.packet_count_label)
        layout.addLayout(telem_layout)
        
        # Plot (now below controls) - Three phase currents
        pg.setConfigOptions(antialias=True)
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setLabel('left', 'Current (pu)')
        self.plot_widget.setLabel('bottom', 'Sample Index')
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.plot_widget.setYRange(-1.5, 1.5)
        
        # Three curves with different colors (no legend to reduce overhead)
        self.osc_curve_a = self.plot_widget.plot(pen=pg.mkPen('r', width=2))
        self.osc_curve_b = self.plot_widget.plot(pen=pg.mkPen('g', width=2))
        self.osc_curve_c = self.plot_widget.plot(pen=pg.mkPen('b', width=2))
        
        # Add simple text label instead of legend
        label_text = QtWidgets.QLabel("Red=Phase A  |  Green=Phase B  |  Blue=Phase C")
        label_text.setStyleSheet("color: gray; font-size: 9pt;")
        layout.addWidget(label_text)
        
        layout.addWidget(self.plot_widget)
        
        return widget
    
    def _build_dashboard(self) -> QtWidgets.QWidget:
        """Build telemetry dashboard panel."""
        widget = QtWidgets.QWidget()
        widget.setMaximumWidth(500)
        layout = QtWidgets.QVBoxLayout(widget)
        
        title = QtWidgets.QLabel("Telemetry Dashboard")
        title.setStyleSheet("font-size: 14pt; font-weight: bold;")
        layout.addWidget(title)
        
        # Scrollable area
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_content = QtWidgets.QWidget()
        self.dashboard_layout = QtWidgets.QVBoxLayout(scroll_content)
        
        # Store labels for updates
        self.dashboard_labels = {}
        
        # Add sections
        self._add_dashboard_section("Measurements", [
            ('grid_voltage_magnitude_pu', 'Grid V Mag (pu)'),
            ('grid_voltage_a_pu', 'Grid V A (pu)'),
            ('grid_voltage_b_pu', 'Grid V B (pu)'),
            ('grid_voltage_c_pu', 'Grid V C (pu)'),
            ('grid_current_magnitude_pu', 'Grid I Mag (pu)'),
            ('grid_current_a_pu', 'Grid I A (pu)'),
            ('grid_current_b_pu', 'Grid I B (pu)'),
            ('grid_current_c_pu', 'Grid I C (pu)'),
            ('dc_link_current_pu', 'DC Link I (pu)'),
        ], 'measurements')
        
        self._add_dashboard_section("Observer", [
            ('estimated_voltage_magnitude_pu', 'Est. V Mag (pu)'),
            ('estimated_frequency_pu', 'Est. Freq (pu)'),
            ('estimated_angle_rad', 'Est. Angle (rad)'),
        ], 'observer')
        
        self._add_dashboard_section("Controller", [
            ('dc_link_current_reference', 'I Ref (pu)'),
            ('dc_link_current_measured', 'I Meas (pu)'),
            ('current_error_pu', 'I Error (pu)'),
        ], 'controller')
        
        self.dashboard_layout.addStretch()
        scroll.setWidget(scroll_content)
        layout.addWidget(scroll)
        
        return widget
    
    def _add_dashboard_section(self, title: str, fields: list, category: str):
        """Add section to dashboard."""
        # Section title
        section_title = QtWidgets.QLabel(title)
        section_title.setStyleSheet("font-size: 11pt; font-weight: bold; margin-top: 8px;")
        self.dashboard_layout.addWidget(section_title)
        
        # Fields
        for field_key, field_label in fields:
            row = QtWidgets.QHBoxLayout()
            label = QtWidgets.QLabel(field_label + ":")
            label.setMinimumWidth(160)
            value_label = QtWidgets.QLabel("---")
            value_label.setStyleSheet("font-family: monospace; font-weight: bold;")
            row.addWidget(label)
            row.addWidget(value_label)
            row.addStretch()
            self.dashboard_layout.addLayout(row)
            
            # Store reference
            self.dashboard_labels[f"{category}.{field_key}"] = value_label
    
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
    
    def _on_start(self):
        """Start telemetry streaming."""
        if self.network_client.start_telemetry():
            logger.info("Telemetry started")
        else:
            QtWidgets.QMessageBox.warning(self, "Error", "Failed to start telemetry")
    
    def _on_stop(self):
        """Stop telemetry streaming."""
        if self.network_client.stop_telemetry():
            logger.info("Telemetry stopped")
        else:
            QtWidgets.QMessageBox.warning(self, "Error", "Failed to stop telemetry")
    
    def _on_clear(self):
        """Clear oscilloscope data."""
        self.osc_buffer.clear()
        self.sample_index = 0
        logger.info("Data cleared")
    
    def _on_connection_changed(self, connected: bool):
        """Handle connection status change."""
        if connected:
            self.status_label.setText("Connected")
            self.status_label.setStyleSheet("color: green; font-weight: bold;")
            self.connect_btn.setEnabled(False)
            self.disconnect_btn.setEnabled(True)
            self.start_btn.setEnabled(True)
            self.stop_btn.setEnabled(True)
            logger.info("Connection established")
        else:
            self.status_label.setText("Disconnected")
            self.status_label.setStyleSheet("color: red; font-weight: bold;")
            self.connect_btn.setEnabled(True)
            self.disconnect_btn.setEnabled(False)
            self.start_btn.setEnabled(False)
            self.stop_btn.setEnabled(False)
            logger.info("Connection closed")
    
    def _on_packet(self, packet: GridSideTelemetryPacket):
        """Handle packet - DIRECT processing."""
        try:
            # Decimation
            self.packet_counter += 1
            if self.packet_counter % self.decimate_factor != 0:
                return
            
            if self.verbose and self.packet_counter % 100 == 0:
                logger.info(f"Packet #{self.packet_counter}, sample={self.sample_index}")
            
            # Update oscilloscope - plot three phase currents
            if packet.measurements:
                meas = packet.measurements
                
                # Get the three phase currents
                current_a = meas.grid_current_a_pu
                current_b = meas.grid_current_b_pu
                current_c = meas.grid_current_c_pu
                
                # Append to buffer as a tuple
                self.osc_buffer.append(self.sample_index, (current_a, current_b, current_c))
                self.sample_index += 1
                
                # Debug: log current values occasionally to verify they're changing
                if self.verbose and self.packet_counter % 500 == 0:
                    logger.info(f"Currents: A={current_a:.6f}, B={current_b:.6f}, C={current_c:.6f}")
                
                # Update dashboard values
                self.latest_values['measurements']['grid_voltage_magnitude_pu'] = meas.grid_voltage_magnitude_pu
                self.latest_values['measurements']['grid_voltage_a_pu'] = meas.grid_voltage_a_pu
                self.latest_values['measurements']['grid_voltage_b_pu'] = meas.grid_voltage_b_pu
                self.latest_values['measurements']['grid_voltage_c_pu'] = meas.grid_voltage_c_pu
                self.latest_values['measurements']['grid_current_magnitude_pu'] = meas.grid_current_magnitude_pu
                self.latest_values['measurements']['grid_current_a_pu'] = meas.grid_current_a_pu
                self.latest_values['measurements']['grid_current_b_pu'] = meas.grid_current_b_pu
                self.latest_values['measurements']['grid_current_c_pu'] = meas.grid_current_c_pu
                self.latest_values['measurements']['dc_link_current_pu'] = meas.dc_link_current_pu
            
            if packet.observer:
                obs = packet.observer
                self.latest_values['observer']['estimated_voltage_magnitude_pu'] = obs.estimated_voltage_magnitude_pu
                self.latest_values['observer']['estimated_frequency_pu'] = obs.estimated_frequency_pu
                self.latest_values['observer']['estimated_angle_rad'] = obs.estimated_angle_rad
            
            if packet.controller:
                ctrl = packet.controller
                self.latest_values['controller']['dc_link_current_reference'] = ctrl.dc_link_current_reference
                self.latest_values['controller']['dc_link_current_measured'] = ctrl.dc_link_current_measured
                self.latest_values['controller']['current_error_pu'] = ctrl.current_error_pu
        
        except Exception as e:
            logger.error(f"Packet error: {e}", exc_info=True)
    
    def _update_plot(self):
        """Update oscilloscope - 100ms timer."""
        try:
            # Check if any data exists
            if len(self.osc_buffer.x_data) == 0:
                return
            
            # Get arrays for all three phase currents
            x_data, y_data_tuple = self.osc_buffer.get_arrays()
            
            # Unpack the y-data tuple into separate arrays for each phase
            y_data_a = y_data_tuple[:, 0]
            y_data_b = y_data_tuple[:, 1]
            y_data_c = y_data_tuple[:, 2]
            
            # Update all three curves - do this as fast as possible
            try:
                self.osc_curve_a.setData(x_data, y_data_a)
            except Exception as e:
                logger.error(f"Error updating curve A: {e}")
            
            try:
                self.osc_curve_b.setData(x_data, y_data_b)
            except Exception as e:
                logger.error(f"Error updating curve B: {e}")
            
            try:
                self.osc_curve_c.setData(x_data, y_data_c)
            except Exception as e:
                logger.error(f"Error updating curve C: {e}")
            
            # Update packet counter display
            self.packet_count_label.setText(f"Packets: {self.packet_counter}")
            
            # Auto-range X axis to show recent data (do this last, less critical)
            if len(x_data) > 0:
                latest = x_data[-1]
                self.plot_widget.setXRange(max(0, latest - 1000), latest, padding=0)
        
        except Exception as e:
            logger.error(f"Plot update error: {e}", exc_info=True)
    
    def _update_dashboard(self):
        """Update dashboard labels - 200ms timer."""
        try:
            for category, values in self.latest_values.items():
                for key, value in values.items():
                    label_key = f"{category}.{key}"
                    if label_key in self.dashboard_labels:
                        if isinstance(value, float):
                            self.dashboard_labels[label_key].setText(f"{value:.4f}")
                        else:
                            self.dashboard_labels[label_key].setText(str(value))
        
        except Exception as e:
            logger.error(f"Dashboard error: {e}", exc_info=True)
    
    def closeEvent(self, event):
        """Clean shutdown."""
        logger.info("Closing")
        self.plot_timer.stop()
        self.dashboard_timer.stop()
        if self.network_client.is_connected():
            self.network_client.disconnect()
        event.accept()


def parse_args():
    """Parse command line arguments."""
    import argparse
    parser = argparse.ArgumentParser(description='Extended Minimal Monitor')
    parser.add_argument('--host', default='192.168.2.100', help='IP address')
    parser.add_argument('--port', type=int, default=7000, help='TCP port')
    parser.add_argument('--verbose', action='store_true', help='Verbose logging')
    parser.add_argument('--decimate', type=int, default=1, help='Process 1 in N packets')
    return parser.parse_args()


def main():
    """Main entry point."""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
        datefmt='%H:%M:%S'
    )
    
    args = parse_args()
    
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle('Fusion')
    
    window = TelemetryMonitorWindow(
        host=args.host,
        port=args.port,
        verbose=args.verbose,
        decimate=args.decimate
    )
    window.show()
    
    logger.info("Extended minimal monitor started")
    return app.exec_()


if __name__ == '__main__':
    sys.exit(main())
