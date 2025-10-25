#!/usr/bin/env python3
"""MINIMAL oscilloscope - ultra-simplified for stability.

Architecture:
- Four subplots: voltage, currents, observer, DC link control
- Python deque for data (automatic FIFO, no ring buffer math)
- Direct packet processing (no queues, no deferred work)
- One timer for plot updates (100ms)
- No caching, no complex buffer logic
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


class TelemetryMonitorWindow(QtWidgets.QMainWindow):
    """Minimal telemetry monitor with single-plot oscilloscope."""
    
    def __init__(self, host: str, port: int, verbose: bool = False, decimate: int = 1, window_size: int = 1000):
        super().__init__()
        
        logger.info(f"=== Minimal Monitor (verbose={verbose}, decimate={decimate}, window={window_size}) ===")
        
        self.setWindowTitle("Minimal SFC Monitor - 4 Plots")
        self.resize(1400, 1000)
        
        self.verbose = verbose
        self.decimate_factor = max(1, decimate)
        self.packet_counter = 0
        self.sample_index = 0
        self.window_size = window_size  # Number of samples to display on X-axis
        
        # Simple deque buffers - automatically drop oldest when full
        self.voltage_buffer = deque(maxlen=2000)
        self.current_buffer = deque(maxlen=2000)  # Will store (sample_idx, ia, ib, ic)
        self.observer_buffer = deque(maxlen=2000)  # Will store (sample_idx, voltage_est, freq_est, angle_est)
        self.dclink_buffer = deque(maxlen=2000)  # Will store (sample_idx, ref, measured, error)
        
        # Build minimal UI
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)
        
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
        layout.addLayout(conn_layout)
        
        # Telemetry controls
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
        telem_layout.addWidget(self.status_label)
        layout.addLayout(telem_layout)
        
        # Four plots in 2x2 grid
        plots_widget = QtWidgets.QWidget()
        plots_layout = QtWidgets.QGridLayout(plots_widget)
        plots_layout.setSpacing(5)
        
        # Plot 1: Grid Voltage (top-left)
        self.voltage_plot = pg.PlotWidget()
        self.voltage_plot.setLabel('left', 'Grid Voltage (pu)')
        self.voltage_plot.setLabel('bottom', 'Sample Index')
        self.voltage_plot.showGrid(x=True, y=True, alpha=0.3)
        self.voltage_curve = self.voltage_plot.plot(pen=pg.mkPen(color='g', width=2))
        plots_layout.addWidget(self.voltage_plot, 0, 0)
        
        # Plot 2: Grid Phase Currents (top-right)
        self.current_plot = pg.PlotWidget()
        self.current_plot.setLabel('left', 'Grid Current (pu)')
        self.current_plot.setLabel('bottom', 'Sample Index')
        self.current_plot.showGrid(x=True, y=True, alpha=0.3)
        self.current_curve_a = self.current_plot.plot(pen=pg.mkPen('r', width=2))
        self.current_curve_b = self.current_plot.plot(pen=pg.mkPen('g', width=2))
        self.current_curve_c = self.current_plot.plot(pen=pg.mkPen('b', width=2))
        plots_layout.addWidget(self.current_plot, 0, 1)
        
        # Plot 3: Observer (bottom-left)
        self.observer_plot = pg.PlotWidget()
        self.observer_plot.setLabel('left', 'Observer')
        self.observer_plot.setLabel('bottom', 'Sample Index')
        self.observer_plot.showGrid(x=True, y=True, alpha=0.3)
        self.observer_voltage_curve = self.observer_plot.plot(pen=pg.mkPen('c', width=2))  # Cyan for voltage estimate
        self.observer_freq_curve = self.observer_plot.plot(pen=pg.mkPen('m', width=2))  # Magenta for frequency
        plots_layout.addWidget(self.observer_plot, 1, 0)
        
        # Plot 4: DC Link Control (bottom-right)
        self.dclink_plot = pg.PlotWidget()
        self.dclink_plot.setLabel('left', 'DC Link Current (pu)')
        self.dclink_plot.setLabel('bottom', 'Sample Index')
        self.dclink_plot.showGrid(x=True, y=True, alpha=0.3)
        self.dclink_ref_curve = self.dclink_plot.plot(pen=pg.mkPen('y', width=2))  # Yellow for reference
        self.dclink_measured_curve = self.dclink_plot.plot(pen=pg.mkPen('w', width=2))  # White for measured
        plots_layout.addWidget(self.dclink_plot, 1, 1)
        
        layout.addWidget(plots_widget, 1)  # Plots take all remaining space
        
        # Add legend labels
        legend_layout = QtWidgets.QHBoxLayout()
        legend_layout.addWidget(QtWidgets.QLabel("Currents: R=A, G=B, B=C"))
        legend_layout.addWidget(QtWidgets.QLabel("  |  "))
        legend_layout.addWidget(QtWidgets.QLabel("Observer: Cyan=V_est, Magenta=Freq"))
        legend_layout.addWidget(QtWidgets.QLabel("  |  "))
        legend_layout.addWidget(QtWidgets.QLabel("DC Link: Yellow=Ref, White=Meas"))
        legend_layout.addStretch()
        for i in range(legend_layout.count()):
            widget = legend_layout.itemAt(i).widget()
            if widget:
                widget.setStyleSheet("color: gray; font-size: 9pt;")
        layout.addLayout(legend_layout)
        
        # Network client
        self.network_client = UnifiedNetworkClient(host, port, verbose=verbose, parent=self)
        self.network_client.connected_changed.connect(self._on_connection_changed)
        self.network_client.telemetry_packet_received.connect(self._on_packet)
        
        # Plot update timer (100ms = 10 Hz, very conservative)
        self.plot_timer = QtCore.QTimer(self)
        self.plot_timer.timeout.connect(self._update_plot)
        self.plot_timer.start(100)
        
        logger.info("Minimal monitor initialized")
    
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
        if connected:
            self.status_label.setText("Connected - Ready")
        else:
            self.status_label.setText("Disconnected")
    
    def _on_start(self):
        """Start telemetry."""
        self.voltage_buffer.clear()
        self.current_buffer.clear()
        self.observer_buffer.clear()
        self.dclink_buffer.clear()
        self.sample_index = 0
        self.packet_counter = 0
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
    
    def _on_packet(self, packet: GridSideTelemetryPacket):
        """Handle incoming telemetry packet - DIRECT processing, no queue."""
        try:
            self.packet_counter += 1
            
            # Decimation
            if self.packet_counter % self.decimate_factor != 0:
                return
            
            # Log first packet
            if self.packet_counter == self.decimate_factor:
                logger.info(f"FIRST PACKET: seq={packet.header.sequence_number}")
            
            # Extract values
            if packet.measurements:
                meas = packet.measurements
                voltage = meas.grid_voltage_magnitude_pu
                current_a = meas.grid_current_a_pu
                current_b = meas.grid_current_b_pu
                current_c = meas.grid_current_c_pu
                dc_link_current = meas.dc_link_current_pu
                
                # Add to buffers (deque auto-drops oldest)
                self.voltage_buffer.append((float(self.sample_index), voltage))
                self.current_buffer.append((float(self.sample_index), current_a, current_b, current_c))
            
            # Extract observer data
            if packet.observer:
                obs = packet.observer
                voltage_est = obs.estimated_voltage_magnitude_pu
                freq_est = obs.estimated_frequency_pu
                angle_est = obs.estimated_angle_rad
                
                self.observer_buffer.append((float(self.sample_index), voltage_est, freq_est))
            
            # Extract DC link control data
            if packet.controller:
                ctrl = packet.controller
                dc_ref = ctrl.dc_link_current_reference
                dc_meas = ctrl.dc_link_current_measured
                
                self.dclink_buffer.append((float(self.sample_index), dc_ref, dc_meas))
            
            self.sample_index += 1
                
            # Log periodically
            if self.verbose and self.sample_index % 100 == 0:
                logger.debug(f"Sample {self.sample_index}, buffers: V={len(self.voltage_buffer)}, I={len(self.current_buffer)}, Obs={len(self.observer_buffer)}, DC={len(self.dclink_buffer)}")
        
        except Exception as e:
            logger.error(f"Packet error: {e}", exc_info=True)
    
    def _update_plot(self):
        """Update plots from buffers - called by timer."""
        try:
            # Update voltage plot
            if len(self.voltage_buffer) > 0:
                x_data = np.array([p[0] for p in self.voltage_buffer])
                y_data = np.array([p[1] for p in self.voltage_buffer])
                self.voltage_curve.setData(x_data, y_data)
                
                # Auto-range to show recent data
                if len(x_data) > 0:
                    latest = x_data[-1]
                    self.voltage_plot.setXRange(max(0, latest - self.window_size), latest, padding=0)
            
            # Update current plots
            if len(self.current_buffer) > 0:
                x_data = np.array([p[0] for p in self.current_buffer])
                y_data_a = np.array([p[1] for p in self.current_buffer])
                y_data_b = np.array([p[2] for p in self.current_buffer])
                y_data_c = np.array([p[3] for p in self.current_buffer])
                
                self.current_curve_a.setData(x_data, y_data_a)
                self.current_curve_b.setData(x_data, y_data_b)
                self.current_curve_c.setData(x_data, y_data_c)
                
                # Auto-range to show recent data
                if len(x_data) > 0:
                    latest = x_data[-1]
                    self.current_plot.setXRange(max(0, latest - self.window_size), latest, padding=0)
            
            # Update observer plots
            if len(self.observer_buffer) > 0:
                x_data = np.array([p[0] for p in self.observer_buffer])
                y_voltage_est = np.array([p[1] for p in self.observer_buffer])
                y_freq_est = np.array([p[2] for p in self.observer_buffer])
                
                self.observer_voltage_curve.setData(x_data, y_voltage_est)
                self.observer_freq_curve.setData(x_data, y_freq_est)
                
                # Auto-range to show recent data
                if len(x_data) > 0:
                    latest = x_data[-1]
                    self.observer_plot.setXRange(max(0, latest - self.window_size), latest, padding=0)
            
            # Update DC link control plots
            if len(self.dclink_buffer) > 0:
                x_data = np.array([p[0] for p in self.dclink_buffer])
                y_ref = np.array([p[1] for p in self.dclink_buffer])
                y_measured = np.array([p[2] for p in self.dclink_buffer])
                
                self.dclink_ref_curve.setData(x_data, y_ref)
                self.dclink_measured_curve.setData(x_data, y_measured)
                
                # Auto-range to show recent data
                if len(x_data) > 0:
                    latest = x_data[-1]
                    self.dclink_plot.setXRange(max(0, latest - self.window_size), latest, padding=0)
        
        except Exception as e:
            logger.error(f"Plot error: {e}", exc_info=True)


def main():
    """Entry point."""
    import argparse
    parser = argparse.ArgumentParser(description="Minimal SFC Monitor")
    parser.add_argument('--host', default='192.168.2.100')
    parser.add_argument('--port', type=int, default=7000)
    parser.add_argument('--verbose', action='store_true')
    parser.add_argument('--decimate', type=int, default=1, help='Process 1 out of N packets')
    parser.add_argument('--window', type=int, default=1000, help='Number of samples to display on X-axis')
    args = parser.parse_args()
    
    app = QtWidgets.QApplication(sys.argv)
    window = TelemetryMonitorWindow(args.host, args.port, args.verbose, args.decimate, args.window)
    window.show()
    return app.exec_()


if __name__ == '__main__':
    sys.exit(main())
