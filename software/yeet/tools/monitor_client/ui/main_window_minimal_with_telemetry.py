#!/usr/bin/env python3
"""MINIMAL oscilloscope with telemetry panel - ultra-simplified for stability.

Architecture:
- Four subplots: voltage, currents, observer, DC link control
- Right panel showing all telemetry values in real-time
- Python deque for data (automatic FIFO, no ring buffer math)
- Direct packet processing (no queues, no deferred work)
- One timer for plot updates (100ms)
- Separate timer for dashboard updates (200ms)
- No caching, no complex buffer logic
"""

import sys
import logging
from collections import deque
from typing import Optional

import numpy as np
from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg

from monitor_client.core.unified_network_client import UnifiedNetworkClient
from monitor_client.core.telemetry_protocol import GridSideTelemetryPacket

logger = logging.getLogger(__name__)


class TelemetryMonitorWindow(QtWidgets.QMainWindow):
    """Minimal telemetry monitor with 4 plots and telemetry panel."""
    
    def __init__(self, host: str, port: int, verbose: bool = False, decimate: int = 1, window_size: int = 1000):
        super().__init__()
        
        logger.info(f"=== Minimal Monitor with Telemetry (verbose={verbose}, decimate={decimate}, window={window_size}) ===")
        
        self.setWindowTitle("Minimal SFC Monitor - 4 Plots + Telemetry")
        self.resize(1800, 1000)
        
        # Set dark theme for entire application
        self.setStyleSheet("""
            QMainWindow {
                background-color: #2b2b2b;
            }
            QWidget {
                background-color: #2b2b2b;
                color: #ffffff;
            }
            QPushButton {
                background-color: #3c3c3c;
                color: #ffffff;
                border: 1px solid #555555;
                padding: 5px;
                border-radius: 3px;
            }
            QPushButton:hover {
                background-color: #4a4a4a;
            }
            QPushButton:pressed {
                background-color: #2a2a2a;
            }
            QPushButton:disabled {
                background-color: #2b2b2b;
                color: #666666;
            }
            QLabel {
                color: #ffffff;
                background-color: transparent;
            }
            QScrollArea {
                border: none;
                background-color: #1e1e1e;
            }
        """)
        
        self.verbose = verbose
        self.decimate_factor = max(1, decimate)
        self.packet_counter = 0
        self.sample_index = 0
        self.window_size = window_size  # Number of samples to display on X-axis
        
        # Simple deque buffers - automatically drop oldest when full
        self.voltage_buffer = deque(maxlen=2000)
        self.current_buffer = deque(maxlen=2000)  # Will store (sample_idx, ia, ib, ic)
        self.observer_buffer = deque(maxlen=2000)  # Will store (sample_idx, voltage_est, freq_est)
        self.dclink_buffer = deque(maxlen=2000)  # Will store (sample_idx, ref, measured)
        
        # Latest telemetry values for dashboard
        self.latest_packet = None
        
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
        main_layout.addLayout(telem_layout)
        
        # Main content: plots on left, telemetry panel on right
        content_layout = QtWidgets.QHBoxLayout()
        
        # Left side: Four plots in 2x2 grid
        plots_widget = QtWidgets.QWidget()
        plots_layout = QtWidgets.QGridLayout(plots_widget)
        plots_layout.setSpacing(5)
        
        # Plot 1: Grid Voltage (top-left)
        self.voltage_plot = pg.PlotWidget()
        self.voltage_plot.setBackground('#1e1e1e')
        self.voltage_plot.setLabel('left', 'Grid Voltage (pu)', color='#ffffff')
        self.voltage_plot.setLabel('bottom', 'Sample Index', color='#ffffff')
        self.voltage_plot.showGrid(x=True, y=True, alpha=0.3)
        self.voltage_plot.getAxis('left').setTextPen('#ffffff')
        self.voltage_plot.getAxis('bottom').setTextPen('#ffffff')
        self.voltage_curve = self.voltage_plot.plot(pen=pg.mkPen(color='g', width=2))
        plots_layout.addWidget(self.voltage_plot, 0, 0)
        
        # Plot 2: Grid Phase Currents (top-right)
        self.current_plot = pg.PlotWidget()
        self.current_plot.setBackground('#1e1e1e')
        self.current_plot.setLabel('left', 'Grid Current (pu)', color='#ffffff')
        self.current_plot.setLabel('bottom', 'Sample Index', color='#ffffff')
        self.current_plot.showGrid(x=True, y=True, alpha=0.3)
        self.current_plot.getAxis('left').setTextPen('#ffffff')
        self.current_plot.getAxis('bottom').setTextPen('#ffffff')
        self.current_curve_a = self.current_plot.plot(pen=pg.mkPen('r', width=2))
        self.current_curve_b = self.current_plot.plot(pen=pg.mkPen('g', width=2))
        self.current_curve_c = self.current_plot.plot(pen=pg.mkPen('b', width=2))
        plots_layout.addWidget(self.current_plot, 0, 1)
        
        # Plot 3: Observer (bottom-left)
        self.observer_plot = pg.PlotWidget()
        self.observer_plot.setBackground('#1e1e1e')
        self.observer_plot.setLabel('left', 'Observer', color='#ffffff')
        self.observer_plot.setLabel('bottom', 'Sample Index', color='#ffffff')
        self.observer_plot.showGrid(x=True, y=True, alpha=0.3)
        self.observer_plot.getAxis('left').setTextPen('#ffffff')
        self.observer_plot.getAxis('bottom').setTextPen('#ffffff')
        self.observer_voltage_curve = self.observer_plot.plot(pen=pg.mkPen('c', width=2))  # Cyan
        self.observer_freq_curve = self.observer_plot.plot(pen=pg.mkPen('m', width=2))  # Magenta
        plots_layout.addWidget(self.observer_plot, 1, 0)
        
        # Plot 4: DC Link Control (bottom-right)
        self.dclink_plot = pg.PlotWidget()
        self.dclink_plot.setBackground('#1e1e1e')
        self.dclink_plot.setLabel('left', 'DC Link Current (pu)', color='#ffffff')
        self.dclink_plot.setLabel('bottom', 'Sample Index', color='#ffffff')
        self.dclink_plot.showGrid(x=True, y=True, alpha=0.3)
        self.dclink_plot.getAxis('left').setTextPen('#ffffff')
        self.dclink_plot.getAxis('bottom').setTextPen('#ffffff')
        self.dclink_ref_curve = self.dclink_plot.plot(pen=pg.mkPen('y', width=2))  # Yellow
        self.dclink_measured_curve = self.dclink_plot.plot(pen=pg.mkPen('w', width=2))  # White
        plots_layout.addWidget(self.dclink_plot, 1, 1)
        
        content_layout.addWidget(plots_widget, 3)  # 75% width for plots
        
        # Right side: Telemetry panel
        telemetry_panel = self._build_telemetry_panel()
        content_layout.addWidget(telemetry_panel, 1)  # 25% width for telemetry
        
        main_layout.addLayout(content_layout, 1)  # Content takes all remaining space
        
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
        main_layout.addLayout(legend_layout)
        
        # Network client
        self.network_client = UnifiedNetworkClient(host, port, verbose=verbose, parent=self)
        self.network_client.connected_changed.connect(self._on_connection_changed)
        self.network_client.telemetry_packet_received.connect(self._on_packet)
        
        # Plot update timer (100ms = 10 Hz)
        self.plot_timer = QtCore.QTimer(self)
        self.plot_timer.timeout.connect(self._update_plot)
        self.plot_timer.start(100)
        
        # Dashboard update timer (200ms = 5 Hz, slower to reduce overhead)
        self.dashboard_timer = QtCore.QTimer(self)
        self.dashboard_timer.timeout.connect(self._update_dashboard)
        self.dashboard_timer.start(200)
        
        logger.info("Minimal monitor with telemetry initialized")
    
    def _build_telemetry_panel(self) -> QtWidgets.QWidget:
        """Build right-side telemetry panel with all values."""
        panel = QtWidgets.QWidget()
        panel.setMaximumWidth(400)
        panel.setStyleSheet("background-color: #1e1e1e;")  # Dark background
        panel_layout = QtWidgets.QVBoxLayout(panel)
        panel_layout.setSpacing(2)
        panel_layout.setContentsMargins(5, 5, 5, 5)
        
        # Title
        title = QtWidgets.QLabel("TELEMETRY VALUES")
        title.setStyleSheet("font-weight: bold; font-size: 12pt; color: #00ffff; background-color: #1e1e1e;")
        title.setAlignment(QtCore.Qt.AlignCenter)
        panel_layout.addWidget(title)
        
        # Scroll area for all values
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        scroll.setStyleSheet("background-color: #1e1e1e; border: none;")
        
        scroll_content = QtWidgets.QWidget()
        scroll_content.setStyleSheet("background-color: #1e1e1e;")
        scroll_layout = QtWidgets.QVBoxLayout(scroll_content)
        scroll_layout.setSpacing(1)
        
        # Dictionary to store label references
        self.telemetry_labels = {}
        
        # Helper to add sections
        def add_section(title: str, fields: list):
            # Section header
            header = QtWidgets.QLabel(f"▼ {title}")
            header.setStyleSheet("font-weight: bold; font-size: 10pt; color: #ffff00; margin-top: 5px; background-color: #1e1e1e;")
            scroll_layout.addWidget(header)
            
            # Fields
            for field_name, field_key in fields:
                row = QtWidgets.QHBoxLayout()
                name_label = QtWidgets.QLabel(f"{field_name}:")
                name_label.setStyleSheet("color: #d0d0d0; font-size: 9pt; background-color: #1e1e1e;")
                name_label.setMinimumWidth(180)
                
                value_label = QtWidgets.QLabel("---")
                value_label.setStyleSheet("color: #ffffff; font-size: 9pt; font-family: monospace; background-color: #1e1e1e;")
                value_label.setAlignment(QtCore.Qt.AlignRight)
                
                row.addWidget(name_label)
                row.addWidget(value_label)
                scroll_layout.addLayout(row)
                
                self.telemetry_labels[field_key] = value_label
        
        # Add all sections with ACTUAL fields from telemetry protocol
        add_section("Grid Measurements", [
            ("Voltage Magnitude", "grid_voltage_magnitude_pu"),
            ("Voltage A", "grid_voltage_a_pu"),
            ("Voltage B", "grid_voltage_b_pu"),
            ("Voltage C", "grid_voltage_c_pu"),
            ("Current Magnitude", "grid_current_magnitude_pu"),
            ("Current A", "grid_current_a_pu"),
            ("Current B", "grid_current_b_pu"),
            ("Current C", "grid_current_c_pu"),
            ("DC Link Current", "dc_link_current_pu"),
            ("Is Commuting", "is_commuting"),
        ])
        
        add_section("Observer", [
            ("Est. Voltage Mag.", "estimated_voltage_magnitude_pu"),
            ("Est. Frequency", "estimated_frequency_pu"),
            ("Est. Angle (rad)", "estimated_angle_rad"),
        ])
        
        add_section("Controller", [
            ("DC Ref", "dc_link_current_reference"),
            ("DC Measured", "dc_link_current_measured"),
            ("Current Error", "current_error_pu"),
            ("Controller Output", "controller_output_pu"),
            ("Operating Mode", "operating_mode"),
            ("Saturated", "controller_saturated"),
            ("Freeze Integrator", "freeze_integrator"),
        ])
        
        add_section("Modulator", [
            ("Is Enabled", "is_enabled"),
            ("Firing Angle Input", "firing_angle_input_pu"),
            ("Firing Angle Output", "firing_angle_output_pu"),
            ("Force Commutation Ack", "force_commutation_ack"),
        ])
        
        add_section("State Machine", [
            ("Current State", "current_state"),
            ("Pulse Releases", "pulse_releases"),
            ("Auxiliaries On", "auxiliaries_are_on"),
            ("Breakers On", "breakers_are_on"),
            ("Start Command", "start_command"),
            ("Stop Command", "stop_command"),
            ("Current Extinguished", "is_current_extinguished"),
            ("Shutdown Requested", "is_shutdown_requested"),
        ])
        
        scroll_layout.addStretch()
        scroll.setWidget(scroll_content)
        panel_layout.addWidget(scroll)
        
        return panel
    
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
            
            # Store latest packet for dashboard
            self.latest_packet = packet
            
            # Decimation for plotting
            if self.packet_counter % self.decimate_factor != 0:
                return
            
            # Log first packet
            if self.packet_counter == self.decimate_factor:
                logger.info(f"FIRST PACKET: seq={packet.header.sequence_number}")
            
            # Extract values for plotting
            if packet.measurements:
                meas = packet.measurements
                voltage = meas.grid_voltage_magnitude_pu
                current_a = meas.grid_current_a_pu
                current_b = meas.grid_current_b_pu
                current_c = meas.grid_current_c_pu
                
                # Add to buffers (deque auto-drops oldest)
                self.voltage_buffer.append((float(self.sample_index), voltage))
                self.current_buffer.append((float(self.sample_index), current_a, current_b, current_c))
            
            # Extract observer data
            if packet.observer:
                obs = packet.observer
                voltage_est = obs.estimated_voltage_magnitude_pu
                freq_est = obs.estimated_frequency_pu
                
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
    
    def _update_dashboard(self):
        """Update telemetry dashboard from latest packet."""
        if not self.latest_packet:
            return
        
        try:
            packet = self.latest_packet
            
            # Update measurements
            if packet.measurements:
                meas = packet.measurements
                self._set_label_value("grid_voltage_magnitude_pu", meas.grid_voltage_magnitude_pu)
                self._set_label_value("grid_voltage_a_pu", meas.grid_voltage_a_pu)
                self._set_label_value("grid_voltage_b_pu", meas.grid_voltage_b_pu)
                self._set_label_value("grid_voltage_c_pu", meas.grid_voltage_c_pu)
                self._set_label_value("grid_current_magnitude_pu", meas.grid_current_magnitude_pu)
                self._set_label_value("grid_current_a_pu", meas.grid_current_a_pu)
                self._set_label_value("grid_current_b_pu", meas.grid_current_b_pu)
                self._set_label_value("grid_current_c_pu", meas.grid_current_c_pu)
                self._set_label_value("dc_link_current_pu", meas.dc_link_current_pu)
                self._set_label_text("is_commuting", "YES" if meas.is_commuting else "NO")
            
            # Update observer
            if packet.observer:
                obs = packet.observer
                self._set_label_value("estimated_voltage_magnitude_pu", obs.estimated_voltage_magnitude_pu)
                self._set_label_value("estimated_frequency_pu", obs.estimated_frequency_pu)
                self._set_label_value("estimated_angle_rad", obs.estimated_angle_rad)
            
            # Update controller
            if packet.controller:
                ctrl = packet.controller
                self._set_label_value("dc_link_current_reference", ctrl.dc_link_current_reference)
                self._set_label_value("dc_link_current_measured", ctrl.dc_link_current_measured)
                self._set_label_value("current_error_pu", ctrl.current_error_pu)
                self._set_label_value("controller_output_pu", ctrl.controller_output_pu)
                self._set_label_text("operating_mode", "Natural" if ctrl.operating_mode == 1 else "Forced")
                self._set_label_text("controller_saturated", "YES" if ctrl.controller_saturated else "NO")
                self._set_label_text("freeze_integrator", "YES" if ctrl.freeze_integrator else "NO")
            
            # Update modulator
            if packet.modulator:
                mod = packet.modulator
                self._set_label_text("is_enabled", "YES" if mod.is_enabled else "NO")
                self._set_label_value("firing_angle_input_pu", mod.firing_angle_input_pu)
                self._set_label_value("firing_angle_output_pu", mod.firing_angle_output_pu)
                self._set_label_text("force_commutation_ack", "YES" if mod.force_commutation_ack else "NO")
            
            # Update state machine
            if packet.state_machine:
                sm = packet.state_machine
                # Map state enum to readable names
                state_names = {
                    0: 'IDLE',
                    1: 'AUXILIARIES_ON',
                    2: 'BREAKERS_OFF',
                    3: 'BREAKERS_ON',
                    4: 'OPERATION_OFF',
                    5: 'OPERATION_ON',
                    6: 'SHUTDOWN_REQUESTED'
                }
                state_name = state_names.get(sm.current_state, f"UNKNOWN({sm.current_state})")
                self._set_label_text("current_state", state_name)
                self._set_label_text("pulse_releases", "YES" if sm.pulse_releases else "NO")
                self._set_label_text("auxiliaries_are_on", "YES" if sm.auxiliaries_are_on else "NO")
                self._set_label_text("breakers_are_on", "YES" if sm.breakers_are_on else "NO")
                self._set_label_text("start_command", "YES" if sm.start_command else "NO")
                self._set_label_text("stop_command", "YES" if sm.stop_command else "NO")
                self._set_label_text("is_current_extinguished", "YES" if sm.is_current_extinguished else "NO")
                self._set_label_text("is_shutdown_requested", "YES" if sm.is_shutdown_requested else "NO")
        
        except Exception as e:
            logger.error(f"Dashboard update error: {e}", exc_info=True)
    
    def _set_label_value(self, key: str, value: float):
        """Set numeric label value with formatting."""
        if key in self.telemetry_labels:
            self.telemetry_labels[key].setText(f"{value:.6f}")
    
    def _set_label_text(self, key: str, text: str):
        """Set text label value."""
        if key in self.telemetry_labels:
            self.telemetry_labels[key].setText(text)


def main():
    """Entry point."""
    import argparse
    parser = argparse.ArgumentParser(description="Minimal SFC Monitor with Telemetry")
    parser.add_argument('--host', default='192.168.2.100')
    parser.add_argument('--port', type=int, default=7000)
    parser.add_argument('--verbose', action='store_true')
    parser.add_argument('--decimate', type=int, default=1, help='Process 1 out of N packets for plotting')
    parser.add_argument('--window', type=int, default=1000, help='Number of samples to display on X-axis')
    args = parser.parse_args()
    
    app = QtWidgets.QApplication(sys.argv)
    window = TelemetryMonitorWindow(args.host, args.port, args.verbose, args.decimate, args.window)
    window.show()
    return app.exec_()


if __name__ == '__main__':
    sys.exit(main())
