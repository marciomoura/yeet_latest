#!/usr/bin/env python3
"""Real-time telemetry dashboard for SFC control system.

This application provides a live dashboard displaying telemetry data from
the embedded grid-side controller. Signals are organized into logical groups
with digital displays showing real-time values.

Features:
    - Connection management (connect/disconnect to controller)
    - Telemetry control (start/stop streaming)
    - Digital displays for all signals organized by functional group
    - Real-time value updates at high rate
    - Connection statistics and monitoring
    - Extensible architecture for adding plotting and logging

Dependencies:
    pip install PyQt5

Usage:
    python telemetry_dashboard.py --host 192.168.2.100 --port 7000

Architecture:
    - TelemetryNetworkClient: Handles TCP communication and decoding
    - SignalManager: Manages signal data storage and retrieval
    - DashboardWindow: Main UI with signal displays
"""

import argparse
import logging
import sys
from typing import Optional

from PyQt5 import QtCore, QtGui, QtWidgets

from telemetry_network_client import TelemetryNetworkClient
from telemetry_protocol import GridSideTelemetryPacket
from telemetry_signal_manager import SignalManager


# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


# UI update intervals
UI_UPDATE_INTERVAL_MS = 50      # Signal display refresh (20 Hz)
STATS_UPDATE_INTERVAL_MS = 500  # Statistics refresh (2 Hz)

# Default buffer size for signal history
DEFAULT_BUFFER_SIZE = 10000


class SignalDisplayWidget(QtWidgets.QWidget):
    """Widget displaying a single signal with name, value, and unit.
    
    Shows signal in digital display format:
        Signal Name: [  value  ] unit
    
    Example:
        Grid Voltage Mag.: [ 1.023 ] pu
        DC Link Current:   [ 0.456 ] pu
    """
    
    def __init__(self, display_name: str, unit: str, enum_map: Optional[dict] = None, parent=None):
        """Initialize signal display widget.
        
        Args:
            display_name: Human-readable signal name
            unit: Physical unit (e.g., 'pu', 'rad', 'Hz')
            enum_map: Optional mapping from numeric value to text (for enums)
            parent: Parent widget
        """
        super().__init__(parent)
        
        self.display_name = display_name
        self.unit = unit
        self.enum_map = enum_map
        
        # Create layout
        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        # Signal name label
        self.name_label = QtWidgets.QLabel(f"{display_name}:", self)
        self.name_label.setMinimumWidth(180)
        self.name_label.setStyleSheet("font-weight: bold;")
        layout.addWidget(self.name_label)
        
        # Value display (LCD-style)
        self.value_display = QtWidgets.QLineEdit(self)
        self.value_display.setReadOnly(True)
        self.value_display.setAlignment(QtCore.Qt.AlignCenter)
        self.value_display.setMinimumWidth(100)
        self.value_display.setMaximumWidth(120)
        self.value_display.setStyleSheet("""
            QLineEdit {
                background-color: #1a1a1a;
                color: #00ff00;
                font-family: 'Courier New', monospace;
                font-size: 14px;
                font-weight: bold;
                border: 2px solid #333333;
                border-radius: 3px;
                padding: 2px;
            }
        """)
        self.value_display.setText("--")
        layout.addWidget(self.value_display)
        
        # Unit label
        self.unit_label = QtWidgets.QLabel(unit, self)
        self.unit_label.setMinimumWidth(40)
        layout.addWidget(self.unit_label)
        
        layout.addStretch()
    
    def set_value(self, value: Optional[float]) -> None:
        """Update displayed value.
        
        Args:
            value: Signal value or None if unavailable
        """
        if value is None:
            self.value_display.setText("--")
        else:
            # If enum mapping provided, convert to text
            if self.enum_map is not None:
                int_value = int(value)
                text = self.enum_map.get(int_value, f"UNKNOWN({int_value})")
            else:
                # Format numeric value based on magnitude
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
    """Widget displaying a group of related signals.
    
    Groups signals logically (e.g., measurements, controller) in a
    vertical layout with title.
    """
    
    def __init__(self, group_name: str, title: str, parent=None):
        """Initialize signal group widget.
        
        Args:
            group_name: Internal group name (e.g., 'measurements')
            title: Display title for group box
            parent: Parent widget
        """
        super().__init__(title, parent)
        
        self.group_name = group_name
        self.signal_displays = {}
        
        # Create layout
        self.layout = QtWidgets.QVBoxLayout(self)
        self.layout.setSpacing(5)
        
        # Style
        self.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                font-size: 13px;
                border: 2px solid #667eea;
                border-radius: 5px;
                margin-top: 10px;
                padding-top: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }
        """)
    
    def add_signal_display(self, signal_name: str, display_name: str, unit: str, 
                           enum_map: Optional[dict] = None) -> None:
        """Add signal display to group.
        
        Args:
            signal_name: Internal signal name
            display_name: Human-readable name
            unit: Physical unit
            enum_map: Optional mapping from numeric value to text (for enums)
        """
        display = SignalDisplayWidget(display_name, unit, enum_map, self)
        self.signal_displays[signal_name] = display
        self.layout.addWidget(display)
    
    def update_signal(self, signal_name: str, value: Optional[float]) -> None:
        """Update specific signal value.
        
        Args:
            signal_name: Signal name
            value: New value or None
        """
        display = self.signal_displays.get(signal_name)
        if display:
            display.set_value(value)
    
    def update_all_signals(self, values: dict) -> None:
        """Update all signals from dictionary.
        
        Args:
            values: Dictionary mapping signal name to value
        """
        for signal_name, value in values.items():
            self.update_signal(signal_name, value)


class DashboardWindow(QtWidgets.QMainWindow):
    """Main dashboard window with connection controls and signal displays.
    
    Provides complete interface for:
        - Connecting to controller
        - Starting/stopping telemetry
        - Viewing real-time signal values
        - Monitoring connection statistics
    """
    
    def __init__(self, host: str, port: int, buffer_size: int, verbose: bool = False):
        """Initialize dashboard window.
        
        Args:
            host: Controller IP address
            port: Controller TCP port
            buffer_size: Signal history buffer size
            verbose: Enable verbose logging for debugging
        """
        super().__init__()
        
        self.setWindowTitle("SFC Telemetry Dashboard")
        self.resize(900, 800)
        
        # Create signal manager
        self.signal_manager = SignalManager(buffer_size=buffer_size)
        
        # Create network client
        self.network_client = TelemetryNetworkClient(host=host, port=port, verbose=verbose)
        self.network_client.set_packet_callback(self._on_packet_received)
        
        # Build UI
        self._build_ui(host, port)
        
        # Setup timers for UI updates
        self.ui_update_timer = QtCore.QTimer(self)
        self.ui_update_timer.timeout.connect(self._update_signal_displays)
        self.ui_update_timer.start(UI_UPDATE_INTERVAL_MS)
        
        self.stats_update_timer = QtCore.QTimer(self)
        self.stats_update_timer.timeout.connect(self._update_statistics)
        self.stats_update_timer.start(STATS_UPDATE_INTERVAL_MS)
        
        self.statusBar().showMessage("Disconnected")
        
        logger.info("Dashboard initialized")
    
    def _build_ui(self, host: str, port: int) -> None:
        """Build main UI components.
        
        Args:
            host: Default host address
            port: Default port number
        """
        # Central widget and main layout
        central = QtWidgets.QWidget(self)
        self.setCentralWidget(central)
        main_layout = QtWidgets.QVBoxLayout(central)
        
        # Connection controls
        connection_box = self._build_connection_controls(host, port)
        main_layout.addWidget(connection_box)
        
        # Telemetry controls
        control_box = self._build_telemetry_controls()
        main_layout.addWidget(control_box)
        
        # Signal displays in scroll area
        scroll_area = QtWidgets.QScrollArea(self)
        scroll_area.setWidgetResizable(True)
        scroll_area.setFrameShape(QtWidgets.QFrame.NoFrame)
        
        signals_widget = QtWidgets.QWidget()
        signals_layout = QtWidgets.QVBoxLayout(signals_widget)
        
        # Create signal group displays
        self.signal_group_widgets = {}
        self._create_signal_group_displays(signals_layout)
        
        signals_layout.addStretch()
        scroll_area.setWidget(signals_widget)
        main_layout.addWidget(scroll_area, stretch=1)
        
        # Statistics display
        stats_box = self._build_statistics_display()
        main_layout.addWidget(stats_box)
    
    def _build_connection_controls(self, host: str, port: int) -> QtWidgets.QGroupBox:
        """Build connection control widgets.
        
        Args:
            host: Default host
            port: Default port
            
        Returns:
            QGroupBox with connection controls
        """
        box = QtWidgets.QGroupBox("Connection", self)
        layout = QtWidgets.QGridLayout(box)
        
        # Host input
        layout.addWidget(QtWidgets.QLabel("Host:"), 0, 0)
        self.host_edit = QtWidgets.QLineEdit(host, self)
        self.host_edit.setMinimumWidth(150)
        layout.addWidget(self.host_edit, 0, 1)
        
        # Port input
        layout.addWidget(QtWidgets.QLabel("Port:"), 0, 2)
        self.port_spin = QtWidgets.QSpinBox(self)
        self.port_spin.setRange(1, 65535)
        self.port_spin.setValue(port)
        layout.addWidget(self.port_spin, 0, 3)
        
        # Connect/Disconnect buttons
        self.connect_button = QtWidgets.QPushButton("Connect", self)
        self.connect_button.clicked.connect(self._on_connect_clicked)
        layout.addWidget(self.connect_button, 1, 0, 1, 2)
        
        self.disconnect_button = QtWidgets.QPushButton("Disconnect", self)
        self.disconnect_button.clicked.connect(self._on_disconnect_clicked)
        self.disconnect_button.setEnabled(False)
        layout.addWidget(self.disconnect_button, 1, 2, 1, 2)
        
        return box
    
    def _build_telemetry_controls(self) -> QtWidgets.QGroupBox:
        """Build telemetry control widgets.
        
        Returns:
            QGroupBox with telemetry controls
        """
        box = QtWidgets.QGroupBox("Telemetry Control", self)
        layout = QtWidgets.QHBoxLayout(box)
        
        # Start button
        self.start_button = QtWidgets.QPushButton("Start Streaming", self)
        self.start_button.clicked.connect(self._on_start_clicked)
        self.start_button.setEnabled(False)
        layout.addWidget(self.start_button)
        
        # Stop button
        self.stop_button = QtWidgets.QPushButton("Stop Streaming", self)
        self.stop_button.clicked.connect(self._on_stop_clicked)
        self.stop_button.setEnabled(False)
        layout.addWidget(self.stop_button)
        
        # Clear button
        self.clear_button = QtWidgets.QPushButton("Clear Data", self)
        self.clear_button.clicked.connect(self._on_clear_clicked)
        layout.addWidget(self.clear_button)
        
        layout.addStretch()
        
        return box
    
    def _create_signal_group_displays(self, parent_layout: QtWidgets.QVBoxLayout) -> None:
        """Create signal display widgets for all groups.
        
        Args:
            parent_layout: Layout to add group widgets to
        """
        # State machine enumeration mappings
        sfc_state_map = {
            0: 'IDLE',
            1: 'AUXILIARIES_ON',
            2: 'BREAKERS_OFF',
            3: 'BREAKERS_ON',
            4: 'OPERATION_OFF',
            5: 'OPERATION_ON',
            6: 'SHUTDOWN_REQUESTED'
        }
        
        # Measurements group
        meas_widget = SignalGroupWidget('measurements', 'Grid & DC Measurements', self)
        meas_widget.add_signal_display('grid_voltage_magnitude_pu', 'Grid Voltage Mag.', 'pu')
        meas_widget.add_signal_display('grid_voltage_a_pu', 'Grid Voltage Phase A', 'pu')
        meas_widget.add_signal_display('grid_voltage_b_pu', 'Grid Voltage Phase B', 'pu')
        meas_widget.add_signal_display('grid_voltage_c_pu', 'Grid Voltage Phase C', 'pu')
        meas_widget.add_signal_display('dc_link_current_pu', 'DC Link Current', 'pu')
        meas_widget.add_signal_display('is_commuting', 'Is Commuting', '')
        self.signal_group_widgets['measurements'] = meas_widget
        parent_layout.addWidget(meas_widget)
        
        # Observer group
        obs_widget = SignalGroupWidget('observer', 'Grid Observer', self)
        obs_widget.add_signal_display('estimated_voltage_magnitude_pu', 'Est. Voltage Mag.', 'pu')
        obs_widget.add_signal_display('estimated_frequency_pu', 'Estimated Frequency', 'pu')
        obs_widget.add_signal_display('estimated_angle_rad', 'Estimated Angle', 'rad')
        self.signal_group_widgets['observer'] = obs_widget
        parent_layout.addWidget(obs_widget)
        
        # Modulator group
        mod_widget = SignalGroupWidget('modulator', 'Grid-Side Modulator', self)
        mod_widget.add_signal_display('is_enabled', 'Is Enabled', '')
        mod_widget.add_signal_display('firing_angle_input_pu', 'Firing Angle Input', 'pu')
        mod_widget.add_signal_display('firing_angle_output_pu', 'Firing Angle Output', 'pu')
        mod_widget.add_signal_display('force_commutation_ack', 'Force Commutation Ack', '')
        self.signal_group_widgets['modulator'] = mod_widget
        parent_layout.addWidget(mod_widget)
        
        # Controller group
        ctrl_widget = SignalGroupWidget('controller', 'Current Controller', self)
        ctrl_widget.add_signal_display('dc_link_current_reference', 'DC Current Reference', 'pu')
        ctrl_widget.add_signal_display('dc_link_current_measured', 'DC Current Measured', 'pu')
        ctrl_widget.add_signal_display('current_error_pu', 'Current Error', 'pu')
        ctrl_widget.add_signal_display('controller_output_pu', 'Controller Output', 'pu')
        ctrl_widget.add_signal_display('operating_mode', 'Operating Mode', '')
        ctrl_widget.add_signal_display('controller_saturated', 'Controller Saturated', '')
        ctrl_widget.add_signal_display('freeze_integrator', 'Freeze Integrator', '')
        self.signal_group_widgets['controller'] = ctrl_widget
        parent_layout.addWidget(ctrl_widget)
        
        # State machine group
        state_widget = SignalGroupWidget('state_machine', 'State Machine', self)
        state_widget.add_signal_display('current_state', 'Current State', '', enum_map=sfc_state_map)
        state_widget.add_signal_display('pulse_releases', 'Pulses Are Released', '')
        state_widget.add_signal_display('auxiliaries_are_on', 'Auxiliaries Are On', '')
        state_widget.add_signal_display('breakers_are_on', 'Breakers Are On', '')
        state_widget.add_signal_display('start_command', 'Start Command', '')
        state_widget.add_signal_display('stop_command', 'Stop Command', '')
        state_widget.add_signal_display('is_current_extinguished', 'Current Extinguished', '')
        state_widget.add_signal_display('is_shutdown_requested', 'Shutdown Requested', '')
        self.signal_group_widgets['state_machine'] = state_widget
        parent_layout.addWidget(state_widget)
    
    def _build_statistics_display(self) -> QtWidgets.QGroupBox:
        """Build statistics display widgets.
        
        Returns:
            QGroupBox with statistics labels
        """
        box = QtWidgets.QGroupBox("Connection Statistics", self)
        layout = QtWidgets.QGridLayout(box)
        
        # Statistics labels
        layout.addWidget(QtWidgets.QLabel("Packets Received:"), 0, 0)
        self.packets_received_label = QtWidgets.QLabel("0", self)
        layout.addWidget(self.packets_received_label, 0, 1)
        
        layout.addWidget(QtWidgets.QLabel("Packets Lost:"), 0, 2)
        self.packets_lost_label = QtWidgets.QLabel("0", self)
        layout.addWidget(self.packets_lost_label, 0, 3)
        
        layout.addWidget(QtWidgets.QLabel("Loss Rate:"), 0, 4)
        self.loss_rate_label = QtWidgets.QLabel("0.0%", self)
        layout.addWidget(self.loss_rate_label, 0, 5)
        
        layout.addWidget(QtWidgets.QLabel("Bytes Received:"), 1, 0)
        self.bytes_received_label = QtWidgets.QLabel("0", self)
        layout.addWidget(self.bytes_received_label, 1, 1)
        
        layout.addWidget(QtWidgets.QLabel("Decode Errors:"), 1, 2)
        self.decode_errors_label = QtWidgets.QLabel("0", self)
        layout.addWidget(self.decode_errors_label, 1, 3)
        
        layout.setColumnStretch(6, 1)
        
        return box
    
    def _on_connect_clicked(self) -> None:
        """Handle connect button click."""
        host = self.host_edit.text().strip()
        port = self.port_spin.value()
        
        self.network_client.host = host
        self.network_client.port = port
        
        if self.network_client.connect():
            self.connect_button.setEnabled(False)
            self.disconnect_button.setEnabled(True)
            self.start_button.setEnabled(True)
            self.stop_button.setEnabled(True)
            self.statusBar().showMessage(f"Connected to {host}:{port}")
            logger.info(f"Connected to {host}:{port}")
        else:
            QtWidgets.QMessageBox.critical(
                self, "Connection Failed",
                f"Unable to connect to {host}:{port}"
            )
    
    def _on_disconnect_clicked(self) -> None:
        """Handle disconnect button click."""
        self.network_client.disconnect()
        self.connect_button.setEnabled(True)
        self.disconnect_button.setEnabled(False)
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(False)
        self.statusBar().showMessage("Disconnected")
        logger.info("Disconnected")
    
    def _on_start_clicked(self) -> None:
        """Handle start streaming button click."""
        if self.network_client.send_command('START'):
            logger.info("Telemetry streaming started")
            self.statusBar().showMessage("Streaming telemetry...")
        else:
            QtWidgets.QMessageBox.warning(
                self, "Command Failed",
                "Failed to send START command"
            )
    
    def _on_stop_clicked(self) -> None:
        """Handle stop streaming button click."""
        if self.network_client.send_command('STOP'):
            logger.info("Telemetry streaming stopped")
            self.statusBar().showMessage("Streaming stopped")
        else:
            QtWidgets.QMessageBox.warning(
                self, "Command Failed",
                "Failed to send STOP command"
            )
    
    def _on_clear_clicked(self) -> None:
        """Handle clear data button click."""
        self.signal_manager.clear_all()
        self.network_client.reset_statistics()
        logger.info("Data cleared")
    
    def _on_packet_received(self, packet: GridSideTelemetryPacket) -> None:
        """Callback invoked when telemetry packet received.
        
        Called from network receive thread. Updates signal manager.
        
        Args:
            packet: Decoded telemetry packet
        """
        self.signal_manager.update_from_packet(packet)
    
    def _update_signal_displays(self) -> None:
        """Update all signal displays with latest values (UI thread)."""
        for group_name, group_widget in self.signal_group_widgets.items():
            group = self.signal_manager.get_group(group_name)
            if group:
                values = group.get_all_latest_values()
                group_widget.update_all_signals(values)
    
    def _update_statistics(self) -> None:
        """Update statistics display (UI thread)."""
        stats = self.network_client.get_statistics()
        
        self.packets_received_label.setText(str(stats['packets_received']))
        self.packets_lost_label.setText(str(stats['packets_lost']))
        self.loss_rate_label.setText(f"{stats['loss_rate_percent']:.2f}%")
        self.bytes_received_label.setText(str(stats['bytes_received']))
        self.decode_errors_label.setText(str(stats['decode_errors']))
    
    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        """Handle window close event."""
        # Ensure graceful disconnect
        if self.network_client.connected:
            self.network_client.send_command('STOP')
            self.network_client.disconnect()
        
        logger.info("Dashboard closed")
        event.accept()


def parse_args() -> argparse.Namespace:
    """Parse command line arguments.
    
    Returns:
        Parsed arguments
    """
    parser = argparse.ArgumentParser(
        description="SFC Telemetry Dashboard - Real-time control system monitoring"
    )
    parser.add_argument(
        '--host',
        default='192.168.2.100',
        help='Controller IP address (default: 192.168.2.100)'
    )
    parser.add_argument(
        '--port',
        type=int,
        default=7000,
        help='Controller TCP port (default: 7000)'
    )
    parser.add_argument(
        '--buffer-size',
        type=int,
        default=DEFAULT_BUFFER_SIZE,
        help='Signal history buffer size (default: 10000)'
    )
    parser.add_argument(
        '--verbose',
        action='store_true',
        help='Enable verbose logging for decode debugging'
    )
    return parser.parse_args()


def main() -> int:
    """Main application entry point.
    
    Returns:
        Exit code
    """
    args = parse_args()
    
    app = QtWidgets.QApplication(sys.argv)
    
    # Set application style
    app.setStyle('Fusion')
    
    # Create and show main window
    window = DashboardWindow(
        host=args.host,
        port=args.port,
        buffer_size=args.buffer_size,
        verbose=args.verbose
    )
    window.show()
    
    logger.info("Application started")
    return app.exec_()


if __name__ == '__main__':
    sys.exit(main())
