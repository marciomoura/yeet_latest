#!/usr/bin/env python3
"""GUI-based TCP client for real-time MCU data streaming.

This tool connects directly to the embedded TCP server, streams bulk packets,
maintains a rolling signal history, and renders a continuously updating plot.

Dependencies (install via pip if needed):
    pip install PyQt5 pyqtgraph

Usage:
    python network_gui_client.py --host 192.168.2.100 --port 7000

The GUI provides:
    * Connect / Disconnect controls
    * Start / Stop streaming commands
    * Amplitude / Noise parameter updates
    * Real-time waveform display with automatic dropping of oldest samples
    * CSV export of the currently retained data window
"""

import argparse
import csv
import sys
import threading
from collections import deque
from pathlib import Path

from PyQt5 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg

from network_client import MCUClient  # Reuse the TCP streaming client implementation


DEFAULT_MAX_DISPLAY_POINTS = 5000        # Points rendered on the chart
DEFAULT_BUFFER_POINTS = 20000            # Rolling window retained for export
FETCH_BATCH_POINTS = 5000                # Max samples fetched per polling iteration
UI_UPDATE_INTERVAL_MS = 50               # Plot refresh interval
STATS_UPDATE_INTERVAL_MS = 500           # Stats label refresh


class StreamingWindow(QtWidgets.QMainWindow):
    """Main application window handling connection, plotting, and controls."""

    def __init__(self, host: str, port: int, buffer_points: int) -> None:
        super().__init__()

        self.setWindowTitle("MCU Real-Time Recorder")
        self.resize(1100, 700)

        # Networking client (large buffer to match GUI rolling window)
        self.client = MCUClient(host=host, port=port, buffer_size=max(buffer_points, FETCH_BATCH_POINTS))
        self.receive_thread = None

        # Local rolling storage for display/export (sample_index, value)
        self.buffer_points = buffer_points
        self.display_buffer: deque[tuple[int, float]] = deque(maxlen=buffer_points)
        self.last_sample_index = -1

        self._build_ui(host, port)
        self._configure_plot()
        self._connect_signals()

        # Timers for UI updates
        self.plot_timer = QtCore.QTimer(self)
        self.plot_timer.timeout.connect(self.update_plot)
        self.plot_timer.start(UI_UPDATE_INTERVAL_MS)

        self.stats_timer = QtCore.QTimer(self)
        self.stats_timer.timeout.connect(self.update_stats)
        self.stats_timer.start(STATS_UPDATE_INTERVAL_MS)

        self.statusBar().showMessage("Disconnected")

    # ------------------------------------------------------------------
    # UI construction helpers
    # ------------------------------------------------------------------
    def _build_ui(self, host: str, port: int) -> None:
        central = QtWidgets.QWidget(self)
        self.setCentralWidget(central)

        main_layout = QtWidgets.QVBoxLayout(central)

        # Connection controls
        connection_box = QtWidgets.QGroupBox("Connection", self)
        connection_layout = QtWidgets.QGridLayout(connection_box)

        self.host_edit = QtWidgets.QLineEdit(host, self)
        self.port_spin = QtWidgets.QSpinBox(self)
        self.port_spin.setRange(1, 65535)
        self.port_spin.setValue(port)

        self.connect_button = QtWidgets.QPushButton("Connect", self)
        self.disconnect_button = QtWidgets.QPushButton("Disconnect", self)
        self.disconnect_button.setEnabled(False)

        connection_layout.addWidget(QtWidgets.QLabel("Host:"), 0, 0)
        connection_layout.addWidget(self.host_edit, 0, 1)
        connection_layout.addWidget(QtWidgets.QLabel("Port:"), 0, 2)
        connection_layout.addWidget(self.port_spin, 0, 3)
        connection_layout.addWidget(self.connect_button, 1, 0, 1, 2)
        connection_layout.addWidget(self.disconnect_button, 1, 2, 1, 2)

        # Streaming controls
        control_box = QtWidgets.QGroupBox("Controls", self)
        control_layout = QtWidgets.QGridLayout(control_box)

        self.start_button = QtWidgets.QPushButton("Start Streaming", self)
        self.stop_button = QtWidgets.QPushButton("Stop Streaming", self)
        self.stop_button.setEnabled(False)

        self.amplitude_spin = QtWidgets.QDoubleSpinBox(self)
        self.amplitude_spin.setRange(0.0, 10.0)
        self.amplitude_spin.setSingleStep(0.1)
        self.amplitude_spin.setValue(1.0)

        self.noise_spin = QtWidgets.QDoubleSpinBox(self)
        self.noise_spin.setRange(0.0, 1.0)
        self.noise_spin.setSingleStep(0.01)
        self.noise_spin.setValue(0.1)

        self.apply_amp_button = QtWidgets.QPushButton("Apply Amplitude", self)
        self.apply_noise_button = QtWidgets.QPushButton("Apply Noise", self)
        self.export_button = QtWidgets.QPushButton("Export CSV", self)

        control_layout.addWidget(self.start_button, 0, 0, 1, 2)
        control_layout.addWidget(self.stop_button, 0, 2, 1, 2)
        control_layout.addWidget(QtWidgets.QLabel("Amplitude"), 1, 0)
        control_layout.addWidget(self.amplitude_spin, 1, 1)
        control_layout.addWidget(self.apply_amp_button, 1, 2)
        control_layout.addWidget(QtWidgets.QLabel("Noise"), 2, 0)
        control_layout.addWidget(self.noise_spin, 2, 1)
        control_layout.addWidget(self.apply_noise_button, 2, 2)
        control_layout.addWidget(self.export_button, 3, 0, 1, 3)

        # Plot widget
        self.plot_widget = pg.PlotWidget(self)
        self.plot_curve = self.plot_widget.plot([], [], pen=pg.mkPen('#667eea', width=1.5))

        # Statistics labels
        stats_box = QtWidgets.QGroupBox("Statistics", self)
        stats_layout = QtWidgets.QGridLayout(stats_box)

        self.packets_label = QtWidgets.QLabel("0", self)
        self.samples_label = QtWidgets.QLabel("0", self)
        self.loss_label = QtWidgets.QLabel("0", self)
        self.buffer_label = QtWidgets.QLabel("0", self)
        self.latest_label = QtWidgets.QLabel("--", self)

        stats_layout.addWidget(QtWidgets.QLabel("Packets Received:"), 0, 0)
        stats_layout.addWidget(self.packets_label, 0, 1)
        stats_layout.addWidget(QtWidgets.QLabel("Total Samples:"), 0, 2)
        stats_layout.addWidget(self.samples_label, 0, 3)
        stats_layout.addWidget(QtWidgets.QLabel("Packets Lost:"), 1, 0)
        stats_layout.addWidget(self.loss_label, 1, 1)
        stats_layout.addWidget(QtWidgets.QLabel("Client Buffer Size:"), 1, 2)
        stats_layout.addWidget(self.buffer_label, 1, 3)
        stats_layout.addWidget(QtWidgets.QLabel("Latest Value:"), 2, 0)
        stats_layout.addWidget(self.latest_label, 2, 1)

        # Assemble layout
        main_layout.addWidget(connection_box)
        main_layout.addWidget(control_box)
        main_layout.addWidget(self.plot_widget, stretch=1)
        main_layout.addWidget(stats_box)

    def _configure_plot(self) -> None:
        self.plot_widget.setBackground('w')
        self.plot_widget.showGrid(x=True, y=True, alpha=0.2)
        self.plot_widget.setLabel('left', 'Signal Value')
        self.plot_widget.setLabel('bottom', 'Sample Index')

    def _connect_signals(self) -> None:
        self.connect_button.clicked.connect(self.connect_to_mcu)
        self.disconnect_button.clicked.connect(self.disconnect_from_mcu)
        self.start_button.clicked.connect(self.start_streaming)
        self.stop_button.clicked.connect(self.stop_streaming)
        self.apply_amp_button.clicked.connect(self.apply_amplitude)
        self.apply_noise_button.clicked.connect(self.apply_noise)
        self.export_button.clicked.connect(self.export_csv)

    # ------------------------------------------------------------------
    # Connection management
    # ------------------------------------------------------------------
    def connect_to_mcu(self) -> None:
        if self.client.connected:
            QtWidgets.QMessageBox.information(self, "Already Connected", "Client is already connected.")
            return

        host = self.host_edit.text().strip()
        port = int(self.port_spin.value())

        self.client.host = host
        self.client.port = port

        if not self.client.connect():
            QtWidgets.QMessageBox.critical(self, "Connection Failed", "Unable to connect to MCU.")
            return

        # Start receive loop thread
        self.receive_thread = threading.Thread(target=self.client.receive_loop, daemon=True)
        self.receive_thread.start()

        # Reset local buffers for new session
        self.display_buffer.clear()
        self.last_sample_index = -1
        self.plot_curve.clear()
        self.latest_label.setText("--")

        self.connect_button.setEnabled(False)
        self.disconnect_button.setEnabled(True)
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(True)
        self.statusBar().showMessage(f"Connected to {host}:{port}")

    def disconnect_from_mcu(self) -> None:
        if self.client.connected:
            self.client.stop_streaming()
            self.client.disconnect()

        self.connect_button.setEnabled(True)
        self.disconnect_button.setEnabled(False)
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(False)
        self.statusBar().showMessage("Disconnected")

    # ------------------------------------------------------------------
    # Streaming controls
    # ------------------------------------------------------------------
    def start_streaming(self) -> None:
        if not self.client.connected:
            QtWidgets.QMessageBox.warning(self, "Not Connected", "Connect to the MCU before streaming.")
            return
        self.client.start_streaming()

    def stop_streaming(self) -> None:
        if self.client.connected:
            self.client.stop_streaming()

    def apply_amplitude(self) -> None:
        if not self.client.connected:
            return
        amplitude = float(self.amplitude_spin.value())
        self.client.set_amplitude(amplitude)

    def apply_noise(self) -> None:
        if not self.client.connected:
            return
        noise = float(self.noise_spin.value())
        self.client.set_noise(noise)

    # ------------------------------------------------------------------
    # Data handling / plotting
    # ------------------------------------------------------------------
    def update_plot(self) -> None:
        if not self.client.connected:
            return

        new_points = self.client.get_data_since(self.last_sample_index, max_points=FETCH_BATCH_POINTS)
        if not new_points:
            return

        # Append new samples to rolling buffer
        for point in new_points:
            self.display_buffer.append((point['sample_index'], point['value']))
        self.last_sample_index = new_points[-1]['sample_index']

        if len(self.display_buffer) == 0:
            return

        # Prepare data for plotting, keep only the configured display window
        if len(self.display_buffer) > DEFAULT_MAX_DISPLAY_POINTS:
            relevant = list(self.display_buffer)[-DEFAULT_MAX_DISPLAY_POINTS:]
        else:
            relevant = list(self.display_buffer)

        x_vals = [sample for sample, _ in relevant]
        y_vals = [value for _, value in relevant]
        self.plot_curve.setData(x_vals, y_vals)

        # Update latest value label immediately for responsiveness
        self.latest_label.setText(f"{y_vals[-1]:.3f}")

    def update_stats(self) -> None:
        stats = self.client.get_stats()
        self.packets_label.setText(str(stats.get('packets_received', 0)))
        self.samples_label.setText(str(stats.get('total_samples', 0)))
        self.loss_label.setText(str(stats.get('packets_lost', 0)))
        self.buffer_label.setText(str(stats.get('buffer_size', 0)))

    # ------------------------------------------------------------------
    # Utilities
    # ------------------------------------------------------------------
    def export_csv(self) -> None:
        if len(self.display_buffer) == 0:
            QtWidgets.QMessageBox.information(self, "No Data", "There is no data to export.")
            return

        timestamp = QtCore.QDateTime.currentDateTime().toString('yyyyMMdd_hhmmss')
        default_name = f"mcu_recording_{timestamp}.csv"
        file_path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Export CSV", default_name, "CSV Files (*.csv)")
        if not file_path:
            return

        try:
            path_obj = Path(file_path)
            with path_obj.open('w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(["Sample Index", "Signal Value"])
                for sample_index, value in self.display_buffer:
                    writer.writerow([sample_index, f"{value:.6f}"])
            QtWidgets.QMessageBox.information(self, "Export Complete", f"Saved to {file_path}")
        except OSError as exc:
            QtWidgets.QMessageBox.critical(self, "Export Failed", f"Could not write file: {exc}")

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:  # noqa: N802
        # Ensure graceful disconnect
        try:
            self.stop_streaming()
            self.disconnect_from_mcu()
        finally:
            event.accept()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="MCU real-time recorder GUI")
    parser.add_argument('--host', default='192.168.2.100', help='MCU IP address (default: 192.168.2.100)')
    parser.add_argument('--port', type=int, default=7000, help='MCU TCP port (default: 7000)')
    parser.add_argument('--buffer-size', type=int, default=DEFAULT_BUFFER_POINTS,
                        help='Number of samples to retain locally (default: 20000)')
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    app = QtWidgets.QApplication(sys.argv)
    pg.setConfigOptions(antialias=True)

    window = StreamingWindow(host=args.host, port=args.port, buffer_points=args.buffer_size)
    window.show()

    return app.exec_()


if __name__ == '__main__':
    sys.exit(main())
