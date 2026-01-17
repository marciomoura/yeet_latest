#!/usr/bin/env python3
"""Signal management for telemetry dashboard.

This module provides the SignalManager class which handles all signal
data storage, updating, and retrieval for the telemetry dashboard.
Manages rolling buffers for time-series data and provides thread-safe
access to signal values.

Architecture:
    - SignalGroup: Organized collection of related signals
    - SignalHistory: Rolling buffer for time-series data
    - SignalManager: Central manager coordinating all signal groups

Example Usage:
    manager = SignalManager(buffer_size=10000)
    manager.update_from_packet(telemetry_packet)
    
    # Get latest values
    grid_voltage = manager.get_latest_value('measurements', 'grid_voltage_magnitude_filtered_pu')
    
    # Get time series
    history = manager.get_time_series('measurements', 'dc_link_current_pu')
"""

import threading
from collections import deque
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Union

from ..core.telemetry_protocol import GridSideTelemetryPacket, MachineSideTelemetryPacket, GridSignalIndex, MachineSignalIndex


@dataclass
class SignalValue:
    """Single signal value with timestamp.
    
    Attributes:
        timestamp_ms: Controller timestamp in milliseconds
        value: Signal value (float or int)
        sequence: Packet sequence number
    """
    timestamp_ms: int
    value: float
    sequence: int


@dataclass
class SignalHistory:
    """Rolling buffer for signal time-series data.
    
    Maintains a fixed-size deque of signal values for plotting.
    Thread-safe access via lock.
    
    Attributes:
        name: Signal name (e.g., 'grid_voltage_magnitude_pu')
        unit: Physical unit (e.g., 'pu', 'rad', 'Hz')
        display_name: Human-readable name for UI
        values: Rolling buffer of signal values
        max_size: Maximum buffer size
    """
    name: str
    unit: str
    display_name: str
    max_size: int
    values: deque = field(default_factory=deque)
    _lock: threading.Lock = field(default_factory=threading.Lock, repr=False)
    
    def __post_init__(self):
        """Initialize values deque with max size."""
        self.values = deque(maxlen=self.max_size)
    
    def append(self, value: SignalValue) -> None:
        """Append new value to history (thread-safe).
        
        Args:
            value: SignalValue to append
        """
        with self._lock:
            self.values.append(value)
    
    def get_latest(self) -> Optional[SignalValue]:
        """Get most recent value (thread-safe).
        
        Returns:
            Latest SignalValue or None if empty
        """
        with self._lock:
            return self.values[-1] if self.values else None
    
    def get_all(self) -> List[SignalValue]:
        """Get copy of all values (thread-safe).
        
        Returns:
            List of all SignalValue objects in buffer
        """
        with self._lock:
            return list(self.values)
    
    def get_time_series(self) -> Tuple[List[float], List[float]]:
        """Get time series data for plotting.
        
        Returns:
            Tuple of (timestamps_ms, values) lists
        """
        with self._lock:
            if not self.values:
                return [], []
            timestamps = [v.timestamp_ms for v in self.values]
            vals = [v.value for v in self.values]
            return timestamps, vals
    
    def clear(self) -> None:
        """Clear all values (thread-safe)."""
        with self._lock:
            self.values.clear()


@dataclass
class SignalGroup:
    """Collection of related signals (e.g., measurements, controller).
    
    Attributes:
        name: Group name (e.g., 'measurements', 'observer')
        display_name: Human-readable name for UI
        signals: Dictionary mapping signal name to SignalHistory
    """
    name: str
    display_name: str
    signals: Dict[str, SignalHistory] = field(default_factory=dict)
    
    def add_signal(self, signal: SignalHistory) -> None:
        """Add signal to group.
        
        Args:
            signal: SignalHistory to add
        """
        self.signals[signal.name] = signal
    
    def get_signal(self, name: str) -> Optional[SignalHistory]:
        """Get signal by name.
        
        Args:
            name: Signal name
            
        Returns:
            SignalHistory or None if not found
        """
        return self.signals.get(name)
    
    def get_all_latest_values(self) -> Dict[str, Optional[float]]:
        """Get latest values for all signals in group.
        
        Returns:
            Dictionary mapping signal name to latest value
        """
        return {
            name: sig.get_latest().value if sig.get_latest() else None
            for name, sig in self.signals.items()
        }
    
    def clear_all(self) -> None:
        """Clear all signal histories in group."""
        for signal in self.signals.values():
            signal.clear()


class SignalManager:
    """Central manager for all telemetry signals.
    
    Organizes signals into logical groups and provides unified interface
    for updating from telemetry packets and retrieving signal data.
    Thread-safe for concurrent access from network and UI threads.
    """
    
    def __init__(self, buffer_size: int = 50000):
        """Initialize signal manager.
        
        Args:
            buffer_size: Maximum number of samples to store per signal
        """
        self.buffer_size = buffer_size
        self.groups: Dict[str, SignalGroup] = {}
        self._lock = threading.Lock()
        self.current_side = "grid"  # Track which side is currently initialized
        self._initialize_grid_side_signal_groups()
    
    def switch_to_grid_side(self) -> None:
        """Switch signal manager to grid-side configuration."""
        if self.current_side != "grid":
            self.clear_all()
            self._initialize_grid_side_signal_groups()
            self.current_side = "grid"
    
    def switch_to_machine_side(self) -> None:
        """Switch signal manager to machine-side configuration."""
        if self.current_side != "machine":
            self.clear_all()
            self._initialize_machine_side_signal_groups()
            self.current_side = "machine"
    
    def _initialize_grid_side_signal_groups(self) -> None:
        """Initialize all grid-side signal groups with their signals.
        
        Signal indices match C++ grid_side_telemetry_builder.cpp exactly.
        All 46 signals across 7 groups (indices 0-45).
        """
        
        # Measurements group
        measurements = SignalGroup('measurements', 'Grid & DC Measurements')
        measurements.add_signal(SignalHistory('grid_voltage_magnitude_filtered_pu', 'pu', 'Grid Voltage Mag (Filtered)', self.buffer_size))
        measurements.add_signal(SignalHistory('grid_voltage_a_pu', 'pu', 'Grid Voltage Phase A', self.buffer_size))
        measurements.add_signal(SignalHistory('grid_voltage_b_pu', 'pu', 'Grid Voltage Phase B', self.buffer_size))
        measurements.add_signal(SignalHistory('grid_voltage_c_pu', 'pu', 'Grid Voltage Phase C', self.buffer_size))
        measurements.add_signal(SignalHistory('grid_current_magnitude_pu', 'pu', 'Grid Current Magnitude', self.buffer_size))
        measurements.add_signal(SignalHistory('grid_current_a_pu', 'pu', 'Grid Current Phase A', self.buffer_size))
        measurements.add_signal(SignalHistory('grid_current_b_pu', 'pu', 'Grid Current Phase B', self.buffer_size))
        measurements.add_signal(SignalHistory('grid_current_c_pu', 'pu', 'Grid Current Phase C', self.buffer_size))
        measurements.add_signal(SignalHistory('dc_link_current_pu', 'pu', 'DC Link Current', self.buffer_size))
        self.groups['measurements'] = measurements
        
        # Observer group
        observer = SignalGroup('observer', 'Grid Observer')
        observer.add_signal(SignalHistory('estimated_voltage_magnitude_pu', 'pu', 'Est. Voltage Magnitude', self.buffer_size))
        observer.add_signal(SignalHistory('estimated_frequency_pu', 'pu', 'Est. Frequency (pu)', self.buffer_size))
        observer.add_signal(SignalHistory('estimated_angle_rad', 'rad', 'Est. Angle', self.buffer_size))
        self.groups['observer'] = observer
        
        # Modulator group
        modulator = SignalGroup('modulator', 'Grid-Side Modulator')
        modulator.add_signal(SignalHistory('firing_angle_rad', 'rad', 'Firing Angle', self.buffer_size))
        modulator.add_signal(SignalHistory('is_enabled', '', 'Is Enabled', self.buffer_size))
        modulator.add_signal(SignalHistory('force_commutation_ack', '', 'Force Commutation Ack', self.buffer_size))
        self.groups['modulator'] = modulator
        
        # Controller group
        controller = SignalGroup('controller', 'Current Controller')
        controller.add_signal(SignalHistory('dc_link_current_reference_pu', 'pu', 'DC Current Reference', self.buffer_size))
        controller.add_signal(SignalHistory('dc_link_current_measured_pu', 'pu', 'DC Current Measured', self.buffer_size))
        controller.add_signal(SignalHistory('current_error_pu', 'pu', 'Current Error', self.buffer_size))
        controller.add_signal(SignalHistory('controller_output_rad', 'rad', 'Controller Output', self.buffer_size))
        controller.add_signal(SignalHistory('operating_mode', '', 'Operating Mode', self.buffer_size))
        controller.add_signal(SignalHistory('controller_saturated', '', 'Controller Saturated', self.buffer_size))
        controller.add_signal(SignalHistory('freeze_integrator', '', 'Freeze Integrator', self.buffer_size))
        self.groups['controller'] = controller
        
        # State machine group
        state_machine = SignalGroup('state_machine', 'State Machine')
        state_machine.add_signal(SignalHistory('current_state_id', '', 'Current State', self.buffer_size))
        state_machine.add_signal(SignalHistory('pulse_releases', '', 'Pulse Releases', self.buffer_size))
        state_machine.add_signal(SignalHistory('auxiliaries_are_on', '', 'Auxiliaries Are On', self.buffer_size))
        state_machine.add_signal(SignalHistory('mcb_is_closed', '', 'Breakers Are On', self.buffer_size))
        state_machine.add_signal(SignalHistory('start_command', '', 'Start Command', self.buffer_size))
        state_machine.add_signal(SignalHistory('stop_command', '', 'Stop Command', self.buffer_size))
        state_machine.add_signal(SignalHistory('is_current_extinguished', '', 'Current Extinguished', self.buffer_size))
        state_machine.add_signal(SignalHistory('shutdown_request', '', 'Shutdown Requested', self.buffer_size))
        self.groups['state_machine'] = state_machine
        
        # Operation mode monitor group
        operation_mode = SignalGroup('operation_mode', 'Operation Mode Monitor')
        operation_mode.add_signal(SignalHistory('current_mode', '', 'Current Mode', self.buffer_size))
        operation_mode.add_signal(SignalHistory('has_mode_changed', '', 'Mode Changed', self.buffer_size))
        operation_mode.add_signal(SignalHistory('is_current_zero', '', 'Current Is Zero', self.buffer_size))
        operation_mode.add_signal(SignalHistory('is_discontinuous_mode', '', 'Is Discontinuous Mode', self.buffer_size))
        self.groups['operation_mode'] = operation_mode
        
        # Pulse generator group
        pulse_generator = SignalGroup('pulse_generator', 'Pulse Generator')
        pulse_generator.add_signal(SignalHistory('pulse_p1', '', 'Pulse P1 (T1)', self.buffer_size))
        pulse_generator.add_signal(SignalHistory('pulse_p2', '', 'Pulse P2 (T2)', self.buffer_size))
        pulse_generator.add_signal(SignalHistory('pulse_p3', '', 'Pulse P3 (T3)', self.buffer_size))
        pulse_generator.add_signal(SignalHistory('pulse_p4', '', 'Pulse P4 (T4)', self.buffer_size))
        pulse_generator.add_signal(SignalHistory('pulse_p5', '', 'Pulse P5 (T5)', self.buffer_size))
        pulse_generator.add_signal(SignalHistory('pulse_p6', '', 'Pulse P6 (T6)', self.buffer_size))
        pulse_generator.add_signal(SignalHistory('pulse_active', '', 'Pulse Active', self.buffer_size))
        self.groups['pulse_generator'] = pulse_generator

        # Protections group
        protections = SignalGroup('protections', 'Grid-Side Protections')
        protections.add_signal(SignalHistory('is_tripped', '', 'Is Tripped', self.buffer_size))
        protections.add_signal(SignalHistory('is_inhibited', '', 'Is Inhibited', self.buffer_size))
        protections.add_signal(SignalHistory('any_fault_active', '', 'Any Fault Active', self.buffer_size))
        protections.add_signal(SignalHistory('overcurrent_tripped', '', 'Overcurrent Tripped', self.buffer_size))
        protections.add_signal(SignalHistory('overvoltage_tripped', '', 'Overvoltage Tripped', self.buffer_size))
        protections.add_signal(SignalHistory('undervoltage_tripped', '', 'Undervoltage Tripped', self.buffer_size))
        protections.add_signal(SignalHistory('overcurrent_warning', '', 'Overcurrent Warning', self.buffer_size))
        protections.add_signal(SignalHistory('overvoltage_warning', '', 'Overvoltage Warning', self.buffer_size))
        protections.add_signal(SignalHistory('undervoltage_warning', '', 'Undervoltage Warning', self.buffer_size))
        protections.add_signal(SignalHistory('communication_loss_tripped', '', 'Comm Loss Tripped', self.buffer_size))
        protections.add_signal(SignalHistory('communication_loss_warning', '', 'Comm Loss Warning', self.buffer_size))
        self.groups['protections'] = protections

        # Fiber Optic RX group
        fiber_rx = SignalGroup('fiber_rx', 'Fiber Optic RX (from Machine)')
        fiber_rx.add_signal(SignalHistory('measured_speed_pu', 'pu', 'Machine Speed', self.buffer_size))
        fiber_rx.add_signal(SignalHistory('dc_link_current_reference_pu', 'pu', 'DC Current Ref', self.buffer_size))
        fiber_rx.add_signal(SignalHistory('machine_voltage_magnitude_pu', 'pu', 'Machine Voltage Mag', self.buffer_size))
        fiber_rx.add_signal(SignalHistory('force_commutation_request', '', 'Force Comm Request', self.buffer_size))
        fiber_rx.add_signal(SignalHistory('shutdown_request', '', 'Shutdown Request', self.buffer_size))
        fiber_rx.add_signal(SignalHistory('machine_protection_tripped', '', 'Machine Tripped', self.buffer_size))
        fiber_rx.add_signal(SignalHistory('protection_inhibit', '', 'Protection Inhibit', self.buffer_size))
        self.groups['fiber_rx'] = fiber_rx

        # Hardware debugging group
        hardware_debugging = SignalGroup('hardware_debugging', 'Hardware Debugging')
        hardware_debugging.add_signal(SignalHistory('v_ab_counts_peak', 'counts', 'Vab Peak Counts', self.buffer_size))
        hardware_debugging.add_signal(SignalHistory('v_bc_counts_peak', 'counts', 'Vbc Peak Counts', self.buffer_size))
        hardware_debugging.add_signal(SignalHistory('i_a_counts_peak', 'counts', 'Ia Peak Counts', self.buffer_size))
        hardware_debugging.add_signal(SignalHistory('i_c_counts_peak', 'counts', 'Ic Peak Counts', self.buffer_size))
        self.groups['hardware_debugging'] = hardware_debugging
    
    def _initialize_machine_side_signal_groups(self) -> None:
        """Initialize all machine-side signal groups with their signals.
        
        Signal indices match C++ machine_side_telemetry_builder.cpp exactly.
        All 52 signals across 8 groups (indices 0-51).
        """
        
        # Machine Measurements group
        measurements = SignalGroup('measurements', 'Machine Measurements & Commutation')
        measurements.add_signal(SignalHistory('machine_voltage_magnitude_filtered_pu', 'pu', 'Machine Voltage Mag (Filtered)', self.buffer_size))
        measurements.add_signal(SignalHistory('machine_voltage_a_pu', 'pu', 'Machine Voltage Phase A', self.buffer_size))
        measurements.add_signal(SignalHistory('machine_voltage_b_pu', 'pu', 'Machine Voltage Phase B', self.buffer_size))
        measurements.add_signal(SignalHistory('machine_voltage_c_pu', 'pu', 'Machine Voltage Phase C', self.buffer_size))
        measurements.add_signal(SignalHistory('machine_voltage_magnitude', 'pu', 'Machine Voltage Magnitude', self.buffer_size))
        self.groups['measurements'] = measurements
        
        # Observer group
        observer = SignalGroup('observer', 'Machine Observer')
        observer.add_signal(SignalHistory('estimated_frequency_pu', 'pu', 'Est. Frequency (pu)', self.buffer_size))
        observer.add_signal(SignalHistory('estimated_position_pu', 'pu', 'Est. Position (pu)', self.buffer_size))
        self.groups['observer'] = observer
        
        # Modulator group
        modulator = SignalGroup('modulator', 'Machine-Side Modulator')
        modulator.add_signal(SignalHistory('firing_angle_rad', 'rad', 'Firing Angle', self.buffer_size))
        modulator.add_signal(SignalHistory('is_enabled', '', 'Is Enabled', self.buffer_size))
        modulator.add_signal(SignalHistory('release_pulses', '', 'Release Pulses', self.buffer_size))
        modulator.add_signal(SignalHistory('request_force_commutation', '', 'Request Force Commutation', self.buffer_size))
        modulator.add_signal(SignalHistory('current_rotor_sector', '', 'Current Rotor Sector', self.buffer_size))
        self.groups['modulator'] = modulator
        
        # Machine-Side Controller group
        machine_controller = SignalGroup('machine_controller', 'Machine-Side Controller')
        machine_controller.add_signal(SignalHistory('firing_angle', 'rad', 'Firing Angle', self.buffer_size))
        machine_controller.add_signal(SignalHistory('dc_link_current', 'pu', 'DC Link Current', self.buffer_size))
        machine_controller.add_signal(SignalHistory('machine_speed', 'pu', 'Machine Speed', self.buffer_size))
        self.groups['machine_controller'] = machine_controller
        
        # Torque Controller group
        torque_controller = SignalGroup('torque_controller', 'Torque Controller')
        torque_controller.add_signal(SignalHistory('current_reference_pu', 'pu', 'Current Reference', self.buffer_size))
        torque_controller.add_signal(SignalHistory('is_saturated', '', 'Is Saturated', self.buffer_size))
        self.groups['torque_controller'] = torque_controller
        
        # Speed Controller group
        speed_controller = SignalGroup('speed_controller', 'Speed Controller')
        speed_controller.add_signal(SignalHistory('torque_reference', 'pu', 'Torque Reference', self.buffer_size))
        speed_controller.add_signal(SignalHistory('measured_speed_pu', 'pu', 'Measured Speed', self.buffer_size))
        speed_controller.add_signal(SignalHistory('pi_output', 'pu', 'PI Output', self.buffer_size))
        speed_controller.add_signal(SignalHistory('is_saturated', '', 'Is Saturated', self.buffer_size))
        self.groups['speed_controller'] = speed_controller
        
        # Startup Sequencer group
        startup_sequencer = SignalGroup('startup_sequencer', 'Startup Sequencer')
        startup_sequencer.add_signal(SignalHistory('speed_reference_pu', 'pu', 'Speed Reference', self.buffer_size))
        startup_sequencer.add_signal(SignalHistory('virtual_rotor_frequency_pu', 'pu', 'Virtual Rotor Freq', self.buffer_size))
        startup_sequencer.add_signal(SignalHistory('virtual_rotor_angle_pu', 'pu', 'Virtual Rotor Angle', self.buffer_size))
        startup_sequencer.add_signal(SignalHistory('virtual_torque_reference_pu', 'pu', 'Virtual Torque Ref', self.buffer_size))
        startup_sequencer.add_signal(SignalHistory('release_speed_controller', '', 'Rel. Speed Ctrl', self.buffer_size))
        startup_sequencer.add_signal(SignalHistory('release_excitation', '', 'Rel. Excitation', self.buffer_size))
        startup_sequencer.add_signal(SignalHistory('release_torque_controller', '', 'Rel. Torque Ctrl', self.buffer_size))
        startup_sequencer.add_signal(SignalHistory('release_machine_side_pulses', '', 'Rel. Machine Pulses', self.buffer_size))
        startup_sequencer.add_signal(SignalHistory('is_startup_complete', '', 'Startup Complete', self.buffer_size))
        self.groups['startup_sequencer'] = startup_sequencer
        
        # Excitation Controller group
        excitation_controller = SignalGroup('excitation_controller', 'Excitation Controller')
        excitation_controller.add_signal(SignalHistory('field_current_reference_pu', 'pu', 'Field Current Ref', self.buffer_size))
        excitation_controller.add_signal(SignalHistory('prefluxing_elapsed_time_s', 's', 'Prefluxing Time', self.buffer_size))
        self.groups['excitation_controller'] = excitation_controller
        
        # Pulse generator group
        pulse_generator = SignalGroup('pulse_generator', 'Pulse Generator')
        pulse_generator.add_signal(SignalHistory('pulse_p1', '', 'Pulse P1 (T1)', self.buffer_size))
        pulse_generator.add_signal(SignalHistory('pulse_p2', '', 'Pulse P2 (T2)', self.buffer_size))
        pulse_generator.add_signal(SignalHistory('pulse_p3', '', 'Pulse P3 (T3)', self.buffer_size))
        pulse_generator.add_signal(SignalHistory('pulse_p4', '', 'Pulse P4 (T4)', self.buffer_size))
        pulse_generator.add_signal(SignalHistory('pulse_p5', '', 'Pulse P5 (T5)', self.buffer_size))
        self.groups['pulse_generator'] = pulse_generator

        # Protections group
        protections = SignalGroup('protections', 'Machine-Side Protections')
        protections.add_signal(SignalHistory('is_tripped', '', 'Is Tripped', self.buffer_size))
        protections.add_signal(SignalHistory('is_inhibited', '', 'Is Inhibited', self.buffer_size))
        protections.add_signal(SignalHistory('any_fault_active', '', 'Any Fault Active', self.buffer_size))
        protections.add_signal(SignalHistory('over_flux_tripped', '', 'Over Flux Tripped', self.buffer_size))
        protections.add_signal(SignalHistory('under_flux_tripped', '', 'Under Flux Tripped', self.buffer_size))
        protections.add_signal(SignalHistory('stall_tripped', '', 'Stall Tripped', self.buffer_size))
        protections.add_signal(SignalHistory('over_flux_warning', '', 'Over Flux Warning', self.buffer_size))
        protections.add_signal(SignalHistory('under_flux_warning', '', 'Under Flux Warning', self.buffer_size))
        protections.add_signal(SignalHistory('stall_warning', '', 'Stall Warning', self.buffer_size))
        protections.add_signal(SignalHistory('communication_loss_tripped', '', 'Comm Loss Tripped', self.buffer_size))
        protections.add_signal(SignalHistory('communication_loss_warning', '', 'Comm Loss Warning', self.buffer_size))
        self.groups['protections'] = protections

        # Fiber Optic RX group
        fiber_rx = SignalGroup('fiber_rx', 'Fiber Optic RX (from Grid)')
        fiber_rx.add_signal(SignalHistory('dc_link_current_pu', 'pu', 'DC Link Current', self.buffer_size))
        fiber_rx.add_signal(SignalHistory('grid_voltage_magnitude_pu', 'pu', 'Grid Voltage Mag', self.buffer_size))
        fiber_rx.add_signal(SignalHistory('current_state_id', '', 'Grid State ID', self.buffer_size))
        fiber_rx.add_signal(SignalHistory('is_forced_commutation', '', 'Is Forced Comm', self.buffer_size))
        fiber_rx.add_signal(SignalHistory('force_commutation_acknowledged', '', 'Force Comm Ack', self.buffer_size))
        fiber_rx.add_signal(SignalHistory('release_pulses', '', 'Release Pulses', self.buffer_size))
        fiber_rx.add_signal(SignalHistory('grid_release_protections', '', 'Grid Rel Protections', self.buffer_size))
        self.groups['fiber_rx'] = fiber_rx
    
    def update_from_packet(self, packet: Union[GridSideTelemetryPacket, MachineSideTelemetryPacket]) -> None:
        """Update all signal histories from telemetry packet (grid or machine side).
        
        Auto-detects packet type and updates signals accordingly.
        If side changes, automatically reinitializes signal groups.
        
        Args:
            packet: Decoded telemetry packet (grid or machine side)
        """
        # Auto-detect side change
        if isinstance(packet, GridSideTelemetryPacket):
            self.switch_to_grid_side()
            self._update_from_grid_packet(packet)
        elif isinstance(packet, MachineSideTelemetryPacket):
            self.switch_to_machine_side()
            self._update_from_machine_packet(packet)
    
    def _update_from_grid_packet(self, packet: GridSideTelemetryPacket) -> None:
        """Update signal histories from grid-side telemetry packet.
        
        Args:
            packet: Grid-side telemetry packet
        """
        timestamp_ms = packet.header.timestamp_ms
        sequence = packet.header.sequence_number
        
        # Update measurements
        meas = packet.measurements
        self._append_signal('measurements', 'grid_voltage_magnitude_filtered_pu', meas.grid_voltage_magnitude_pu, timestamp_ms, sequence)
        self._append_signal('measurements', 'grid_voltage_a_pu', meas.grid_voltage_a_pu, timestamp_ms, sequence)
        self._append_signal('measurements', 'grid_voltage_b_pu', meas.grid_voltage_b_pu, timestamp_ms, sequence)
        self._append_signal('measurements', 'grid_voltage_c_pu', meas.grid_voltage_c_pu, timestamp_ms, sequence)
        self._append_signal('measurements', 'grid_current_magnitude_pu', meas.grid_current_magnitude_pu, timestamp_ms, sequence)
        self._append_signal('measurements', 'grid_current_a_pu', meas.grid_current_a_pu, timestamp_ms, sequence)
        self._append_signal('measurements', 'grid_current_b_pu', meas.grid_current_b_pu, timestamp_ms, sequence)
        self._append_signal('measurements', 'grid_current_c_pu', meas.grid_current_c_pu, timestamp_ms, sequence)
        self._append_signal('measurements', 'dc_link_current_pu', meas.dc_link_current_pu, timestamp_ms, sequence)
        
        # Update observer
        obs = packet.observer
        self._append_signal('observer', 'estimated_voltage_magnitude_pu', obs.estimated_voltage_magnitude_pu, timestamp_ms, sequence)
        self._append_signal('observer', 'estimated_frequency_pu', obs.estimated_frequency_pu, timestamp_ms, sequence)
        self._append_signal('observer', 'estimated_angle_rad', obs.estimated_reference_angle_pu * 2.0 * 3.14159, timestamp_ms, sequence)
        
        # Update modulator
        mod = packet.modulator
        self._append_signal('modulator', 'firing_angle_rad', mod.firing_angle_output_pu * 3.14159, timestamp_ms, sequence)
        self._append_signal('modulator', 'is_enabled', float(mod.release_pulses_output), timestamp_ms, sequence)
        self._append_signal('modulator', 'force_commutation_ack', float(mod.request_force_commutation), timestamp_ms, sequence)
        
        # Update controller
        ctrl = packet.controller
        self._append_signal('controller', 'dc_link_current_reference_pu', ctrl.current_reference, timestamp_ms, sequence)
        self._append_signal('controller', 'dc_link_current_measured_pu', ctrl.current_measured, timestamp_ms, sequence)
        self._append_signal('controller', 'current_error_pu', ctrl.current_error_pu, timestamp_ms, sequence)
        self._append_signal('controller', 'controller_output_rad', ctrl.pi_output_pu * 3.14159, timestamp_ms, sequence)
        self._append_signal('controller', 'operating_mode', float(mod.operating_mode), timestamp_ms, sequence)
        self._append_signal('controller', 'controller_saturated', float(ctrl.is_saturated), timestamp_ms, sequence)
        self._append_signal('controller', 'freeze_integrator', float(ctrl.freeze_integrator), timestamp_ms, sequence)
        
        # Update state machine
        sm = packet.state_machine
        self._append_signal('state_machine', 'current_state_id', float(sm.current_state), timestamp_ms, sequence)
        self._append_signal('state_machine', 'pulse_releases', float(sm.release_pulses), timestamp_ms, sequence)
        self._append_signal('state_machine', 'auxiliaries_are_on', float(sm.auxiliaries_are_on), timestamp_ms, sequence)
        self._append_signal('state_machine', 'mcb_is_closed', float(sm.mcb_is_closed), timestamp_ms, sequence)
        self._append_signal('state_machine', 'start_command', float(sm.start_command), timestamp_ms, sequence)
        self._append_signal('state_machine', 'stop_command', float(sm.stop_command), timestamp_ms, sequence)
        self._append_signal('state_machine', 'is_current_extinguished', float(sm.is_current_extinguished), timestamp_ms, sequence)
        self._append_signal('state_machine', 'shutdown_request', float(sm.is_shutdown_requested), timestamp_ms, sequence)
        
        # Update operation mode monitor
        omm = packet.operation_mode_monitor
        self._append_signal('operation_mode', 'current_mode', float(omm.mode), timestamp_ms, sequence)
        self._append_signal('operation_mode', 'has_mode_changed', float(omm.mode_changed), timestamp_ms, sequence)
        self._append_signal('operation_mode', 'is_current_zero', float(omm.current_zero), timestamp_ms, sequence)
        self._append_signal('operation_mode', 'is_discontinuous_mode', float(omm.is_discontinuous_mode), timestamp_ms, sequence)
        
        # Update pulse generator
        pg = packet.pulse_generator
        self._append_signal('pulse_generator', 'pulse_p1', float(pg.p1), timestamp_ms, sequence)
        self._append_signal('pulse_generator', 'pulse_p2', float(pg.p2), timestamp_ms, sequence)
        self._append_signal('pulse_generator', 'pulse_p3', float(pg.p3), timestamp_ms, sequence)
        self._append_signal('pulse_generator', 'pulse_p4', float(pg.p4), timestamp_ms, sequence)
        self._append_signal('pulse_generator', 'pulse_p5', float(pg.p5), timestamp_ms, sequence)
        self._append_signal('pulse_generator', 'pulse_p6', float(pg.p6), timestamp_ms, sequence)
        self._append_signal('pulse_generator', 'pulse_active', float(pg.release_pulses_output), timestamp_ms, sequence)

        # Update protections
        prot = packet.protections
        self._append_signal('protections', 'is_tripped', float(prot.is_tripped), timestamp_ms, sequence)
        self._append_signal('protections', 'is_inhibited', float(prot.is_inhibited), timestamp_ms, sequence)
        self._append_signal('protections', 'any_fault_active', float(prot.any_fault_active), timestamp_ms, sequence)
        self._append_signal('protections', 'overcurrent_tripped', float(prot.overcurrent_tripped), timestamp_ms, sequence)
        self._append_signal('protections', 'overvoltage_tripped', float(prot.overvoltage_tripped), timestamp_ms, sequence)
        self._append_signal('protections', 'undervoltage_tripped', float(prot.undervoltage_tripped), timestamp_ms, sequence)
        self._append_signal('protections', 'overcurrent_warning', float(prot.overcurrent_warning), timestamp_ms, sequence)
        self._append_signal('protections', 'overvoltage_warning', float(prot.overvoltage_warning), timestamp_ms, sequence)
        self._append_signal('protections', 'undervoltage_warning', float(prot.undervoltage_warning), timestamp_ms, sequence)
        self._append_signal('protections', 'communication_loss_tripped', float(prot.communication_loss_tripped), timestamp_ms, sequence)
        self._append_signal('protections', 'communication_loss_warning', float(prot.communication_loss_warning), timestamp_ms, sequence)

        # Update fiber optic RX
        frx = packet.fiber_rx
        self._append_signal('fiber_rx', 'measured_speed_pu', frx.measured_speed_pu, timestamp_ms, sequence)
        self._append_signal('fiber_rx', 'dc_link_current_reference_pu', frx.dc_link_current_reference_pu, timestamp_ms, sequence)
        self._append_signal('fiber_rx', 'machine_voltage_magnitude_pu', frx.machine_voltage_magnitude_pu, timestamp_ms, sequence)
        self._append_signal('fiber_rx', 'force_commutation_request', float(frx.force_commutation_request), timestamp_ms, sequence)
        self._append_signal('fiber_rx', 'shutdown_request', float(frx.shutdown_request), timestamp_ms, sequence)
        self._append_signal('fiber_rx', 'machine_protection_tripped', float(frx.machine_protection_tripped), timestamp_ms, sequence)
        self._append_signal('fiber_rx', 'protection_inhibit', float(frx.protection_inhibit), timestamp_ms, sequence)

        # Update hardware debugging
        hw = packet.hardware_debug
        self._append_signal('hardware_debugging', 'v_ab_counts_peak', hw.voltage_ab_counts_peak, timestamp_ms, sequence)
        self._append_signal('hardware_debugging', 'v_bc_counts_peak', hw.voltage_bc_counts_peak, timestamp_ms, sequence)
        self._append_signal('hardware_debugging', 'i_a_counts_peak', hw.current_a_counts_peak, timestamp_ms, sequence)
        self._append_signal('hardware_debugging', 'i_c_counts_peak', hw.current_c_counts_peak, timestamp_ms, sequence)
    
    def _update_from_machine_packet(self, packet: MachineSideTelemetryPacket) -> None:
        """Update signal histories from machine-side telemetry packet.
        
        Args:
            packet: Machine-side telemetry packet
        """
        timestamp_ms = packet.header.timestamp_ms
        sequence = packet.header.sequence_number
        
        # Update measurements
        meas = packet.measurements
        self._append_signal('measurements', 'machine_voltage_magnitude_filtered_pu', meas.machine_voltage_magnitude_pu, timestamp_ms, sequence)
        self._append_signal('measurements', 'machine_voltage_a_pu', meas.machine_voltage_a_pu, timestamp_ms, sequence)
        self._append_signal('measurements', 'machine_voltage_b_pu', meas.machine_voltage_b_pu, timestamp_ms, sequence)
        self._append_signal('measurements', 'machine_voltage_c_pu', meas.machine_voltage_c_pu, timestamp_ms, sequence)
        self._append_signal('measurements', 'machine_voltage_magnitude', meas.estimated_voltage_magnitude_pu, timestamp_ms, sequence)
        
        # Update observer
        obs = packet.observer
        self._append_signal('observer', 'estimated_frequency_pu', obs.estimated_frequency_pu, timestamp_ms, sequence)
        self._append_signal('observer', 'estimated_position_pu', obs.estimated_reference_angle_pu, timestamp_ms, sequence)
        
        # Update modulator
        mod = packet.modulator
        self._append_signal('modulator', 'firing_angle_rad', mod.firing_angle_pu * 3.14159, timestamp_ms, sequence)
        self._append_signal('modulator', 'is_enabled', float(mod.release_pulses_output), timestamp_ms, sequence)
        self._append_signal('modulator', 'release_pulses', float(mod.release_pulses_output), timestamp_ms, sequence)
        self._append_signal('modulator', 'request_force_commutation', float(mod.request_force_commutation), timestamp_ms, sequence)
        self._append_signal('modulator', 'current_rotor_sector', mod.current_rotor_sector_normalized, timestamp_ms, sequence)
        
        # Update machine controller
        ctrl = packet.controller
        self._append_signal('machine_controller', 'firing_angle', ctrl.firing_angle_rad, timestamp_ms, sequence)
        self._append_signal('machine_controller', 'dc_link_current', ctrl.dc_link_current_pu, timestamp_ms, sequence)
        self._append_signal('machine_controller', 'machine_speed', ctrl.machine_speed_pu, timestamp_ms, sequence)
        
        # Update torque controller
        tc = packet.torque_controller
        self._append_signal('torque_controller', 'current_reference_pu', tc.current_reference_pu, timestamp_ms, sequence)
        self._append_signal('torque_controller', 'is_saturated', float(tc.is_saturated), timestamp_ms, sequence)
        
        # Update speed controller
        sc = packet.speed_controller
        self._append_signal('speed_controller', 'torque_reference', sc.torque_reference_pu, timestamp_ms, sequence)
        self._append_signal('speed_controller', 'measured_speed_pu', sc.measured_speed_pu, timestamp_ms, sequence)
        self._append_signal('speed_controller', 'pi_output', sc.pi_output_pu, timestamp_ms, sequence)
        self._append_signal('speed_controller', 'is_saturated', float(sc.is_saturated), timestamp_ms, sequence)
        
        # Update startup sequencer
        ss = packet.startup_sequencer
        self._append_signal('startup_sequencer', 'speed_reference_pu', ss.speed_reference_pu, timestamp_ms, sequence)
        self._append_signal('startup_sequencer', 'virtual_rotor_frequency_pu', ss.virtual_rotor_frequency_pu, timestamp_ms, sequence)
        self._append_signal('startup_sequencer', 'virtual_rotor_angle_pu', ss.virtual_rotor_angle_pu, timestamp_ms, sequence)
        self._append_signal('startup_sequencer', 'virtual_torque_reference_pu', ss.virtual_torque_reference_pu, timestamp_ms, sequence)
        self._append_signal('startup_sequencer', 'release_speed_controller', float(ss.release_speed_controller), timestamp_ms, sequence)
        self._append_signal('startup_sequencer', 'release_excitation', float(ss.release_excitation), timestamp_ms, sequence)
        self._append_signal('startup_sequencer', 'release_torque_controller', float(ss.release_torque_controller), timestamp_ms, sequence)
        self._append_signal('startup_sequencer', 'release_machine_side_pulses', float(ss.release_machine_side_pulses), timestamp_ms, sequence)
        self._append_signal('startup_sequencer', 'is_startup_complete', float(ss.is_startup_complete), timestamp_ms, sequence)
        
        # Update excitation controller
        ec = packet.excitation_controller
        self._append_signal('excitation_controller', 'field_current_reference_pu', ec.field_current_reference_pu, timestamp_ms, sequence)
        self._append_signal('excitation_controller', 'prefluxing_elapsed_time_s', ec.prefluxing_elapsed_time_s, timestamp_ms, sequence)
        
        # Update pulse generator
        pg = packet.pulse_generator
        self._append_signal('pulse_generator', 'pulse_p1', float(pg.p1), timestamp_ms, sequence)
        self._append_signal('pulse_generator', 'pulse_p2', float(pg.p2), timestamp_ms, sequence)
        self._append_signal('pulse_generator', 'pulse_p3', float(pg.p3), timestamp_ms, sequence)
        self._append_signal('pulse_generator', 'pulse_p4', float(pg.p4), timestamp_ms, sequence)
        self._append_signal('pulse_generator', 'pulse_p5', float(pg.p5), timestamp_ms, sequence)

        # Update protections
        prot = packet.protections
        self._append_signal('protections', 'is_tripped', float(prot.is_tripped), timestamp_ms, sequence)
        self._append_signal('protections', 'is_inhibited', float(prot.is_inhibited), timestamp_ms, sequence)
        self._append_signal('protections', 'any_fault_active', float(prot.any_fault_active), timestamp_ms, sequence)
        self._append_signal('protections', 'over_flux_tripped', float(prot.over_flux_tripped), timestamp_ms, sequence)
        self._append_signal('protections', 'under_flux_tripped', float(prot.under_flux_tripped), timestamp_ms, sequence)
        self._append_signal('protections', 'stall_tripped', float(prot.stall_tripped), timestamp_ms, sequence)
        self._append_signal('protections', 'over_flux_warning', float(prot.over_flux_warning), timestamp_ms, sequence)
        self._append_signal('protections', 'under_flux_warning', float(prot.under_flux_warning), timestamp_ms, sequence)
        self._append_signal('protections', 'stall_warning', float(prot.stall_warning), timestamp_ms, sequence)
        self._append_signal('protections', 'communication_loss_tripped', float(prot.communication_loss_tripped), timestamp_ms, sequence)
        self._append_signal('protections', 'communication_loss_warning', float(prot.communication_loss_warning), timestamp_ms, sequence)
        self._append_signal('protections', 'over_flux', packet.signals[MachineSignalIndex.PROTECTION_OVER_FLUX], timestamp_ms, sequence)
        self._append_signal('protections', 'under_flux', packet.signals[MachineSignalIndex.PROTECTION_UNDER_FLUX], timestamp_ms, sequence)
        self._append_signal('protections', 'stall', packet.signals[MachineSignalIndex.PROTECTION_STALL], timestamp_ms, sequence)
        self._append_signal('protections', 'communication_loss', packet.signals[MachineSignalIndex.PROTECTION_COMM_LOSS], timestamp_ms, sequence)
        self._append_signal('protections', 'overspeed', packet.signals[MachineSignalIndex.PROTECTION_OVERSPEED], timestamp_ms, sequence)
        self._append_signal('protections', 'machine_overvoltage', packet.signals[MachineSignalIndex.PROTECTION_OVERVOLTAGE], timestamp_ms, sequence)

        # Update fiber optic RX
        frx = packet.fiber_optic
        self._append_signal('fiber_rx', 'dc_link_current_pu', frx.dc_link_current_pu, timestamp_ms, sequence)
        self._append_signal('fiber_rx', 'grid_voltage_magnitude_pu', frx.grid_voltage_magnitude_pu, timestamp_ms, sequence)
        self._append_signal('fiber_rx', 'current_state_id', float(frx.current_state_id), timestamp_ms, sequence)
        self._append_signal('fiber_rx', 'is_forced_commutation', float(frx.is_forced_commutation), timestamp_ms, sequence)
        self._append_signal('fiber_rx', 'force_commutation_acknowledged', float(frx.force_commutation_acknowledged), timestamp_ms, sequence)
        self._append_signal('fiber_rx', 'release_pulses', float(frx.release_pulses), timestamp_ms, sequence)
        self._append_signal('fiber_rx', 'grid_release_protections', float(frx.grid_release_protections), timestamp_ms, sequence)

    def _append_signal(self, group_name: str, signal_name: str, value: float, timestamp_ms: int, sequence: int) -> None:
        """Append value to signal history (internal helper).
        
        Args:
            group_name: Signal group name
            signal_name: Signal name within group
            value: Signal value
            timestamp_ms: Timestamp in milliseconds
            sequence: Packet sequence number
        """
        group = self.groups.get(group_name)
        if group:
            signal = group.get_signal(signal_name)
            if signal:
                signal.append(SignalValue(timestamp_ms, value, sequence))
    
    def get_latest_value(self, group_name: str, signal_name: str) -> Optional[float]:
        """Get latest value for specific signal.
        
        Args:
            group_name: Signal group name
            signal_name: Signal name within group
            
        Returns:
            Latest signal value or None if not found/empty
        """
        group = self.groups.get(group_name)
        if group:
            signal = group.get_signal(signal_name)
            if signal:
                latest = signal.get_latest()
                return latest.value if latest else None
        return None
    
    def get_time_series(self, group_name: str, signal_name: str) -> Tuple[List[float], List[float]]:
        """Get time series data for specific signal.
        
        Args:
            group_name: Signal group name
            signal_name: Signal name within group
            
        Returns:
            Tuple of (timestamps_ms, values) lists
        """
        group = self.groups.get(group_name)
        if group:
            signal = group.get_signal(signal_name)
            if signal:
                return signal.get_time_series()
        return [], []
    
    def get_group(self, group_name: str) -> Optional[SignalGroup]:
        """Get signal group by name.
        
        Args:
            group_name: Group name
            
        Returns:
            SignalGroup or None if not found
        """
        return self.groups.get(group_name)
    
    def get_all_groups(self) -> List[SignalGroup]:
        """Get all signal groups.
        
        Returns:
            List of all SignalGroup objects
        """
        return list(self.groups.values())
    
    def clear_all(self) -> None:
        """Clear all signal histories."""
        with self._lock:
            for group in self.groups.values():
                group.clear_all()
    
    def get_statistics(self) -> Dict[str, int]:
        """Get statistics about signal storage.
        
        Returns:
            Dictionary with statistics (total_signals, buffer_size, etc.)
        """
        total_signals = sum(len(group.signals) for group in self.groups.values())
        total_samples = sum(
            len(signal.values)
            for group in self.groups.values()
            for signal in group.signals.values()
        )
        
        return {
            'total_groups': len(self.groups),
            'total_signals': total_signals,
            'buffer_size': self.buffer_size,
            'total_samples': total_samples
        }
