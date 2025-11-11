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
    
    Example:
        manager = SignalManager(buffer_size=10000)
        
        # Update from packet
        manager.update_from_packet(packet)
        
        # Get latest value
        voltage = manager.get_latest_value('measurements', 'grid_voltage_magnitude_filtered_pu')
        
        # Get time series for plotting
        times, values = manager.get_time_series('measurements', 'dc_link_current_pu')
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
        All 50 signals across 7 groups (indices 0-49).
        """
        
        # Measurements group (indices 0-13)
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
        measurements.add_signal(SignalHistory('is_commuting', '', 'Is Commuting', self.buffer_size))
        measurements.add_signal(SignalHistory('grid_voltage_alpha_pu', 'pu', 'Grid Voltage Alpha', self.buffer_size))
        measurements.add_signal(SignalHistory('grid_voltage_beta_pu', 'pu', 'Grid Voltage Beta', self.buffer_size))
        measurements.add_signal(SignalHistory('grid_voltage_magnitude_filtered_pu_2', 'pu', 'Grid Voltage Mag (Filtered) #2', self.buffer_size))
        measurements.add_signal(SignalHistory('grid_side_frequency_pu', 'pu', 'Grid Side Frequency', self.buffer_size))
        self.groups['measurements'] = measurements
        
        # Observer group (indices 14-19)
        observer = SignalGroup('observer', 'Grid Observer')
        observer.add_signal(SignalHistory('estimated_voltage_magnitude_pu', 'pu', 'Est. Voltage Magnitude', self.buffer_size))
        observer.add_signal(SignalHistory('estimated_frequency_pu', 'pu', 'Est. Frequency (pu)', self.buffer_size))
        observer.add_signal(SignalHistory('estimated_angle_rad', 'rad', 'Est. Angle', self.buffer_size))
        observer.add_signal(SignalHistory('estimated_frequency_hz', 'Hz', 'Est. Frequency (Hz)', self.buffer_size))
        observer.add_signal(SignalHistory('estimated_voltage_alpha_pu', 'pu', 'Est. Voltage Alpha', self.buffer_size))
        observer.add_signal(SignalHistory('estimated_voltage_beta_pu', 'pu', 'Est. Voltage Beta', self.buffer_size))
        self.groups['observer'] = observer
        
        # Modulator group (indices 20-23)
        modulator = SignalGroup('modulator', 'Grid-Side Modulator')
        modulator.add_signal(SignalHistory('firing_angle_rad', 'rad', 'Firing Angle', self.buffer_size))
        modulator.add_signal(SignalHistory('is_enabled', '', 'Is Enabled', self.buffer_size))
        modulator.add_signal(SignalHistory('force_commutation_ack', '', 'Force Commutation Ack', self.buffer_size))
        modulator.add_signal(SignalHistory('is_requesting_pi_reset', '', 'Is Requesting PI Reset', self.buffer_size))
        self.groups['modulator'] = modulator
        
        # Controller group (indices 24-30)
        controller = SignalGroup('controller', 'Current Controller')
        controller.add_signal(SignalHistory('dc_link_current_reference_pu', 'pu', 'DC Current Reference', self.buffer_size))
        controller.add_signal(SignalHistory('dc_link_current_measured_pu', 'pu', 'DC Current Measured', self.buffer_size))
        controller.add_signal(SignalHistory('current_error_pu', 'pu', 'Current Error', self.buffer_size))
        controller.add_signal(SignalHistory('controller_output_rad', 'rad', 'Controller Output', self.buffer_size))
        controller.add_signal(SignalHistory('operating_mode', '', 'Operating Mode', self.buffer_size))
        controller.add_signal(SignalHistory('controller_saturated', '', 'Controller Saturated', self.buffer_size))
        controller.add_signal(SignalHistory('freeze_integrator', '', 'Freeze Integrator', self.buffer_size))
        self.groups['controller'] = controller
        
        # State machine group (indices 31-38)
        state_machine = SignalGroup('state_machine', 'State Machine')
        state_machine.add_signal(SignalHistory('current_state_id', '', 'Current State', self.buffer_size))
        state_machine.add_signal(SignalHistory('pulse_releases', '', 'Pulse Releases', self.buffer_size))
        state_machine.add_signal(SignalHistory('auxiliaries_are_on', '', 'Auxiliaries Are On', self.buffer_size))
        state_machine.add_signal(SignalHistory('breakers_are_on', '', 'Breakers Are On', self.buffer_size))
        state_machine.add_signal(SignalHistory('start_command', '', 'Start Command', self.buffer_size))
        state_machine.add_signal(SignalHistory('stop_command', '', 'Stop Command', self.buffer_size))
        state_machine.add_signal(SignalHistory('is_current_extinguished', '', 'Current Extinguished', self.buffer_size))
        state_machine.add_signal(SignalHistory('shutdown_request', '', 'Shutdown Requested', self.buffer_size))
        self.groups['state_machine'] = state_machine
        
        # Operation mode monitor group (indices 39-42)
        operation_mode = SignalGroup('operation_mode', 'Operation Mode Monitor')
        operation_mode.add_signal(SignalHistory('current_mode', '', 'Current Mode', self.buffer_size))
        operation_mode.add_signal(SignalHistory('has_mode_changed', '', 'Mode Changed', self.buffer_size))
        operation_mode.add_signal(SignalHistory('is_current_zero', '', 'Current Is Zero', self.buffer_size))
        operation_mode.add_signal(SignalHistory('is_discontinuous_mode', '', 'Is Discontinuous Mode', self.buffer_size))
        self.groups['operation_mode'] = operation_mode
        
        # Pulse generator group (indices 43-49)
        pulse_generator = SignalGroup('pulse_generator', 'Pulse Generator')
        pulse_generator.add_signal(SignalHistory('pulse_p1', '', 'Pulse P1 (T1)', self.buffer_size))
        pulse_generator.add_signal(SignalHistory('pulse_p2', '', 'Pulse P2 (T2)', self.buffer_size))
        pulse_generator.add_signal(SignalHistory('pulse_p3', '', 'Pulse P3 (T3)', self.buffer_size))
        pulse_generator.add_signal(SignalHistory('pulse_p4', '', 'Pulse P4 (T4)', self.buffer_size))
        pulse_generator.add_signal(SignalHistory('pulse_p5', '', 'Pulse P5 (T5)', self.buffer_size))
        pulse_generator.add_signal(SignalHistory('pulse_p6', '', 'Pulse P6 (T6)', self.buffer_size))
        pulse_generator.add_signal(SignalHistory('pulse_active', '', 'Pulse Active', self.buffer_size))
        self.groups['pulse_generator'] = pulse_generator
    
    def _initialize_machine_side_signal_groups(self) -> None:
        """Initialize all machine-side signal groups with their signals.
        
        Signal indices match C++ machine_side_telemetry_builder.cpp exactly.
        All 50 signals across 8 groups (indices 0-49).
        """
        
        # Machine Measurements group (indices 0-8)
        measurements = SignalGroup('measurements', 'Machine Measurements & Commutation')
        measurements.add_signal(SignalHistory('machine_voltage_magnitude_filtered_pu', 'pu', 'Machine Voltage Mag (Filtered)', self.buffer_size))
        measurements.add_signal(SignalHistory('machine_voltage_a_pu', 'pu', 'Machine Voltage Phase A', self.buffer_size))
        measurements.add_signal(SignalHistory('machine_voltage_b_pu', 'pu', 'Machine Voltage Phase B', self.buffer_size))
        measurements.add_signal(SignalHistory('machine_voltage_c_pu', 'pu', 'Machine Voltage Phase C', self.buffer_size))
        measurements.add_signal(SignalHistory('machine_voltage_magnitude', 'pu', 'Machine Voltage Magnitude', self.buffer_size))
        measurements.add_signal(SignalHistory('is_commuting', '', 'Is Commuting', self.buffer_size))
        self.groups['measurements'] = measurements
        
        # Observer group (indices 9-12)
        observer = SignalGroup('observer', 'Machine Observer')
        observer.add_signal(SignalHistory('estimated_frequency_pu', 'pu', 'Est. Frequency (pu)', self.buffer_size))
        observer.add_signal(SignalHistory('estimated_position_pu', 'pu', 'Est. Position (pu)', self.buffer_size))
        observer.add_signal(SignalHistory('filtered_voltage_alpha_pu', 'pu', 'Filtered Voltage Alpha', self.buffer_size))
        observer.add_signal(SignalHistory('filtered_voltage_beta_pu', 'pu', 'Filtered Voltage Beta', self.buffer_size))
        self.groups['observer'] = observer
        
        # Modulator group (indices 13-17)
        modulator = SignalGroup('modulator', 'Machine-Side Modulator')
        modulator.add_signal(SignalHistory('firing_angle_rad', 'rad', 'Firing Angle', self.buffer_size))
        modulator.add_signal(SignalHistory('is_enabled', '', 'Is Enabled', self.buffer_size))
        modulator.add_signal(SignalHistory('release_pulses', '', 'Release Pulses', self.buffer_size))
        modulator.add_signal(SignalHistory('request_force_commutation', '', 'Request Force Commutation', self.buffer_size))
        modulator.add_signal(SignalHistory('current_rotor_sector', '', 'Current Rotor Sector', self.buffer_size))
        self.groups['modulator'] = modulator
        
        # Machine-Side Controller group (indices 18-20)
        machine_controller = SignalGroup('machine_controller', 'Machine-Side Controller')
        machine_controller.add_signal(SignalHistory('firing_angle', 'rad', 'Firing Angle', self.buffer_size))
        machine_controller.add_signal(SignalHistory('dc_link_current', 'pu', 'DC Link Current', self.buffer_size))
        machine_controller.add_signal(SignalHistory('machine_speed', 'pu', 'Machine Speed', self.buffer_size))
        self.groups['machine_controller'] = machine_controller
        
        # Torque Controller group (indices 21-33)
        torque_controller = SignalGroup('torque_controller', 'Torque Controller')
        torque_controller.add_signal(SignalHistory('current_reference_pu', 'pu', 'Current Reference', self.buffer_size))
        torque_controller.add_signal(SignalHistory('base_torque_reference', 'pu', 'Base Torque Reference', self.buffer_size))
        torque_controller.add_signal(SignalHistory('total_torque_reference', 'pu', 'Total Torque Reference', self.buffer_size))
        torque_controller.add_signal(SignalHistory('limited_torque_reference', 'pu', 'Limited Torque Reference', self.buffer_size))
        torque_controller.add_signal(SignalHistory('is_saturated', '', 'Is Saturated', self.buffer_size))
        self.groups['torque_controller'] = torque_controller
        
        # Speed Controller group (indices 34-39)
        speed_controller = SignalGroup('speed_controller', 'Speed Controller')
        speed_controller.add_signal(SignalHistory('torque_reference', 'pu', 'Torque Reference', self.buffer_size))
        speed_controller.add_signal(SignalHistory('measured_speed_pu', 'pu', 'Measured Speed', self.buffer_size))
        speed_controller.add_signal(SignalHistory('speed_error', 'pu', 'Speed Error', self.buffer_size))
        speed_controller.add_signal(SignalHistory('pi_output', '', 'PI Output', self.buffer_size))
        speed_controller.add_signal(SignalHistory('is_active', '', 'Is Active', self.buffer_size))
        speed_controller.add_signal(SignalHistory('is_saturated', '', 'Is Saturated', self.buffer_size))
        self.groups['speed_controller'] = speed_controller
        
        # Startup Sequencer group (indices 34-42)
        startup_sequencer = SignalGroup('startup_sequencer', 'Startup Sequencer')
        startup_sequencer.add_signal(SignalHistory('speed_reference_pu', 'pu', 'Speed Reference', self.buffer_size))
        startup_sequencer.add_signal(SignalHistory('virtual_rotor_frequency_pu', 'pu', 'Virtual Rotor Frequency', self.buffer_size))
        startup_sequencer.add_signal(SignalHistory('virtual_rotor_angle_pu', 'pu', 'Virtual Rotor Angle', self.buffer_size))
        startup_sequencer.add_signal(SignalHistory('virtual_torque_reference_pu', 'pu', 'Virtual Torque Reference', self.buffer_size))
        startup_sequencer.add_signal(SignalHistory('release_speed_controller', '', 'Release Speed Controller', self.buffer_size))
        startup_sequencer.add_signal(SignalHistory('release_excitation', '', 'Release Excitation', self.buffer_size))
        startup_sequencer.add_signal(SignalHistory('release_torque_controller', '', 'Release Torque Controller', self.buffer_size))
        startup_sequencer.add_signal(SignalHistory('release_machine_side_pulses', '', 'Release Machine Pulses', self.buffer_size))
        startup_sequencer.add_signal(SignalHistory('is_startup_complete', '', 'Startup Complete', self.buffer_size))
        self.groups['startup_sequencer'] = startup_sequencer
        
        # Excitation Controller group (indices 43-44)
        excitation_controller = SignalGroup('excitation_controller', 'Excitation Controller')
        excitation_controller.add_signal(SignalHistory('field_current_reference_pu', 'pu', 'Field Current Reference', self.buffer_size))
        excitation_controller.add_signal(SignalHistory('prefluxing_elapsed_time_s', 's', 'Prefluxing Elapsed Time', self.buffer_size))
        self.groups['excitation_controller'] = excitation_controller
        
        # Pulse Generator group (indices 45-49)
        pulse_generator = SignalGroup('pulse_generator', 'Pulse Generator')
        pulse_generator.add_signal(SignalHistory('pulse_p1', '', 'Pulse P1 (T1)', self.buffer_size))
        pulse_generator.add_signal(SignalHistory('pulse_p2', '', 'Pulse P2 (T2)', self.buffer_size))
        pulse_generator.add_signal(SignalHistory('pulse_p3', '', 'Pulse P3 (T3)', self.buffer_size))
        pulse_generator.add_signal(SignalHistory('pulse_p4', '', 'Pulse P4 (T4)', self.buffer_size))
        pulse_generator.add_signal(SignalHistory('pulse_p5', '', 'Pulse P5 (T5)', self.buffer_size))
        self.groups['pulse_generator'] = pulse_generator
    
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
        
        # Update measurements (indices 0-13)
        self._append_signal('measurements', 'grid_voltage_magnitude_filtered_pu', packet.get_signal(GridSignalIndex.GRID_VOLTAGE_MAGNITUDE_FILTERED), timestamp_ms, sequence)
        self._append_signal('measurements', 'grid_voltage_a_pu', packet.get_signal(GridSignalIndex.GRID_VOLTAGE_A), timestamp_ms, sequence)
        self._append_signal('measurements', 'grid_voltage_b_pu', packet.get_signal(GridSignalIndex.GRID_VOLTAGE_B), timestamp_ms, sequence)
        self._append_signal('measurements', 'grid_voltage_c_pu', packet.get_signal(GridSignalIndex.GRID_VOLTAGE_C), timestamp_ms, sequence)
        self._append_signal('measurements', 'grid_current_magnitude_pu', packet.get_signal(GridSignalIndex.GRID_CURRENT_MAGNITUDE), timestamp_ms, sequence)
        self._append_signal('measurements', 'grid_current_a_pu', packet.get_signal(GridSignalIndex.GRID_CURRENT_A), timestamp_ms, sequence)
        self._append_signal('measurements', 'grid_current_b_pu', packet.get_signal(GridSignalIndex.GRID_CURRENT_B), timestamp_ms, sequence)
        self._append_signal('measurements', 'grid_current_c_pu', packet.get_signal(GridSignalIndex.GRID_CURRENT_C), timestamp_ms, sequence)
        self._append_signal('measurements', 'dc_link_current_pu', packet.get_signal(GridSignalIndex.DC_LINK_CURRENT), timestamp_ms, sequence)
        self._append_signal('measurements', 'is_commuting', float(packet.get_bool_signal(GridSignalIndex.IS_COMMUTING)), timestamp_ms, sequence)
        self._append_signal('measurements', 'grid_voltage_alpha_pu', packet.get_signal(GridSignalIndex.GRID_VOLTAGE_ALPHA), timestamp_ms, sequence)
        self._append_signal('measurements', 'grid_voltage_beta_pu', packet.get_signal(GridSignalIndex.GRID_VOLTAGE_BETA), timestamp_ms, sequence)
        self._append_signal('measurements', 'grid_voltage_magnitude_filtered_pu_2', packet.get_signal(GridSignalIndex.GRID_VOLTAGE_MAGNITUDE_FILTERED_DUP), timestamp_ms, sequence)
        self._append_signal('measurements', 'grid_side_frequency_pu', packet.get_signal(GridSignalIndex.GRID_SIDE_FREQUENCY), timestamp_ms, sequence)
        
        # Update observer (indices 14-19)
        self._append_signal('observer', 'estimated_voltage_magnitude_pu', packet.get_signal(GridSignalIndex.ESTIMATED_VOLTAGE_MAGNITUDE), timestamp_ms, sequence)
        self._append_signal('observer', 'estimated_frequency_pu', packet.get_signal(GridSignalIndex.ESTIMATED_FREQUENCY_PU), timestamp_ms, sequence)
        self._append_signal('observer', 'estimated_angle_rad', packet.get_signal(GridSignalIndex.ESTIMATED_ANGLE_RAD), timestamp_ms, sequence)
        self._append_signal('observer', 'estimated_frequency_hz', packet.get_signal(GridSignalIndex.ESTIMATED_FREQUENCY_HZ), timestamp_ms, sequence)
        self._append_signal('observer', 'estimated_voltage_alpha_pu', packet.get_signal(GridSignalIndex.ESTIMATED_VOLTAGE_ALPHA), timestamp_ms, sequence)
        self._append_signal('observer', 'estimated_voltage_beta_pu', packet.get_signal(GridSignalIndex.ESTIMATED_VOLTAGE_BETA), timestamp_ms, sequence)
        
        # Update modulator (indices 20-23)
        self._append_signal('modulator', 'firing_angle_rad', packet.get_signal(GridSignalIndex.FIRING_ANGLE_RAD), timestamp_ms, sequence)
        self._append_signal('modulator', 'is_enabled', float(packet.get_bool_signal(GridSignalIndex.MODULATOR_ENABLED)), timestamp_ms, sequence)
        self._append_signal('modulator', 'force_commutation_ack', float(packet.get_bool_signal(GridSignalIndex.FORCE_COMMUTATION_ACK)), timestamp_ms, sequence)
        self._append_signal('modulator', 'is_requesting_pi_reset', float(packet.get_bool_signal(GridSignalIndex.IS_REQUESTING_PI_RESET)), timestamp_ms, sequence)
        
        # Update controller (indices 24-30)
        self._append_signal('controller', 'dc_link_current_reference_pu', packet.get_signal(GridSignalIndex.DC_LINK_CURRENT_REFERENCE), timestamp_ms, sequence)
        self._append_signal('controller', 'dc_link_current_measured_pu', packet.get_signal(GridSignalIndex.DC_LINK_CURRENT_MEASURED), timestamp_ms, sequence)
        self._append_signal('controller', 'current_error_pu', packet.get_signal(GridSignalIndex.CURRENT_ERROR), timestamp_ms, sequence)
        self._append_signal('controller', 'controller_output_rad', packet.get_signal(GridSignalIndex.CONTROLLER_OUTPUT), timestamp_ms, sequence)
        self._append_signal('controller', 'operating_mode', packet.get_signal(GridSignalIndex.OPERATING_MODE), timestamp_ms, sequence)
        self._append_signal('controller', 'controller_saturated', float(packet.get_bool_signal(GridSignalIndex.CONTROLLER_SATURATED)), timestamp_ms, sequence)
        self._append_signal('controller', 'freeze_integrator', float(packet.get_bool_signal(GridSignalIndex.FREEZE_INTEGRATOR)), timestamp_ms, sequence)
        
        # Update state machine (indices 31-38)
        self._append_signal('state_machine', 'current_state_id', packet.get_signal(GridSignalIndex.STATE_MACHINE_STATE), timestamp_ms, sequence)
        self._append_signal('state_machine', 'pulse_releases', float(packet.get_bool_signal(GridSignalIndex.PULSE_RELEASES)), timestamp_ms, sequence)
        self._append_signal('state_machine', 'auxiliaries_are_on', float(packet.get_bool_signal(GridSignalIndex.AUXILIARIES_ON)), timestamp_ms, sequence)
        self._append_signal('state_machine', 'breakers_are_on', float(packet.get_bool_signal(GridSignalIndex.BREAKERS_ON)), timestamp_ms, sequence)
        self._append_signal('state_machine', 'start_command', float(packet.get_bool_signal(GridSignalIndex.START_COMMAND)), timestamp_ms, sequence)
        self._append_signal('state_machine', 'stop_command', float(packet.get_bool_signal(GridSignalIndex.STOP_COMMAND)), timestamp_ms, sequence)
        self._append_signal('state_machine', 'is_current_extinguished', float(packet.get_bool_signal(GridSignalIndex.CURRENT_EXTINGUISHED)), timestamp_ms, sequence)
        self._append_signal('state_machine', 'shutdown_request', float(packet.get_bool_signal(GridSignalIndex.SHUTDOWN_REQUESTED)), timestamp_ms, sequence)
        
        # Update operation mode monitor (indices 39-42)
        self._append_signal('operation_mode', 'current_mode', packet.get_signal(GridSignalIndex.OPERATION_MODE), timestamp_ms, sequence)
        self._append_signal('operation_mode', 'has_mode_changed', float(packet.get_bool_signal(GridSignalIndex.MODE_CHANGED)), timestamp_ms, sequence)
        self._append_signal('operation_mode', 'is_current_zero', float(packet.get_bool_signal(GridSignalIndex.CURRENT_ZERO)), timestamp_ms, sequence)
        self._append_signal('operation_mode', 'is_discontinuous_mode', float(packet.get_bool_signal(GridSignalIndex.DISCONTINUOUS_MODE)), timestamp_ms, sequence)
        
        # Update pulse generator (indices 43-49)
        self._append_signal('pulse_generator', 'pulse_p1', float(packet.get_bool_signal(GridSignalIndex.FIRING_PULSE_1)), timestamp_ms, sequence)
        self._append_signal('pulse_generator', 'pulse_p2', float(packet.get_bool_signal(GridSignalIndex.FIRING_PULSE_2)), timestamp_ms, sequence)
        self._append_signal('pulse_generator', 'pulse_p3', float(packet.get_bool_signal(GridSignalIndex.FIRING_PULSE_3)), timestamp_ms, sequence)
        self._append_signal('pulse_generator', 'pulse_p4', float(packet.get_bool_signal(GridSignalIndex.FIRING_PULSE_4)), timestamp_ms, sequence)
        self._append_signal('pulse_generator', 'pulse_p5', float(packet.get_bool_signal(GridSignalIndex.FIRING_PULSE_5)), timestamp_ms, sequence)
        self._append_signal('pulse_generator', 'pulse_p6', float(packet.get_bool_signal(GridSignalIndex.FIRING_PULSE_6)), timestamp_ms, sequence)
        self._append_signal('pulse_generator', 'pulse_active', float(packet.get_bool_signal(GridSignalIndex.PULSE_GENERATOR_ACTIVE)), timestamp_ms, sequence)
    
    def _update_from_machine_packet(self, packet: MachineSideTelemetryPacket) -> None:
        """Update signal histories from machine-side telemetry packet.
        
        Args:
            packet: Machine-side telemetry packet
        """
        timestamp_ms = packet.header.timestamp_ms
        sequence = packet.header.sequence_number
        
        # Update measurements (indices 0-8)
        self._append_signal('measurements', 'machine_voltage_magnitude_filtered_pu', packet.get_signal(MachineSignalIndex.MACHINE_VOLTAGE_MAGNITUDE_FILTERED), timestamp_ms, sequence)
        self._append_signal('measurements', 'machine_voltage_a_pu', packet.get_signal(MachineSignalIndex.MACHINE_VOLTAGE_ABC_A), timestamp_ms, sequence)
        self._append_signal('measurements', 'machine_voltage_b_pu', packet.get_signal(MachineSignalIndex.MACHINE_VOLTAGE_ABC_B), timestamp_ms, sequence)
        self._append_signal('measurements', 'machine_voltage_c_pu', packet.get_signal(MachineSignalIndex.MACHINE_VOLTAGE_ABC_C), timestamp_ms, sequence)
        self._append_signal('measurements', 'machine_voltage_magnitude', packet.get_signal(MachineSignalIndex.MACHINE_VOLTAGE_MAGNITUDE), timestamp_ms, sequence)
        self._append_signal('measurements', 'is_commuting', float(packet.get_bool_signal(MachineSignalIndex.IS_COMMUTING)), timestamp_ms, sequence)
        
        # Update observer (indices 9-12)
        self._append_signal('observer', 'estimated_frequency_pu', packet.get_signal(MachineSignalIndex.ESTIMATED_FREQUENCY_PU), timestamp_ms, sequence)
        self._append_signal('observer', 'estimated_position_pu', packet.get_signal(MachineSignalIndex.ESTIMATED_POSITION_PU), timestamp_ms, sequence)
        self._append_signal('observer', 'filtered_voltage_alpha_pu', packet.get_signal(MachineSignalIndex.FILTERED_VOLTAGE_ALPHA), timestamp_ms, sequence)
        self._append_signal('observer', 'filtered_voltage_beta_pu', packet.get_signal(MachineSignalIndex.FILTERED_VOLTAGE_BETA), timestamp_ms, sequence)
        
        # Update modulator (indices 15-19)
        self._append_signal('modulator', 'firing_angle_rad', packet.get_signal(MachineSignalIndex.FIRING_ANGLE_RAD), timestamp_ms, sequence)
        self._append_signal('modulator', 'is_enabled', float(packet.get_bool_signal(MachineSignalIndex.MODULATOR_ENABLED)), timestamp_ms, sequence)
        self._append_signal('modulator', 'release_pulses', float(packet.get_bool_signal(MachineSignalIndex.RELEASE_PULSES)), timestamp_ms, sequence)
        self._append_signal('modulator', 'request_force_commutation', float(packet.get_bool_signal(MachineSignalIndex.REQUEST_FORCE_COMMUTATION)), timestamp_ms, sequence)
        self._append_signal('modulator', 'current_rotor_sector', packet.get_signal(MachineSignalIndex.CURRENT_ROTOR_SECTOR), timestamp_ms, sequence)
        
        # Update machine controller (indices 18-20)
        self._append_signal('machine_controller', 'firing_angle', packet.get_signal(MachineSignalIndex.CONTROLLER_FIRING_ANGLE), timestamp_ms, sequence)
        self._append_signal('machine_controller', 'dc_link_current', packet.get_signal(MachineSignalIndex.CONTROLLER_DC_LINK_CURRENT), timestamp_ms, sequence)
        self._append_signal('machine_controller', 'machine_speed', packet.get_signal(MachineSignalIndex.CONTROLLER_MACHINE_SPEED), timestamp_ms, sequence)
        
        # Update torque controller (indices 21-33)
        self._append_signal('torque_controller', 'current_reference_pu', packet.get_signal(MachineSignalIndex.TORQUE_CURRENT_REFERENCE), timestamp_ms, sequence)
        self._append_signal('torque_controller', 'base_torque_reference', packet.get_signal(MachineSignalIndex.TORQUE_BASE_REFERENCE), timestamp_ms, sequence)
        self._append_signal('torque_controller', 'total_torque_reference', packet.get_signal(MachineSignalIndex.TORQUE_TOTAL_REFERENCE), timestamp_ms, sequence)
        self._append_signal('torque_controller', 'limited_torque_reference', packet.get_signal(MachineSignalIndex.TORQUE_LIMITED_REFERENCE), timestamp_ms, sequence)
        self._append_signal('torque_controller', 'is_saturated', float(packet.get_bool_signal(MachineSignalIndex.TORQUE_IS_SATURATED)), timestamp_ms, sequence)
        
        # Update speed controller (indices 34-39)
        self._append_signal('speed_controller', 'torque_reference', packet.get_signal(MachineSignalIndex.SPEED_TORQUE_REFERENCE), timestamp_ms, sequence)
        self._append_signal('speed_controller', 'measured_speed_pu', packet.get_signal(MachineSignalIndex.SPEED_MEASURED_SPEED_PU), timestamp_ms, sequence)
        self._append_signal('speed_controller', 'speed_error', packet.get_signal(MachineSignalIndex.SPEED_ERROR), timestamp_ms, sequence)
        self._append_signal('speed_controller', 'pi_output', packet.get_signal(MachineSignalIndex.SPEED_PI_OUTPUT), timestamp_ms, sequence)
        self._append_signal('speed_controller', 'is_active', float(packet.get_bool_signal(MachineSignalIndex.SPEED_IS_ACTIVE)), timestamp_ms, sequence)
        self._append_signal('speed_controller', 'is_saturated', float(packet.get_bool_signal(MachineSignalIndex.SPEED_IS_SATURATED)), timestamp_ms, sequence)
        
        # Update startup sequencer (indices 34-42)
        self._append_signal('startup_sequencer', 'speed_reference_pu', packet.get_signal(MachineSignalIndex.STARTUP_SPEED_REFERENCE), timestamp_ms, sequence)
        self._append_signal('startup_sequencer', 'virtual_rotor_frequency_pu', packet.get_signal(MachineSignalIndex.STARTUP_VIRTUAL_ROTOR_FREQUENCY), timestamp_ms, sequence)
        self._append_signal('startup_sequencer', 'virtual_rotor_angle_pu', packet.get_signal(MachineSignalIndex.STARTUP_VIRTUAL_ROTOR_ANGLE), timestamp_ms, sequence)
        self._append_signal('startup_sequencer', 'virtual_torque_reference_pu', packet.get_signal(MachineSignalIndex.STARTUP_VIRTUAL_TORQUE_REFERENCE), timestamp_ms, sequence)
        self._append_signal('startup_sequencer', 'release_speed_controller', float(packet.get_bool_signal(MachineSignalIndex.STARTUP_RELEASE_SPEED_CONTROLLER)), timestamp_ms, sequence)
        self._append_signal('startup_sequencer', 'release_excitation', float(packet.get_bool_signal(MachineSignalIndex.STARTUP_RELEASE_EXCITATION)), timestamp_ms, sequence)
        self._append_signal('startup_sequencer', 'release_torque_controller', float(packet.get_bool_signal(MachineSignalIndex.STARTUP_RELEASE_TORQUE_CONTROLLER)), timestamp_ms, sequence)
        self._append_signal('startup_sequencer', 'release_machine_side_pulses', float(packet.get_bool_signal(MachineSignalIndex.STARTUP_RELEASE_MACHINE_SIDE_PULSES)), timestamp_ms, sequence)
        self._append_signal('startup_sequencer', 'is_startup_complete', float(packet.get_bool_signal(MachineSignalIndex.STARTUP_IS_STARTUP_COMPLETE)), timestamp_ms, sequence)
        
        # Update excitation controller (indices 43-44)
        self._append_signal('excitation_controller', 'field_current_reference_pu', packet.get_signal(MachineSignalIndex.EXCITATION_FIELD_CURRENT_REFERENCE), timestamp_ms, sequence)
        self._append_signal('excitation_controller', 'prefluxing_elapsed_time_s', packet.get_signal(MachineSignalIndex.EXCITATION_PREFLUXING_ELAPSED_TIME), timestamp_ms, sequence)
        
        # Update pulse generator (indices 45-49)
        self._append_signal('pulse_generator', 'pulse_p1', float(packet.get_bool_signal(MachineSignalIndex.PULSE_P1)), timestamp_ms, sequence)
        self._append_signal('pulse_generator', 'pulse_p2', float(packet.get_bool_signal(MachineSignalIndex.PULSE_P2)), timestamp_ms, sequence)
        self._append_signal('pulse_generator', 'pulse_p3', float(packet.get_bool_signal(MachineSignalIndex.PULSE_P3)), timestamp_ms, sequence)
        self._append_signal('pulse_generator', 'pulse_p4', float(packet.get_bool_signal(MachineSignalIndex.PULSE_P4)), timestamp_ms, sequence)
        self._append_signal('pulse_generator', 'pulse_p5', float(packet.get_bool_signal(MachineSignalIndex.PULSE_P5)), timestamp_ms, sequence)
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
        measurements.add_signal(SignalHistory('is_commuting', '', 'Is Commuting', self.buffer_size))
        measurements.add_signal(SignalHistory('grid_voltage_alpha_pu', 'pu', 'Grid Voltage Alpha', self.buffer_size))
        measurements.add_signal(SignalHistory('grid_voltage_beta_pu', 'pu', 'Grid Voltage Beta', self.buffer_size))
        measurements.add_signal(SignalHistory('grid_voltage_magnitude_filtered_pu_2', 'pu', 'Grid Voltage Mag (Filtered) #2', self.buffer_size))
        measurements.add_signal(SignalHistory('grid_side_frequency_pu', 'pu', 'Grid Side Frequency', self.buffer_size))
        self.groups['measurements'] = measurements
        
        # Observer group (indices 14-19)
        observer = SignalGroup('observer', 'Grid Observer')
        observer.add_signal(SignalHistory('estimated_voltage_magnitude_pu', 'pu', 'Est. Voltage Magnitude', self.buffer_size))
        observer.add_signal(SignalHistory('estimated_frequency_pu', 'pu', 'Est. Frequency (pu)', self.buffer_size))
        observer.add_signal(SignalHistory('estimated_angle_rad', 'rad', 'Est. Angle', self.buffer_size))
        observer.add_signal(SignalHistory('estimated_frequency_hz', 'Hz', 'Est. Frequency (Hz)', self.buffer_size))
        observer.add_signal(SignalHistory('estimated_voltage_alpha_pu', 'pu', 'Est. Voltage Alpha', self.buffer_size))
        observer.add_signal(SignalHistory('estimated_voltage_beta_pu', 'pu', 'Est. Voltage Beta', self.buffer_size))
        self.groups['observer'] = observer
        
        # Modulator group (indices 20-23)
        modulator = SignalGroup('modulator', 'Grid-Side Modulator')
        modulator.add_signal(SignalHistory('firing_angle_rad', 'rad', 'Firing Angle', self.buffer_size))
        modulator.add_signal(SignalHistory('is_enabled', '', 'Is Enabled', self.buffer_size))
        modulator.add_signal(SignalHistory('force_commutation_ack', '', 'Force Commutation Ack', self.buffer_size))
        modulator.add_signal(SignalHistory('is_requesting_pi_reset', '', 'Is Requesting PI Reset', self.buffer_size))
        self.groups['modulator'] = modulator
        
        # Controller group (indices 24-30)
        controller = SignalGroup('controller', 'Current Controller')
        controller.add_signal(SignalHistory('dc_link_current_reference_pu', 'pu', 'DC Current Reference', self.buffer_size))
        controller.add_signal(SignalHistory('dc_link_current_measured_pu', 'pu', 'DC Current Measured', self.buffer_size))
        controller.add_signal(SignalHistory('current_error_pu', 'pu', 'Current Error', self.buffer_size))
        controller.add_signal(SignalHistory('controller_output_rad', 'rad', 'Controller Output', self.buffer_size))
        controller.add_signal(SignalHistory('operating_mode', '', 'Operating Mode', self.buffer_size))
        controller.add_signal(SignalHistory('controller_saturated', '', 'Controller Saturated', self.buffer_size))
        controller.add_signal(SignalHistory('freeze_integrator', '', 'Freeze Integrator', self.buffer_size))
        self.groups['controller'] = controller
        
        # State machine group (indices 31-38)
        state_machine = SignalGroup('state_machine', 'State Machine')
        state_machine.add_signal(SignalHistory('current_state_id', '', 'Current State', self.buffer_size))
        state_machine.add_signal(SignalHistory('pulse_releases', '', 'Pulse Releases', self.buffer_size))
        state_machine.add_signal(SignalHistory('auxiliaries_are_on', '', 'Auxiliaries Are On', self.buffer_size))
        state_machine.add_signal(SignalHistory('breakers_are_on', '', 'Breakers Are On', self.buffer_size))
        state_machine.add_signal(SignalHistory('start_command', '', 'Start Command', self.buffer_size))
        state_machine.add_signal(SignalHistory('stop_command', '', 'Stop Command', self.buffer_size))
        state_machine.add_signal(SignalHistory('is_current_extinguished', '', 'Current Extinguished', self.buffer_size))
        state_machine.add_signal(SignalHistory('shutdown_request', '', 'Shutdown Requested', self.buffer_size))
        self.groups['state_machine'] = state_machine
        
        # Operation mode monitor group (indices 39-42)
        operation_mode = SignalGroup('operation_mode', 'Operation Mode Monitor')
        operation_mode.add_signal(SignalHistory('current_mode', '', 'Current Mode', self.buffer_size))
        operation_mode.add_signal(SignalHistory('has_mode_changed', '', 'Mode Changed', self.buffer_size))
        operation_mode.add_signal(SignalHistory('is_current_zero', '', 'Current Is Zero', self.buffer_size))
        operation_mode.add_signal(SignalHistory('is_discontinuous_mode', '', 'Is Discontinuous Mode', self.buffer_size))
        self.groups['operation_mode'] = operation_mode
        
        # Pulse generator group (indices 43-49)
        pulse_generator = SignalGroup('pulse_generator', 'Pulse Generator')
        pulse_generator.add_signal(SignalHistory('pulse_p1', '', 'Pulse P1 (T1)', self.buffer_size))
        pulse_generator.add_signal(SignalHistory('pulse_p2', '', 'Pulse P2 (T2)', self.buffer_size))
        pulse_generator.add_signal(SignalHistory('pulse_p3', '', 'Pulse P3 (T3)', self.buffer_size))
        pulse_generator.add_signal(SignalHistory('pulse_p4', '', 'Pulse P4 (T4)', self.buffer_size))
        pulse_generator.add_signal(SignalHistory('pulse_p5', '', 'Pulse P5 (T5)', self.buffer_size))
        pulse_generator.add_signal(SignalHistory('pulse_p6', '', 'Pulse P6 (T6)', self.buffer_size))
        pulse_generator.add_signal(SignalHistory('pulse_active', '', 'Pulse Active', self.buffer_size))
        self.groups['pulse_generator'] = pulse_generator
    
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
