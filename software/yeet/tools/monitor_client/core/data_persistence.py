#!/usr/bin/env python3
"""Data persistence and export for telemetry logging.

Provides options to:
1. Keep data in memory indefinitely (for post-run analysis)
2. Export data to JSON for offline visualization
3. Stream data to disk for very long recordings
"""

import json
import logging
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional
from collections import deque


logger = logging.getLogger(__name__)


@dataclass
class TelemetrySample:
    """Single telemetry sample with complete signal array."""
    timestamp_ms: int
    sequence_number: int
    signal_count: int
    signals: List[float]  # Complete signal array from packet
    
    # Convenience properties for common signals
    @property
    def voltage_magnitude(self) -> float:
        """Grid voltage magnitude (index 0)."""
        return self.signals[0] if len(self.signals) > 0 else 0.0
    
    @property
    def current_magnitude(self) -> float:
        """Grid current magnitude (index 4)."""
        return self.signals[4] if len(self.signals) > 4 else 0.0
    
    @property
    def dc_link_current(self) -> float:
        """DC link current (index 8)."""
        return self.signals[8] if len(self.signals) > 8 else 0.0
    
    @property
    def observer_voltage(self) -> float:
        """Estimated voltage magnitude (index 13)."""
        return self.signals[13] if len(self.signals) > 13 else 0.0
    
    @property
    def observer_frequency(self) -> float:
        """Estimated frequency (index 17)."""
        return self.signals[17] if len(self.signals) > 17 else 0.0
    
    @property
    def firing_angle(self) -> float:
        """Firing angle output (index 20)."""
        return self.signals[20] if len(self.signals) > 20 else 0.0
    

class DataPersistence:
    """Manage telemetry data persistence and export."""
    
    def __init__(self, enable_persistence: bool = False, export_path: Optional[Path] = None):
        """Initialize data persistence.
        
        Args:
            enable_persistence: If True, keep all data in memory (uses more RAM)
                               If False, only rolling window kept in deques
            export_path: Optional path to export data to (CSV or HDF5)
        """
        self.enable_persistence = enable_persistence
        self.export_path = export_path
        
        # In-memory storage (only if persistence enabled)
        self.all_samples: List[TelemetrySample] = [] if enable_persistence else None
        
        # Metadata
        self.start_time: Optional[datetime] = None
        self.end_time: Optional[datetime] = None
        self.total_packets: int = 0
        self.total_samples_stored: int = 0
        
        if enable_persistence:
            logger.info("Data persistence ENABLED - all samples will be kept in memory")
        else:
            logger.info("Data persistence DISABLED - only rolling window (10k samples) kept")
    
    def add_sample(self, sample: TelemetrySample) -> None:
        """Add a telemetry sample to persistent storage.
        
        Args:
            sample: TelemetrySample to store
        """
        if not self.enable_persistence:
            return
        
        if self.start_time is None:
            self.start_time = datetime.now()
        
        self.all_samples.append(sample)
        self.total_samples_stored += 1
        self.end_time = datetime.now()
    
    def export_json(self, filepath: Optional[Path] = None, data_buffers: Optional[Dict] = None, signal_names: Optional[List[str]] = None) -> bool:
        """Export metadata and telemetry data to JSON.
        
        Args:
            filepath: Path to export to
            data_buffers: Optional dict of deque buffers to export from
            signal_names: Optional list of descriptive names for signals (when using persistence)
            
        Returns:
            True if export successful, False otherwise
        """
        target_path = filepath or (self.export_path.with_suffix('.json') if self.export_path else None)
        if not target_path:
            logger.error("No export path specified")
            return False
        
        try:
            target_path = Path(target_path)
            target_path.parent.mkdir(parents=True, exist_ok=True)
            
            # Base metadata format compatible with plot_sampler_data.py
            output_data = {
                'test_suite': 'SFC Monitor Session',
                'test_name': f'Telemetry Export {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}',
                'persistence_enabled': self.enable_persistence,
                'start_time': self.start_time.isoformat() if self.start_time else None,
                'end_time': self.end_time.isoformat() if self.end_time else None,
                'total_packets_received': self.total_packets,
                'total_samples_stored': self.total_samples_stored,
                'duration_seconds': (
                    (self.end_time - self.start_time).total_seconds() 
                    if self.start_time and self.end_time else None
                ),
                'test_data': []
            }
            
            if self.enable_persistence and self.all_samples:
                # Add time data
                time_data = [s.timestamp_ms for s in self.all_samples]
                output_data['test_data'].append({'name': 'time', 'data': time_data})
                
                # Add sequences for reference
                seq_data = [s.sequence_number for s in self.all_samples]
                output_data['test_data'].append({'name': 'sequence', 'data': seq_data})

                # Add signals
                if self.all_samples:
                    num_signals = self.all_samples[0].signal_count
                    for i in range(num_signals):
                        signal_data = [s.signals[i] if i < len(s.signals) else 0.0 for s in self.all_samples]
                        
                        # Use provided name or default to raw signal index
                        if signal_names and i < len(signal_names):
                            name = signal_names[i]
                        else:
                            name = f'Raw: signal_{i:03d}'
                            
                        output_data['test_data'].append({
                            'name': name,
                            'data': signal_data,
                            'dimension': 1
                        })
            elif data_buffers:
                # Export from rolling window buffers
                # Use 'timestamp_ms' for 'time' if available
                if 'timestamp_ms' in data_buffers:
                    output_data['test_data'].append({'name': 'time', 'data': list(data_buffers['timestamp_ms'])})
                else:
                    # Fallback to index
                    first_buf = next(iter(data_buffers.values())) if data_buffers else []
                    output_data['test_data'].append({'name': 'time', 'data': list(range(len(first_buf)))})

                for name, buffer in data_buffers.items():
                    if name == 'timestamp_ms':
                        continue
                    output_data['test_data'].append({
                        'name': name,
                        'data': list(buffer),
                        'dimension': 1
                    })
            
            with open(target_path, 'w') as f:
                json.dump(output_data, f, indent=2)
            
            logger.info(f"Exported {len(output_data['test_data'])} signals to {target_path}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to export JSON: {e}")
            return False
    
    def get_statistics(self) -> Dict:
        """Get statistics about stored data.
        
        Returns:
            Dictionary with statistics
        """
        duration = None
        if self.start_time and self.end_time:
            duration = (self.end_time - self.start_time).total_seconds()
        
        return {
            'persistence_enabled': self.enable_persistence,
            'total_samples_stored': self.total_samples_stored,
            'total_packets': self.total_packets,
            'start_time': self.start_time.isoformat() if self.start_time else None,
            'end_time': self.end_time.isoformat() if self.end_time else None,
            'duration_seconds': duration,
            'memory_used_mb': self._estimate_memory_usage_mb(),
        }
    
    def _estimate_memory_usage_mb(self) -> float:
        """Estimate memory used by stored samples.
        
        Returns:
            Estimated memory usage in MB
        """
        if not self.all_samples:
            return 0.0
        
        # Rough estimate: ~100 bytes per sample
        bytes_used = len(self.all_samples) * 100
        return bytes_used / (1024 * 1024)
    
    def clear(self) -> None:
        """Clear all stored data."""
        if self.enable_persistence and self.all_samples:
            logger.info(f"Clearing {len(self.all_samples)} stored samples")
            self.all_samples.clear()
        self.total_samples_stored = 0
