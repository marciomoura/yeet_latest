#!/usr/bin/env python3
"""Data persistence and export for telemetry logging.

Provides options to:
1. Keep data in memory indefinitely (for post-run analysis)
2. Export data to CSV/HDF5 for offline visualization
3. Stream data to disk for very long recordings
"""

import csv
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
    
    def export_csv(self, filepath: Optional[Path] = None, data_buffers: Optional[Dict] = None) -> bool:
        """Export data to CSV file.
        
        Can export either persistent in-memory data or rolling window buffers.
        
        Args:
            filepath: Path to export to (uses self.export_path if not specified)
            data_buffers: Optional dict of deque buffers to export from
                         {
                             'voltage': deque(...),
                             'current': deque(...),
                             'observer_voltage': deque(...),
                             'dc_link': deque(...),
                         }
        
        Returns:
            True if export successful, False otherwise
        """
        target_path = filepath or self.export_path
        if not target_path:
            logger.error("No export path specified")
            return False
        
        try:
            target_path = Path(target_path)
            target_path.parent.mkdir(parents=True, exist_ok=True)
            
            with open(target_path, 'w', newline='') as f:
                writer = csv.writer(f)
                
                # Write header - include all signal indices
                header = [
                    'sample_index',
                    'timestamp_ms',
                    'sequence_number',
                    'signal_count',
                ]
                # Add columns for all possible signals (0-255)
                for i in range(256):
                    header.append(f'signal_{i:03d}')
                writer.writerow(header)
                
                # Write data
                if self.enable_persistence and self.all_samples:
                    # Export persistent data with full signal array
                    for idx, sample in enumerate(self.all_samples):
                        row = [
                            idx,
                            sample.timestamp_ms,
                            sample.sequence_number,
                            sample.signal_count,
                        ]
                        # Add signal values, pad with empty strings for missing signals
                        for sig_idx in range(256):
                            if sig_idx < len(sample.signals):
                                row.append(sample.signals[sig_idx])
                            else:
                                row.append('')
                        writer.writerow(row)
                    row_count = len(self.all_samples)
                elif data_buffers:
                    # Export rolling window data (fallback for non-persistent mode)
                    # Reconstruct by zipping all buffers together
                    num_samples = len(data_buffers.get('voltage', deque()))
                    for idx in range(num_samples):
                        try:
                            row = [
                                idx,
                                data_buffers['timestamp'][idx] if 'timestamp' in data_buffers else 0,
                                data_buffers['sequence'][idx] if 'sequence' in data_buffers else idx,
                                len(data_buffers),  # signal count estimate
                            ]
                            # Add available buffer data
                            for sig_idx in range(256):
                                # Map buffer keys to signal indices (basic mapping)
                                if sig_idx == 0 and 'voltage' in data_buffers:
                                    row.append(data_buffers['voltage'][idx])
                                elif sig_idx == 4 and 'current' in data_buffers:
                                    row.append(data_buffers['current'][idx])
                                elif sig_idx == 8 and 'dc_link' in data_buffers:
                                    row.append(data_buffers['dc_link'][idx])
                                elif sig_idx == 13 and 'observer_voltage' in data_buffers:
                                    row.append(data_buffers['observer_voltage'][idx])
                                elif sig_idx == 17 and 'observer_frequency' in data_buffers:
                                    row.append(data_buffers['observer_frequency'][idx])
                                elif sig_idx == 20 and 'firing_angle' in data_buffers:
                                    row.append(data_buffers['firing_angle'][idx])
                                else:
                                    row.append('')
                            writer.writerow(row)
                        except IndexError:
                            # Buffers may have different lengths, skip misaligned samples
                            break
                    row_count = idx + 1
                else:
                    logger.warning("No data to export")
                    return False
            
            logger.info(f"Exported {row_count} samples to {target_path}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to export CSV: {e}")
            return False
    
    def export_json(self, filepath: Optional[Path] = None) -> bool:
        """Export metadata and statistics to JSON.
        
        Args:
            filepath: Path to export to
            
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
            
            metadata = {
                'persistence_enabled': self.enable_persistence,
                'start_time': self.start_time.isoformat() if self.start_time else None,
                'end_time': self.end_time.isoformat() if self.end_time else None,
                'total_packets_received': self.total_packets,
                'total_samples_stored': self.total_samples_stored,
                'duration_seconds': (
                    (self.end_time - self.start_time).total_seconds() 
                    if self.start_time and self.end_time else None
                ),
            }
            
            with open(target_path, 'w') as f:
                json.dump(metadata, f, indent=2)
            
            logger.info(f"Exported metadata to {target_path}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to export JSON metadata: {e}")
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
