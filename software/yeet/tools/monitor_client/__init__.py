"""
Monitor Client Package
======================

A professional telemetry monitoring application for embedded LCI control systems.

Modules:
    core: Network communication and protocol handling
    ui: User interface components
    utils: Utility functions and helpers
"""

__version__ = "1.0.0"
__author__ = "Techperts"

from .core.telemetry_protocol import GridSideTelemetryPacket
from .core.parameter_protocol import (
    ParameterValueType,
    PacketType,
    ParameterMetadata,
    ParameterValue
)

__all__ = [
    'GridSideTelemetryPacket',
    'ParameterValueType',
    'PacketType',
    'ParameterMetadata',
    'ParameterValue',
]
