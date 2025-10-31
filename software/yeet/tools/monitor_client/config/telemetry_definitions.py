#!/usr/bin/env python3
"""Telemetry signal definitions convenience module.

This module provides backward compatibility by re-exporting definitions from
the explicit machine and grid side modules. For new code, import directly from:
  - grid_side_telemetry_definitions.py
  - machine_side_telemetry_definitions.py
"""

from .grid_side_telemetry_definitions import (
    get_grid_side_telemetry,
    get_default_visible_sections as get_grid_default_sections
)
from .machine_side_telemetry_definitions import (
    get_machine_side_telemetry,
    get_default_visible_sections as get_machine_default_sections
)


def get_default_visible_sections(target: str) -> list:
    """Get default visible sections for a given target.
    
    Args:
        target: Either 'grid' or 'machine'
        
    Returns:
        List of section names that should be visible by default
    """
    if target == 'grid':
        return get_grid_default_sections()
    if target == 'machine':
        return get_machine_default_sections()
    return []


__all__ = [
    'get_grid_side_telemetry',
    'get_machine_side_telemetry',
    'get_default_visible_sections',
]
