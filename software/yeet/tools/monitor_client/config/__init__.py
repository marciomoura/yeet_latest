"""Configuration module for the monitor client.

Contains plot definitions and other configuration data separated from UI code.
"""

from .plot_definitions import (
    get_grid_side_plots,
    get_machine_side_plots,
    get_default_visible_groups
)

from .telemetry_definitions import (
    get_grid_side_telemetry,
    get_machine_side_telemetry
)

__all__ = [
    'get_grid_side_plots',
    'get_machine_side_plots',
    'get_default_visible_groups',
    'get_grid_side_telemetry',
    'get_machine_side_telemetry'
]
