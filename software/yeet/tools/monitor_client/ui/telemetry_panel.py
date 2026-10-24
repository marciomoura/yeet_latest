#!/usr/bin/env python3
"""Dedicated telemetry panel widget for displaying signal values.

This widget encapsulates telemetry dashboard management, independent from
networking, parameters, and data persistence - similar to oscilloscope_widget.

Architecture:
- Self-contained telemetry display with signal organization
- Scrollable area with collapsible sections
- Support for both grid-side and machine-side signals
- Easy integration with any telemetry packet type
"""

import logging
from typing import Dict, List, Optional

from PyQt5 import QtWidgets, QtCore

from ..config.grid_side_telemetry_definitions import (
    get_grid_side_telemetry,
    get_default_visible_sections as get_grid_default_sections
)
from ..config.machine_side_telemetry_definitions import (
    get_machine_side_telemetry,
    get_default_visible_sections as get_machine_default_sections
)


logger = logging.getLogger(__name__)


class TelemetryPanel(QtWidgets.QWidget):
    """Self-contained telemetry panel for displaying signal values.
    
    Manages:
    - Telemetry sections (State Machine, Measurements, Observer, etc.)
    - Signal labels and value updates
    - Scrollable display with consistent formatting
    - Support for different telemetry configurations (grid vs machine)
    
    External code updates signal values via update_signal() and
    can define custom layouts via add_section().
    """
    
    def __init__(self, parent=None, max_width: int = 500):
        """Initialize telemetry panel.
        
        Args:
            parent: Parent widget
            max_width: Maximum width of the panel (default: 400px)
        """
        super().__init__(parent)
        
        self.max_width = max_width
        
        # Store labels for direct access
        # Structure: {(section_title, signal_name): QLabel}
        self.signal_labels: Dict[tuple, QtWidgets.QLabel] = {}
        
        # Store sections for organization
        # Structure: {section_title: [field_names]}
        self.sections: Dict[str, List[str]] = {}
        
        # Track current section being added (for label key generation)
        self._current_section: str = ""
        
        # Initialize UI
        self._init_ui()
    
    def _init_ui(self):
        """Initialize widget layout."""
        self.setMaximumWidth(self.max_width)
        
        # Main scrollable area
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        # scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        
        # Content widget
        self.content_widget = QtWidgets.QWidget()
        self.content_layout = QtWidgets.QVBoxLayout(self.content_widget)
        self.content_layout.setSpacing(5)
        self.content_layout.setContentsMargins(5, 5, 5, 5)
        
        scroll.setWidget(self.content_widget)
        
        # Wrap in main layout
        main_layout = QtWidgets.QVBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.addWidget(scroll)
        
        self.setLayout(main_layout)
    
    def add_header(self, title: str):
        """Add a main header to the panel.
        
        Args:
            title: Header text (e.g., "TELEMETRY", "MACHINE TELEMETRY")
        """
        header = QtWidgets.QLabel(title)
        header.setStyleSheet("color: #00ffff; font-size: 14pt; font-weight: bold;")
        self.content_layout.insertWidget(0, header)
    
    def add_section(self, title: str, fields: List[str]):
        """Add a telemetry section with fields.
        
        Args:
            title: Section title (e.g., "State Machine", "Measurements")
            fields: List of field names to display
        """
        self._current_section = title
        
        section_label = QtWidgets.QLabel(title)
        section_label.setStyleSheet("color: #ffff00; font-weight: bold; font-size: 11pt; margin-top: 10px;")
        self.content_layout.addWidget(section_label)
        
        for field in fields:
            field_layout = QtWidgets.QHBoxLayout()
            field_layout.setSpacing(5)
            
            # Field name label
            name_label = QtWidgets.QLabel(field + ":")
            name_label.setStyleSheet("color: #d0d0d0; font-size: 9pt;")
            name_label.setMinimumWidth(200)
            field_layout.addWidget(name_label)
            
            # Field value label
            value_label = QtWidgets.QLabel("---")
            value_label.setStyleSheet("color: #ffffff; font-family: 'Courier New'; font-size: 9pt;")
            value_label.setAlignment(QtCore.Qt.AlignRight)
            field_layout.addWidget(value_label)
            
            # Store reference for updates using (section, field) tuple as key
            self.signal_labels[(title, field)] = value_label
            self.content_layout.addLayout(field_layout)
        
        # Store section mapping
        self.sections[title] = fields
    
    def update_signal(self, signal_name: str, value: any, format_str: str = None, section: str = None):
        """Update a single signal value display.
        
        Args:
            signal_name: Signal name (key in signal_labels)
            value: Signal value (any type)
            format_str: Optional format string (e.g., ":.4f" for floats, default auto)
            section: Optional section name for disambiguation when multiple sections have same field name
        """
        # Try to find the label - if section provided, use it; otherwise search all sections
        label = None
        if section is not None:
            key = (section, signal_name)
            label = self.signal_labels.get(key)
        else:
            # Search for signal in any section (backward compatibility)
            for (sec, sig), lbl in self.signal_labels.items():
                if sig == signal_name:
                    label = lbl
                    break
        
        if label is None:
            logger.warning(f"Signal not found: {signal_name}" + (f" in section {section}" if section else ""))
            return
        
        # Auto-format based on value type if format_str not provided
        if format_str is None:
            if isinstance(value, bool):
                text = "YES" if value else "NO"
            elif isinstance(value, float):
                text = f"{value:.4f}"
            elif isinstance(value, int):
                text = str(value)
            else:
                text = str(value)
        else:
            try:
                text = format(value, format_str)
            except (ValueError, TypeError):
                text = str(value)
        
        label.setText(text)
    
    def update_signal_from_dict(self, data_dict: Dict[str, any]):
        """Update multiple signals from a dictionary.
        
        Args:
            data_dict: Dictionary mapping signal names to values
        """
        for signal_name, value in data_dict.items():
            self.update_signal(signal_name, value)
    
    def clear_all(self):
        """Clear all signal displays (reset to '---')."""
        for label in self.signal_labels.values():
            label.setText("---")
    
    def get_signal_value(self, signal_name: str, section: str = None) -> Optional[str]:
        """Get the current display value of a signal.
        
        Args:
            signal_name: Signal name
            section: Optional section name for disambiguation
            
        Returns:
            Current text value or None if signal doesn't exist
        """
        if section is not None:
            key = (section, signal_name)
            label = self.signal_labels.get(key)
            return label.text() if label else None
        else:
            # Search all sections
            for (sec, sig), label in self.signal_labels.items():
                if sig == signal_name:
                    return label.text()
            return None
    
    def set_signal_color(self, signal_name: str, color: str, section: str = None):
        """Change the color of a signal value label.
        
        Args:
            signal_name: Signal name
            color: Color string (e.g., "#ff0000" for red, "green", etc.)
            section: Optional section name for disambiguation
        """
        label = None
        if section is not None:
            key = (section, signal_name)
            label = self.signal_labels.get(key)
        else:
            # Search all sections
            for (sec, sig), lbl in self.signal_labels.items():
                if sig == signal_name:
                    label = lbl
                    break
        
        if label is None:
            logger.warning(f"Signal not found: {signal_name}")
            return
        
        label.setStyleSheet(f"color: {color}; font-family: 'Courier New'; font-size: 9pt;")
    
    def highlight_signal(self, signal_name: str, highlight: bool = True, section: str = None):
        """Highlight or unhighlight a signal value.
        
        Args:
            signal_name: Signal name
            highlight: Whether to highlight (True) or normal (False)
            section: Optional section name for disambiguation
        """
        if highlight:
            self.set_signal_color(signal_name, "#ff00ff", section)  # Magenta highlight
        else:
            self.set_signal_color(signal_name, "#ffffff", section)  # Normal white
    
    def get_label(self, signal_name: str, section: str = None) -> Optional[QtWidgets.QLabel]:
        """Get direct reference to a signal label.
        
        Args:
            signal_name: Signal name
            section: Optional section name for disambiguation
            
        Returns:
            QLabel reference or None
        """
        if section is not None:
            key = (section, signal_name)
            return self.signal_labels.get(key)
        else:
            # Search all sections
            for (sec, sig), label in self.signal_labels.items():
                if sig == signal_name:
                    return label
            return None


class GridSideTelemetryPanel(TelemetryPanel):
    """Pre-configured telemetry panel for grid-side signals."""
    
    def __init__(self, parent=None):
        """Initialize grid-side telemetry panel."""
        super().__init__(parent, max_width=500)
        self._setup_grid_layout()
    
    def _setup_grid_layout(self):
        """Setup standard grid-side telemetry layout using definitions."""
        self.add_header("TELEMETRY - GRID SIDE")
        
        # Load grid-side definitions
        definitions = get_grid_side_telemetry()
        
        # Add each section
        for section_key, section_data in definitions.items():
            # Use title from definition, fallback to snake_case conversion
            section_title = section_data.get('title', section_key.replace('_', ' ').title())
            fields = [item['name'] for item in section_data['fields']]
            self.add_section(section_title, fields)


class MachineSideTelemetryPanel(TelemetryPanel):
    """Pre-configured telemetry panel for machine-side signals."""
    
    def __init__(self, parent=None):
        """Initialize machine-side telemetry panel."""
        super().__init__(parent, max_width=500)
        self._setup_machine_layout()
    
    def _setup_machine_layout(self):
        """Setup standard machine-side telemetry layout using definitions."""
        self.add_header("TELEMETRY - MACHINE SIDE")
        
        # Load machine-side definitions
        definitions = get_machine_side_telemetry()
        
        # Add each section
        for section_key, section_data in definitions.items():
            # Use title from definition, fallback to snake_case conversion
            section_title = section_data.get('title', section_key.replace('_', ' ').title())
            fields = [item['name'] for item in section_data['fields']]
            self.add_section(section_title, fields)
