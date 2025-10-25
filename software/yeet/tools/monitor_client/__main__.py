#!/usr/bin/env python3
"""
Monitor Client - Main Entry Point
==================================

Launch the telemetry monitoring application.

Usage:
    python -m monitor_client [options]
    
    Or from the tools directory:
    python monitor.py [options]

Options:
    --host HOST          MCU IP address (default: 192.168.2.100)
    --port PORT          MCU telemetry port (default: 7000)
    --window SECONDS     Time window in seconds (default: 5.0)
    --verbose           Enable verbose logging
"""

import sys
from pathlib import Path

# Add parent directory to path to ensure imports work
sys.path.insert(0, str(Path(__file__).parent.parent))

from monitor_client.ui.main_window import main

if __name__ == "__main__":
    sys.exit(main())
