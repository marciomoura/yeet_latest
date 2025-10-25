#!/usr/bin/env python3
"""
Monitor Client Launcher
========================

Convenience launcher for the monitor client application.

Usage:
    python monitor.py [options]

Options:
    --host HOST          MCU IP address (default: 192.168.2.100)
    --port PORT          MCU telemetry port (default: 7000)
    --window SECONDS     Time window in seconds (default: 5.0)
    --verbose           Enable verbose logging

Examples:
    # Launch with default settings
    python monitor.py

    # Connect to different host
    python monitor.py --host 192.168.1.100

    # Enable verbose logging
    python monitor.py --verbose

    # Custom time window
    python monitor.py --window 10.0
"""

import sys
from pathlib import Path

# Add current directory to path
sys.path.insert(0, str(Path(__file__).parent))

from monitor_client.ui.main_window import main

if __name__ == "__main__":
    sys.exit(main())
