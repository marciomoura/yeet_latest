#!/usr/bin/env python3
"""Launch SFC monitor with parameter configuration support.

This monitor includes:
- Left panel: Parameter list with load button and double-click editing
- Center: Four oscilloscope plots (voltage, currents, observer, DC link)
- Right panel: Real-time telemetry dashboard

Usage:
    python monitor_with_parameters.py --host 192.168.2.100 --port 7000
    python monitor_with_parameters.py --decimate 10 --window 500
"""

import sys
import logging
import argparse

# Parse args early to check for verbose flag
parser = argparse.ArgumentParser(add_help=False)
parser.add_argument('--verbose', action='store_true')
args, _ = parser.parse_known_args()

# Configure logging
logging.basicConfig(
    level=logging.DEBUG if args.verbose else logging.WARNING,
    format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
    datefmt='%H:%M:%S'
)

# Import and run
from monitor_client.ui.main_window import main

if __name__ == '__main__':
    sys.exit(main())
