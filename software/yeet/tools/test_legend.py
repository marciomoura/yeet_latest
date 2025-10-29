#!/usr/bin/env python3
"""Test legend visibility in pyqtgraph"""

import pyqtgraph as pg
from PyQt5 import QtWidgets
import sys

app = QtWidgets.QApplication(sys.argv)

# Create a plot
plot = pg.PlotItem(title='Legend Test')
plot.setLabel('left', 'Value (pu)')
plot.setLabel('bottom', 'Sample')

# Create some curves with names
curve1 = plot.plot(pen=pg.mkPen('r', width=2), name='Signal 1 (Red)')
curve2 = plot.plot(pen=pg.mkPen('g', width=2), name='Signal 2 (Green)')
curve3 = plot.plot(pen=pg.mkPen('b', width=2), name='Signal 3 (Blue)')

# Test 1: Default legend
print("Test 1: Default legend")
legend1 = plot.addLegend()
print(f"  Legend: {legend1}")
print(f"  Legend items: {legend1.items if legend1 else 'None'}")

# Test 2: Check available methods
print("\nTest 2: Available legend methods:")
if legend1:
    methods = [m for m in dir(legend1) if not m.startswith('_')]
    print(f"  Methods: {methods[:10]}")

# Check if text is visible
print("\nTest 3: Checking text rendering...")
if legend1:
    for item_pair in legend1.items:
        print(f"  Item: {item_pair}")
        if len(item_pair) > 1:
            print(f"    Text label: {item_pair[1]}")
            if hasattr(item_pair[1], 'color'):
                print(f"    Has color attr")

print("\nDone!")
