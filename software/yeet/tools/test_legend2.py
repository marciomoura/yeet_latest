#!/usr/bin/env python3
"""Test legend with explicit addItem"""

import pyqtgraph as pg
from PyQt5 import QtWidgets
import sys

app = QtWidgets.QApplication(sys.argv)

# Create a plot
plot = pg.PlotItem(title='Legend Test')
plot.setLabel('left', 'Value (pu)')
plot.setLabel('bottom', 'Sample')
plot.showGrid(x=True, y=True)

# Create some curves WITH names
curve1 = plot.plot(pen=pg.mkPen('r', width=2), name='Signal 1 (Red)')
curve2 = plot.plot(pen=pg.mkPen('g', width=2), name='Signal 2 (Green)')
curve3 = plot.plot(pen=pg.mkPen('b', width=2), name='Signal 3 (Blue)')

# Add legend
print("Adding legend...")
legend = plot.addLegend(offset=(10, 10))
print(f"Legend items after addLegend: {len(legend.items)}")

# Check if we need to manually add items
print(f"\nChecking plot items...")
for item in plot.listDataItems():
    print(f"  Item: {item}, Name: {item.name()}")

# Try to add items manually
print(f"\nTrying manual addItem...")
legend.clear()
for item in plot.listDataItems():
    if item.name():
        print(f"  Adding to legend: {item.name()}")
        legend.addItem(item, item.name())

print(f"Legend items after manual add: {len(legend.items)}")

# Now test styling
print(f"\nLegend items content:")
for item in legend.items:
    print(f"  {item}")
