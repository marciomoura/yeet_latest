#!/usr/bin/env python3
"""Plot telemetry data exported from the monitor tool.

This script is adapted from plot_sampler_data.py to support telemetry exports.
It groups signals by category and creates interactive subplots.
"""

import os
import json
import argparse
import glob
import webbrowser
from typing import List, Dict, Any
from pathlib import Path

import plotly.graph_objects as go
from plotly.subplots import make_subplots


def create_plot_for_multidimensional_data(entry: Dict[str, Any], time_timeseries: List[float], figure: go.Figure, decimate_factor: int = 1):
    """Handle multi-dimensional data plotting with decimation."""
    decimated_time = time_timeseries[::decimate_factor]
    dimension = entry.get("dimension", 1)
    
    if dimension > 1:
        data = entry["data"]
        components = list(zip(*data))
        for i, component in enumerate(components):
            decimated_component = component[::decimate_factor]
            figure.add_trace(
                go.Scatter(
                    x=decimated_time,
                    y=decimated_component,
                    mode="lines",
                    name=f"{entry['name']}[{i}]"
                )
            )
    else:
        decimated_data = entry["data"][::decimate_factor]
        figure.add_trace(
            go.Scatter(
                x=decimated_time,
                y=decimated_data,
                mode="lines",
                name=entry["name"]
            )
        )


def parse_signal_name(name: str) -> tuple[str, str]:
    """Parse signal name into category and signal."""
    if ":" in name:
        parts = name.split(":", 1)
        category = parts[0].strip()
        signal = parts[1].strip()
        return category, signal
    return "General", name


def group_signals_by_category(data_timeseries: List[Dict[str, Any]]) -> Dict[str, List[Dict[str, Any]]]:
    """Group signals by their category prefix."""
    groups = {}
    for entry in data_timeseries:
        category, signal_name = parse_signal_name(entry["name"])
        if category not in groups:
            groups[category] = []
        entry_copy = entry.copy()
        entry_copy["display_name"] = signal_name
        entry_copy["category"] = category
        groups[category].append(entry_copy)
    
    for category in groups:
        groups[category].sort(key=lambda x: x["display_name"])
    return groups


def create_plot_for_multidimensional_data_subplot(entry: Dict[str, Any], time_timeseries: List[float], 
                                                  figure: go.Figure, row: int, col: int, decimate_factor: int = 1, 
                                                  legend_group: str = None):
    """Handle multi-dimensional data plotting with decimation for subplots."""
    decimated_time = time_timeseries[::decimate_factor]
    dimension = entry.get("dimension", 1)
    
    if dimension > 1:
        data = entry["data"]
        components = list(zip(*data))
        for i, component in enumerate(components):
            decimated_component = component[::decimate_factor]
            display_name = f"[{legend_group}] {entry['name']}[{i}]" if legend_group != "General" else f"{entry['name']}[{i}]"
            figure.add_trace(
                go.Scatter(
                    x=decimated_time,
                    y=decimated_component,
                    mode="lines",
                    name=display_name,
                ),
                row=row, col=col
            )
    else:
        decimated_data = entry["data"][::decimate_factor]
        display_name = f"[{legend_group}] {entry['name']}" if legend_group != "General" else entry['name']
        figure.add_trace(
            go.Scatter(
                x=decimated_time,
                y=decimated_data,
                mode="lines",
                name=display_name,
            ),
            row=row, col=col
        )


def create_plot(input_path: str, output_path: str, decimate_factor: int = 1):
    """Create a multi-subplot plot from telemetry JSON."""
    with open(input_path, "r") as file:
        data = json.load(file)

    test_suite: str = data.get("test_suite", "SFC Monitor")
    test_name: str = data.get("test_name", "Telemetry Session")
    plot_name = f"Suite: {test_suite} - Test: {test_name}"

    # Extract time data - check for 'time' or 'timestamp_ms'
    time_entry = next((entry for entry in data["test_data"] if entry["name"] in ["time", "timestamp_ms"]), None)
    if not time_entry:
        print(f"Warning: No time signal found in {input_path}. Using index as time.")
        # Create a dummy time entry based on first available signal
        first_signal = next(entry for entry in data["test_data"] if "data" in entry)
        time_timeseries_raw = list(range(len(first_signal["data"])))
    else:
        time_timeseries_raw = time_entry["data"]

    # Strict increasing time validation (matches plot_sampler_data.py)
    last_valid_index = len(time_timeseries_raw)
    for i in range(1, len(time_timeseries_raw)):
        if time_timeseries_raw[i] < time_timeseries_raw[i-1]:
            last_valid_index = i
            break
    
    time_series = time_timeseries_raw[:last_valid_index]
    
    # Extract data entries
    data_timeseries = []
    for entry in data["test_data"]:
        if entry["name"] not in ["time", "timestamp_ms", "sequence"]:
            entry_copy = entry.copy()
            if "data" in entry_copy:
                entry_copy["data"] = entry_copy["data"][:last_valid_index]
            data_timeseries.append(entry_copy)
    
    if not data_timeseries:
        print(f"No data signals found in {input_path}")
        return

    data_timeseries.sort(key=lambda x: x["name"])
    signal_groups = group_signals_by_category(data_timeseries)
    
    num_subplots = len(signal_groups)
    subplot_titles = sorted(signal_groups.keys())
    
    # Increase vertical spacing and adjust for better visibility (matches plot_sampler_data.py)
    vertical_spacing = max(0.08, 0.15 / num_subplots)
    
    figure = make_subplots(
        rows=num_subplots,
        cols=1,
        subplot_titles=subplot_titles,
        vertical_spacing=vertical_spacing,
        shared_xaxes=True
    )
    
    if decimate_factor > 1:
        print(f"Plotting '{os.path.basename(input_path)}' with decimation factor: {decimate_factor}...")
    
    # Create plots for each category
    for idx, category in enumerate(sorted(signal_groups.keys()), start=1):
        for entry in signal_groups[category]:
            # Use the display_name for the legend
            original_name = entry["name"]
            entry["name"] = entry["display_name"]
            
            # Plot to the appropriate subplot with legend group
            create_plot_for_multidimensional_data_subplot(
                entry, time_series, figure, 
                row=idx, col=1, decimate_factor=decimate_factor,
                legend_group=category
            )
            entry["name"] = original_name

    # Calculate appropriate height (matches plot_sampler_data.py)
    subplot_height = max(400, 600 // num_subplots)
    total_height = subplot_height * num_subplots + 200
    
    # Update layout with enhanced legend (matches plot_sampler_data.py)
    figure.update_layout(
        template="plotly_dark",
        title=dict(
            text=plot_name,
            font=dict(size=16),
            xanchor='center',
            x=0.5,
            yanchor='top',
            y=0.98
        ),
        showlegend=True,
        legend=dict(
            title=dict(
                text="Signals",
                font=dict(size=14, color="white")
            ),
            yanchor="top",
            y=0.95,
            xanchor="left",
            x=1.02,
            bgcolor="rgba(0,0,0,0.8)",
            bordercolor="rgba(255,255,255,0.3)",
            borderwidth=1,
            font=dict(size=11),
            itemsizing='constant',
            tracegroupgap=10,
            itemclick='toggle',
            itemdoubleclick='toggleothers'
        ),
        height=total_height,
        margin=dict(t=80, b=80, l=80, r=300),
        hovermode='x unified'
    )
    
    # Update axes (matches plot_sampler_data.py)
    for idx in range(1, num_subplots + 1):
        figure.update_xaxes(
            title_text="Time", 
            row=idx, col=1,
            showgrid=True,
            gridwidth=1,
            gridcolor="rgba(128,128,128,0.3)",
            showticklabels=True,
            tickmode='auto',
            tickcolor='white',
            tickwidth=1,
            ticklen=5,
            ticks='outside'
        )
        figure.update_yaxes(
            title_text="Values", 
            row=idx, col=1, 
            showgrid=True, 
            gridwidth=1, 
            gridcolor="rgba(128,128,128,0.3)",
            zeroline=True,
            zerolinewidth=1,
            zerolinecolor="rgba(128,128,128,0.5)",
            showticklabels=True,
            tickcolor='white',
            tickwidth=1,
            ticklen=5,
            ticks='outside'
        )
    
    # Add custom buttons (matches plot_sampler_data.py)
    figure.update_layout(
        updatemenus=[
            dict(
                type="buttons",
                direction="left",
                buttons=[
                    dict(args=["visible", "legendonly"], label="Hide All", method="restyle"),
                    dict(args=["visible", True], label="Show All", method="restyle")
                ],
                pad={"r": 10, "t": 10},
                showactive=False,
                x=0.02, xanchor="left", y=1.02, yanchor="bottom"
            )
        ]
    )
    
    figure.write_html(file=output_path, config={"editable": True})
    print(f"Plot saved to {output_path}")


def main():
    parser = argparse.ArgumentParser(description="Plot telemetry data exported from SFC Monitor.")
    parser.add_argument("input", type=str, help="Path to input JSON file or directory.")
    parser.add_argument("output_dir", type=str, nargs='?', default=".", help="Directory to save HTML outputs.")
    parser.add_argument("--move", type=str, help="Path to move the input JSON and output HTML files.")
    parser.add_argument("--decimate", type=int, default=1, help="Decimation factor (plot every Nth point).")
    parser.add_argument("--open", action="store_true", help="Open plots in browser after creation.")
    
    args = parser.parse_args()
    input_path = Path(args.input)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    if input_path.is_dir():
        input_files = list(input_path.glob("*telemetry_export_*.json"))
    else:
        input_files = [input_path]
        
    for input_file in input_files:
        output_file = output_dir / input_file.with_suffix(".html").name
        create_plot(str(input_file), str(output_file), decimate_factor=args.decimate)
        
        # Move logic (matches plot_sampler_data.py)
        if args.move:
            move_path = Path(args.move)
            move_path.mkdir(parents=True, exist_ok=True)
            dst_input = move_path / input_file.name
            dst_output = move_path / output_file.name
            
            if dst_input.exists(): dst_input.unlink()
            if dst_output.exists(): dst_output.unlink()
            
            # Use shutil to move or just rename if on same drive
            import shutil
            shutil.move(str(input_file), str(dst_input))
            shutil.move(str(output_file), str(dst_output))
            print(f"Moved files to {move_path}")
            if args.open:
                webbrowser.open(f"file://{dst_output.absolute()}")
        elif args.open:
            webbrowser.open(f"file://{output_file.absolute()}")


if __name__ == "__main__":
    main()
