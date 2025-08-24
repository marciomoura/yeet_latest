import os
import json
import argparse
import plotly.graph_objects as go
import glob
import webbrowser
from typing import List, Dict, Any


# MODIFIED: Added decimate_factor parameter
def create_plot_for_multidimensional_data(entry: Dict[str, Any], time_timeseries: List[float], figure: go.Figure, decimate_factor: int = 1):
    """Handle multi-dimensional data plotting with decimation."""
    # Apply decimation to the shared time data
    decimated_time = time_timeseries[::decimate_factor]

    dimension = entry.get("dimension", 1)
    if dimension > 1:
        data = entry["data"]
        # Transpose the data to get individual component time series
        components = list(zip(*data))
        # Plot each component
        for i, component in enumerate(components):
            # Apply decimation to each component's data
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
        # Handle scalar data as before, but with decimation
        decimated_data = entry["data"][::decimate_factor]
        figure.add_trace(
            go.Scatter(
                x=decimated_time,
                y=decimated_data,
                mode="lines",
                name=entry["name"]
            )
        )


def add_checker_failures(failures: List[Dict[str, Any]], figure: go.Figure):
    """Add vertical lines and draggable annotations for checker failures at the top region."""
    if not failures:
        return

    # For each failure, add a vertical red dashed line and an annotation (the annotation itself is draggable)
    for failure in failures:
        figure.add_vline(
            x=failure["time"],
            line_dash="dash",
            line_color="red",
            line_width=2,
            opacity=0.7
        )
        
        figure.add_annotation(
            x=failure["time"],
            y=0.92,  # Positioned near the top (below the title)
            xref="x",
            yref="paper",
            text=f"{failure['name']}: {failure['message']}",
            showarrow=True,
            arrowhead=2,
            arrowsize=1,
            arrowwidth=2,
            arrowcolor="red",
            bgcolor="rgba(255,0,0,0.1)",
            bordercolor="red",
            borderwidth=2,
            borderpad=4,
            yanchor="bottom",
            hovertext="Double-click to toggle editing. Drag to move."
        )


def parse_signal_name(name: str) -> tuple[str, str]:
    """Parse signal name into category and signal.
    
    Args:
        name: Signal name, potentially with category separator ":":
        
    Returns:
        Tuple of (category, signal_name). If no separator, category is None.
    """
    if ":" in name:
        parts = name.split(":", 1)
        category = parts[0].strip()
        signal = parts[1].strip()
        return category, signal
    return None, name


def group_signals_by_category(data_timeseries: List[Dict[str, Any]]) -> Dict[str, List[Dict[str, Any]]]:
    """Group signals by their category prefix.
    
    Args:
        data_timeseries: List of data entries
        
    Returns:
        Dictionary mapping category names to lists of data entries
    """
    groups = {}
    
    for entry in data_timeseries:
        category, signal_name = parse_signal_name(entry["name"])
        
        if category is None:
            category = "General"  # Default category for ungrouped signals
        
        if category not in groups:
            groups[category] = []
        
        # Create a copy of the entry with updated name
        entry_copy = entry.copy()
        entry_copy["display_name"] = signal_name
        entry_copy["category"] = category
        groups[category].append(entry_copy)
    
    # Sort signals within each category alphabetically
    for category in groups:
        groups[category].sort(key=lambda x: x["display_name"])
    
    return groups


# MODIFIED: Added decimate_factor parameter
def create_plot(input_path: str, output_path: str, decimate_factor: int = 1):
    with open(input_path, "r") as file:
        data = json.load(file)

    test_suite: str = data["test_suite"]
    test_name: str = data["test_name"]
    plot_name = f"Suite: {test_suite} - Test: {test_name}"

    # Extract time data
    time_entry = next(entry for entry in data["test_data"] if entry["name"] == "time")
    time_timeseries = time_entry["data"]
    
    # Get all non-time data entries and sort alphabetically
    data_timeseries = [entry for entry in data["test_data"] if entry["name"] != "time"]
    data_timeseries.sort(key=lambda x: x["name"])
    
    # Group signals by category
    signal_groups = group_signals_by_category(data_timeseries)
    
    # Create subplots based on number of categories
    from plotly.subplots import make_subplots
    
    num_subplots = len(signal_groups)
    subplot_titles = sorted(signal_groups.keys())
    
    # Increase vertical spacing and adjust for better visibility
    vertical_spacing = max(0.08, 0.15 / num_subplots)  # Dynamic spacing based on number of subplots
    
    figure = make_subplots(
        rows=num_subplots,
        cols=1,
        subplot_titles=subplot_titles,
        vertical_spacing=vertical_spacing,
        shared_xaxes=True
    )
    
    # Inform the user if decimation is active
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
                entry, time_timeseries, figure, 
                row=idx, col=1, decimate_factor=decimate_factor,
                legend_group=category
            )
            
            # Restore original name
            entry["name"] = original_name

    # Add checker failures to all subplots
    if "checker_failures" in data:
        add_checker_failures_to_subplots(data["checker_failures"], figure, num_subplots)

    # Calculate appropriate height based on number of subplots
    # Minimum 400px per subplot, with additional space for titles and margins
    subplot_height = max(400, 600 // num_subplots)
    total_height = subplot_height * num_subplots + 200  # Extra space for titles and margins
    
    # Update layout with enhanced legend
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
            x=1.02,  # Position legend to the right of the plot area
            bgcolor="rgba(0,0,0,0.8)",
            bordercolor="rgba(255,255,255,0.3)",
            borderwidth=1,
            font=dict(size=11),
            itemsizing='constant',
            tracegroupgap=10,  # Add gap between trace groups
            itemclick='toggle',  # Single click toggles individual trace
            itemdoubleclick='toggleothers'  # Double click isolates trace
        ),
        height=total_height,
        margin=dict(t=80, b=80, l=80, r=300),  # Increased right margin for legend space
        hovermode='x unified'  # Show all y-values at the same x position
    )
    
    # Update x-axis labels and ticks for ALL subplots
    for idx in range(1, num_subplots + 1):
        figure.update_xaxes(
            title_text="Time", 
            row=idx, 
            col=1,
            showgrid=True,
            gridwidth=1,
            gridcolor="rgba(128,128,128,0.3)",
            showticklabels=True,  # Show tick labels
            tickmode='auto',      # Automatic tick placement
            tickcolor='white',    # Color of tick marks
            tickwidth=1,          # Width of tick marks
            ticklen=5,            # Length of tick marks
            ticks='outside'       # Position ticks outside the plot area
        )
    
    # Update y-axis labels for each subplot with better formatting
    for idx in range(1, num_subplots + 1):
        figure.update_yaxes(
            title_text="Values", 
            row=idx, 
            col=1, 
            showgrid=True, 
            gridwidth=1, 
            gridcolor="rgba(128,128,128,0.3)",
            zeroline=True,
            zerolinewidth=1,
            zerolinecolor="rgba(128,128,128,0.5)",
            showticklabels=True,  # Show tick labels on y-axis too
            tickcolor='white',    # Color of tick marks
            tickwidth=1,          # Width of tick marks
            ticklen=5,            # Length of tick marks
            ticks='outside'       # Position ticks outside the plot area
        )
    
    # Add custom buttons for better signal control
    figure.update_layout(
        updatemenus=[
            dict(
                type="buttons",
                direction="left",
                buttons=list([
                    dict(
                        args=["visible", "legendonly"],
                        label="Hide All",
                        method="restyle"
                    ),
                    dict(
                        args=["visible", True],
                        label="Show All",
                        method="restyle"
                    )
                ]),
                pad={"r": 10, "t": 10},
                showactive=False,
                x=0.02,
                xanchor="left",
                y=1.02,
                yanchor="bottom"
            )
        ]
    )
    
    # Write the plot to file
    figure.write_html(file=output_path, config={"editable": True})


def create_plot_for_multidimensional_data_subplot(entry: Dict[str, Any], time_timeseries: List[float], 
                                                  figure: go.Figure, row: int, col: int, decimate_factor: int = 1, 
                                                  legend_group: str = None):
    """Handle multi-dimensional data plotting with decimation for subplots."""
    # Apply decimation to the shared time data
    decimated_time = time_timeseries[::decimate_factor]

    dimension = entry.get("dimension", 1)
    if dimension > 1:
        data = entry["data"]
        components = list(zip(*data))
        for i, component in enumerate(components):
            decimated_component = component[::decimate_factor]
            # Add category prefix to name for clarity in legend
            display_name = f"[{legend_group}] {entry['name']}[{i}]" if legend_group != "General" else f"{entry['name']}[{i}]"
            figure.add_trace(
                go.Scatter(
                    x=decimated_time,
                    y=decimated_component,
                    mode="lines",
                    name=display_name,
                    # Remove legendgroup to allow individual control
                    # legendgroup=legend_group,
                    # legendgrouptitle_text=legend_group if i == 0 else None
                ),
                row=row, col=col
            )
    else:
        decimated_data = entry["data"][::decimate_factor]
        # Add category prefix to name for clarity in legend
        display_name = f"[{legend_group}] {entry['name']}" if legend_group != "General" else entry['name']
        figure.add_trace(
            go.Scatter(
                x=decimated_time,
                y=decimated_data,
                mode="lines",
                name=display_name,
                # Remove legendgroup to allow individual control
                # legendgroup=legend_group,
                # legendgrouptitle_text=legend_group
            ),
            row=row, col=col
        )


def add_checker_failures_to_subplots(failures: List[Dict[str, Any]], figure: go.Figure, num_subplots: int):
    """Add vertical lines for checker failures across all subplots."""
    if not failures:
        return

    for failure in failures:
        # Add vertical line to all subplots
        for idx in range(1, num_subplots + 1):
            figure.add_vline(
                x=failure["time"],
                line_dash="dash",
                line_color="red",
                line_width=2,
                opacity=0.7,
                row=idx, col=1
            )
        
        # Add annotation only to the first subplot
        figure.add_annotation(
            x=failure["time"],
            y=1,
            xref="x",
            yref="y domain",
            text=f"{failure['name']}: {failure['message']}",
            showarrow=True,
            arrowhead=2,
            arrowsize=1,
            arrowwidth=2,
            arrowcolor="red",
            bgcolor="rgba(255,0,0,0.1)",
            bordercolor="red",
            borderwidth=2,
            borderpad=4,
            yanchor="bottom",
            hovertext="Double-click to toggle editing. Drag to move.",
            row=1, col=1
        )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot data from JSON file.")
    parser.add_argument("input_dir", type=str, help="Path to the input JSON file.")
    parser.add_argument("output_dir", type=str, help="Path to save the output HTML file.")
    # Add a parsing argument to move the html and json files to a new directory
    parser.add_argument("--move", type=str, help="Path to move the input JSON and output HTML files.")

    # NEW: Add a command-line argument for the decimation factor
    parser.add_argument(
        "--decimate",
        type=int,
        default=1,
        help="Decimation factor for plotting. Plots every Nth point. Default is 1 (no decimation)."
    )
    
    args = parser.parse_args()

    # Validate the decimation factor
    if args.decimate < 1:
        raise ValueError("Decimation factor must be an integer greater than or equal to 1.")

    input_files = glob.glob(os.path.join(args.input_dir, "testing_*.json"))

    # For every input testing file found, create a plot.
    for input_file in input_files:
        output_file = os.path.join(
            args.output_dir, os.path.basename(input_file).replace(".json", ".html")
        )
        # MODIFIED: Pass the decimation factor from args to create_plot
        create_plot(input_file, output_file, decimate_factor=args.decimate)

        # If requested, move the files from the working directory to the destination
        if args.move:
            if not os.path.exists(args.move):
                os.makedirs(args.move)
            dst_input_file = os.path.join(args.move, os.path.basename(input_file))
            dst_output_file = os.path.join(args.move, os.path.basename(output_file))
            if os.path.exists(dst_input_file):
                os.remove(dst_input_file)
            if os.path.exists(dst_output_file):
                os.remove(dst_output_file)
            os.rename(input_file, dst_input_file)
            os.rename(output_file, dst_output_file)
            print(f"Opening: {dst_output_file}...")
            webbrowser.open(dst_output_file)