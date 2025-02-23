import os
import json
import argparse
import plotly.graph_objects as go
import glob
import webbrowser
from typing import List, Dict, Any


def create_plot_for_multidimensional_data(entry: Dict[str, Any], time_timeseries: List[float], figure: go.Figure):
    """Handle multi-dimensional data plotting"""
    dimension = entry.get("dimension", 1)
    if dimension > 1:
        data = entry["data"]
        # Transpose the data to get individual component time series
        components = list(zip(*data))
        # Plot each component
        for i, component in enumerate(components):
            figure.add_trace(
                go.Scatter(
                    x=time_timeseries,
                    y=component,
                    mode="lines",
                    name=f"{entry['name']}[{i}]"
                )
            )
    else:
        # Handle scalar data as before
        figure.add_trace(
            go.Scatter(
                x=time_timeseries,
                y=entry["data"],
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
            # # Remove top-level editable from layout; annotations here can be subject to configuration in the HTML export
            # editable=True,
            hovertext="Double-click to toggle editing. Drag to move."
        )



def create_plot(input_path: str, output_path: str):
    with open(input_path, "r") as file:
        data = json.load(file)

    test_suite: str = data["test_suite"]
    test_name: str = data["test_name"]
    plot_name = f"Suite: {test_suite} - Test: {test_name}"

    # Extract time data
    time_entry = next(entry for entry in data["test_data"] if entry["name"] == "time")
    time_timeseries = time_entry["data"]
    
    # Get all non-time data entries
    data_timeseries = [entry for entry in data["test_data"] if entry["name"] != "time"]

    figure = go.Figure()

    # Create plots for each data entry
    for entry in data_timeseries:
        create_plot_for_multidimensional_data(entry, time_timeseries, figure)

    # Add checker failures (vertical lines and draggable annotations) if present
    if "checker_failures" in data:
        add_checker_failures(data["checker_failures"], figure)

    # Update layout with title at the top and a failure summary annotation at the bottom
    figure.update_layout(
        template="plotly_dark",
        xaxis=dict(
            showgrid=True,
            gridwidth=1,
            gridcolor="LightGray",
            title="Time",
            domain=[0, 0.95]  # reduce plot height slightly to make room
        ),
        yaxis=dict(
            showgrid=True,
            gridwidth=1,
            gridcolor="LightGray",
            title="Values"
        ),
        title=dict(
            text=plot_name,
            font=dict(size=14),
            xanchor='left',
            x=0,
            yanchor='top',
            y=0.98
        ),
        showlegend=True,
        legend=dict(
            yanchor="top",
            y=0.99,
            xanchor="left",
            x=0.01,
            bgcolor="rgba(0,0,0,0.3)"
        ),
        margin=dict(t=150, b=200, l=50, r=50)  # Increase bottom margin to fully show the summary annotation
    )

    # Write the plot to file and pass a config making annotations editable
    figure.write_html(file=output_path, config={"editable": True})


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot data from JSON file.")
    parser.add_argument("input_dir", type=str, help="Path to the input JSON file.")
    parser.add_argument("output_dir", type=str, help="Path to save the output HTML file.")
    # Add a parsing argument to move the html and json files to a new directory
    parser.add_argument("--move", type=str, help="Path to move the input JSON and output HTML files.")
    args = parser.parse_args()

    input_files = glob.glob(os.path.join(args.input_dir, "testing_*.json"))

    # For every input testing file found, create a plot.
    for input_file in input_files:
        output_file = os.path.join(
            args.output_dir, os.path.basename(input_file).replace(".json", ".html")
        )
        create_plot(input_file, output_file)

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
