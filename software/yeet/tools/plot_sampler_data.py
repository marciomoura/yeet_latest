import os
import json
import argparse
import plotly.graph_objects as go
import glob
import webbrowser
from typing import List, Dict, Union, Any


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

    # Update layout with dark theme and grid
    figure.update_layout(
        template="plotly_dark",
        xaxis=dict(
            showgrid=True,
            gridwidth=1,
            gridcolor="LightGray",
            title="Time"
        ),
        yaxis=dict(
            showgrid=True,
            gridwidth=1,
            gridcolor="LightGray",
            title="Values"
        ),
        title=plot_name,
        showlegend=True,
        legend=dict(
            yanchor="top",
            y=0.99,
            xanchor="left",
            x=0.01,
            bgcolor="rgba(0,0,0,0.3)"
        )
    )

    # Write the plot to file
    figure.write_html(file=output_path)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot data from JSON file.")
    parser.add_argument("input_dir", type=str, help="Path to the input JSON file.")
    parser.add_argument(
        "output_dir", type=str, help="Path to save the output HTML file."
    )
    # add a parsing argument to move the html and json files to a new directory
    parser.add_argument(
        "--move",
        type=str,
        help="Path to move the input JSON and output HTML files.",
    )
    args = parser.parse_args()

    input_files = glob.glob(os.path.join(args.input_dir, "testing_*.json"))

    # For every input testing file that is found, create a plot
    for input_file in input_files:
        output_file = os.path.join(
            args.output_dir, os.path.basename(input_file).replace(".json", ".html")
        )

        # Create a plot found in the input file json structure
        # Export the plotly plot to the output file
        create_plot(input_file, output_file)

        # If requested to move the files, then move them from the working directory to the destination
        if args.move:
            # If folder to move all files does not exist, then create it
            if not os.path.exists(args.move):
                os.makedirs(args.move)

            dst_input_file: str = os.path.join(args.move, os.path.basename(input_file))
            dst_output_file: str = os.path.join(
                args.move, os.path.basename(output_file)
            )

            # If files exists, then remove them
            if os.path.exists(dst_input_file):
                os.remove(dst_input_file)

            if os.path.exists(dst_output_file):
                os.remove(dst_output_file)

            # Rename from source to destination
            os.rename(input_file, dst_input_file)
            os.rename(output_file, dst_output_file)

            # Open the output file in the browser
            print(f"Opening: {dst_output_file}...")
            webbrowser.open(dst_output_file)
