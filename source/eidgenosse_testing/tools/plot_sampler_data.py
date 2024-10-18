import os
import json
import argparse
import plotly.graph_objects as go
import glob
import webbrowser


def main(input_path, output_path):
    with open(input_path, "r") as file:
        data = json.load(file)

    # Unpack the json array into a dictionary
    test_suite: str = data["test_suite"]
    test_name: str = data["test_name"]

    plot_name = f"Suite: {test_suite} - Test: {test_name}"
    time_timeseries = [
        entry["data"] for entry in data["test_data"] if entry["name"] == "time"
    ][0]
    data_timeseries = [entry for entry in data["test_data"] if entry["name"] != "time"]

    figure = go.Figure()

    for entry in data_timeseries:
        figure.add_trace(
            go.Scatter(
                x=time_timeseries, y=entry["data"], mode="lines", name=entry["name"]
            )
        )

    figure.update_layout(template="plotly_dark")

    # set vertical and horizontal grid
    figure.update_layout(
        xaxis=dict(showgrid=True, gridwidth=1, gridcolor="LightGray"),
        yaxis=dict(showgrid=True, gridwidth=1, gridcolor="LightGray"),
    )

    # set the title and axis labels
    figure.update_layout(
        title=plot_name,
        xaxis_title="Values",
        yaxis_title="Time",
    )

    # write the file in the output path
    figure.write_html(file=output_path)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot data from JSON file.")
    parser.add_argument("input_dir", type=str, help="Path to the input JSON file.")
    parser.add_argument(
        "output_dir", type=str, help="Path to save the output HTML file."
    )
    args = parser.parse_args()

    input_files = glob.glob(os.path.join(args.input_dir, "testing_*.json"))

    for input_file in input_files:
        output_file = os.path.join(
            args.output_dir, os.path.basename(input_file).replace(".json", ".html")
        )

        print(f"Processing input file: {input_file}...")
        main(input_file, output_file)
        print(f"Writing output file: {output_file}...")

        webbrowser.open(output_file)
