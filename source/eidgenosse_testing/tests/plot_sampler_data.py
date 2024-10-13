import json
import unittest
import matplotlib.pyplot as plt


def plot_data():
    with open(
        "C:/Users/CHMABRI9/Repositories/eidgenosse_platform/build/bin/data.json", "r"
    ) as file:
        data = json.load(file)

    time = []
    # Unpack the json array into a dictionary
    for entry in data:
        if entry["name"] == "time":
            time = entry["data"]

    plt.figure(figsize=(10, 5))
    for entry in data:
        if entry["name"] != "time":
            plt.plot(time, entry["data"], label=entry["name"])
    # plt.bar(keys, values, color="blue")
    plt.xlabel("Keys")
    plt.ylabel("Values")
    # plt.title("Data from JSON File")
    plt.show()


if __name__ == "__main__":
    # Run unittests

    # Plot data
    plot_data()
