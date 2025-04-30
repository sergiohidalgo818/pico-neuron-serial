import os
import argparse
import pandas as pd
import matplotlib.pyplot as plt


def plot_model_data(directory: str, filename: str, name_model: str, separator: str):
    if not os.path.exists("graphs/execution_" + name_model + "/"):
        os.makedirs("graphs/execution_" + name_model + "/")

    print("ploting " + filename)
    data_frame = pd.read_csv(directory + filename, sep=separator, decimal=".")
    data_frame = data_frame[["x", "time"]]

    plt.figure(figsize=(12, 6))

    plt.plot(data_frame["time"], data_frame["x"])
    # plt.xlim((data_frame["time"].min(), data_frame["time"].max()))
    # plt.ylim(top=5)
    #
    name = filename.split(".")[0]

    plt.xlabel("time (s)")
    plt.ylabel("voltage (mV)")
    plt.title(name)

    plt.savefig("graphs/execution_" + name_model + "/" + name + ".png")
    plt.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--directory", type=str, default="data/")
    parser.add_argument("-f", "--filename", type=str, default="", required=True)
    parser.add_argument("-s", "--separator", type=str, default=" ")
    args = parser.parse_args()
    plot_model_data(
        args.directory, args.filename, args.filename.split(".")[0], args.separator
    )
