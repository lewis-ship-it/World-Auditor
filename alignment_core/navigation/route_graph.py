import matplotlib.pyplot as plt
import numpy as np


def generate_speed_graph(distances, speeds):

    cumulative = np.cumsum(distances)

    plt.figure()

    plt.plot(cumulative, speeds)

    plt.xlabel("Distance")
    plt.ylabel("Max Safe Speed")

    plt.title("Robot Speed Profile")

    plt.grid(True)

    return plt