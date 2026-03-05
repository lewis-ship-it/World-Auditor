import json
import numpy as np


def load_map(file):
    """
    Loads a map or path file uploaded by the user.

    Expected format:
    [
        {"x":0,"y":0},
        {"x":5,"y":3},
        {"x":10,"y":6}
    ]
    """

    data = json.load(file)

    path = []

    for p in data:
        path.append((float(p["x"]), float(p["y"])))

    return np.array(path)