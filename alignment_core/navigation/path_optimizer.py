import numpy as np


def compute_path_distances(path):

    distances = []

    for i in range(len(path) - 1):

        p1 = path[i]
        p2 = path[i + 1]

        d = np.linalg.norm(p2 - p1)

        distances.append(d)

    return np.array(distances)


def compute_total_distance(path):

    distances = compute_path_distances(path)

    return float(np.sum(distances))