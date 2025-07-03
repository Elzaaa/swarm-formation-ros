import numpy as np 
import matplotlib.pyplot as plt
import clusters as cl

def open_matrix() -> np.ndarray: 
    b = np.loadtxt('raw_map.txt', dtype=int)
    return b

def slice_map(map_matrix: np.ndarray, distance: float, resolution: float) -> np.ndarray:
    """    Slices the map matrix based on the given distance and resolution.
    :param map_matrix: The full map matrix.
    :param distance: The distance to slice the map.
    :param resolution: The resolution of the map.
    :return: A sliced portion of the map matrix.
    """
    shape = map_matrix.shape
    start = (int((shape[0] / 2 )-( distance / resolution)), int((shape[1] / 2 )- (distance / resolution)))
    end = (int(shape[0] / 2 + (distance / resolution)), int(shape[1] / 2 + (distance / resolution)))

    return map_matrix[start[0]:end[0], start[1]:end[1]]


if __name__ == "__main__":
    map = open_matrix()
    sliced_map = slice_map(map, distance=3.5, resolution=0.05)
    obstacles = cl.find_obstacles(sliced_map)
    
    print("Obstacles found at:", obstacles)

    labels = cl.find_clusters(obstacles)
    unique_labels = set(labels) - {-1}  # Exclude noise label (-1)

    centers = cl.find_obstacles_info(labels, unique_labels, obstacles)
    print("Cluster centers found at:", centers)

    # Number of clusters in labels, ignoring noise if present.
    n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
    n_noise_ = list(labels).count(-1)

    boundaries = cl.find_boundaries(labels, unique_labels, obstacles)
    print("Boundaries found at:", boundaries)
    print("Boundaries x: ", boundaries[:, 1])

    print("Estimated number of clusters: %d" % n_clusters_)
    print("Estimated number of noise points: %d" % n_noise_)

    plt.scatter(obstacles[:, 1], obstacles[:, 0], c=labels)
    plt.show()

