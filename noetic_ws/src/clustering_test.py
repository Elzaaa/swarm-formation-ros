import numpy as np 
import matplotlib.pyplot as plt
from clusters import find_obstacles, find_clusters, find_clusters_centers

def open_matrix() -> np.ndarray: 
    b = np.loadtxt('map.txt', dtype=int)
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
    obstacles = find_obstacles(sliced_map)
    
    print("Obstacles found at:", obstacles)
    
    # # Example of how to visualize the obstacles on the map
    # plt.imshow(map, cmap='gray', interpolation='nearest')
    # plt.scatter(obstacles[:, 1], obstacles[:, 0], color='red', label='Obstacles')
    # plt.title('Map with Obstacles')
    # plt.xlabel('X Coordinate')
    # plt.ylabel('Y Coordinate')
    # plt.legend()
    # plt.show()

    # inertias = []

    # for i in range(1,11):
    #     kmeans = KMeans(n_clusters=i)
    #     kmeans.fit(obstacles)
    #     inertias.append(kmeans.inertia_)

    # plt.plot(range(1,11), inertias, marker='o')
    # plt.title('Elbow method')
    # plt.xlabel('Number of clusters')
    # plt.ylabel('Inertia')
    # plt.show()

    labels = find_clusters(obstacles)
    unique_labels = set(labels) - {-1}  # Exclude noise label (-1)

    centers = find_clusters_centers(labels, unique_labels, obstacles)
    print("Cluster centers found at:", centers)

    # Number of clusters in labels, ignoring noise if present.
    n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
    n_noise_ = list(labels).count(-1)

    print("Estimated number of clusters: %d" % n_clusters_)
    print("Estimated number of noise points: %d" % n_noise_)

    plt.scatter(obstacles[:, 1], obstacles[:, 0], c=labels)
    plt.show()

