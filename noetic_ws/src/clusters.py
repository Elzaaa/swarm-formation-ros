import numpy as np 
from sklearn.cluster import DBSCAN



def find_obstacles(map_matrix: np.ndarray) -> np.ndarray:
    """Finds the coordinates of obstacles in the map matrix.
    :param map_matrix: The full map matrix.
    :return: An array of coordinates where obstacles are located.
    """
    return np.argwhere(map_matrix == 100)

def find_clusters(X: np.ndarray) -> np.ndarray:
    """Finds clusters in the given data using KMeans.
    :param X: The data points to cluster.
    :param n_clusters: The number of clusters to find.
    :return: An array of cluster labels for each point.
    """
    db = DBSCAN(eps=3, min_samples=5).fit(X)
    return db.labels_

def find_clusters_centers(labels, unique_labels, X) -> np.ndarray:
    """Finds the centers of clusters in the map maunique_labels = set(labels) - {-1}  # Exclude noise label (-1)trix.
    :return: An array of cluster centers.
    """
    centers = []
    for label in unique_labels:
        cluster_points = X[labels == label]
        if len(cluster_points) > 50:
            continue
        print(f"Cluster {label} has {len(cluster_points)} points.")

        center = cluster_points.mean(axis=0)
        centers.append(center)
    return np.array(centers).astype(int)

