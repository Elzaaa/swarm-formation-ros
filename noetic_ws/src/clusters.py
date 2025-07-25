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

def find_obstacles_info(labels, unique_labels, X):
    """Finds the centers of clusters in the map maunique_labels = set(labels) - {-1}  # Exclude noise label (-1)trix.
    :return: An array of cluster centers.
    """
    centers = []
    radius = []
    for label in unique_labels:
        cluster_points = X[labels == label]
        print(f"Cluster {label} has {len(cluster_points)} points.")
        if len(cluster_points) > 100:
            continue
        center = cluster_points.mean(axis=0)
        distances = np.linalg.norm(cluster_points - center, axis=1)
        centers.append(center)
        radius.append(distances.max())
    return np.array(centers).astype(int), np.array(radius).astype(int)

def find_boundaries(labels, unique_labels, X):
    """Finds the centers of clusters in the map maunique_labels = set(labels) - {-1}  # Exclude noise label (-1)trix.
    :return: An array of cluster centers.
    """
    max_length = 0
    max_label = -1
    for label in unique_labels:
        cluster_points = X[labels == label]  
        if len(cluster_points) > 100:
           if len(cluster_points) > max_length:
                max_length = len(cluster_points)
                max_label = label
                print(f"Boundary Cluster {label} has {len(cluster_points)} points.")

    if (max_label == -1):
        print("No valid boundary cluster found.")
        return np.array([])
    else:
        print(f"Boundary Cluster {max_label} has {len(cluster_points)} points.")
        return np.array(X[labels == max_label]).astype(int)

