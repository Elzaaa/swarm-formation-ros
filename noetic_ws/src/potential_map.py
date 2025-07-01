import numpy as np 
import matplotlib.pyplot as plt
import clusters as cl
from scipy.spatial.distance import cdist
from mpl_toolkits.mplot3d import Axes3D 


def open_matrix(file_name: str) -> np.ndarray: 
    b = np.loadtxt(file_name, dtype=int)
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

def calculate_potential_field(map_matrix, obstacles, radius, goal, repulsive_gain=100, attractive_gain=0.02, repulsive_range=5):
    rows, cols = map_matrix.shape
    xs, ys = np.indices((rows, cols))
    coords = np.stack([xs, ys], axis=-1).reshape(-1, 2)

    # Attractive potential (vectorized)
    d_goal = np.linalg.norm(coords - np.array(goal), axis=1)
    U_att = 0.5 * attractive_gain * np.power(d_goal, 2)
    U_att = U_att.reshape(rows, cols)

    # Repulsive potential (vectorized for all obstacles)
    # obstacles = np.argwhere(map_matrix == 100)
    # unknowns = np.argwhere(map_matrix == -1)
    all_repulsors = obstacles  # To treat unknowns as obstacles: np.vstack([obstacles, unknowns])
    U_rep = np.zeros((rows * cols,), dtype=float)

    if all_repulsors.shape[0] > 0:
        # Compute distances from all cells to all obstacles
        dists = cdist(coords, all_repulsors)
        mask = (dists < radius + repulsive_range) & (dists != 0)
        with np.errstate(divide='ignore'):
            rep_term = 0.5 * repulsive_gain * (1.0/dists - 1.0/repulsive_range)**2
        rep_term[~mask] = 0
        U_rep = np.sum(rep_term, axis=1)
    U_rep = U_rep.reshape(rows, cols)

    # High potential for obstacles and unknowns
    potential_map = U_rep
    # high_potential_mask = (map_matrix == 100) #| (map_matrix == -1)
    # potential_map[high_potential_mask] = 150

    return potential_map