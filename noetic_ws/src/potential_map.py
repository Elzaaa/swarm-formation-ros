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

def calculate_potential_field(map_matrix, obstacles, radius, boundaries, goal, repulsive_gain=5500, attractive_gain=5.0, repulsive_range=5):
    rows, cols = map_matrix.shape
    xs, ys = np.indices((rows, cols))
    coords = np.stack([xs, ys], axis=-1).reshape(-1, 2)

    # Attractive potential (vectorized)
    if goal is not None:
        d_goal = np.linalg.norm(coords - np.array(goal), axis=1)
        U_att = 0.5 * attractive_gain * np.power(d_goal, 2)
        U_att = U_att.reshape(rows, cols)
    else : 
        U_att = np.zeros((rows * cols,), dtype=float).reshape(rows, cols)
        
    # Repulsive potential (vectorized for all obstacles)
    # obstacles = np.argwhere(map_matrix == 100)
    # unknowns = np.argwhere(map_matrix == -1)
    all_repulsors = obstacles  # To treat unknowns as obstacles: np.vstack([obstacles, unknowns])
    U_rep = np.zeros((rows * cols,), dtype=float)
    U_rep2 = np.zeros((rows * cols,), dtype=float)
    U_rep_boundaries = np.zeros((rows * cols,), dtype=float)

    if all_repulsors.shape[0] > 0:
        # Compute distances from all cells to all obstacles
        dists = cdist(coords, all_repulsors)
        mask = (radius < dists) & (dists < (repulsive_range+radius))  & (dists != 0)
        with np.errstate(divide='ignore'):
            rep_term = 0.5 * repulsive_gain * (1.0/dists - 1.0/repulsive_range)**2
        rep_term[~mask] = 0
        U_rep = np.sum(rep_term, axis=1)

        mask2 = (dists < radius) & (dists != 0)
        with np.errstate(divide='ignore'):
            rep_term2 = np.ones_like(rep_term) * repulsive_gain
        rep_term2[~mask2] = 0
        U_rep2 = np.sum(rep_term2, axis=1)
    U_rep = U_rep.reshape(rows, cols)
    U_rep2 = U_rep2.reshape(rows, cols)

    # Repulsive potential for boundaries
    print("Boundaries length: ", len(boundaries))
    if boundaries is not None and len(boundaries) > 0:
        boundaries = np.array(boundaries)
        dists_boundaries = cdist(coords, boundaries)
        mask_boundaries = (dists_boundaries < repulsive_range) & (dists_boundaries != 0)
        with np.errstate(divide='ignore'):
            rep_term_boundaries = 0.5 * repulsive_gain * (1.0/dists_boundaries - 1.0/repulsive_range)**2
        rep_term_boundaries[~mask_boundaries] = 0
        U_rep_boundaries = np.sum(rep_term_boundaries, axis=1)
        U_rep_boundaries = U_rep_boundaries.reshape(rows, cols)
    else:
        U_rep_boundaries = np.zeros((rows, cols), dtype=float)

    # High potential for obstacles and unknowns
    potential_map = U_att + U_rep + U_rep2 + U_rep_boundaries
    # 
    # high_potential_mask = (map_matrix == 100) #| (map_matrix == -1)
    # potential_map[high_potential_mask] = 150

    return potential_map

def get_next_step(potential_map, ix, iy):
    motion = get_motion_model()
    minp = potential_map[iy][ix]
    minix, miniy = ix, iy
    for i, _ in enumerate(motion):
        inx = int(ix + motion[i][0])
        iny = int(iy + motion[i][1])
        if inx >= len(potential_map) or iny >= len(potential_map[0]) or inx < 0 or iny < 0:
            p = float('inf')
        else:
           
            p = potential_map[iny][inx]
        if p < minp:
            minp = p
            minix = inx
            miniy = iny
    return miniy, minix

def get_path_to_goal(potential_map, start, goal):
    """
    Get the path from start to goal using gradient descent on the potential map.
    Args:
        potential_map: 2D numpy array representing the potential field
        start: (row, col) tuple for starting index
        goal: (row, col) tuple for goal index
    Returns:   
    """
    path = [start]

    while True: 
        current = path[-1]
        if current == goal:
            break
        next_step = get_next_step(potential_map, current[1], current[0])
        if next_step == current or next_step in path:
            print(next_step, current)
            break
        path.append(next_step)
    return path

def get_motion_model():
    """
    Returns a list of possible motion vectors for 8-connected grid movement.
    Each vector is a tuple (dx, dy) representing the change in x and y coordinates.
    """
    return [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]

def gradient_descent_2d(arr, start, max_steps=1000):
    """
    Perform discrete gradient descent on a 2D array from a given start point.
    Args:
        arr: 2D numpy array (the surface)
        start: (row, col) tuple for starting index
        max_steps: maximum number of steps before stopping
    Returns:
        path: list of (row, col) tuples showing the steps taken
    """
    rows, cols = arr.shape
    pos = start
    path = [pos]
    
    for _ in range(max_steps):
        r, c = pos
        min_val = arr[r, c]
        next_pos = pos
        # Explore 8 neighbors
        for dr in [-1, 0, 1]:
            for dc in [-1, 0, 1]:
                if dr == 0 and dc == 0:
                    continue
                nr, nc = r + dr, c + dc
                if 0 <= nr < rows and 0 <= nc < cols:
                    if arr[nr, nc] < min_val:
                        min_val = arr[nr, nc]
                        next_pos = (nr, nc)
        if next_pos == pos:
            # Local minimum reached
            break
        pos = next_pos
        path.append(pos)
    return path