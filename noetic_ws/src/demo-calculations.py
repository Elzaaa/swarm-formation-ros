import numpy as np 
import matplotlib.pyplot as plt
import clusters as cl
from scipy.spatial.distance import cdist
from mpl_toolkits.mplot3d import Axes3D 


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

def calculate_potential_field(map_matrix, obstacles, goal, repulsive_gain=100, attractive_gain=0.02, repulsive_range=5):
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
        mask = (dists < repulsive_range) & (dists != 0)
        with np.errstate(divide='ignore'):
            rep_term = 0.5 * repulsive_gain * (1.0/dists - 1.0/repulsive_range)**2
        rep_term[~mask] = 0
        U_rep = np.sum(rep_term, axis=1)
    U_rep = U_rep.reshape(rows, cols)

    # High potential for obstacles and unknowns
    potential_map = U_att + U_rep
    # high_potential_mask = (map_matrix == 100) | (map_matrix == -1)
    # potential_map[high_potential_mask] = 150

    return potential_map

def attractive_goal_potential(val, x, y)-> float:
    goal = (70,110)
    attractive_gain = 5.0
    d_goal = np.linalg.norm([x - goal[0], y - goal[1]])
    return 0.5 * attractive_gain * np.power(d_goal, 2)

def repulsive_obstacle_potential(x, y) -> float:
    repulsive_gain=100
    repulsive_range=5
    U_rep = 0
    d_obs = np.linalg.norm([x - obstacles[:,0], y - obstacles_cells[:,1]])
    for ox, oy in obstacles:
        print(f"Obstacle at: ({ox}, {oy})")
        d_obs = np.linalg.norm([x - ox, y - oy])
        if d_obs < repulsive_range and d_obs != 0:
            U_rep += 0.5 * repulsive_gain * (1.0/d_obs - 1.0/repulsive_range)**2
    return U_rep

def draw_heatmap(data):
    data = np.array(data)
    plt.pcolor(data,cmap=plt.cm.Blues)

if __name__ == "__main__":
    map = open_matrix()
    sliced_map = slice_map(map, distance=3.5, resolution=0.05)
    obstacles_cells = cl.find_obstacles(sliced_map)
    obstacles_clusters = cl.find_clusters(obstacles_cells)
    
    obstacles = cl.find_clusters_centers(obstacles_clusters, set(obstacles_clusters), obstacles_cells)
 
    potential_map = calculate_potential_field(sliced_map, obstacles, (70,110), repulsive_range=15)
    # rows, cols = np.indices(sliced_map.shape)
    # indices_array = np.dstack((rows,cols))
    # print(indices_array)
    # print(indices_array.shape)
    # # att_func = np.vectorize(attractive_goal_potential)
    # # attractive_map = att_func(sliced_map,cols, rows)
    # # rep_func = np.vectorize(repulsive_obstacle_potential)
    # repulsive_map = rep_func(rows, cols)
    # draw_heatmap(potential_map)
    # plt.show()
    fig = plt.figure(figsize=(8,6))
    ax = fig.add_subplot(111, projection='3d')
    X, Y = np.meshgrid(np.arange(potential_map.shape[1]), np.arange(potential_map.shape[0]))
    surf = ax.plot_surface(X, Y, potential_map, cmap='viridis', edgecolor='k', linewidth=0.5, antialiased=True)
    ax.set_xlabel('Y')
    ax.set_ylabel('X')
    ax.set_zlabel('Potential')
    ax.set_title('Artificial Potential Field Surface')
    fig.colorbar(surf, shrink=0.5, aspect=10)
    plt.show()
