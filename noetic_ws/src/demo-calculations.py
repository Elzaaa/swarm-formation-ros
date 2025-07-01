import numpy as np 
import matplotlib.pyplot as plt
import clusters as cl
from scipy.spatial.distance import cdist
from mpl_toolkits.mplot3d import Axes3D 
from potential_map import calculate_potential_field, slice_map, open_matrix


# def attractive_goal_potential(val, x, y)-> float:
#     goal = (70,110)
#     attractive_gain = 5.0
#     d_goal = np.linalg.norm([x - goal[0], y - goal[1]])
#     return 0.5 * attractive_gain * np.power(d_goal, 2)

# def repulsive_obstacle_potential(x, y) -> float:
#     repulsive_gain=100
#     repulsive_range=5
#     U_rep = 0
#     d_obs = np.linalg.norm([x - obstacles[:,0], y - obstacles_cells[:,1]])
#     for ox, oy in obstacles:
#         print(f"Obstacle at: ({ox}, {oy})")
#         d_obs = np.linalg.norm([x - ox, y - oy])
#         if d_obs < repulsive_range and d_obs != 0:
#             U_rep += 0.5 * repulsive_gain * (1.0/d_obs - 1.0/repulsive_range)**2
#     return U_rep

def draw_heatmap(data):
    data = np.array(data)
    plt.pcolor(data,cmap=plt.cm.Blues)

if __name__ == "__main__":
    map = open_matrix('raw_map.txt')
    sliced_map = slice_map(map, distance=3.5, resolution=0.05)
    obstacles_cells = cl.find_obstacles(sliced_map)
    obstacles_clusters = cl.find_clusters(obstacles_cells)
    
    obstacles, radius = cl.find_clusters_centers(obstacles_clusters, set(obstacles_clusters), obstacles_cells)
 
    potential_map = calculate_potential_field(sliced_map, obstacles, radius, (70,110), repulsive_range=15)

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
