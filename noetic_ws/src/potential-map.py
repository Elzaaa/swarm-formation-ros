import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid
import rospy
import time

def calculate_potential_field(map_matrix, goal, repulsive_gain=100, attractive_gain=10, repulsive_range=5):
    rows, cols = map_matrix.shape
    potential_map = np.zeros_like(map_matrix, dtype=float)

    # Coordinates of all obstacles and unknowns
    obstacles = np.argwhere(map_matrix == 100)
    unknowns = np.argwhere(map_matrix == -1)  # Optional: treat as obstacles

    print("Obstacles found at:", obstacles)
    # Precompute potential for each cell
    for x in range(150):
        for y in range(150):
            # Attractive potential (to goal)
            d_goal = np.linalg.norm([x - goal[0], y - goal[1]])
            U_att = 0.5 * attractive_gain * np.power(d_goal, 2)

            # Repulsive potential (from obstacles)
            U_rep = 0
            for ox, oy in obstacles:
                d_obs = np.linalg.norm([x - ox, y - oy])
                if d_obs < repulsive_range and d_obs != 0:
                    U_rep += 0.5 * repulsive_gain * (1.0/d_obs - 1.0/repulsive_range)**2
            # Optionally treat unknowns as obstacles (uncomment below)
            # for ux, uy in unknowns:
            #     d_un = np.linalg.norm([x - ux, y - uy])
            #     if d_un < repulsive_range and d_un != 0:
            #         U_rep += 0.5 * repulsive_gain * (1.0/d_un - 1.0/repulsive_range)**2

            # Obstacles and unknowns themselves get a very high potential
            if map_matrix[x, y] == 100 or map_matrix[x, y] == -1:
                potential_map[x, y] = 500
            else:
                potential_map[x, y] = U_att + U_rep
            print(x,y)
    return potential_map

def draw_heatmap(data):
    data = np.array(data)
    plt.pcolor(data, vmax=100.0, cmap=plt.cm.Blues)

def map_callback(msg):
    """
    Callback function to process the OccupancyGrid message and calculate the potential field.
    :param msg: OccupancyGrid message containing the map data.
    """
    print("Entered callback")
    print("Map dimensions: {}x{}".format(msg.info.width, msg.info.height))
    print("Map resolution: {}".format(msg.info.resolution))
    print("Map origin: ({}, {})".format(msg.info.origin.position.x, msg.info.origin.position.y))
    map_matrix = np.array(msg.data).reshape((msg.info.height, msg.info.width))
    # goal = (2, 4)  # Example goal position
    # start = time.time()
    # potential_map = calculate_potential_field(map_matrix, goal)
    # print("Potential field calculated.")
    # end = time.time()
    # print(end - start, "seconds to calculate potential field")
    # Draw heatmap
    
    draw_heatmap(map_matrix)
    np.savetxt('map.txt', map_matrix, fmt='%d')
    print("Map saved to map.txt")
    plt.show()


if __name__ == "__main__":

    rospy.init_node('potential_field_calculator')
    rospy.Subscriber('/map', OccupancyGrid, map_callback)
    rospy.spin()
    # Uncomment the following lines to run the example directly without ROS
    # Example map
    # -1: unknown, 0: free, 100: obstacle
    # map_matrix = np.array([
    #     [0,    0,    0,    0,   0,   0,    0,   0,   0, -1],
    #     [0,  100,    0,  100,   0,   0,    0,   0,   0, -1],
    #     [0,    0,    0,    0,   0,   0,    0,   0,   0, -1],
    #     [0,  100,  100,    0,   0,   0,    0,   0,   0, -1],
    #     [0,  100,  100,    0,   0,   0,    0,   0,   0, -1],
    #     [0,    0,    0,    0,   0,   0,    0,   0,   0, -1],
    #     [0,    0,    0,  100, 100,   0,    0, 100, 100, -1],
    #     [0,    0,    0,  100, 100,   0,    0, 100, 100, -1],
    #     [0,    0,    0,    0,   0,  -1,   -1,  -1,  -1, -1]
    # ])

    # goal = (0, 8)  # bottom right corner as goal

    # potential_map = calculate_potential_field(map_matrix, goal)
    # print("map calculated")
    # draw_heatmap(potential_map)
    # plt.show()

    # np.set_printoptions(precision=2, suppress=True)
    # print("Potential Field Map:")
    # print(potential_map)
    # fig = plt.figure(figsize=(8,6))
    # ax = fig.add_subplot(111, projection='3d')
    # X, Y = np.meshgrid(np.arange(map_matrix.shape[1]), np.arange(map_matrix.shape[0]))
    # surf = ax.plot_surface(X, Y, potential_map, cmap='viridis', edgecolor='k', linewidth=0.5, antialiased=True)
    # ax.set_xlabel('Y')
    # ax.set_ylabel('X')
    # ax.set_zlabel('Potential')
    # ax.set_title('Artificial Potential Field Surface')
    # fig.colorbar(surf, shrink=0.5, aspect=10)
    # plt.show()