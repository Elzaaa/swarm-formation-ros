import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid
import rospy
import time
from potential_map import calculate_potential_field, slice_map
import clusters as cl
from matplotlib.animation import FuncAnimation


class MappingNode:
    def __init__(self):
        # self.node_name = node_name
        # rospy.init_node(self.node_name, anonymous=True)
        # self.map_subscriber = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        # self.rate = rospy.Rate(2)  # 2 Hz
        self.first_run = True  # Flag to check if it's the first run for plotting
        self.potential_map = np.zeros((139,139))  # Initialize potential map
        self.raw_map = np.zeros((384,384)) 
        self.mappable = plt.cm.ScalarMappable(cmap=plt.cm.Blues)
        self.mappable.set_array(self.potential_map)
        self.fig = plt.figure(figsize=(8,6))
        self.ax = self.fig.add_subplot(111, projection='3d')

    def draw_heatmap(self,data):
        data = np.array(data)
        plt.pcolor(data, vmax=500.0, cmap=plt.cm.Blues)

    def plot_init(self):
        X, Y = np.meshgrid(np.arange(self.potential_map.shape[1]), np.arange(self.potential_map.shape[0]))
        self.surf = self.ax.plot_surface(X, Y, self.potential_map, cmap='viridis', edgecolor='k', linewidth=0.5, antialiased=True)
        self.ax.set_xlabel('Y')
        self.ax.set_ylabel('X')
        self.ax.set_zlabel('Potential')
        self.ax.set_title('Artificial Potential Field Surface')
        self.fig.colorbar(self.mappable, shrink=0.5, aspect=10)

    def plot_surface(self,frame : np.ndarray): 
        print("Plotting surface")
        self.ax.clear()  # Clear the previous surface
        X, Y = np.meshgrid(np.arange(self.potential_map.shape[1]), np.arange(self.potential_map.shape[0]))
        self.surf = self.ax.plot_surface(X, Y, self.potential_map, cmap='viridis', edgecolor='k', linewidth=0.5, antialiased=True)
        # self.ax.set_xlabel('Y')
        # self.ax.set_ylabel('X')
        # self.ax.set_zlabel('Potential')
        # self.ax.set_title('Artificial Potential Field Surface')
        # self.fig.colorbar(self.mappable, shrink=0.5, aspect=10)
# Update the mappable with the new
        self.mappable.set_array(self.potential_map.ravel())
        return self.surf
            

    def map_callback(self,msg):
        """
        Callback function to process the OccupancyGrid message and calculate the potential field.
        :param msg: OccupancyGrid message containing the map data.
        """
        print("Entered callback")
        print("Map dimensions: {}x{}".format(msg.info.width, msg.info.height))
        print("Map resolution: {}".format(msg.info.resolution))
        print("Map origin: ({}, {})".format(msg.info.origin.position.x, msg.info.origin.position.y))

        map_matrix = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.raw_map = map_matrix.copy()  # Store the raw map for saving later
        sliced_map = slice_map(map_matrix, distance=3.5, resolution=msg.info.resolution)
        obstacles_cells = cl.find_obstacles(sliced_map)
        obstacles_clusters = cl.find_clusters(obstacles_cells)
        
        obstacles, radius = cl.find_clusters_centers(obstacles_clusters, set(obstacles_clusters), obstacles_cells)
        goal = (70, 110)  # Example goal position
        self.potential_map = calculate_potential_field(sliced_map, obstacles, radius, goal, repulsive_range=15)
        print(self.potential_map.shape)
        # Draw heatmap    
        # draw_heatmap(potential_map)
        # np.savetxt('map.txt', map_matrix, fmt='%d')
        # print("Map saved to map.txt")
        # plt.draw()
    
    def save_map(self):
        """
        Save the current potential map to a file.
        """
        np.savetxt('potential_map.txt', self.potential_map, fmt='%.2f')
        np.savetxt('raw_map.txt', self.raw_map, fmt='%d')
        print("Potential map saved to potential_map.txt")


if __name__ == "__main__":

    try:
        node = MappingNode()
        rospy.init_node('potential_field_calculator')
        rospy.Subscriber('/map', OccupancyGrid, node.map_callback)
        # rospy.spin()
        ani = FuncAnimation(node.fig, node.plot_surface, init_func=node.plot_init, interval=2000,)
        plt.show(block=True)
        node.save_map()
        print("Potential map saved to potential_map.txt")
    except KeyboardInterrupt:
        node.save_map()
        print("ROS node interrupted.")
        rospy.shutdown("ROS node interrupted.")



# ---
# transforms: 
#   - 
#     header: 
#       seq: 0
#       stamp: 
#         secs: 1751293232
#         nsecs: 704368620
#       frame_id: "map"
#     child_frame_id: "odom"
#     transform: 
#       translation: 
#         x: -0.0003384673431575035
#         y: 0.000922874659750596
#         z: 0.0
#       rotation: 
#         x: 0.0
#         y: 0.0
#         z: -0.0007479006023341696
#         w: 0.9999997203223056


# transforms: 
#   - 
#     header: 
#       seq: 0
#       stamp: 
#         secs: 1751293293
#         nsecs: 777533498
#       frame_id: "odom"
#     child_frame_id: "base_footprint"
#     transform: 
#       translation: 
#         x: -0.6811088919639587
#         y: -0.7377679347991943
#         z: 0.0
#       rotation: 
#         x: 0.0
#         y: 0.0
#         z: -0.5176990032196045
#         w: -0.8555628061294556

