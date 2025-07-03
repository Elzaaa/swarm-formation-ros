import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
import rospy
import time
from potential_map import calculate_potential_field, slice_map, gradient_descent_2d, get_path_to_goal
import clusters as cl
from matplotlib.animation import FuncAnimation


class MappingNode:
    def __init__(self):

        self.goal = None
        self.map_origin_x = -10
        self.map_origin_y = -10
        self.map_resolution = 0.05

        self.path = []
        self.potential_map = np.zeros((139,139))  # Initialize potential map
        self.raw_map = np.zeros((384,384)) 

        # self.fig = plt.figure(figsize=(8,6))
        # self.ax1 = self.fig.add_subplot(111, projection='3d')
        # self.ax2 = self.fig.add_subplot(433)

        self.fig, (self.ax1, self.ax2) = plt.subplots(2)

    def draw_heatmap(self,data, path):
        data = np.array(data)
        c = self.ax2.pcolor(data,cmap='viridis')
        self.ax2.set_title('Potential Field Heatmap')

        if len(path) > 0:
            ys, xs = zip(*path)  # Unzip the path coordinates
            self.ax2.plot(xs, ys,marker='*', color='red', label='Path')
        return c
    
    def plot_potential_map(self):
        X, Y = np.meshgrid(np.arange(self.potential_map.shape[1]), np.arange(self.potential_map.shape[0]))
        self.surf = self.ax1.plot_surface(X, Y, self.potential_map, cmap='viridis', edgecolor='k', linewidth=0.5, antialiased=True)

    def plot_init(self):

        c = self.draw_heatmap(self.potential_map, self.path)
        self.fig.colorbar(c, ax=self.ax2)
        # self.plot_potential_map()

    def plot_surface(self,frame : np.ndarray): 
        print("Plotting surface")
        self.ax1.clear()  # Clear the previous surface
        self.ax2.clear()  # Clear the previous surface
 
        self.draw_heatmap(self.potential_map, self.path)
        # self.plot_potential_map()

    
    def set_goal(self,msg: PoseStamped): 
        print("Goal set to : ({},{})".format(msg.pose.position.x, msg.pose.position.y))
        x, y = self.__pose_to_grid_index(msg.pose.position.x, msg.pose.position.y)
        print("Goal grid index: ({},{})".format(x,y))
        self.goal = (y,x)
        self.potential_map = calculate_potential_field(self.sliced_map, self.obstacles, self.radius, self.boundaries, self.goal, repulsive_range=10)
        # self.path = gradient_descent_2d(self.potential_map, self.__pose_to_grid_index(0,0))
        self.path = get_path_to_goal(self.potential_map, self.__pose_to_grid_index(0,0), self.goal)
        print("Path found: ", self.path)
        self.path_pose = [(self.__grid_index_to_pose(x, y)) for y, x in self.path]  # Convert grid indices to poses
        print("Path in poses: ", self.path_pose)
        
            
    def __pose_to_grid_index(self, x: float, y: float) -> tuple: 
        x_index = (int)((x-self.map_origin_x)/self.map_resolution)
        y_index = (int)((y-self.map_origin_y)/self.map_resolution)
        return x_index, y_index

    def __grid_index_to_pose(self, x: int, y: int) -> tuple:
        x_pose = self.map_origin_x + x * self.map_resolution
        y_pose = self.map_origin_y + y * self.map_resolution
        return x_pose, y_pose
    
    def map_callback(self,msg):
        """
        Callback function to process the OccupancyGrid message and calculate the potential field.
        :param msg: OccupancyGrid message containing the map data.
        """
        print("Entered callback")
        print("Map dimensions: {}x{}".format(msg.info.width, msg.info.height))
        print("Map resolution: {}".format(msg.info.resolution))
        print("Map origin: ({}, {})".format(msg.info.origin.position.x, msg.info.origin.position.y))

        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        self.map_resolution = msg.info.resolution
        
        # Convert the OccupancyGrid data to a numpy array and reduce the area of interest
        map_matrix = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.raw_map = map_matrix.copy()  # Store the raw map for saving later
        self.sliced_map = slice_map(map_matrix, distance=3.5, resolution=msg.info.resolution)
        self.map_origin_x = -3.5
        self.map_origin_y = -3.5

        # Find obstacles in the interest area and boundaries
        obstacles_cells = cl.find_obstacles(self.sliced_map)
        obstacles_clusters = cl.find_clusters(obstacles_cells)
        unique_labels = set(obstacles_clusters) - {-1}  # Exclude noise label (-1)
        obstacles, radius = cl.find_obstacles_info(obstacles_clusters, unique_labels, obstacles_cells)
        boundaries = cl.find_boundaries(obstacles_clusters, unique_labels, obstacles_cells)
        self.obstacles = obstacles
        self.radius = radius
        self.boundaries = boundaries
        self.potential_map = calculate_potential_field(self.sliced_map, obstacles, radius, boundaries, self.goal, repulsive_range=15)
    
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
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, node.set_goal )
        rospy.Subscriber('/map', OccupancyGrid, node.map_callback)
        # rospy.spin()
        ani = FuncAnimation(node.fig, node.plot_surface, init_func=node.plot_init, interval=2000,)
        plt.show(block=True)
        # node.save_map()
        # print("Potential map saved to potential_map.txt")
    except KeyboardInterrupt:
        # node.save_map()
        print("ROS node interrupted.")
        rospy.shutdown("ROS node interrupted.")