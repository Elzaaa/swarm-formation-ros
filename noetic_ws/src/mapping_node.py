import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import rospy
import time
from potential_map import calculate_potential_field, slice_map, gradient_descent_2d, get_path_to_goal, get_next_step
import clusters as cl
from matplotlib.animation import FuncAnimation

class MappingNode:
    def __init__(self):

        self.goal = None
        self.map_origin_x = -10
        self.map_origin_y = -10
        self.map_resolution = 0.05

        self.path = []
        self.path_pose = np.array([])
        self.potential_map = np.zeros((139,139))  # Initialize potential map
        self.raw_map = np.zeros((384,384)) 

        self.path_ready = False

        rospy.init_node('potential_field_calculator')
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.set_goal )
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.set_initial_pose)

        # self.fig = plt.figure(figsize=(8,6))
        # self.ax1 = self.fig.add_subplot(111, projection='3d')
        # self.ax2 = self.fig.add_subplot(433)

        self.fig, (self.ax1, self.ax2) = plt.subplots(1,2, figsize=(12, 6))

    def run_animation(self, block: bool):
        ani = FuncAnimation(self.fig, self.plot_surface, init_func=self.plot_init, interval=2000)
        plt.show(block=block)

    def draw_heatmap(self,data, path):
        c = self.ax2.pcolor(data,cmap='viridis', vmax=5000)
        self.ax2.set_title('Potential Field Occupancy Grid')
        self.ax2.set_xlim(125,275)
        self.ax2.set_ylim(125,275)

        if len(path) > 0:
            ys, xs = zip(*path)  # Unzip the path coordinates
            self.ax2.plot(xs, ys,marker='*', color='red', label='Path')
        return c
    
    def plot_map(self):
        self.ax1.set_title('Map with Obstacles and Path')
        self.ax1.set_xlabel('X (m)')
        self.ax1.set_ylabel('Y (m)')
        self.ax1.grid(True)
        self.ax1.scatter(self.obstacles_cells_pose[:,0], self.obstacles_cells_pose[:,1], c='black', label='Obstacles')
        if len(self.path_pose) > 0: 
            self.path_pose
            self.ax1.scatter(self.path_pose[:,0],self.path_pose[:,1], c='red', label='Path', marker='*')
            self.ax1.set_xlim(-3, 3)   # Set x-axis limits: min=0, max=6
            self.ax1.set_ylim(-3, 3)  # 

    def plot_potential_map(self):
        X, Y = np.meshgrid(np.arange(self.potential_map.shape[1]), np.arange(self.potential_map.shape[0]))
        self.surf = self.ax1.plot_surface(X, Y, self.potential_map, cmap='viridis', edgecolor='k', linewidth=0.5, antialiased=True)

    def plot_init(self):

        c = self.draw_heatmap(self.potential_map, self.path)
        self.cbar = self.fig.colorbar(c, ax=self.ax2)
        # self.plot_potential_map()

    def plot_surface(self,frame : np.ndarray): 
        print("Plotting surface")
        self.ax1.clear()  # Clear the previous surface
        self.ax2.clear()  # Clear the previous surface
 
        c = self.draw_heatmap(self.potential_map, self.path)
        self.cbar.update_normal(c)
        self.plot_map()
        # self.plot_potential_map()

    def set_initial_pose(self, msg: PoseWithCovarianceStamped):
        print("Initial position set to: ({},{})".format(msg.pose.pose.position.x, msg.pose.pose.position.y))
        index_x, index_y = self.__pose_to_grid_index(msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.initial_point = (index_y,index_x)
    
    def get_next_position(self, x: float, y: float, range=3):
        ix, iy = self.__pose_to_grid_index(x=x,y=y)
        if ix is None or iy is None:
            new_iy, new_ix = get_next_step(self.potential_map, ix, iy)
        else:
            path = gradient_descent_2d(self.potential_map, (iy,ix), max_steps=range)
        new_iy, new_ix = path[-1] if path else (iy, ix)
        return self.__grid_index_to_pose(new_ix, new_iy)
    
    def set_goal(self,msg: PoseStamped): 
        print("Goal set to : ({},{})".format(msg.pose.position.x, msg.pose.position.y))
        x, y = self.__pose_to_grid_index(msg.pose.position.x, msg.pose.position.y)
        print("Goal grid index: ({},{})".format(x,y))
        self.goal = (y,x)
        self.potential_map = calculate_potential_field(self.raw_map, self.obstacles, self.radius, self.boundaries, self.goal)
        self.path = gradient_descent_2d(self.potential_map, self.initial_point, max_steps=1000)
        # self.path = get_path_to_goal(self.potential_map, self.initial_point, self.goal)

        
        self.path_pose = np.array([[]])
        for iy, ix in self.path:
           x,y = self.__grid_index_to_pose(ix, iy)
           if self.path_pose.size == 0:
               theta = 0.0
           else:
                theta = np.arctan2(y - self.path_pose[-2], x - self.path_pose[-3])
           self.path_pose = np.append(self.path_pose,np.array([[x,y, theta]]))
        self.path_pose = self.path_pose.reshape(-1,3)

        print("Path in poses: ", self.path_pose)
        # nc.send_trajectory_to_turtlebot(self.path_pose)
        self.path_ready = True
        
            
    def __pose_to_grid_index(self, x: float, y: float) -> tuple: 
        x_index = (int)((x-self.map_origin_x)/self.map_resolution)
        y_index = (int)((y-self.map_origin_y)/self.map_resolution)
        return x_index, y_index

    def __grid_index_to_pose(self, ix: int, iy: int) -> tuple:
        x_pose = self.map_origin_x + ix * self.map_resolution
        y_pose = self.map_origin_y + iy * self.map_resolution
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
        # self.sliced_map = slice_map(map_matrix, distance=3.5, resolution=msg.info.resolution)
        # self.map_origin_x = -3.5
        # self.map_origin_y = -3.5

        # Find obstacles in the interest area and boundaries
        obstacles_cells = cl.find_obstacles(self.raw_map)
        self.obstacles_cells = obstacles_cells
        self.obstacles_cells_pose = np.array([self.__grid_index_to_pose(x, y) for y, x in obstacles_cells])
        obstacles_clusters = cl.find_clusters(obstacles_cells)
        unique_labels = set(obstacles_clusters) - {-1}  # Exclude noise label (-1)
        obstacles, radius = cl.find_obstacles_info(obstacles_clusters, unique_labels, obstacles_cells)
        boundaries = cl.find_boundaries(obstacles_clusters, unique_labels, obstacles_cells)
        self.obstacles = obstacles
        self.radius = radius
        self.boundaries = boundaries
        self.potential_map = calculate_potential_field(self.raw_map, obstacles, radius, boundaries, self.goal)
    
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
        node.run_animation(block=True)
        # rospy.spin()

        # node.save_map()
        # print("Potential map saved to potential_map.txt")
    except KeyboardInterrupt:
        # node.save_map()
        print("ROS node interrupted.")
        rospy.shutdown("ROS node interrupted.")