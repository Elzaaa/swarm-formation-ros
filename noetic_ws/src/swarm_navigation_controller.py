import rospy
import threading

from geometry_msgs.msg import Point, Twist, PoseWithCovarianceStamped
import numpy as np
from turtlebot_controller import TurteblotController
from mapping_node import MappingNode


class NavigationController: 

	def __init__(self):
		self.lock = threading.Lock()
		self.RATE = rospy.get_param('/rate', 50)

		self.mapping_node = MappingNode()
		self.turtlebot_leader = TurteblotController(node_name="tb3_0")
		self.turtlebot_follower1 = TurteblotController(node_name="tb3_1")
		self.turtlebot_follower2 = TurteblotController(node_name="tb3_2")
		rospy.Subscriber("tb3_0/amcl_pose", PoseWithCovarianceStamped, self.update_leader_position)
		rospy.Subscriber(f"tb3_1/amcl_pose", PoseWithCovarianceStamped, self.update_follower1_position)
		rospy.Subscriber(f"tb3_2/amcl_pose", PoseWithCovarianceStamped, self.update_follower2_position)
		self.leader_x_pos = 0.0
		self.leader_y_pos = 0.0
		self.follower1_x_pos = 0.0
		self.follower1_y_pos = 0.0
		self.follower2_x_pos = 0.0
		self.follower2_y_pos = 0.0
  
		self.leader_x_goal = 0
		self.leader_y_goal = 0
		self.leader_theta = 0

		self.follower1_x_goal = 0
		self.follower1_y_goal = 0

		self.range = 3
  
		self.path_index = 0

	def update_leader_position(self, msg):
		with self.lock:
			self.leader_x_pos = msg.pose.pose.position.x
			self.leader_y_pos = msg.pose.pose.position.y

	def update_follower1_position(self, msg):
		with self.lock:
			self.follower1_x_pos = msg.pose.pose.position.x
			self.follower1_y_pos = msg.pose.pose.position.y

	def update_follower2_position(self, msg):
		with self.lock:
			self.follower2_x_pos = msg.pose.pose.position.x
			self.follower2_y_pos = msg.pose.pose.position.y

	def navigation_loop(self):
		rate = rospy.Rate(self.RATE)
		rate.sleep()  # Initial sleep to allow subscribers to receive first messages
		while not rospy.is_shutdown():

			
			with self.lock:
				leader_x = self.leader_x_pos
				leader_y = self.leader_y_pos
				follower1_x = self.follower1_x_pos
				follower1_y = self.follower1_y_pos
				follower2_x = self.follower2_x_pos
				follower2_y = self.follower2_y_pos

			if not all([leader_x, leader_y, follower1_x, follower1_y, follower2_x, follower2_y]):
				rospy.loginfo("Waiting for leader and followers positions to be updated...")
				rate.sleep()
				

			if not self.mapping_node.path_ready:
				rospy.loginfo("Waiting for path to be ready...")
				rospy.sleep(1)
				continue
			
			range=2
   
			path = np.asarray(self.mapping_node.path_pose)
			coords = path[:,:2]
			dist = np.linalg.norm(np.array([leader_x, leader_y]) - coords, axis=1)
			print("Calculated distances: {}".format(dist))
			new_index = np.argmin(dist)
			if (new_index > self.path_index):
				self.path_index = new_index
				print("Current path goal: {}".format(path[self.path_index]))
			elif (dist[new_index] <= 0.1):
				self.path_index = new_index + 1

			print("Path Status: {}, {}".format(self.path_index, path.size))
			if (self.path_index+1 > path.size):
				self.path_index -= 1
				# rospy.loginfo("Goal reached")
				# rospy.signal_shutdown()
			
			# leader_x_goal, leader_y_goal = self.mapping_node.get_next_position(leader_x, leader_y, range=range)
			# leader_theta = np.arctan2(leader_y_goal - leader_y, leader_x_goal - leader_x)
			rospy.loginfo(f"Current Leader position: ({leader_x}, {leader_y})")
			leader_x_goal = path[self.path_index][0]
			leader_y_goal = path[self.path_index][1]
			leader_theta = path[self.path_index][2]
			self.turtlebot_leader.set_state_goal(state_goal=[leader_x_goal, leader_y_goal, leader_theta])
   
			rospy.loginfo(f"Current Follower 1 position: ({follower1_x}, {follower1_y})")
			follower1_x_goal, follower1_y_goal, follower1_theta = self.mapping_node.get_follower1_next_position(
				x=follower1_x, y=follower1_y, 
				leader_x=leader_x_goal, leader_y=leader_y_goal, 
				follower2_x=self.follower2_x_pos, follower2_y=self.follower2_y_pos,
				range=range)
			
			self.turtlebot_follower1.set_state_goal(state_goal=[follower1_x_goal, follower1_y_goal, leader_theta])
			
			rospy.loginfo(f"Current Follower 2 position: ({self.follower2_x_pos}, {self.follower2_y_pos})")
			follower2_x_goal, follower2_y_goal, follower2_theta = self.mapping_node.get_follower2_next_position(
				x=self.follower2_x_pos, y=self.follower2_y_pos,
				leader_x=leader_x_goal, leader_y=leader_y_goal, 
				follower1_x=follower1_x_goal, follower1_y=follower1_y_goal,
				range=range)
			
			self.turtlebot_follower2.set_state_goal(state_goal=[follower2_x_goal, follower2_y_goal, leader_theta])

			leader_thread = threading.Thread(target=self.turtlebot_leader.spin)
			follower1_thread = threading.Thread(target=self.turtlebot_follower1.spin)
			follower2_thread = threading.Thread(target=self.turtlebot_follower2.spin)
			leader_thread.start()
			follower1_thread.start()
			follower2_thread.start()
			leader_thread.join()
			follower1_thread.join()
			follower2_thread.join()
			
			rospy.loginfo("Leader and followers have been updated.")
			rate.sleep()
			
	def run_leader(self):
		rate = rospy.Rate(self.RATE)
		rate.sleep()  # Initial sleep to allow subscribers to receive first messages
		while not rospy.is_shutdown():

			if not self.mapping_node.path_ready:
				rospy.loginfo("Waiting for path to be ready...")
				rospy.sleep(1)
				continue

			# path = np.delete(np.asarray(self.mapping_node.path_pose),0,0)
			path = np.asarray(self.mapping_node.path_pose)
			coords = path[:,:2]
			dist = np.linalg.norm(np.array([self.leader_x_pos, self.leader_y_pos]) - coords, axis=1)
   
			print("Calculated distances: {}".format(dist))
			new_index = np.argmin(dist)
			if (new_index > self.path_index):
				self.path_index = new_index
				print("Current path goal: {}".format(path[self.path_index]))
			elif (dist[new_index] <= 0.1):
				self.path_index = new_index + 1

			print("Path Status: {}, {}".format(self.path_index, path.size))
			if (self.path_index+1 > path.size):
				self.path_index -= 1
				# rospy.loginfo("Goal reached")
				# rospy.signal_shutdown()
			
			# leader_x_goal, leader_y_goal = self.mapping_node.get_next_position(leader_x, leader_y, range=range)
			# leader_theta = np.arctan2(leader_y_goal - leader_y, leader_x_goal - leader_x)
			rospy.loginfo(f"Current Leader position: ({self.leader_x_pos}, {self.leader_y_pos})")
			self.leader_x_goal = path[self.path_index][0]
			self.leader_y_goal = path[self.path_index][1]
			self.leader_theta = path[self.path_index][2]
			self.turtlebot_leader.set_state_goal(state_goal=[self.leader_x_goal, 
                                                    		 self.leader_y_goal, 
                                                       		 self.leader_theta])
			self.turtlebot_leader.spin()

	def run_follower1(self):
		rate = rospy.Rate(self.RATE)
		rate.sleep()  # Initial sleep to allow subscribers to receive first messages
		while not rospy.is_shutdown():
			
			if not self.mapping_node.path_ready:
				rospy.sleep(2)
				continue

			rospy.loginfo(f"Current Follower 1 position: ({self.follower1_x_pos }, {self.follower1_y_pos})")
			follower1_x_goal, follower1_y_goal, follower1_theta = self.mapping_node.get_follower1_next_position(
			x=self.follower1_x_pos, y=self.follower1_y_pos, 
			leader_x=self.leader_x_goal, leader_y=self.leader_y_goal, 
			follower2_x=self.follower2_x_pos, follower2_y=self.follower2_y_pos,
			range=self.range)
   
			self.follower1_x_goal = follower1_x_goal
			self.follower1_y_goal = follower1_y_goal
				
			self.turtlebot_follower1.set_state_goal(state_goal=[follower1_x_goal, follower1_y_goal, follower1_theta])
			self.turtlebot_follower1.spin()
   
	def run_follower2(self):
		rate = rospy.Rate(self.RATE)
		rate.sleep()  # Initial sleep to allow subscribers to receive first messages
		while not rospy.is_shutdown():

			if not self.mapping_node.path_ready:
				rospy.sleep(2)
				continue

			rospy.loginfo(f"Current Follower 2 position: ({self.follower2_x_pos}, {self.follower2_y_pos})")
			follower2_x_goal, follower2_y_goal, follower2_theta = self.mapping_node.get_follower2_next_position(
				x=self.follower2_x_pos, y=self.follower2_y_pos,
				leader_x=self.leader_x_goal, leader_y=self.leader_y_goal, 
				follower1_x=self.follower1_x_goal, follower1_y=self.follower1_y_goal,
				range=self.range)
			
			self.turtlebot_follower2.set_state_goal(state_goal=[follower2_x_goal, follower2_y_goal, follower2_theta])
			self.turtlebot_follower2.spin()

# if __name__ == '__main__':
# 	navigationController = NavigationController()
# 	rospy.init_node('navigation_controller_node')
# 	nav_thread = threading.Thread(target=navigationController.navigation_loop)
	
# 	try:
# 		nav_thread.start()
# 		# navigationController.navigation_loop()
# 		print("Navigation loop started.")
# 		navigationController.mapping_node.run_animation(block=True)
# 		# rospy.spin()  # Let ROS handle callbacks
# 	except KeyboardInterrupt:
# 		rospy.signal_shutdown("KeyboardInterrupt")

# 		# nav_thread.join()
# 		print("Exiting...")
# 	except rospy.exceptions.ROSInterruptException as e:
# 		print(f"Error: {e}")
# 		navigationController.mapping_node.close()
# 		nav_thread.join()

if __name__ == '__main__':
	navigationController = NavigationController()
	rospy.init_node('navigation_controller_node')
	leader_thread = threading.Thread(target=navigationController.run_leader)
	follower1_thread = threading.Thread(target=navigationController.run_follower1)
	follower2_thread = threading.Thread(target=navigationController.run_follower2)
	
	try:
		leader_thread.start()
		follower1_thread.start()
		follower2_thread.start()
		# navigationController.navigation_loop()
		print("Navigation loop started.")
		navigationController.mapping_node.run_animation(block=True)
		leader_thread.join()
		follower1_thread.join()
		follower2_thread.join()
		# rospy.spin()  # Let ROS handle callbacks
	except KeyboardInterrupt:
		rospy.signal_shutdown("KeyboardInterrupt")

		# nav_thread.join()
		print("Exiting...")
	except rospy.exceptions.ROSInterruptException as e:
		print(f"Error: {e}")
		navigationController.mapping_node.close()

