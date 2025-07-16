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
				rate.sleep()
				continue

			range=2

			leader_x_goal, leader_y_goal = self.mapping_node.get_next_position(leader_x, leader_y, range=range)
			leader_theta = np.arctan2(leader_y_goal - leader_y, leader_x_goal - leader_x)
			self.turtlebot_leader.set_state_goal(state_goal=[leader_x_goal, leader_y_goal, leader_theta])

			follower1_x_goal, follower1_y_goal = self.mapping_node.get_follower1_next_position(
				x=follower1_x, y=follower1_y, 
				leader_x=leader_x_goal, leader_y=leader_y_goal, 
				follower2_x=follower2_x, follower2_y=follower2_y,
				range=range)
			follower1_theta = np.arctan2(follower1_y_goal - follower1_y, follower1_x_goal - follower1_x)
			self.turtlebot_follower1.set_state_goal(state_goal=[follower1_x_goal, follower1_y_goal, leader_theta])

			follower2_x_goal, follower2_y_goal = self.mapping_node.get_follower2_next_position(
				x=follower2_x, y=follower2_y,
				leader_x=leader_x_goal, leader_y=leader_y_goal, 
				follower1_x=follower1_x, follower1_y=follower1_y,
				range=range)
			follower2_theta = np.arctan2(follower2_y_goal - follower2_y, follower2_x_goal - follower2_x)
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

	
if __name__ == '__main__':
	navigationController = NavigationController()
	rospy.init_node('navigation_controller_node')
	# nav_thread = threading.Thread(target=navigationController.navigation_loop)
	
	try:
		# nav_thread.start()
		navigationController.navigation_loop()
		print("Navigation loop started.")
		rospy.spin()  # Let ROS handle callbacks
	except KeyboardInterrupt:
		rospy.signal_shutdown("KeyboardInterrupt")
		# nav_thread.join()
		print("Exiting...")


