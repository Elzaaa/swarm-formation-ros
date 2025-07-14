import rospy
import threading

from geometry_msgs.msg import Point, Twist, PoseWithCovarianceStamped
import numpy as np
from turtlebot_controller import TurteblotController
from mapping_node import MappingNode


class NavigationController: 

	def __init__(self):
		self.mapping_node = MappingNode()
		self.turtlebot = TurteblotController(node_name="tb3_0")
		self.sub_odom = rospy.Subscriber(f"/amcl_pose", PoseWithCovarianceStamped, self.update_current_position)
		self.x_pos = 0.0
		self.y_pos = 0.0

	def update_current_position(self, msg):
		self.x_pos = msg.pose.pose.position.x
		self.y_pos = msg.pose.pose.position.y
	
	def spin(self):
		while True:
			if not self.mapping_node.path_ready:
				print("Waiting for path to be ready...")
				rospy.sleep(1)
				continue
			x,y = self.mapping_node.get_next_position(self.x_pos, self.y_pos, range=3)
			theta = np.arctan2(y - self.y_pos, x - self.x_pos)
			print(f"Current position: ({self.x_pos}, {self.y_pos}), Next position: ({x}, {y}. {theta})")
			self.turtlebot.set_state_goal(state_goal=[x, y, theta])
			self.turtlebot.spin()
			print("finished spin")


# def send_trajectory_to_turtlebot(trajectory):
# 	"""
# 	Function to send a trajectory to the Turtlebot controller.
# 	:param turtlebot_controller: Instance of TurteblotController.
# 	:param trajectory: List of state trajectories to be followed.
# 	"""
# 	turtlebot_controller = TurteblotController(node_name='tb3_1')
# 	turtlebot_controller.set_state_trajectory(state_trajectory=trajectory)

# 	t1 = threading.Thread(target=turtlebot_controller.spin_trajectory)
# 	t1.start()

# 	t1.join()
# 	print("Threads completed.")
	
if __name__ == '__main__':
	navigationController = NavigationController()
	mapping_node = navigationController.mapping_node
	t1 = threading.Thread(target=mapping_node.run_animation(block=False))
	t2 = threading.Thread(target=navigationController.spin)

	try:
		while 1: 
			t2.start()
			t1.start()
	except KeyboardInterrupt:

		t1.join()
		t2.join()
	
	except rospy.exceptions.ROSInterruptException as e:
		print(f"Error: {e}")
		t1.join()
		t2.join()

