import rospy
import threading

from turtlebot_controller import TurteblotController


# if __name__ == "__main__":
# 	rospy.init_node('ros_controller_node')
	
# turtlebot_leader = TurteblotController(node_name='tb3_1')
# turtlebot_leader.set_state_goal(state_goal=[1.0, 2.0, 0.0])
# turtlebot_leader.spin()

# turtlebot_follower1 = TurteblotController(node_name='tb3_1')
# # turtlebot_follower1.set_state_trajectory(state_trajectory=[[0.25, 0.25, 0.785], [0.5, 0.5, 0.785], [0.75, 0.75, 0.785]])
# turtlebot_follower1.set_state_goal([2.0,3.0,0.785])


# turtlebot_follower2 = TurteblotController(node_name='tb3_0')
# turtlebot_follower2.set_state_goal(state_goal=[2.0, -2.0, 0.0])

if __name__ == "__main__":
	rospy.init_node('ros_controller_node')
	turtlebot_leader = TurteblotController(node_name='tb3_0')
	turtlebot_follower1 = TurteblotController(node_name='tb3_1')
	turtlebot_follower2 = TurteblotController(node_name='tb3_2')


	try:
		while True:
			x = input("Enter desired x position: ")
			y = input("Enter desired y position: ")
			theta = input("Enter desired theta: ")
			leader_thread = threading.Thread(target=turtlebot_leader.spin)
			follower1_thread = threading.Thread(target=turtlebot_follower1.spin)
			follower2_thread = threading.Thread(target=turtlebot_follower2.spin)
			turtlebot_leader.set_state_goal([float(x), float(y), float(theta)])
			turtlebot_follower1.set_state_goal([float(x) - 0.25, float(y) - 0.25, float(theta)])
			turtlebot_follower2.set_state_goal([float(x) - 0.25, float(y) + 0.25, float(theta)])
			
			leader_thread.start()
			follower1_thread.start()
			follower2_thread.start()

			leader_thread.join()
			follower1_thread.join()
			follower2_thread.join()
	except KeyboardInterrupt:
		print("Exiting...")

		leader_thread.join()
		follower1_thread.join()
		follower2_thread.join()
	
	# t1 = threading.Thread(target=turtlebot_follower1.spin)
	# # t2 = threading.Thread(target=turtlebot_follower2.spin)
	# print("Starting spin")
	# t1.start()
	# # t2.start()

	# t1.join()
	# # t2.join()

	# print("Threads completed.")
