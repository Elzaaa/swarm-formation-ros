import rospy
import threading

from turtlebot_controller import TurteblotController


if __name__ == "__main__":
	rospy.init_node('ros_controller_node')
	
# turtlebot_leader = TurteblotController(node_name='tb3_1')
# turtlebot_leader.set_state_goal(state_goal=[1.0, 2.0, 0.0])
# turtlebot_leader.spin()

turtlebot_follower1 = TurteblotController(node_name='tb3_1')
turtlebot_follower1.set_state_goal(state_goal=[2.0, 2.0, 0.0])


turtlebot_follower2 = TurteblotController(node_name='tb3_0')
turtlebot_follower2.set_state_goal(state_goal=[2.0, -2.0, 0.0])

if __name__ == "__main__":
	rospy.init_node('ros_controller_node')
	t1 = threading.Thread(target=turtlebot_follower1.spin)
	t2 = threading.Thread(target=turtlebot_follower2.spin)

	t1.start()
	t2.start()

	t1.join()
	t2.join()
	print("Threads completed.")


	
