import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ros-user/swarm-formation-ros/ros_ws/install/swarm_control'
