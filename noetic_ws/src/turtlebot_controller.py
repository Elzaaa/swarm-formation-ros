
import os, sys
PARENT_DIR = os.path.abspath(__file__ + '/../..')
sys.path.insert(0, PARENT_DIR)
import math

from datetime import datetime, timedelta
import numpy as np

import argparse

#---------------------------------------- Import ROS
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
#from tf.transformations import euler_from_quaternion
import tf.transformations as tftr 
import threading



#----------------------------------------Initialization : : system
# my_sys = systems.Sys3WRobotNI(sys_type="diff_eqn")
                                #  dim_state=dim_state,
                                #  dim_input=dim_input,
                                #  dim_output=dim_output,
                                #  dim_disturb=dim_disturb,
                                #  pars=[],
                                #  ctrl_bnds=ctrl_bnds,
                                #  is_dyn_ctrl=is_dyn_ctrl,
                                #  is_disturb=is_disturb,
                                #  pars_disturb = np.array([[200*dt, 200*dt], [0, 0], [0.3, 0.3]]))

# observation_init = my_sys.out(state_init)

class TurteblotController:

    def __init__(self, node_name):
        
        self.node_name = node_name
        # rospy.init_node(f'{self.node_name}_controller')
        
        self.RATE = rospy.get_param('/rate', 50)
        self.lock = threading.Lock()
        
        #Initialization
        # self.system = my_sys
        self.state_goal = [0.0, 0.0, 0.0]  # Default goal state
        
        self.dt = 0.0
        self.time_start = 0.0
        
        # Topics
        
        self.pub_cmd_vel = rospy.Publisher(f"{self.node_name}/cmd_vel", Twist, queue_size=1, latch=False)
        self.sub_odom = rospy.Subscriber(f"{self.node_name}/odom", Odometry, self.odometry_callback)
        
        self.state = np.zeros(3)
        self.dstate = np.zeros(3)
        self.new_state = np.zeros(3)
        self.new_dstate = np.zeros(3)
        
        self.rotation_counter = 0
        self.prev_theta = 0
        self.new_theta = 0
        
        theta_goal = self.state_goal[2]

        self.ctrl_bnds = np.array([
                [-.22, .22],  # linear velocity limit
                [-2.84, 2.84] # angular velocity limit
        ])

        self.ctrl = N_CTRL(self.ctrl_bnds)
        
        # Complete Rotation Matrix
        self.rotation_matrix = np.zeros((3,3))  # here

    def set_state_goal(self, state_goal):
        """
        Set the goal state for the controller.
        :param state_goal: List or array of [x, y, theta] representing the goal state.
        """
        self.state_goal = state_goal
        rospy.loginfo(f"Goal state set to: {self.state_goal}")
        
    def odometry_callback(self, msg):
    
        self.lock.acquire()
        
        # Read current robot state
        x = msg.pose.pose.position.x
        # Complete for y and orientation
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
          
        rospy.loginfo(msg.pose.pose.position)
        # Transform quat2euler using tf transformations: complete!
        current_rpy = tftr.euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        # Extract Theta (orientation about Z)
        theta = current_rpy[2]
        
        self.state = [x, y, 0, theta]
        
        # Make transform matrix from 'robot body' frame to 'goal' frame
        theta_goal = self.state_goal[2]
        
        # Complete rotation matrix
        rotation_matrix = np.array([ 
            [np.cos(theta_goal), -np.sin(theta_goal), 0], 
            [np.sin(theta_goal), np.cos(theta_goal), 0], 
            [0,0,1] 
        ])
        
        state_matrix = np.vstack([
            self.state_goal[0],
            self.state_goal[1],
            0
        ]) # [x, y, 0] -- column   
        
        # Compute Transformation matrix 
        
        t_matrix = np.block([
            [rotation_matrix, state_matrix],
            [np.array([0, 0, 0, 1])]
        ])
        inv_t_matrix = np.linalg.inv(t_matrix)
        
        # Complete rotation counter for turtlebot3
        
        ''' Your solution goes here (rotation_counter) '''
        if math.copysign(1, self.prev_theta) != math.copysign(1, theta) and abs(self.prev_theta) > np.pi:
            if math.copysign(1, self.prev_theta) == -1:
                self.rotation_counter -= 1
            else: 
                self.rotation_counter += 1
        
        self.prev_theta = theta
        theta = theta + 2 * np.pi * self.rotation_counter
        self.new_theta = theta
        
        # Orientation transform
        new_theta = theta - theta_goal
        
        # Do position transform
        
        ''' 
        Your solution goes here 
        self.new_state = using tranformations :) Have fun!
        '''
        temp_pos = np.array([x,y,0,1])
        
        # self.new_state = np.linalg.inv(t_matrix) @ temp_pos.T
        self.new_state = np.dot(inv_t_matrix, np.transpose(temp_pos))
        self.new_state = [self.new_state[0], self.new_state[1], new_theta]
        rospy.loginfo("New state for {}: {}".format(self.node_name, self.new_state))

        self.lock.release()

    def __check_zero(self, xpos, ypos) -> bool:
        """
        Check if the robot is close to the goal state.
        :param xpos: x position of the robot.
        :param ypos: y position of the robot.
        :param theta: orientation of the robot.
        :return: True if close to goal, False otherwise.
        """
        abs_error = 0.1
        return (abs(xpos) < abs_error and abs(ypos) < abs_error)
        
    def spin(self):
    
        rospy.loginfo('ROS-Preset has been activated!')
        
        start_time = datetime.now()
        rate = rospy.Rate(self.RATE)
        self.time_start = rospy.get_time()
        
        while not rospy.is_shutdown() and datetime.now() - start_time < timedelta(100):
            if not all(self.new_state):
                rate.sleep()
            
            if self.__check_zero(self.new_state[0], self.new_state[1]):
                rospy.loginfo(f"Goal state reached for {self.node_name}!")
                velocity.linear.x = 0
                velocity.angular.z = 0
                self.pub_cmd_vel.publish(velocity)
                break 

            t = rospy.get_time() - self.time_start
            self.t = t
            
            velocity = Twist()
            
            # action = controllers.ctrl_selector('''COMPLETE!''')
            action = self.ctrl.compute_action(self.new_state)
            
            # self.system.receive_action(action)
            # self.ctrl_benchm.receive_sys.state(self.system._state)
            # self.ctrl_benchm.upd_accum_obj(self.new_state, action)
            
            # run_obj = self.ctrl_benchm.run_obj(self.new_state, action)
            # accum_obj = self.ctrl_benchm.accum_obj_val

            for k in range(2):
                action[k] = np.clip(action[k], self.ctrl_bnds[k, 0], self.ctrl_bnds[k, 1])

                
            # self.ctrl_benchm.receive_sys_state(self.new_state)
           
           # Generate ROSmsg from action
            '''
            velocity.linear.x =  ...
            velocity.angular.z =  ...
            
            PUBLISH THE MESSAGE VELOCITY:
            YOUR SOLUTION HERE
            '''
            velocity.linear.x = action[0]
            velocity.angular.z = action[1]
            self.pub_cmd_vel.publish(velocity)
           
            rate.sleep()
           
        rospy.loginfo('Task completed or interrupted!')
     


#----------------------------------------Initialization : : model

#----------------------------------------Main loop

class N_CTRL:

        #####################################################################################################
        ########################## write down here nominal controller class #################################
        #####################################################################################################
        def __init__(self, ctrl_bounds) -> list:
            self.ctrl_bounds = ctrl_bounds
            self.kappa_rho = 0.15
            self.kappa_alpha = 0.17
            self.kappa_betha = - 0.05
            
            
        def compute_action(self, observation):
            polar_coord = self._transform_2_polar(observation)
            v = self.kappa_rho * polar_coord[0]
            w = (self.kappa_alpha * polar_coord[1]) + (self.kappa_betha * polar_coord[2])

            if (np.abs(observation[0]) < 0.1 and np.abs(observation[1]) < 0.1):
                v = 0
            if (np.abs(observation[1]) < 0.1):
                w = 0
            return [v,w]
        
        def _transform_2_polar(self, observation): 
            delta_x = - observation[0]
            delta_y = - observation[1]
            theta = observation[2]

            rho = np.sqrt(np.power(delta_x,2) + np.power(delta_y,2))
            alpha = -theta + np.arctan2(delta_y, delta_x)
            beta = -theta - alpha
            return [rho, alpha, beta]
