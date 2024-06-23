#!/usr/bin/env python

# System Imports
from __future__ import print_function
import os
import math, os
import sys; print(sys.version); sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# External Libraries
import rospy
import numpy as np
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion

# Local Imports
from config.ros import VERBOSE_LEVEL, WARN_FREQ, DEBUG_FREQ
from utils.rostools import set_subscribers
import torch
import numpy as np
from model import PredictionNet

cur_dir = os.getcwd()


class Simulator():

    Kp = rospy.get_param("Kp")
    Kd = rospy.get_param("Kd")
    dt = 1.0/rospy.get_param('max_fps')
    wheel_base = rospy.get_param("wheel_base")

    class State:

        def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
            self.x = x
            self.y = y
            self.yaw = yaw
            self.v = v
            self.rear_x = self.x - ((Simulator.wheel_base / 2) * math.cos(self.yaw))
            self.rear_y = self.y - ((Simulator.wheel_base / 2) * math.sin(self.yaw))

	    self.hist = np.zeros((1, 1, 50))
	    self.net = PredictionNet(out_len=1, batch_size=30)
	    net_state_dict = torch.load('/home/student/catkin_ws/src/agent/scripts/simulator/net_state_dict.pth')
	    self.net.load_state_dict(net_state_dict)
	    
            # print('Vehicle state initialized at Initial Position: %f, %f, %f' %(self.x, self.y, self.yaw))

        def update(self, a, delta, desired_velocity):
            #NOTE Convention for the following equations:
            #       Steering angle is increasing in the CW (in contrast to real VESC increasing IN CCW)
            delta = -delta

	    self.hist[:, :, :-1] = self.hist[:, :, 1:]
	    self.hist[:, :, -1] = desired_velocity
	    

            self.x += self.v * math.cos(self.yaw) * Simulator.dt
            self.y += self.v * math.sin(self.yaw) * Simulator.dt
            self.yaw += (1 / Simulator.wheel_base) * self.v * math.tan(delta) * Simulator.dt
            # self.v += a * Simulator.dt
	    # self.v += self.net(torch.tensor(self.hist).float())[0, 0, 0].detach().numpy() * Simulator.dt
	    
	    v_temp = self.net(torch.tensor(self.hist).float())[0, 0, 0].detach().numpy()
	    if v_temp > 0.15:
		self.v = v_temp
	    else: 
		self.v = 0
            # print(np.rad2deg(delta))
            self.rear_x = self.x - ((Simulator.wheel_base / 2) * math.cos(self.yaw))
            self.rear_y = self.y - ((Simulator.wheel_base / 2) * math.sin(self.yaw))

        def calc_distance(self, point_x, point_y):
            dx = self.rear_x - point_x
            dy = self.rear_y - point_y
            return math.hypot(dx, dy)
        

    def __init__(self):

        # setup publishers
        car_id = str(rospy.get_param('car_id'))
        offline = rospy.get_param("offline")
        self.seq = 0

        pose_topic = rospy.get_param("vicon_car_" + car_id + "_pose_topic")
        twist_topic = rospy.get_param("vicon_car_" + car_id + "_twist_topic")
        self.pub_vicon_pose = rospy.Publisher(pose_topic, PoseStamped, queue_size=1000, latch=True, tcp_nodelay=True)
        self.pub_vicon_twist = rospy.Publisher(twist_topic, TwistStamped, queue_size=1000, latch=True, tcp_nodelay=True)

        init_odom = rospy.wait_for_message(rospy.get_param("odom_topic"), Odometry)
        init_pose = init_odom.pose

        self.prev_time = rospy.get_time()
        self.prev_error = 0

        # setup subscribers
        if offline:
            self.system_ready = True
        else:
            set_subscribers([
                        (rospy.get_param('prescan_status_topic'),   Bool,       self._prescan_status_callback,      True)
                    ], subscribe=True)
        self.system_ready = True
            
        set_subscribers([
            (rospy.get_param('drive_final_topic'), Twist, self._command_callback, False)
        ], subscribe=True)


        # extract initial position
        x0 = init_pose.pose.position.x
        y0 = init_pose.pose.position.y
        [_, _, yaw] = euler_from_quaternion([init_pose.pose.orientation.x, init_pose.pose.orientation.y, init_pose.pose.orientation.z, init_pose.pose.orientation.w])

        # initialize state
        self.current_state = Simulator.State(x=x0, y=y0, yaw=yaw)
        self.desired_velocity = 0.0
        self.desired_steering_angle = 0.0
        
        # setup timer callback
        rospy.Timer(rospy.Duration(Simulator.dt), self._update_callback)

        rospy.loginfo("[%s]: Initialization complete!" % (self.__class__.__name__))

    def _command_callback(self, msg):
        self.desired_velocity = msg.linear.x
        self.desired_steering_angle = msg.angular.z


    def _prescan_status_callback(self, msg):
        self.system_ready = msg.data

    def _update_callback(self, event):


        self.curr_time = rospy.get_time()

        # get new speed (towards desired speed)
        speed_step = self.pd_control(current=self.current_state.v, 
                                          target=self.desired_velocity,
                                          dt=Simulator.dt)
        self.prev_time = self.curr_time
        
        # rospy.logdebug_throttle(DEBUG_FREQ, '[%s] Command from PP: v:%.2f, delta:%.2f, Current Speed: %.2f, Speed Step:%.2f', 
        #                         self.__class__.__name__,
        #                         self.desired_velocity,
        #                         self.desired_steering_angle,
        #                         self.current_state.v,
        #                         speed_step)
        # rospy.logdebug_throttle(DEBUG_FREQ, '[%s] Position: (%.2f, %.2f))',
        #                         self.__class__.__name__,
        #                         self.current_state.x,
        #                         self.current_state.y)

        # update state
        self.current_state.update(speed_step, self.desired_steering_angle, self.desired_velocity)

        # create pose message
        pose_msg = PoseStamped()
        pose_msg.header.seq = self.seq
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "world"
        
        # broadcast pose transforms
        pose_msg.pose.position.x = self.current_state.x
        pose_msg.pose.position.y = self.current_state.y
        pose_msg.pose.position.z = 0.13  

        q = quaternion_from_euler(0, 0, self.current_state.yaw)
        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]

        # create twist message
        twist_msg = TwistStamped()
        twist_msg.header.seq = self.seq
        twist_msg.header.stamp = rospy.Time.now()
        twist_msg.header.frame_id = "world"
        twist_msg.twist = Twist()
        twist_msg.twist.linear.x = self.current_state.v
    
        # publish messages
        self.pub_vicon_pose.publish(pose_msg)
        self.pub_vicon_twist.publish(twist_msg)

        self.seq += 1


    def pd_control(self, current, target, dt):
        
        curr_err = target - current
        
        derivative = (curr_err - self.prev_error) / dt

        step = Simulator.Kp * curr_err + Simulator.Kd * derivative

        self.prev_error = curr_err

        return step


if __name__ == "__main__":
    rospy.init_node('simulator', anonymous=True, log_level=VERBOSE_LEVEL)
    sim_bridge = Simulator()
    rospy.spin()
