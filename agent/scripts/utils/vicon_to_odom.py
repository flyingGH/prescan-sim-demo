#!/usr/bin/env python
from __future__ import print_function

# System Libraries
import os
import sys; sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# External Libraries
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped, Pose, Twist, Point, Quaternion, Vector3
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from nav_msgs.msg import Odometry, Path

# Local Imports
from config.ros import VERBOSE_LEVEL
from utils.rostools import set_subscribers

class ViconToOdom(object):

    def __init__(self):

        # Get params
        self.env = rospy.get_param('env')
        self.agent_name = rospy.get_param('agent')
        self.car_id = str(rospy.get_param('car_id'))
        self.pose_topic = rospy.get_param("vicon_car_" + self.car_id + "_pose_topic")
        self.twist_topic = rospy.get_param("vicon_car_" + self.car_id + "_twist_topic")
        self.odom_topic = rospy.get_param('odom_topic')

        self.waypoints = None

        # Set up publisher
        self.pub_odom = rospy.Publisher(self.odom_topic, Odometry, queue_size=5, latch=True)
        
        # give some time to create publisher
        rospy.sleep(1.0)

        self.pose_msg = PoseStamped()
        self.twist_msg = TwistStamped()

        # Set up subscriber


        if self.env == 'sim' and self.agent_name == 'pure_pursuit':
            set_subscribers([
                (self.pose_topic, PoseStamped, self._pose_callback, False),
 	        (self.twist_topic, TwistStamped, self._twist_callback, False),
                (rospy.get_param('path_topic'), Path, self._path_callback, True)
            ], subscribe=True)

            pose = Pose()
            pose.position = Point(self.waypoints[0], self.waypoints[1], self.waypoints[2])
            q = quaternion_from_euler(0, 0, 0)
            pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
            self.pose_msg.pose = pose
            twist = Twist()
            twist.linear = Vector3(0., 0., 0.)
            twist.angular = Vector3(0., 0., 0.)
            self.twist_msg.twist = twist
        else:
            set_subscribers([
            	(self.pose_topic, PoseStamped, self._pose_callback, True),
            	(self.twist_topic, TwistStamped, self._twist_callback, True),
	        ], subscribe=True)
        self._push_odom()

    # -- Callbacks (private)
    def _path_callback(self, msg):
        self.waypoints = np.array([msg.poses[0].pose.position.x, msg.poses[0].pose.position.y, msg.poses[0].pose.position.z])
        # print(self.waypoints)

    def _pose_callback(self, data):
        self.pose_msg = data
        self._push_odom()

    def _twist_callback(self, data):
        self.twist_msg = data
        self._push_odom()
        
    def _push_odom(self):
        odom_msg = Odometry()
        odom_msg.header = self.pose_msg.header
        odom_msg.pose.pose = self.pose_msg.pose
        odom_msg.twist.twist = self.twist_msg.twist
        # x = odom_msg.pose.pose.orientation.x
        # y = odom_msg.pose.pose.orientation.y
        # z = odom_msg.pose.pose.orientation.z
        # w = odom_msg.pose.pose.orientation.w
        # (_, _, yaw) = euler_from_quaternion([x, y, z, w])
        # print(np.rad2deg(yaw))
        self.pub_odom.publish(odom_msg)

if __name__ == "__main__":

    # Initialize node
    rospy.init_node('vicon_to_odom', anonymous=True, log_level=VERBOSE_LEVEL)

    # Initialize class
    vicon_to_odom = ViconToOdom()

    # Wait for other nodes to finish
    rospy.spin()
