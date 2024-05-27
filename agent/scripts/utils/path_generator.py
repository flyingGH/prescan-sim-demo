#!/usr/bin/env python
from __future__ import print_function

# System Libraries
import os
import sys; sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# External Libraries
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

# Local Imports
from config.ros import VERBOSE_LEVEL, TIMEOUT_READY
from utils.tools import csv_to_waypoints


class PathGenerator(object):

    def __init__(self):

        # Get params
        self.csv_path = rospy.get_param('csv_path')

        # Publisher
        self.path_pub = rospy.Publisher(rospy.get_param("path_topic"), Path, queue_size=10, latch=True)

        # Load waypoints and publish
        self.publish_waypoints()

    def publish_waypoints(self):
        waypoints = csv_to_waypoints(self.csv_path)
        path_msg = Path()
        path_msg.header.frame_id = "map"  # Assuming map frame, can be changed based on requirement

        for wp in waypoints:
            pose = PoseStamped()
            pose.pose.position.x = wp[0]
            pose.pose.position.y = wp[1]
            pose.pose.position.z = wp[2]
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        rospy.loginfo("Path published successfully!")
   


if __name__ == "__main__":

    # Initialize node
    rospy.init_node('path_generator', anonymous=True, log_level=VERBOSE_LEVEL)

    # Initialize class
    path_generator = PathGenerator()

    # Wait for other nodes to finish
    rospy.spin()