#!/usr/bin/env python
from __future__ import print_function

# System Libraries
import os
import re
import math
import sys; sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# External Libraries
import numpy as np
import rospy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Point, Vector3, Pose, Quaternion
from std_msgs.msg import Bool, String, ColorRGBA
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Range
from tf.transformations import euler_from_quaternion

# Local Imports
from config.ros import VERBOSE_LEVEL
from config.definitions import TRAFFIC_LIGHT_INFO, PED_ID_START, TL_ID_START
from utils.rostools import set_subscribers
from utils.tools import dist
from env.traffic_light import TrafficLight

class Visualizer(object):

    def __init__(self):
        
        # initialize dynamic parameters
        self.emergency = False
        self.position = Point(0., 0., 0.,)
        self.traffic_lights = {}
        self.global_yaw = 0.0
        self.number_waypoints = 0

        # Setup publisher
        self.pub_car = rospy.Publisher(rospy.get_param('viz_car_topic'), Marker, queue_size=1, latch=True)
        self.pub_waypoints = rospy.Publisher(rospy.get_param('viz_waypoints_topic'), Marker, queue_size=1, latch=True)
        self.pub_lookahead = rospy.Publisher(rospy.get_param('viz_lookahead_topic'), Marker, queue_size=1, latch=True)
        self.pub_traffic_lights = rospy.Publisher(rospy.get_param('viz_traffic_light_topic'), Marker, queue_size=1, latch=True)
        self.pub_pedestrians = rospy.Publisher(rospy.get_param('viz_pedestrian_topic'), Marker, queue_size=1, latch=True)
        self.pub_text_info = rospy.Publisher(rospy.get_param('viz_text_topic'), Marker, queue_size=1, latch=True)

        # Setup subscribers
        set_subscribers([
            (rospy.get_param('odom_topic'), Odometry, self._odom_callback, False),
            (rospy.get_param('path_topic'), Path, self._path_callback, False),
            (rospy.get_param('traffic_lights_topic'), String, self._traffic_lights_callback, False),
            (rospy.get_param('pedestrian_topic'), String, self._pedestrians_callback, False),
            (rospy.get_param('emergency_flag_topic'), Bool, self._emergency_flag_callback, False),
            ('lookahead_point', Point, self._lookahead_point_callback, False),
            ('active_waypoints', Path, self._active_waypoints_callback, False),
            ('visible_waypoints', Path, self._visible_waypoints_callback, False),
        ], subscribe=True)


    def _emergency_flag_callback(self, emergency_flag_msg):
        self.emergency = emergency_flag_msg.data
   
    def _odom_callback(self, odom_msg):
        """
        Visualize car. Convert odometry to marker information
        """

        # update class params
        self.position = odom_msg.pose.pose.position
        self.global_yaw = euler_from_quaternion((
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z,
            odom_msg.pose.pose.orientation.w
        ))[2]

        # create and publish marker message (position/rotation)
        msg = Marker()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        msg.id = 0
        msg.type = Marker.CUBE
        msg.action = Marker.MODIFY
        msg.pose = odom_msg.pose.pose
        msg.scale = Vector3(0.5, 0.2, 0.1)
        msg.color = ColorRGBA(1., 0., 0., 1.) if self.emergency else ColorRGBA(1., 0.8, 0.8, 1.) # white normal, pink for emergency
        msg.lifetime = rospy.Duration(0)
        self.pub_car.publish(msg)


        # create and publish car text information
        car_speed = np.linalg.norm([odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y])
        msg = Marker()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.id = 0
        msg.type = Marker.TEXT_VIEW_FACING
        msg.action = Marker.MODIFY
        msg.text = "Car Speed: %.2f\nGlobal Yaw: %.2f deg" % (car_speed, self.global_yaw*(180/math.pi))
        msg.pose = Pose(
            Point(self.position.x, self.position.y + 1.0, 0.2),
            Quaternion(0., 0., 0., 0.)
        ) 
        msg.scale.z = 0.2
        msg.color = ColorRGBA(1., 1., 1., 1.)
        msg.lifetime = rospy.Duration(0)
        self.pub_text_info.publish(msg)

        # create and publish text information for traffic light next to it
        for idx, tl in enumerate(self.traffic_lights.values()):
            # check traffic light distance from car
            traffic_light_dist = dist(
                source_pos = [self.position.x, self.position.y],
                target_pos = tl.position[:2]
            )

            msg = Marker()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "map"
            msg.id = tl.id
            msg.type = Marker.TEXT_VIEW_FACING
            msg.action = Marker.MODIFY
            msg.pose = Pose(
                Point(tl.position[0] + 1.5, tl.position[1], 0.2),
                Quaternion(0., 0., 0., 0.,)
            )
            msg.text = "ID: %d\nStatus: %s\nDistance: %.2fm" % (tl.id, tl.color, traffic_light_dist)
            msg.scale.z = 0.2
            msg.color = ColorRGBA(1., 1., 1., 1.)
            msg.lifetime = rospy.Duration(0)
            self.pub_text_info.publish(msg)

    def _path_callback(self, path_msg):
        """
        Visualize path. Convert path to marker information
        """

        # initialize message
        msg = Marker()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.id = 1
        msg.type = Marker.POINTS
        msg.action = Marker.MODIFY
        msg.scale = Vector3(0.03, 0.03, 0.03)
        msg.color = ColorRGBA(1, 0, 1, 1)       # Purple
        msg.lifetime = rospy.Duration(0)
        for pose in path_msg.poses:
            msg.points.append(pose.pose.position)

        # publish message
        self.pub_waypoints.publish(msg)

    def _traffic_lights_callback(self, tl_msg):
        # extract traffic light information (remove all \0 from ros message)
        tl_info = eval(tl_msg.data.split('\0', 1)[0])
        tl_id = TL_ID_START + int(re.search(r'_(\d+)$', tl_info['id']).group(1))
        tl_position = tl_info['position']
        tl_color = tl_info['status']

        # create/update traffic light information TODO make this more modular/efficient
        if tl_id in self.traffic_lights:
            self.traffic_lights[tl_id].update(color=tl_color)
        else:
            self.traffic_lights[tl_id] = TrafficLight(id=tl_id, position=tl_position, color=tl_color)

        # create and publish traffic light in RVIZ
        msg = Marker()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.id = tl_id
        msg.type = Marker.CYLINDER
        msg.action = Marker.MODIFY
        msg.pose = Pose(
            Point(tl_position[0], tl_position[1], 0.2),
            Quaternion(0., 0., 0., 0.,)
        )
        msg.scale = Vector3(0.15, 0.15, 0.4)
        msg.color = TRAFFIC_LIGHT_INFO[tl_color]['color']
        msg.lifetime = rospy.Duration(0)
        self.pub_traffic_lights.publish(msg)

    def _pedestrians_callback(self, pd_msg):
        # extract traffic light information (remove all \0 from ros message)
        pd_info = eval(pd_msg.data.split('\0', 1)[0])
        pd_id = PED_ID_START + int(re.search(r'_(\d+)$', pd_info['id']).group(1))
        pd_range = pd_info['range']
        pd_heading = pd_info['theta']*(math.pi/180)

        # determining pedestrian's global position
        pd_angle = self.global_yaw - pd_heading
        pd_pose = Pose(
            Point(
                self.position.x + pd_range * math.cos(pd_angle),
                self.position.y + pd_range * math.sin(pd_angle),
                0.15
            ),
            Quaternion(0., 0., 0., 0.)
        )

        # create and publish message for pedestrian visualization
        msg = Marker()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.id = pd_id
        msg.type = Marker.CUBE
        msg.action = Marker.MODIFY
        msg.pose = pd_pose
        msg.scale = Vector3(0.12, 0.12, 0.3)
        msg.color = ColorRGBA(1., 1., 0., 0.9)
        msg.lifetime = rospy.Duration(2.0)
        self.pub_pedestrians.publish(msg)
        
        # create and publish text information for pedestrian next to it
        pd_pose.position.x = pd_pose.position.x + 1.0
        msg = Marker()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.id = pd_id
        msg.type = Marker.TEXT_VIEW_FACING
        msg.action = Marker.MODIFY
        msg.text = "ID: %d\nDistance: %.2fm\nAngle w.r.t car:%.2f deg" % (pd_id, pd_range, pd_heading)
        msg.pose = pd_pose 
        msg.scale.z = 0.2
        msg.color = ColorRGBA(1., 1., 1., 1.)
        msg.lifetime = rospy.Duration(2.0)
        self.pub_text_info.publish(msg)

    def _lookahead_point_callback(self, ld_msg):
        """
        Visualize lookahead point.
        """

        # initialize message
        msg = Marker()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.id = 3
        msg.type = Marker.POINTS
        msg.action = Marker.MODIFY
        msg.scale = Vector3(0.05, 0.05, 0.05)
        msg.color = ColorRGBA(0., 1., 1., 1.)       # Cyan
        msg.lifetime = rospy.Duration(0)
        msg.pose = Pose(
            Point(ld_msg.x, ld_msg.y, ld_msg.z),
            Quaternion(0., 0., 0., 0.)
        ) 

        # publish message
        self.pub_lookahead.publish(msg)

    def _active_waypoints_callback(self, aw_msg):
        pass

    def _visible_waypoints_callback(self, vw_msg):
        pass

if __name__ == "__main__":

    # Initialize node
    rospy.init_node('visualizer', anonymous=True, log_level=VERBOSE_LEVEL)

    # Initialize class
    path_generator = Visualizer()

    # Wait for other nodes to finish
    rospy.spin()