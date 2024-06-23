#!/usr/bin/env python
from __future__ import print_function

# System Libraries
import os, json
import math
import re
import sys; print(sys.version); sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# External Libraries
import numpy as np
import rospy
from geometry_msgs.msg import Twist, Point, Vector3, Quaternion, PoseStamped, TwistStamped
from std_msgs.msg import Bool, String
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

# Local Imports
from env.pedestrian import Pedestrian
from env.traffic_light import TrafficLight
from env.driver import Driver

from utils.rostools import set_subscribers
from utils import tools as tls
from config.ros import VERBOSE_LEVEL
from machine import MooreStateMachine

cur_dir = os.getcwd()


class SafetyFilter(object):
    
    def __init__(self):

        # initialize dynamic data
        self.env = rospy.get_param('env')
        self.car_id = str(rospy.get_param('car_id'))
        self.position = Point(0., 0., 0.)
        self.yaw = 0.0
        self.entities = {}
	self.sm = MooreStateMachine('/home/student/catkin_ws/src/agent/scripts/safety/sm_tables.json')

        self.max_speed = rospy.get_param('max_speed') if self.env == 'sim' else rospy.get_param('vesc_driver/speed_max')
            
        self.emergency_callbacks = [
            self._check_tl_emergency,
            self._check_pd_emergency,
            self._check_driver_emergency,
        ]

        self.emergency_state = np.zeros(shape=(len(self.emergency_callbacks),), dtype=bool)
        self.nearest_dists = np.zeros(shape=(len(self.emergency_callbacks),), dtype=float)
        self.nearest_objects = np.zeros(shape=(len(self.emergency_callbacks),), dtype=object)

        # Setup publishers
        self.pub_flag = rospy.Publisher(rospy.get_param("emergency_flag_topic"), Bool, queue_size=1, latch=True)
        self.pub_drive = rospy.Publisher(rospy.get_param("drive_final_topic"), Twist, queue_size=1, latch=True)

        # Setup subscribers
        set_subscribers([
            (rospy.get_param('odom_topic'), Odometry, self._odom_callback, True),
            (rospy.get_param('drive_raw_topic'), Twist, self._filter_callback, True),
            (rospy.get_param('traffic_lights_topic'), String, self._traffic_lights_callback, False),
            (rospy.get_param('pedestrian_topic'), String, self._pedestrians_callback, False)
        ], subscribe=True)

        # determine the number of other drivers from VICON
        pattern = r"\/vrpn_client_node\/Car_(\d)_Tracking\/(\w+)"
        topic_list = rospy.get_published_topics()
        driver_topics = {}
        for topic in topic_list:
            try:
                match = re.match(pattern, topic)
                if match: #and match.group(1) != self.car_id:
                    driver_topics[match.group(1)][match.group(2)] = topic
            except:
                continue
        for id, topics in driver_topics.items():
            rospy.Subscriber(topics['pose'], PoseStamped, self._drivers_pose_callback, callback_args=id)
            rospy.Subscriber(topics['twist'], TwistStamped, self._drivers_twist_callback, callback_args=id)
        self.latest_pose = {}  # Dictionary to store the latest pose message for each car
        self.latest_twist = {}  # Dictionary to store the latest twist message for each car

        # setup timer callback
        rospy.Timer(rospy.Duration(0.1), self._check_pedestrian_timeout)


    # Subscriber Callbacks

    def _odom_callback(self, msg):
        self.position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

    def _traffic_lights_callback(self, msg):
        # extract traffic light information (remove all \0 from ros message)
        tl_info = eval(msg.data.split('\0', 1)[0])
        tl_id = tl_info['id']
        tl_position = tl_info['position']
        tl_color = tl_info['status']

        # create/update traffic light information
        if tl_id in self.entities:
            self.entities[tl_id].update(color=tl_color)
        else:
            self.entities[tl_id] = TrafficLight(id=tl_id, 
                                                position=tl_position, 
                                                color=tl_color)

    def _pedestrians_callback(self, msg):
        # extract pedestrians information (remove all \0 from ros message)
        pd_info = eval(msg.data.split('\0', 1)[0])
        pd_id = pd_info['id']
        pd_distance = pd_info['range']
        pd_velocity = pd_info['vel']
        pd_heading = np.degrees(pd_info['theta'])
        pd_azimuth = np.degrees(pd_info['heading'])

        # create/update pedestrian information
        if pd_id in self.entities:
            self.entities[pd_id].update(velocity=pd_velocity, 
                                        distance=pd_distance, 
                                        azimuth=pd_azimuth, 
                                        heading=pd_heading)
        else:
            self.entities[pd_id] = Pedestrian(id=pd_id, 
                                              velocity=pd_velocity, 
                                              distance=pd_distance, 
                                              azimuth=pd_azimuth,
                                              heading=pd_heading)

    def _drivers_pose_callback(self, msg, car_id):
        # Store the latest pose message
        self.latest_pose[car_id] = msg

        # Update the entity if the latest twist message is available
        if car_id in self.latest_twist:
            self._drivers_callback(car_id)
        
    def _drivers_twist_callback(self, msg, car_id):
        # Store the latest twist message
        self.latest_twist[car_id] = msg

        # Update the entity if the latest pose message is available
        if car_id in self.latest_pose:
            self._drivers_callback(car_id)

    def _drivers_callback(self, car_id):
        # Extract information from the latest messages
        pose_msg = self.latest_pose[car_id]
        twist_msg = self.latest_twist[car_id]

        pos = [pose_msg.pose.position.x, pose_msg.pose.position.y]
        rot = pose_msg.pose.orientation
        _, _, heading = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
        speed = twist_msg.twist.linear.x

        car_entity_id = 'car_' + car_id


        # calculate distance, azimuth from source position and heading
        distance = tls.dist(source_pos=[self.position.x, self.position.y],
                        target_pos=pos)
        azimuth = tls.azimuth(source_pos=[self.position.x, self.position.y],
                              target_pos=pos,
                              source_heading=self.yaw)

        # Create or update the driver entity
        if car_entity_id in self.entities:
            self.entities[car_entity_id].update(velocity=speed,
                                                distance=distance,
                                                azimuth=azimuth,
                                                heading=heading)
        else:
            self.entities[car_entity_id] = Driver(id=car_entity_id,
                                                  velocity=speed,
                                                  distance=distance,
                                                  azimuth=azimuth,
                                                  heading=heading)


    def _filter_callback(self, msg):
        # Inspect normal action
        speed = msg.linear.x
        steering_angle = msg.angular.z
        deceleration = 0
        
        # Check for emergencies whenever you get a new action from agent
        for i in range(len(self.emergency_callbacks)):
            self.emergency_state[i], self.nearest_dists[i], self.nearest_objects[i] = self.emergency_callbacks[i]()

        # rospy.logdebug_throttle(1, 'Emergency States: %s, Nearest Distances: %s' % (self.emergency_state, self.nearest_dists))

        em_idx = np.where(self.emergency_state == True)[0]
        em_entities = self.nearest_dists[em_idx]
        em_objects = self.nearest_objects[em_idx]

        # if there are emergency entities, reduce speed
        if em_entities.size != 0:
            rospy.logwarn_throttle(0.5, 'Emergency detected!')
            current_min_idx = np.argmin(em_entities)
            current_min_dist = em_entities[current_min_idx]
            current_closest_obj = em_objects[current_min_idx]

            # rospy.logwarn_throttle(0.5, 'Current min dist: %.2f, Stop dist: %.2f' % (current_min_dist, current_closest_obj.stop_dist))
	    
            # Check if within stopping range
            #if self.env == 'sim':
                #speed = 0.0
            if current_min_dist < current_closest_obj.min_stop_dist:
		# Take emergency stop
                self.sm.reaction(False, True)
            elif current_min_dist < current_closest_obj.stop_dist:
		# Decelerate, no emergency stop
		self.sm.reaction(True, False)		
            else:
		# No need to decelerate nor stop
		self.sm.reaction(False, False)

	    out = self.sm.get_output()

	    if out == 'STOP':
		speed = 0.0
	    elif out == 'DECEL':

                # Calculate deceleration needed based on distance to the nearest entity and stopping distance
                deceleration = (speed ** 2) / (2 * (current_min_dist - current_closest_obj.min_stop_dist))
                
                # rospy.logwarn_throttle(0.5, 'Speed: %.2f, Deceleration Rate:%.2f' % (speed, deceleration))

                # Apply the deceleration
                speed -= deceleration

                # Ensure speed does not go below zero
                speed = np.clip(speed, 0, self.max_speed)
	    
        # Create message and publish action
        filtered_drive_msg = Twist(Vector3(speed, 0., 0.), Vector3(0., 0., steering_angle))
        flag_msg = Bool(np.any(self.emergency_state))
        self.pub_drive.publish(filtered_drive_msg)
        self.pub_flag.publish(flag_msg)

        
    # Helper functions

    def _check_tl_emergency(self):
        # extract traffic light information
        tl_emergency_color_matrix = np.array([tl.is_emergency_color() for tl in self.entities.values() if isinstance(tl, TrafficLight)])
        tl_dist_matrix = np.array([tl.get_distance(source_pos=[self.position.x, self.position.y]) for tl in self.entities.values() if isinstance(tl, TrafficLight)])
        tl_near_matrix = np.array([tl.is_near(source_pos=[self.position.x, self.position.y]) for tl in self.entities.values() if isinstance(tl, TrafficLight)])
        tl_in_front_matrix = np.array([tl.is_in_front(source_pos=[self.position.x, self.position.y], source_heading=self.yaw) for tl in self.entities.values() if isinstance(tl, TrafficLight)])
        tl_id_matrix = np.array([tl.id for tl in self.entities.values() if isinstance(tl, TrafficLight)])

        if tl_id_matrix.size == 0:
            return False, np.Inf, None

        # build emergency matrix
        tl_emergency_matrix = tl_emergency_color_matrix & tl_near_matrix & tl_in_front_matrix

        # rospy.logdebug_throttle(1, '[TL]: Em Matrix: %s, Color Matrix: %s, Low Distance Matrix: %s, Angle Matrix: %s' % (tl_emergency_matrix, tl_emergency_color_matrix, (tl_dist_matrix <= stop_dist), (np.abs(tl_angle_matrix) <= crit_angle)))
        
        nearest_obj= self.entities[tl_id_matrix[np.argmin(tl_near_matrix)]]
        return np.any(tl_emergency_matrix), tl_dist_matrix.min(), nearest_obj

    def _check_pd_emergency(self):
        pd_dist_matrix = np.array([pd.get_distance() for pd in self.entities.values() if isinstance(pd, Pedestrian)])
        pd_near_matrix = np.array([pd.is_near() for pd in self.entities.values() if isinstance(pd, Pedestrian)])
        pd_id_matrix = np.array([pd.id for pd in self.entities.values() if isinstance(pd, Pedestrian)])

        if pd_id_matrix.size == 0:
            return False, np.Inf, None
        
        # rospy.logdebug_throttle(1, '[PD]: Em Matrix: %s' % (pd_em_matrix))

        nearest_obj = self.entities[pd_id_matrix[np.argmin(pd_dist_matrix)]]
        return np.any(pd_near_matrix), pd_dist_matrix.min(), nearest_obj

    def _check_driver_emergency(self):
        dr_same_dir_matrix = np.array([dr.is_same_direction() for dr in self.entities.values() if isinstance(dr, Driver)])
        dr_dist_matrix = np.array([dr.get_distance() for dr in self.entities.values() if isinstance(dr, Driver)])
        dr_near_matrix = np.array([dr.is_near() for dr in self.entities.values() if isinstance(dr, Driver)])
        dr_id_matrix = np.array([dr.id for dr in self.entities.values() if isinstance(dr, Driver)])

        if dr_id_matrix.size == 0:
            return False, np.Inf, None
        
        dr_emergency_matrix = dr_same_dir_matrix & dr_near_matrix

        nearest_obj = self.entities[dr_id_matrix[np.argmin(dr_dist_matrix)]]
        return np.any(dr_emergency_matrix), dr_dist_matrix.min(), nearest_obj
        

    def _check_pedestrian_timeout(self, event):
        to_delete = [pid for pid, pd in self.entities.items() if isinstance(pd, Pedestrian) and pd.is_obsolete()]
        for pid in to_delete:
            del self.entities[pid]


if __name__ == "__main__":

    # Initialize safety node and class
    rospy.init_node('safety_node', anonymous=True, log_level=VERBOSE_LEVEL)

    # initialize safety node object
    safety_node = SafetyFilter()

    # wait until other nodes terminate
    rospy.spin()
