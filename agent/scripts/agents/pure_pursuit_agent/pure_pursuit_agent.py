# MIT License

# Copyright (c) 2020 Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

# Internal Imports
import json
import os
import time

# External Imports
import rospy
import numpy as np

# Local Imports
from agent_bridge.agent_bridge import AgentBridge
from agents.agent import Agent
import agents.pure_pursuit_agent.pure_pursuit_helper as helper
from config.ros import DEBUG_FREQ, WARN_FREQ, RosMessageType
from utils.tools import calculate_curvature

class PurePursuitAgent(Agent):
    """
    The `PurePursuitAgent` class represents an agent that follows the Pure Pursuit algorithm to make decisions.

    This agent extends the `Agent` class and inherits its methods and attributes.

    Parameters:
        None

    Attributes:
        max_speed (float): The maximum speed allowed for the agent.

    Methods:
        _decide(observation):
            Determines the action to take based on the Pure Pursuit algorithm.

    Usage:
        agent = PurePursuitAgent()
        action = agent._decide(observation)
    """

    def __init__(self,
                 bridge,
                 min_lookahead_distance=0.8,
                 max_lookahead_distance=2.0,
                 wheelbase=0.25,
                 max_reacquire=100.0,
                 waypoint_complete_dist=0.5,
                 max_unaffected_speed=1.2,
                 steering_threshold=0.4):
        """
        Initialize the `PurePursuitAgent` object.

        Parameters:
            min_speed (float): The minimum speed allowed for the agent. Default is -1.0.
            max_speed (float): The maximum speed allowed for the agent. Default is 1.0.
            min_steering_angle (float): The minimum steering angle allowed for the agent. Default is -pi/5.
            max_steering_angle (float): The maximum steering angle allowed for the agent. Default is pi/5.

        Returns:
            None
        """
        super(PurePursuitAgent, self).__init__(bridge)


        # Algorith parameters
        if self.bridge.sloth_mode:
            self.min_lookahead_distance = 0.1
            self.max_lookahead_distance = 0.1
            self.waypoint_complete_dist = 0.1
        else:
            self.min_lookahead_distance = min_lookahead_distance
            self.max_lookahead_distance = max_lookahead_distance
            self.waypoint_complete_dist = waypoint_complete_dist
        self.lookahead_distance = (self.min_lookahead_distance + self.max_lookahead_distance) / 2
        self.lookahead_point = [0., 0.]

        self.wheelbase = wheelbase
        self.max_reacquire = max_reacquire
        self.max_unaffected_speed = max_unaffected_speed
        self.steering_threshold = steering_threshold

        # Initialize waypoints
        self.waypoints_completed = 0
        self.waypoints_reached = 0
        self.num_visible_wpts = self.bridge.num_visible_wpts


    def _satisfy_starting_conditions(self):
        # satisfy the upper class conditions
        super(PurePursuitAgent, self)._satisfy_starting_conditions()
        # wait for bridge to receive waypoint path information
        while (self.bridge.waypoints is None):
            rospy.sleep(0.5)
            rospy.logwarn_throttle(WARN_FREQ, '[%s]: Pure Pursuit waiting for waypoints', self.__class__.__name__)
        # build waypoints array
        self.update_waypoints()

    def _setup_custom_publishers(self):
        self.bridge.setup_custom_publishers({
            # "active_waypoints" : RosMessageType.PATH.value,
            # "visible_waypoints" : RosMessageType.PATH.value,
            "lookahead_point": RosMessageType.POINT.value
        })
    
    def _check_mission_complete(self):
        # Check if all waypoints are reached
        if len(self.active_waypoints) > 1:
            return False
        # mission completed: measure total time and number of emergency activations
        self.time_end = time.time() - self.time_start
        self.emergency_activations = self.bridge.emergency_activations
        rospy.loginfo("[%s]: Mission complete! All waypoints reached!", self.__class__.__name__)
        return True
    
    def _check_mission_failed(self):
        return False
    
    def update_waypoints(self):
        """Update waypoints when change map"""
        self.waypoints = self.bridge.waypoints
        self.active_waypoints = self.bridge.waypoints
        self.visible_waypoints = self.active_waypoints[:min(self.num_visible_wpts, len(self.active_waypoints)), :]

    def _pass_waypoint(self, index):
        # print('self.waypoints:', len(self.waypoints), 'self.active_waypoints:', len(self.active_waypoints), 'self.visible_waypoints:', len(self.visible_waypoints), 'index to delete:', index)
        self.active_waypoints = self.active_waypoints[index:]
        
        self.active_waypoints = np.delete(arr=self.active_waypoints, obj=index, axis=0)

        if len(self.active_waypoints) < self.num_visible_wpts:
            self.visible_waypoints = self.active_waypoints[:len(self.active_waypoints)]
            new_points = self.bridge.waypoints[:(self.num_visible_wpts - len(self.visible_waypoints)), :]
            self.visible_waypoints = np.concatenate((self.visible_waypoints, new_points), axis=0)
        else:
            self.visible_waypoints = self.active_waypoints[:self.num_visible_wpts, :]

        self.waypoints_completed += 1
        rospy.loginfo_throttle(DEBUG_FREQ, "[%s]: W:%d/%d (%.1f%%)" % (self.__class__.__name__, self.waypoints_completed, self.waypoints_completed+len(self.active_waypoints), (100*self.waypoints_completed/len(self.waypoints))))

        # When sloth_mode mode is active, stop at every waypoint for 2 seconds
        if self.bridge.sloth_mode:
            self.bridge.step([0. ,0.])
            rospy.sleep(0.5)
    
    def _get_lookahead_point(self, waypoints, position):        
        # Find nearest point/distance
        wpts = waypoints[:, 0:2]

        nearest_point, nearest_dist, t, i = helper.nearest_point_on_trajectory_py2(position, wpts)
        # rospy.logdebug_throttle(DEBUG_FREQ, "Neareset dist: %.2f", nearest_dist)

        # If it's too near, consider the waypoint passed
        if nearest_dist <= self.waypoint_complete_dist:
            self._pass_waypoint(i)

        # If within lookahead distance, interpolate the point
        if nearest_dist < self.lookahead_distance:
            lookahead_point, i2, t2 = helper.first_point_on_trajectory_intersecting_circle(position, self.lookahead_distance, wpts, i+t, wrap=True)
            if i2 == None:
                return None
            current_waypoint = np.empty(shape=waypoints[i2, :].shape)
            # x, y (waypoint)
            current_waypoint[0:2] = lookahead_point
            current_waypoint[2] = waypoints[i2, 2]
            return current_waypoint
        
        # If it is outside the lookahead distance, just set the point as the actual physical waypoint
        elif nearest_dist < self.max_reacquire:
            return waypoints[i, :]
        
        # If it is outside the reacquire distance, return nothing
        else:
            return None


    def _decide(self, observation):
        """
        Determine the action to take based on the Pure Pursuit algorithm.

        Parameters:
            observation (any): The observation received from the environment.

        Returns:
            list: The action to be taken, represented as [speed, steering_angle].
        """
        pose_x = observation['pose_x']
        pose_y = observation['pose_y']

        position = np.array([pose_x, pose_y])       # car position
        pose_theta = observation['pose_theta']      # car steering angle

        waypoints = self.visible_waypoints
        # rospy.logdebug_throttle(DEBUG_FREQ, "%s", waypoints)

        # calculate lookahead distance dynamically
        density = 5 # TODO make this dynamic from prescan map density
        upcoming_curvature = calculate_curvature(waypoints[:2*density+1:density, :2])
        self.lookahead_distance = self.max_lookahead_distance - (self.max_lookahead_distance - self.min_lookahead_distance) * upcoming_curvature
        # rospy.logdebug_throttle(DEBUG_FREQ, "[%s]: Upcoming track curvature:%.2f, Look-ahead distance:%.2f", self.__class__.__name__, upcoming_curvature, self.lookahead_distance)

        self.lookahead_point = self._get_lookahead_point(waypoints, position)
        if self.lookahead_point is None:
            rospy.logwarn_throttle(1, "No lookahead point found! Reducing speed to safe speed.")
            return [0.0, 0.0], {}
        
        steering_angle = helper.get_actuation(pose_theta, self.lookahead_point, position, self.lookahead_distance, self.wheelbase)
        speed_reduction_factor = max(0, 1 - (abs(steering_angle) - self.steering_threshold)/(self.max_steering_angle - self.steering_threshold))
        speed = min(self.max_speed, max(self.max_unaffected_speed, self.max_speed*speed_reduction_factor))
        return [speed, -steering_angle], {
            # "active_waypoints" : self.active_waypoints,
            # "visible_waypoints" : self.visible_waypoints,
            "lookahead_point": self.lookahead_point}


    def _print_stats(self):
        super(PurePursuitAgent, self)._print_stats()

        rospy.loginfo("[%s]: -------------------------------------", self.__class__.__name__)
        rospy.loginfo("[%s]:   M I S S I O N   C O M P L E T E D  ", self.__class__.__name__)
        rospy.loginfo("[%s]: -------------------------------------", self.__class__.__name__)
        rospy.loginfo("[%s]: Min look-ahead distance: %.2f", self.__class__.__name__, self.min_lookahead_distance)
        rospy.loginfo("[%s]: Max look-ahead distance: %.2f", self.__class__.__name__, self.max_lookahead_distance)
        rospy.loginfo("[%s]: -------------------------------------", self.__class__.__name__)
        rospy.loginfo('[%s]: Emergency activations: %.2fs' % (self.__class__.__name__, self.emergency_activations))
        rospy.loginfo('[%s]: Total time elapsed: %.2fs' % (self.__class__.__name__, self.time_end))
        rospy.loginfo("[%s]: -------------------------------------", self.__class__.__name__)


    def _save_stats(self):
        super(PurePursuitAgent, self)._save_stats()

        stats = {
            'min_lookahead_distance' : self.min_lookahead_distance,
            'min_lookahead_distance' : self.max_lookahead_distance,
            'emergency_activations' : self.emergency_activations,
            'time_end' : self.time_end
        }

        # save stats to file
        root_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
        with open(os.path.join(root_path, 'stats', 'ld_%s.json' % (str(self.lookahead_distance))), 'w') as fp:
            json.dump(stats, fp, indent=4)