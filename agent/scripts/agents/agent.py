# System Imports
from __future__ import print_function
import time

# External Imports
import numpy as np
import rospy

# Local Imports
from config.ros import DEBUG_FREQ, WARN_FREQ

class Agent(object):
    """
    The `Agent` class represents a base agent that provides common functionality and attributes for other agents to inherit from.

    Attributes:
        min_speed (float): The minimum speed allowed for the agent.
        max_speed (float): The maximum speed allowed for the agent.
        min_steering_angle (float): The minimum steering angle allowed for the agent.
        max_steering_angle (float): The maximum steering angle allowed for the agent.
        safe_speed (float): The safe speed calculated as 0.1 times the maximum speed.

    Methods:
        decide(observation):
            Decide on the action to take given the observation. This method automatically applies clipping to the action values.
        _decide(observation):
            Placeholder method that should be overridden by child classes to determine the action to take based on the agent's logic.
        _clip(action):
            Clips the action values to the specified speed and steering angle bounds.

    Usage:
        # Create a base agent object (Note: Agent is not intended to be used directly)
        agent = Agent()
    """

    def __init__(self, bridge):
        """
        Initialize the `Agent` object.
        """

        # parameters
        self.bridge = bridge
        self.max_speed = self.bridge.max_speed
        self.min_steering_angle = self.bridge.min_steering_angle
        self.max_steering_angle = self.bridge.max_steering_angle
        self.agent_update_period = (1 / (self.bridge.max_fps + 1e-10))      # Set agent refresh rate

        # statistics
        self.time_start = time.time()
        self.time_end = time.time()
        self.emergency_activations = 0

        rospy.loginfo(self.__class__.__name__ +  ' selected!')


    def execute_loop(self):
        
        # check starting conditions
        self._satisfy_starting_conditions()

        # start loop
        while not rospy.is_shutdown():

            # check if mission is complete
            if self._check_mission_complete():
                self._step([0., 0.])
                self._print_stats()
                # self._save_stats()
                return
            
            # check if mission is failed
            if self._check_mission_failed():
                self._step([0., 0.])
                self._print_stats()
                # self._save_stats()
                return

            # produce normal action pipeline as given by agent.
            # NEW: if emergency system is enabled and emergency occurs,
            #      the normal action will be overridden by the safety filter system
            action, info = self.decide(self.bridge.observation)
            # rospy.logdebug_throttle(DEBUG_FREQ, "[%s]: Action (%s)", self.__class__.__name__, action)

            # broadcast info
            self._broadcast(info)

            # apply action (send action to ROS)
            self._step(action)

            # sleep
            rospy.sleep(min(0.005, self.agent_update_period))


    # Private methods
    def _satisfy_starting_conditions(self):
        # setup custom publishers
        self._setup_custom_publishers()

        # wait for other processes
        while not self.bridge.system_ready:
            rospy.sleep(0.1)
            rospy.logwarn_throttle(WARN_FREQ, '[%s]: Waiting for Prescan!', self.__class__.__name__)

    def _setup_custom_publishers(self):
        pass

    def _check_mission_complete(self):
        pass

    def _check_mission_failed(self):
        pass

    def _is_emergency(self):
        return self.bridge.emergency_flag
    
    def _broadcast(self, info):
        self.bridge.broadcast(info)

    def _decide(self, observation):
        """
        Placeholder method that should be overridden by child classes to determine the action to take based on the agent's logic.

        Parameters:
            observation (any): The observation received from the environment.

        Returns:
            list: The action to be taken, represented as [speed, steering_angle].
        """
        pass

    def _clip(self, action):
        """
        Clips the action values to the specified speed and steering angle bounds.

        Parameters:
            action (list): The action to be clipped, represented as [speed, steering_angle].

        Returns:
            list: The clipped action values within the speed and steering angle bounds.
        """
        return np.clip(action, 
                       a_min=np.array([0., self.min_steering_angle]), 
                       a_max=np.array([self.max_speed, self.max_steering_angle])
                       ).tolist()
    
    def _step(self, action):
        self.bridge.step(action)

    def _print_stats(self):
        pass

    def _save_stats(self):
        pass

    # Public methods

    def decide(self, observation):
        """
        Decide on the action to take given the observation. This method automatically applies clipping to the action values.

        Parameters:
            observation (any): The observation received from the environment.

        Returns:
            list: The action to be taken, represented as [speed, steering_angle].
        """
        action, info = self._decide(observation)          # Call the overridden _decide method in child classes
        return self._clip(action), info                   # Apply _clip to the action and return the clipped action

