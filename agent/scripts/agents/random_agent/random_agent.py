# External Imports
import rospy
import numpy as np

# Local Imports
from agent_bridge.agent_bridge import AgentBridge
from agents.agent import Agent


class RandomAgent(Agent):
    """
    The `RandomAgent` class represents an agent that takes random actions within specified bounds.

    This agent extends the `Agent` class and inherits its methods and attributes.

    Parameters:
        None

    Attributes:
        start (float): The start time of the current action.
        action_duration (float): The duration of each action in seconds.

    Methods:
        _decide(observation):
            Returns a random action within the speed and steering angle bounds.

    Usage:
        agent = RandomAgent()
        action = agent._decide(observation)
    """

    def __init__(self, bridge, action_duration=0.8):
        """
        Initialize the `RandomAgent` object.

        Parameters:
            min_speed (float): The minimum speed allowed for the agent. Default is -1.0.
            max_speed (float): The maximum speed allowed for the agent. Default is 1.0.
            min_steering_angle (float): The minimum steering angle allowed for the agent. Default is -pi/5.
            max_steering_angle (float): The maximum steering angle allowed for the agent. Default is pi/5.

        Returns:
            None
        """
        super(RandomAgent, self).__init__(bridge)
        self.start = rospy.get_time()
        self.action_duration = action_duration
        self.action = [0., 0.]

    def _decide(self, observation):
        """
        Decide on a random action within the specified speed and steering angle bounds.

        Parameters:
            observation (any): The observation received from the environment.

        Returns:
            list: A random action within the speed and steering angle bounds.
        """
        if rospy.get_time() - self.start >= self.action_duration:
            rospy.logdebug("Time to change random speed and steering angle")
            angle_direction = np.sign(np.random.rand()-0.5)
            self.action = [self.max_speed/2 + np.random.rand() * (self.max_speed/2), np.random.rand() * self.max_steering_angle * angle_direction]
            self.start = rospy.get_time()
        
        return self.action, {}