# External Imports
import rospy
import numpy as np

# Local Imports
from agent_bridge.agent_bridge import AgentBridge
from agents.agent import Agent


class StraightAgent(Agent):
    """
    The `StraightAgent` class represents an agent that moves straight at a constant speed.

    This agent extends the `Agent` class and inherits its methods and attributes.

    Parameters:
        None

    Attributes:
        None

    Methods:
        _decide(observation):
            Returns an action of moving straight at a constant speed.

    Usage:
        agent = StraightAgent()
        action = agent._decide(observation)
    """

    def __init__(self, bridge):
        """
        Initialize the `StraightAgent` object.

        Parameters:
            min_speed (float): The minimum speed allowed for the agent. Default is -1.0.
            max_speed (float): The maximum speed allowed for the agent. Default is 1.0.
            min_steering_angle (float): The minimum steering angle allowed for the agent. Default is -pi/5.
            max_steering_angle (float): The maximum steering angle allowed for the agent. Default is pi/5.

        Returns:
            None
        """
        super(StraightAgent, self).__init__(bridge)

    def _decide(self, observation):
        """
        Decide on the action to take given the observation.

        This method returns an action of moving straight at a constant speed.

        Parameters:
            observation (any): The observation received from the environment.

        Returns:
            list: An action of [max_speed, 0.], indicating moving straight at the maximum speed.
        """
        return [self.max_speed, 0.], {}