# External Imports
import numpy as np

# Local Imports
from agent_bridge.agent_bridge import AgentBridge
from agents.agent import Agent


class IdleAgent(Agent):
    """
    The `IdleAgent` class represents an idle agent that does not take any actions.

    This agent extends the `Agent` class and inherits its methods and attributes.

    Parameters:
        None

    Attributes:
        None

    Methods:
        _decide(observation):
            Returns a default action of [0., 0.] without any decision-making based on the observation.

    Usage:
        agent = IdleAgent()
        action = agent._decide(observation)
    """

    def __init__(self, bridge):
        """
        Initialize the `IdleAgent` object.

        Parameters:
            min_speed (float): The minimum speed allowed for the agent. Default is -1.0.
            max_speed (float): The maximum speed allowed for the agent. Default is 1.0.
            min_steering_angle (float): The minimum steering angle allowed for the agent. Default is -pi/5.
            max_steering_angle (float): The maximum steering angle allowed for the agent. Default is pi/5.

        Returns:
            None
        """
        super(IdleAgent, self).__init__(bridge)

    def _decide(self, observation):
        """
        Decide on the action to take given the observation.

        This method returns a default action of [0., 0.] without any decision-making based on the observation.

        Parameters:
            observation (any): The observation received from the environment.

        Returns:
            list: A default action of [0., 0.] indicating no action to be taken.
        """
        return [0., 0.], {}