# system imports
import rospy

# local imports
from .agent_bridge import AgentBridge

class AgentBridgeSim(AgentBridge):

    def __init__(self):
        super(AgentBridgeSim, self).__init__()

        # parameters
        self.max_speed = rospy.get_param('max_speed')
            
    def step(self, action):
        super(AgentBridgeSim, self).step(action)

    def reset(self):
        super(AgentBridgeSim, self).reset()
