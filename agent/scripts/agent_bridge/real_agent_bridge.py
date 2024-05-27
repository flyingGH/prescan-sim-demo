# system imports
import rospy

# local imports
from .agent_bridge import AgentBridge

class AgentBridgeReal(AgentBridge):

    def __init__(self):
        super(AgentBridgeReal, self).__init__()

        # parameters
        self.max_speed = rospy.get_param('sloth_speed') if self.sloth_mode else rospy.get_param('vesc_driver/speed_max')

    def step(self, action):
        super(AgentBridgeReal, self).step(action)

    def reset(self):
        super(AgentBridgeReal, self).reset()
