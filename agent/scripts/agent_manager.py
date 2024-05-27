#!/usr/bin/env python

# System Imports
from __future__ import print_function
import os
import sys; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# External Imports
import rospy

# Local Imports
from agent_bridge.real_agent_bridge import AgentBridgeReal
from agent_bridge.sim_agent_bridge import AgentBridgeSim
from agents.idle_agent.idle_agent import IdleAgent
from agents.random_agent.random_agent import RandomAgent
from agents.straight_agent.straight_agent import StraightAgent
from agents.pure_pursuit_agent.pure_pursuit_agent import PurePursuitAgent
from config.ros import VERBOSE_LEVEL


class AgentManager():

    def __init__(self):

        # Initialize bridge environment
        env = rospy.get_param('env')
        if env == 'sim':
            self.bridge = AgentBridgeSim()
        elif env == 'real_car':
            self.bridge = AgentBridgeReal()
        else:
            raise ValueError('Unknown environment')

        # Initialize agent
        agent = rospy.get_param('agent')
        if agent == "straight":
            self.agent = StraightAgent(self.bridge)
        elif agent == "pure_pursuit":
            self.agent = PurePursuitAgent(self.bridge,
                                          min_lookahead_distance=rospy.get_param("min_lookahead_distance"),
                                          max_lookahead_distance=rospy.get_param("max_lookahead_distance"),
                                          wheelbase=1.0,
                                          max_reacquire=rospy.get_param("max_reacquire"),
                                          waypoint_complete_dist=rospy.get_param("waypoint_complete_dist"))
        elif agent == "idle":
            self.agent = IdleAgent(self.bridge)
        elif agent == "random":
            self.agent = RandomAgent(self.bridge)
        else:
            raise ValueError("Agent not valid")

    def initiate_loop(self):
        self.agent.execute_loop()

        
if __name__ == "__main__":
    # initialize ROS node
    rospy.init_node('agent_manager', anonymous=True, log_level=VERBOSE_LEVEL)

    # initiate agent and start loop
    am = AgentManager()

    try:
        am.initiate_loop()
    except rospy.ROSInterruptException:     # terminate ROS
        rospy.logwarn('[%s] : Initiating shutdown sequence!', __file__)
        rospy.signal_shutdown('Shutting down')
    # wait for other nodes
    # rospy.spin()