#!/usr/bin/env python
from __future__ import print_function
import sys
print(sys.version)


# ======================= I M P O R T S =======================

# External Libraries
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

# ==================== P A R A M E T E R S ====================

# Misc
VERBOSE_LEVEL = rospy.DEBUG

class CmdvelToVesc(object):
      
    def __init__(self):
            
        # Get Parameters
        self.motor_speed_topic = rospy.get_param("motor_speed_topic")
        self.servo_position_topic = rospy.get_param("servo_position_topic")
        self.cmd_vel_topic = rospy.get_param("drive_final_topic")

        # Set up subscriber
        rospy.Subscriber(self.cmd_vel_topic, Twist, self._twist_callback)

        # Set up publisher
        self.pub_motor_speed = rospy.Publisher(self.motor_speed_topic, Float64, queue_size = 1)
        self.pub_servo_position = rospy.Publisher(self.servo_position_topic, Float64, queue_size = 1)

    def _twist_callback(self, data):
        # Get twist data
        self.twist_speed = data.linear.x
        self.twist_steering_angle = data.angular.z

        # Calculate motor speed & servo position
        self.desired_motor_speed = 0 + 10000 * self.twist_speed
        self.desired_servo_position = 0.5 + 0.35 * self.twist_steering_angle

        # Publish motor speed & servo position (Float64 messages)
        self.pub_motor_speed.publish(self.desired_motor_speed)
        self.pub_servo_position.publish(self.desired_servo_position)
        

if __name__ == "__main__":

    # Initialize node
    rospy.init_node('cmd_vel_to_vesc', anonymous=True, log_level=VERBOSE_LEVEL)

    # Initialize class
    cmd_vel_to_vesc = CmdvelToVesc()

    # Wait for other nodes to finish
    rospy.spin()