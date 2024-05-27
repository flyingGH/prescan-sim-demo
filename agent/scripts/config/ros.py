import rospy
from enum import Enum


# ROS verbosity settings
VERBOSE_LEVEL = rospy.DEBUG

# ROS Logging Parameters
DEBUG_FREQ = 1; WARN_FREQ = 1; INFO_FREQ = 1

# ROS subscriber parameters
TIMEOUT_READY = 10


# Ros messages
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import ColorRGBA, Bool

class RosMessageType(Enum):
    POINT = Point
    PATH = Path
    TWIST = Twist