# System Libraries
from __future__ import print_function
import sys; print(sys.version)

# External Libraries
import numpy as np
import rospy
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import ColorRGBA, Bool
from visualization_msgs.msg import Marker
from tf.transformations import euler_from_quaternion
from tf import TransformBroadcaster

# Local Libraries
from utils.rostools import set_subscribers, create_message

class AgentBridge(object):

    def __init__(self):
        
        # general parameters
        self.agent_name = rospy.get_param('agent')
        self.max_fps = rospy.get_param('max_fps')
        self.num_visible_wpts = rospy.get_param("visible_waypoints")
        self.safety = rospy.get_param("safety")
        self.offline = rospy.get_param("offline")
        
        # speed parameters
        self.sloth_mode = rospy.get_param('sloth_mode', default=False)
        self.min_steering_angle = -1
        self.max_steering_angle = 1

        # class-related vars
        self.observation = {}
        self.done = False
        self.system_ready = False
        self.emergency_flag = False
        self.prev_emergency_flag = False
        self.emergency_activations = 0


        # publishers
        if self.safety:
            self.pub_drive = rospy.Publisher(rospy.get_param('drive_raw_topic'), Twist, queue_size=1)
        else:
            self.pub_drive = rospy.Publisher(rospy.get_param('drive_final_topic'), Twist, queue_size=1)


        self.pubs_custom = {}

        self.br = TransformBroadcaster()

        # initiate waypoints
        if self.agent_name == 'pure_pursuit':
            path_msg = rospy.wait_for_message(rospy.get_param('path_topic'), Path, timeout=180)

            if path_msg is None:
                rospy.logerr("No path message received!")
                rospy.signal_shutdown()
            
            _wp_list = []
            for i in range(len(path_msg.poses)):
                _wp_list.append([path_msg.poses[i].pose.position.x, path_msg.poses[i].pose.position.y, path_msg.poses[i].pose.position.z])
            self.waypoints = np.array(_wp_list)
            self.observation['waypoints'] = self.waypoints
            rospy.loginfo("[%s]: Path from prescan received successfully!", self.__class__.__name__)

        # make sure that all subscribers receive messages
        set_subscribers([
            (rospy.get_param('scan_topic'),             LaserScan,  self._scan_callback,                False),
            (rospy.get_param('odom_topic'),             Odometry,   self._odom_callback,                True),
            (rospy.get_param('emergency_flag_topic'),   Bool,       self._emergency_flag_callback,      False)
        ], subscribe=True)

        if self.offline:
            self.system_ready = True
        else:
            set_subscribers([
                        (rospy.get_param('prescan_status_topic'),   Bool,       self._prescan_status_callback,      True)
                    ], subscribe=True)
        self.system_ready = True

        rospy.loginfo(self.__class__.__name__ + " initialization complete!")

    # -- Callbacks (private)
    def _path_callback(self, msg):
        _wp_list = []
        for i in range(len(msg.poses)):
            _wp_list.append([msg.poses[i].pose.position.x, msg.poses[i].pose.position.y, msg.poses[i].pose.position.z])
        self.waypoints = np.array(_wp_list)
        self.observation['waypoints'] = self.waypoints
        rospy.loginfo("[%s]: Path from prescan received successfully!", self.__class__.__name__)

    def _scan_callback(self, msg):
        # self.observation['scan'] = msg.ranges
        self.scan = msg.ranges

    def _raceinfo_callback(self, msg):
        self.observation['collision'] = msg.collision

    def _done_callback(self, msg):
        self.done = msg.data

    def _odom_callback(self, msg):
        self.observation["pose_x"] = msg.pose.pose.position.x
        self.observation["pose_y"] = msg.pose.pose.position.y

        speed_x = msg.twist.twist.linear.x
        speed_y = msg.twist.twist.linear.y
        self.speed = np.linalg.norm([speed_x, speed_y])
        self.observation["speed"] = self.speed
        self.rotation = msg.pose.pose.orientation
        _, _, self.observation["pose_theta"] = euler_from_quaternion([self.rotation.x, self.rotation.y, self.rotation.z, self.rotation.w])

        # publish broadcast for relative frame
        self.br.sendTransform(
            (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
            (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
            rospy.Time.now(),
            "base_link",        # child frame (destination)
            "map"               # source frame
        )

    def _emergency_flag_callback(self, msg):
        self.prev_emergency_flag = self.emergency_flag
        self.emergency_flag = msg.data

        if not self.prev_emergency_flag and self.emergency_flag:
            self.emergency_activations += 1

    def _prescan_status_callback(self, msg):
        self.system_ready = msg.data


    # --- Visualize (private)


    # --- Env Methods (public)

    def setup_custom_publishers(self, info):
        for k, v in info.items():
            self.pubs_custom[k] = rospy.Publisher(name='/' + k,
                                                  data_class=v,
                                                  queue_size=1,
                                                  latch=True)

    def step(self, action):
        msg = Twist()
        msg.linear.x = float(action[0])
        msg.angular.z = float(action[1])
        self.pub_drive.publish(msg)

    def broadcast(self, info):
        for k, v in info.items():
            msg = create_message(v)
            self.pubs_custom[k].publish(msg)

    def reset(self):
        pass

    def visualize_lookahead(self, lookahead_point):
        if lookahead_point is None:
            rospy.logdebug('[%s] Visualization ignored, no lookahead point given!', self.__class__.__name__)
            return

        m = Marker()
        m.header.frame_id = "map"
        m.id = 1
        m.type = Marker.POINTS
        m.action = Marker.ADD
        # m.ns = "racecar"

        m.scale.x = 0.2
        m.scale.y = 0.2
        m.scale.z = 0.2

        m.color = ColorRGBA(0, 1, 1, 1)       # Cyan
        m.lifetime = rospy.Duration(0.1)
        
        p = Point()
        p.x = lookahead_point[0]
        p.y = lookahead_point[1]
        p.z = 0.05
        m.points.append(p)
            
        self.pub_viz_lookahead.publish(m)
