# External imports
import rospy
import numpy as np
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from nav_msgs.msg import Path

# Local imports
from tf.transformations import quaternion_from_euler


def set_subscribers(subscriber_list, subscribe=True, timeout=30, warn_freq=1):
    """ Checking that all systems are ready """
    for subscriber in subscriber_list:
        topic, topic_class, topic_callback, required = subscriber
        
        if subscribe:
            rospy.Subscriber(topic, topic_class, topic_callback)
        
        if not required:
            continue
        
        data = None
        while data is None and not rospy.is_shutdown():
            try:
                data = rospy.wait_for_message(topic, topic_class, timeout)
                if not topic_callback is None:
                    topic_callback(data)
                rospy.logdebug("[%s]: Subscriber %s ready!", rospy.get_caller_id(), topic)
            except:
                rospy.logwarn_throttle(warn_freq, "[%s]: Topic class %s not ready yet, trying from topic %s", rospy.get_caller_id(), topic_class, topic)

def create_message(data):

    def create_pose_message(*args):
        pose = PoseStamped()
        pose.header = rospy.Time.now()
        if len(args) == 5:  # Handle (x, y, roll, pitch, yaw) case
            x, y, roll, pitch, yaw = args
            pose.pose.position = Point(x, y, 0)
            [qx, qy, qz, qw] = quaternion_from_euler(roll, pitch, yaw)
            pose.pose.orientation = Quaternion(qx, qy, qz, qw)
        else:
            arg_names = ["x", "y", "z", "roll", "pitch", "yaw"]
            for i, arg in enumerate(args):
                if i < 3:  # positional values
                    setattr(pose.pose.position, arg_names[i], arg)
                else:  # orientation values
                    [qx, qy, qz, qw] = quaternion_from_euler(*args[3:])
                    pose.pose.orientation = Quaternion(qx, qy, qz, qw)
                    break  # No need to iterate further after setting orientation
        return pose

    msg = None
    
    # fill data based on the incoming data format
    if isinstance(data, np.ndarray) and data.ndim == 2:
        msg = Path()
        msg.header = rospy.Time.now()
        for p in data:
            pose = create_pose_message(*p)
            msg.poses.append(pose)
    elif isinstance(data, (list, tuple)) or isinstance(data, np.ndarray) & data.ndim==1:
        if len(data) == 2:  # X, Y
            msg = Point(data[0], data[1], 0)
        elif len(data) == 3:  # X, Y, Z
            msg = Point(data[0], data[1], data[2])
        else:
            rospy.logwarn("[%s]: Unrecognized data point format!", rospy.get_caller_id())
    else:
        rospy.logwarn("[%s]: Unrecognized data format!", rospy.get_caller_id())
    
    return msg