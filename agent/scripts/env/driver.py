# External libraries
import numpy as np
import rospy

# Local imports
from env.entity import DynamicEntity


class Driver(DynamicEntity):

    def __init__(self, id, velocity, distance, azimuth, heading):
        """
        Initializes the Driver object.

        Args:
            id (int): Unique identifier for the driver.
            velocity (float): Initial velocity of the driver.
            distance (float): Initial distance of the driver from a source object.
            azimuth (float): Initial azimuth of the source object.
            heading (float): Initial heading of the driver.
        """
        super(Driver, self).__init__(id, 
                                         stop_dist=rospy.get_param("driver_stop_dist"),
                                         min_stop_dist=rospy.get_param("driver_min_stop_dist"),
                                         timeout=rospy.get_param("driver_timeout"),
                                         velocity=velocity,
                                         distance=distance,
                                         azimuth=azimuth,
                                         heading=heading)
    
        self.driver_same_dir_range = rospy.get_param("driver_same_dir_range")

    def is_same_direction(self, source_heading):
        return np.abs(self.heading - source_heading) <= self.driver_same_dir_range