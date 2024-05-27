# External libraries
import rospy

# Local imports
from env.entity import DynamicEntity


class Pedestrian(DynamicEntity):
    """
    Represents a pedestrian as a specialized form of a dynamic entity.

    Inherits from DynamicEntity and is specifically tailored for pedestrian dynamics in a simulation or tracking environment.

    Attributes:
        Inherited from DynamicEntity: velocity, distance, azimuth, heading.
    """

    def __init__(self, id, velocity, distance, azimuth, heading):
        """
        Initializes the Pedestrian object.

        Args:
            id (int): Unique identifier for the pedestrian.
            velocity (float): Initial velocity of the pedestrian.
            distance (float): Initial distance of the pedestrian from a source object.
            azimuth (float): Initial azimuth of the source object.
            heading (float): Initial heading of the pedestrian.
        """
        super(Pedestrian, self).__init__(id, 
                                         stop_dist=rospy.get_param("pedestrian_stop_dist"),
                                         min_stop_dist=rospy.get_param("pedestrian_min_stop_dist"),
                                         timeout=rospy.get_param("pedestrian_timeout"),
                                         velocity=velocity,
                                         distance=distance,
                                         azimuth=azimuth,
                                         heading=heading)        
    
    