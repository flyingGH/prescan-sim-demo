# External libraries
import rospy
import numpy as np

# Local imports
from env.entity import StaticEntity
from config.definitions import TRAFFIC_LIGHT_INFO


class TrafficLight(StaticEntity):
    """
    Represents a traffic light as a specialized form of a static entity.

    Inherits from StaticEntity and adds specific attributes and methods related to traffic lights.

    Attributes:
        crit_angle (float): Critical angle parameter for the traffic light.
        color (str): Current color of the traffic light.
    """
    
    def __init__(self, id, position, color):
        """
        Initializes the TrafficLight object.

        Args:
            id (int): Unique identifier for the traffic light.
            position (tuple): The position of the traffic light.
            color (str): Initial color of the traffic light.
        """
        
        # Initialize the base StaticEntity class
        super(TrafficLight, self).__init__(id=id, 
                                           stop_dist = rospy.get_param("traffic_light_stop_dist"), 
                                           min_stop_dist = rospy.get_param("traffic_light_min_stop_dist"), 
                                           position = position)

        # subclass specific parameters
        self.crit_angle = rospy.get_param("traffic_light_crit_angle")

        # Update the traffic light with the current color
        self.update(color)
    
    def update(self, color):
        """
        Updates the traffic light's color and refreshes the last updated timestamp.

        Args:
            color (str): New color to update the traffic light with.
        """
        super(TrafficLight, self).update()
        self.color = color
    
    def is_emergency_color(self):
        """
        Determines if the current state of the traffic light represents an emergency.

        Returns:
            bool: True if the current color represents an emergency, False otherwise.
        """
        return TRAFFIC_LIGHT_INFO[self.color]['emergency']
    
    def is_in_front(self, source_pos, source_heading):
        azimuth = self.get_azimuth(source_pos=source_pos, source_heading=source_heading)
        return np.abs(azimuth) < self.crit_angle