# System imports
import math
import time
from enum import Enum

# Local imports
from utils import tools as tls


class Entity(object):
    """
    Base class for representing an entity.

    Attributes:
        id (int): Unique identifier for the entity.
        stop_dist (float): Stopping distance parameter for the entity.
        min_stop_dist (float): Minimum stopping distance for the entity.
        last_updated (float): Timestamp of the last update.
    """

    def __init__(self, 
                 id, 
                 stop_dist, 
                 min_stop_dist):
        """
        Initializes the Entity object.

        Args:
            id (int): Unique identifier for the entity.
            stop_dist (float): Stopping distance parameter for the entity.
            min_stop_dist (float): Minimum stopping distance for the entity.
        """

        self.id = id
        self.stop_dist = stop_dist
        self.min_stop_dist = min_stop_dist
        self.last_updated = time.time()

    def update(self):
        """
        Updates the last_updated timestamp to the current time.
        """
        self.last_updated = time.time()

    def get_azimuth(self):
        """
        Placeholder method to get the azimuth of the entity.
        Should be implemented in subclasses.
        """
        pass

    def get_distance(self):
        """
        Placeholder method to get the distance of the entity.
        Should be implemented in subclasses.
        """
        pass
    
    def is_near(self):
        """
        Placeholder method to check whther source object is nead this entity.
        Should be implemented in subclasses.
        """
        pass


class StaticEntity(Entity):
    """
    Represents a static entity.

    Attributes:
        position (tuple): The position of the static entity.
    """

    def __init__(self,
                 id, 
                 stop_dist, 
                 min_stop_dist, 
                 position):
        """
        Initializes the StaticEntity object.

        Args:
            id (int): Unique identifier for the entity.
            stop_dist (float): Stopping distance parameter for the entity.
            min_stop_dist (float): Minimum stopping distance for the entity.
            position (tuple): The position of the static entity.
        """

        super(StaticEntity, self).__init__(id, stop_dist, min_stop_dist)
        self.position = position

    def get_distance(self, source_pos):
        """
        Calculates the distance from a source position to this entity.

        Args:
            source_pos (tuple): The position of the source from which the distance is calculated.

        Returns:
            float: The calculated distance.
        """
        return tls.dist(source_pos=source_pos, target_pos=self.position)
    
    def get_azimuth(self, source_pos, source_heading=0):
        """
        Calculates the azimuth from a source position and heading to this entity.

        Args:
            source_pos (tuple): The position of the source.
            source_heading (float): The heading of the source.

        Returns:
            float: The calculated azimuth.
        """
        return tls.azimuth(source_pos=source_pos, target_pos=self.position, source_heading=source_heading)

    def is_near(self, source_pos):
        return self.get_distance(source_pos) < self.stop_dist


class DynamicEntity(Entity):
    """
    Represents a dynamic entity that moves within the environment such as pedestrians and other cars.

    Attributes:
        velocity (float): The velocity of the dynamic entity.
        distance (float): The distance of the dynamic entity from a source object.
        azimuth (float): The azimuth of the source object.
        heading (float): The heading of the dynamic entity.
    """

    def __init__(self, 
                 id, 
                 stop_dist,
                 min_stop_dist,
                 timeout,
                 velocity,
                 distance,
                 azimuth,
                 heading):
        """
        Initializes the DynamicEntity object.

        Args:
            id (int): Unique identifier for the entity.
            stop_dist (float): Stopping distance parameter for the entity.
            min_stop_dist (float): Minimum stopping distance for the entity.
            velocity (float): The velocity of the dynamic entity.
            distance (float): The distance of the dynamic entity from a source object.
            azimuth (float): The azimuth of the dsource object..
            heading (float): The heading of the dynamic entity.
        """

        super(DynamicEntity, self).__init__(id, stop_dist, min_stop_dist)

        self.timeout = timeout

        self.update(velocity=velocity,
                    distance=distance,
                    azimuth=azimuth,
                    heading=heading)


    def update(self, velocity, distance, azimuth, heading):
        """
        Updates the dynamic entity with new movement parameters.

        Args:
            velocity (float): The new velocity of the entity.
            distance (float): The new distance of the entity from a source object.
            azimuth (float): The new azimuth of the source object.
            heading (float): The new heading of the entity.
        """
        super(DynamicEntity, self).update()
        self.velocity = velocity
        self.distance = distance
        self.azimuth = azimuth
        self.heading = heading
    
    def get_azimuth(self):
        """
        Returns the azimuth of the dynamic entity.

        Returns:
            float: The azimuth of the entity.
        """
        return self.azimuth

    def get_distance(self):
        """
        Returns the distance of the dynamic entity.

        Returns:
            float: The distance of the entity.
        """
        return self.distance

    def is_obsolete(self):
        return time.time() - self.last_updated > self.timeout
    
    def is_near(self):
        return self.distance < self.stop_dist