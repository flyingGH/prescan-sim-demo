# system imports
from enum import Enum

# external imports
from std_msgs.msg import ColorRGBA

class EmergencyState(Enum):
    NORMAL = 0
    TRAFFIC_LIGHT = 1
    PEDESTRIAN = 2

TRAFFIC_LIGHT_INFO = {
    'red' : {
        'color' : ColorRGBA(1., 0., 0., 1.),
        'emergency' : True
    },
    'orange' : {
        'color' : ColorRGBA(1., 0.5, 0., 1.),
        'emergency' : True
    },
    'green' : {
        'color' : ColorRGBA(0., 1., 0., 1.),
        'emergency' : False
    }
}

# ids
TL_ID_START = 1000
PED_ID_START = 2000

