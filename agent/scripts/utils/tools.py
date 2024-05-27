# System imports
import csv
import math
from collections import Sequence

# External imports
import numpy as np

# Local imports
from config.definitions import TRAFFIC_LIGHT_INFO

def scale(arr, low=-1.0, high=1.0, clip_min=-1.0, clip_max=1.0):
    # Clamp data between [clip_min, clip_max]
    data = np.clip(arr, a_min=low, a_max=high)
    # Scale values between [low, high]
    data = (data - clip_min) / (clip_max - clip_min)
    data = data * (high - low) + low
    return data

def csv_to_waypoints(path, dilation=1):
    with open(path) as f:
        csv_lines = [tuple(line) for line in csv.reader(f)]
        dims = len(csv_lines[0])
        if dims == 2:
            return np.array([(float(csv_lines[pt][0]), float(csv_lines[pt][1]), 1.0) for pt in range(0, len(csv_lines), dilation)])
        else:
            return np.array([(float(csv_lines[pt][0]), float(csv_lines[pt][1]), float(csv_lines[pt][2])) for pt in range(0, len(csv_lines), dilation)])


def min_data_zones(data, zones=1):
    # make sure zones divide exactly the data
    assert len(data) % zones == 0
    lidar_data = np.array(data)
    lidar_data = lidar_data.reshape(zones, -1)
    lidar_data = np.min(lidar_data, axis=1)
    return lidar_data

def calculate_curvature(points):
    # get next three points
    a = points[0]
    b = points[1]
    c = points[2]

    area = abs(a[0] * (b[1] - c[1]) + b[0] * (c[1] - a[1]) + c[0] * (a[1] - b[1])) / 2
    curvature = (4 * area) / (np.linalg.norm(a - b) * np.linalg.norm(b - c) * np.linalg.norm(c - a))
    return np.clip(curvature, 0.0, 1.0)

def is_sequence(data):
    return isinstance(data, (list, tuple, np.ndarray, Sequence))

# ----- GEOMETRIC FUNCTIONS ------

def dist(source_pos, target_pos):
    return sum((a - b) ** 2 for a, b in zip(source_pos, target_pos)) ** 0.5

# def angle(source_pos, target_pos):
#     return math.atan2(
#                 source_pos[1] - target_pos[1], 
#                 source_pos[0] - target_pos[0]
#             ) + math.atan2(
#                 source_pos[1],
#                 source_pos[0]
#             )

def azimuth(source_pos, target_pos, source_heading=0):
    return source_heading - math.atan2(
        target_pos[1] - source_pos[1], 
        target_pos[0] - source_pos[0]
    )

def position(source_pos, distance, azimuth):
    return [
        source_pos[0] + distance * np.cos(azimuth), # TODO verify sin vs cos
        source_pos[1] + distance * np.sin(azimuth)  # TODO verify cos vs sin
    ]
