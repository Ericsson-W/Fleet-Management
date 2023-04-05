from PIL.Image import radial_gradient
import numpy as np
from copy import deepcopy

from numpy.core.fromnumeric import argmax

def angle(vector, radians=True):
    """Calculate angle of single 2D vector"""
    angles = np.arctan2(vector[1], vector[0])
    if not radians:
        angles = 180 / np.pi * angles
    return angles

def calc_angle(vector, t_start, t_end, radians=True):
    """Calculate angle of vector"""
    angles = np.arctan2(vector[t_start:t_end, 1], vector[t_start:t_end, 0])
    if not radians:
        angles = 180 / np.pi * angles
    return angles


def unit_vector(vector):
    return vector / np.linalg.norm(vector)

def angle_between_vectors(vector1, vector2, radians=True):
    v1_u = unit_vector(vector1)
    v2_u = unit_vector(vector2)

    angle = np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

    if not radians:
        return 180 / (np.pi * angle)

    return angle

def position_range(vehicles, params):
    """Get range (max - min) for the position of all vehicles"""

    if params['simulator']['zoom_to_ego']:
        ego_id = params['simulator']['ego_vehicle_id']

        vehicle = vehicles[ego_id]

        orig_x = vehicle['Position'][:, 0][~np.isnan(vehicle['Position'][:, 0])]
        orig_y = vehicle['Position'][:, 1][~np.isnan(vehicle['Position'][:, 1])]
        max_x = max(orig_x)
        max_y = max(orig_y)
        min_x = min(orig_x)
        min_y = min(orig_y)

    else:

        max_x = -999999
        max_y = -999999
        min_x = 999999
        min_y = 999999
        # length = scale*max(car_params['length'], car_params['width'])
        for n, vehicle in enumerate(vehicles.values()):
            orig_x = vehicle['Position'][:, 0][~np.isnan(vehicle['Position'][:, 0])]
            orig_y = vehicle['Position'][:, 1][~np.isnan(vehicle['Position'][:, 1])]
            max_x = max(max_x, max(orig_x))
            max_y = max(max_y, max(orig_y))
            min_x = min(min_x, min(orig_x))
            min_y = min(min_y, min(orig_y))


    return min_x, max_x, min_y, max_y

def norm_2(vector):
    """Norm squared of a vector"""
    return np.power(np.linalg.norm(vector, axis=1), 2)

def vehicle_corners(vehicle, dimensions, scale, t_start, t_end):
    """
    Dictionary with 4 corners of vehicle
    :return: corners = dict(left_front, right_front, left_rear, right_rear)
    """
    corner_nan = np.array([np.nan, np.nan])
    corners_nan = {
        'left_front': corner_nan,
        'right_front': corner_nan,
        'left_rear': corner_nan,
        'right_rear': corner_nan
    }
    corners_arr = []
    car_length, car_width = dimensions
    length_half = car_length * scale / 2
    width_half = car_width * scale / 2
    left_front = np.array([length_half, width_half])
    right_front = np.array([length_half, -width_half])
    left_rear = np.array([-length_half, width_half])
    right_rear = np.array([-length_half, -width_half])
    corners = {
        'left_front': left_front,
        'right_front': right_front,
        'left_rear': left_rear,
        'right_rear': right_rear
    }
    for t in range(t_start, min(len(vehicle['Time']), t_end)):
        rot_corners = deepcopy(corners_nan)
        if not np.isnan(vehicle['Position'][t]).any():
            theta = vehicle['Angle'][t]
            sin = np.sin(theta)
            cos = np.cos(theta)
            # Rotate car and add position to find corners
            for corner in corners:
                rot_corner = np.array([np.nan, np.nan])
                rot_corner[0] = corners[corner][0] * cos - corners[corner][1] * sin + vehicle['Position'][t][0]
                rot_corner[1] = corners[corner][0] * sin + corners[corner][1] * cos + vehicle['Position'][t][1]
                rot_corners[corner] = rot_corner
        corners_arr.append(rot_corners)
    return corners_arr


########################################################################################################################
###############################################  For Risk Metrics ######################################################
########################################################################################################################

def rotate(vector, angle):
    """Rotate vector"""
    cos_r = np.cos(angle)
    sin_r = np.sin(angle)
    r = np.array([[cos_r, sin_r], [-sin_r, cos_r]])
    return np.dot(vector, r)

def project_vector(vector, ego_angle):
    """Project vector onto vector_proj_on"""
    theta = angle(vector, radians=True) - ego_angle
    return np.array([np.cos(theta), np.sin(theta)])

def same_orientation(angle1, angle2, radians=True):
    """
    Check if orientation between 2 vehicles is similar
    return: boolean
    """
    eps_degrees = 30
    res = False
    if radians:
        if abs(angle1 - angle2) <= eps_degrees * np.pi / 180:
            res = True
    else:
        if abs(angle1 - angle2) <= eps_degrees:
            res = True
    return res

def dist_lon(pos1, angle1, pos2):
    """Longitudinal distance"""
    point_angle = pos1 + [np.cos(angle1), np.sin(angle1)]
    d = np.abs(np.cross(point_angle - pos1, pos2 - pos1)) / np.linalg.norm(point_angle - pos1)
    return d

def dist_lat(pos1, angle1, pos2):
    """Lateral distance"""
    point_angle = pos1 + [-np.sin(angle1), np.cos(angle1)]
    d = np.abs(np.cross(point_angle - pos1, pos2 - pos1)) / np.linalg.norm(point_angle - pos1)
    return d