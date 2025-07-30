import math
import numpy as np
from carla import Vector3D

def transform_velocity_to_vehicle_frame(velocity, vehicle_transform):
    """
    Transform velocity from world frame to vehicle frame
    """
    world2ego_transform = vehicle_transform.get_inverse_matrix()

    # Rotation matrix only
    world2ego_transform = np.array(world2ego_transform)
    R = world2ego_transform[:3, :3]
    v_ego = R @ np.array([velocity.x, velocity.y, velocity.z])

    # carla 左手坐标系转换为 ISO8855 右手坐标系
    return Vector3D(float(v_ego[0]), float(-v_ego[1]), float(v_ego[2]))

def transform_angular_velocity_to_vehicle_frame(angular_velocity, vehicle_transform):
    """
    Transform angular velocity from world frame to vehicle frame
    """
    world2ego_transform = vehicle_transform.get_inverse_matrix()
    world2ego_transform = np.array(world2ego_transform)
    R = world2ego_transform[:3, :3]
    w_ego = R @ np.array([angular_velocity.x, angular_velocity.y, angular_velocity.z])
    # carla 左手坐标系转换为 ISO8855 右手坐标系
    return Vector3D(float(w_ego[0]), float(-w_ego[1]), float(-w_ego[2]))


def get_vehicle_info(vehicle, cal_spd=False):
    """Returns vehicle info."""
    vehicle_info = {}
    acceleration = vehicle.get_acceleration()  # m/s^2
    angular_velocity = vehicle.get_angular_velocity()  # deg/s
    location = vehicle.get_location()
    velocity = vehicle.get_velocity()

    # convert velocity from world frame to vehicle frame
    velocity = transform_velocity_to_vehicle_frame(velocity, vehicle.get_transform())
    angular_velocity = transform_angular_velocity_to_vehicle_frame(angular_velocity, vehicle.get_transform())

    vehicle_info["angular_velocity_degps"] = [
        angular_velocity.x,
        angular_velocity.y,
        angular_velocity.z,
    ]
    vehicle_info["velocity"] = [velocity.x, velocity.y, velocity.z]
    if cal_spd:
        spd = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2) * 3.6  # km/h
        vehicle_info["speed_kmph"] = spd
        vehicle_info["yaw_velocity_degps"] = angular_velocity.z  # deg/s

    return vehicle_info


def get_kmh_speed(vehicle):
    """Returns speed in km/h."""
    velocity = vehicle.get_velocity()
    speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
    return speed * 3.6  # Convert m/s to km/h


def get_mph_speed(vehicle):
    """Returns speed in mph."""
    velocity = vehicle.get_velocity()
    speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
    return speed * 2.23694  #
