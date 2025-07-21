import math


def get_vehicle_info(vehicle, cal_spd=False):
    """Returns vehicle info."""
    vehicle_info = {}
    acceleration = vehicle.get_acceleration()  # m/s^2
    angular_velocity = vehicle.get_angular_velocity()  # deg/s
    location = vehicle.get_location()
    velocity = vehicle.get_velocity()

    vehicle_info["acceleration"] = [acceleration.x, acceleration.y, acceleration.z]
    vehicle_info["angular_velocity"] = [
        angular_velocity.x,
        angular_velocity.y,
        angular_velocity.z,
    ]
    vehicle_info["location"] = [location.x, location.y, location.z]
    vehicle_info["velocity"] = [velocity.x, velocity.y, velocity.z]
    if cal_spd:
        spd = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2) * 3.6  # km/h
        vehicle_info["speed"] = spd
        vehicle_info["yaw_velocity"] = angular_velocity.z  # deg/s

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
