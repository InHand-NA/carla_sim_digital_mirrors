import multiprocessing
import sys
import carla
import random
import time
import cv2
import numpy as np
import math
import pygame
from pathlib import Path
from multiprocessing import Lock
from shared_memory_dict import SharedMemoryDict
import json
import os
import signal

from config import Config, mirror_parameters
from carla_gather_vehicle_data import get_vehicle_info
from carla_gather_lane import gather_lane_data
from data_processor import process_data, DATA_DIR, data_processor_loop


MB = 1000 * 1000

config = Config()
smd = SharedMemoryDict(name="tokens", size=20 * MB)
lock = Lock()


# To use multiprocessing.Lock on write operations of shared memory dict set environment variable SHARED_MEMORY_USE_LOCK=1.


# smd = SharedMemoryDict(name="tokens", size=10000000)

# frame_count = 0


# Render object to keep and pass the PyGame surface
class RenderObject(object):
    def __init__(self, width, height):
        init_image = np.random.randint(0, 255, (height, width, 3), dtype="uint8")
        self.surface = pygame.surfarray.make_surface(init_image.swapaxes(0, 1))


# Camera sensor callback, reshapes raw data from camera into 2D RGB and applies to PyGame surface
# This is the driver's view
def pygame_callback(data, obj):
    img = np.reshape(np.copy(data.raw_data), (data.height, data.width, 4))
    img = img[:, :, :3]
    img = img[:, :, ::-1]

    obj.surface = pygame.surfarray.make_surface(img.swapaxes(0, 1))


# Camera sensor callback, reshapes raw data from camera into 2D RGB and applies to PyGame surface
# This is the driver's view in left and right rear mirrors
def process_image_data(image_data, view_id, flip=False):
    img = np.reshape(
        np.copy(image_data.raw_data), (image_data.height, image_data.width, 4)
    )
    img = img[:, :, :3]
    img = img[:, :, ::-1]
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    if flip:
        img = cv2.flip(img, 1)
    lock.acquire()
    smd[view_id] = img
    lock.release()


def set_ego_autopilot_args(vehicle, tm):
    # Disable auto lane change
    tm.auto_lane_change(vehicle, True)
    # Set random speed
    #percentage = random.randint(-50, 0)
    #tm.vehicle_percentage_speed_difference(vehicle, percentage)
    tm.set_desired_speed(vehicle, 100)
    # Set keep right rule
    # tm.set_keep_right_rule(vehicle, True)
    tm.random_left_lanechange_percentage(vehicle, 15)
    tm.random_right_lanechange_percentage(vehicle, 15)
    # Ignore lights, signs and vehicles
    tm.ignore_lights_percentage(vehicle, 0)  # 忽略红绿灯
    tm.ignore_signs_percentage(vehicle, 10)  # 忽略交通标志
    tm.ignore_vehicles_percentage(vehicle, 0)  # 忽略其他车辆


class controller:
    def __init__(self, vehicle, config: Config):
        self.vehicle = vehicle
        self.config = config
        self._control = carla.VehicleControl()
        self._steer_cache = 0.0

        # initialize steering wheel
        pygame.joystick.init()

        joystick_count = pygame.joystick.get_count()
        # if joystick_count > 1:
        #    raise ValueError("Please Connect one Joystick")
        if joystick_count == 0:
            raise ValueError("No joystick connected")

        self.control_device = config.default_control
        if self.control_device == "fanatec":
            self._joystick = pygame.joystick.Joystick(1)
        else:
            self._joystick = pygame.joystick.Joystick(0)

        self._joystick.init()

        self._steer_idx = config.controls[self.control_device]["steering_wheel"]
        self._throttle_idx = config.controls[self.control_device]["throttle"]
        self._brake_idx = config.controls[self.control_device]["brake"]
        self._reverse_idx = config.controls[self.control_device]["reverse"]
        self._handbrake_idx = config.controls[self.control_device]["handbrake"]
        self.vehicle = vehicle

    def parse_vehicle_wheel(self):
        numAxes = self._joystick.get_numaxes()
        jsInputs = [float(self._joystick.get_axis(i)) for i in range(numAxes)]
        # print (jsInputs)
        jsButtons = [
            float(self._joystick.get_button(i))
            for i in range(self._joystick.get_numbuttons())
        ]

        # Invert Control Signals if needed
        if self.control_device == "fanatec":
            ic = -1
        else:
            ic = 1

        # Custom function to map range of inputs [1, -1] to outputs [0, 1] i.e 1 from inputs means nothing is pressed
        # For the steering, it seems fine as it is
        # Steering
        K1 = 1.0  # 0.55
        steerCmd = K1 * math.tan(1.1 * jsInputs[self._steer_idx])
        steer_dead_zone = self.config.steering_dead_zone
        if steerCmd >= -1 * steer_dead_zone and steerCmd <= steer_dead_zone:
            steerCmd = 0

        # Throttle
        K2 = 1.6  # 1.6
        throttleCmd = (
            K2
            + (2.05 * math.log10(-0.7 * ic * jsInputs[self._throttle_idx] + 1.4) - 1.2)
            / 0.92
        )
        if throttleCmd <= 0:
            throttleCmd = 0
        elif throttleCmd > 1:
            throttleCmd = 1
        throttleCmd = abs(1 - throttleCmd)

        # Brake
        brakeCmd = (
            1.6
            + (2.05 * math.log10(-0.7 * ic * jsInputs[self._brake_idx] + 1.4) - 1.2)
            / 0.92
        )
        if brakeCmd <= 0:
            brakeCmd = 0
        elif brakeCmd > 1:
            brakeCmd = 1
        brakeCmd = abs(1 - brakeCmd)

        self._control.steer = steerCmd
        self._control.brake = brakeCmd
        self._control.throttle = throttleCmd
        self._control.hand_brake = bool(jsButtons[self._handbrake_idx])

        self.vehicle.apply_control(self._control)


def get_vehicle_metadata(vehicle):
    bbox = vehicle.bounding_box
    extent = bbox.extent  # Vector3D(x, y, z)
    metadata = {
        "id": vehicle.id,
        "name": vehicle.type_id,
        "size": [
            round(extent.x * 2, 2),
            round(extent.y * 2, 2),
            round(extent.z * 2, 2),
        ],
    }
    #print(f"vehicle metadata: {metadata}")
    return metadata


def save_metadata(
    map_name,
    weather_preset_name,
    vehicle,
    dashcam_height,
    dashcam_width,
    dashcam_fov,
    dashcam_location,
    dashcam_rotation,
    fps,
    task_name,
):
    veh_meta = get_vehicle_metadata(vehicle)

    metadata = {
        "map_name": map_name,
        "weather_preset_name": weather_preset_name,
        "vehicle": veh_meta,
        "dashcam": {
            "height": dashcam_height,
            "width": dashcam_width,
            "fov": dashcam_fov,
            "location": [dashcam_location[0], dashcam_location[1], dashcam_location[2]],
            "rpy": [dashcam_rotation[0], dashcam_rotation[1], dashcam_rotation[2]],
            "fps": fps,
        },
    }
    metadata_file = os.path.join(DATA_DIR, f"{task_name}", "metadata.json")
    task_dir = os.path.dirname(metadata_file)
    if not os.path.exists(task_dir):
        os.makedirs(task_dir)

    with open(metadata_file, "w") as f:
        json.dump(metadata, f)


def get_weather_preset(weather_preset_name: str):
    weather_presets = {
        "ClearNoon": carla.WeatherParameters.ClearNoon,
        "CloudyNoon": carla.WeatherParameters.CloudyNoon,
        "WetNoon": carla.WeatherParameters.WetNoon,
        "WetCloudyNoon": carla.WeatherParameters.WetCloudyNoon,
        "SoftRainNoon": carla.WeatherParameters.SoftRainNoon,
        "MidRainyNoon": carla.WeatherParameters.MidRainyNoon,
        "HardRainNoon": carla.WeatherParameters.HardRainNoon,
        "ClearSunset": carla.WeatherParameters.ClearSunset,
        "CloudySunset": carla.WeatherParameters.CloudySunset,
        "WetSunset": carla.WeatherParameters.WetSunset,
        "WetCloudySunset": carla.WeatherParameters.WetCloudySunset,
        "SoftRainSunset": carla.WeatherParameters.SoftRainSunset,
        "MidRainSunset": carla.WeatherParameters.MidRainSunset,
        "HardRainSunset": carla.WeatherParameters.HardRainSunset,
    }
    if weather_preset_name not in weather_presets:
        raise ValueError(f"Invalid weather preset name: {weather_preset_name}")
    return weather_presets[weather_preset_name]


class Simulator(object):
    def __init__(self, config: Config, smd: SharedMemoryDict, lock: Lock):
        self.config = config
        self.vehicle_list = []
        self.mp = mirror_parameters()
        self.smd = smd
        self.lock = lock
        pass

    def stop_sensors(self, sensors_list):
        for sensor in sensors_list:
            sensor.stop()

    def init_sim(self):
        config = self.config
        self.vehicle_list = []
        self.mp = mirror_parameters()

        # Connect to the client
        client = carla.Client(config.host, config.port)
        client.set_timeout(config.timeout)
        self.client = client

        world = client.load_world(config.world)
        self.world = world
        """
        # Large World Loading
        world = client.load_world('Town11')
        settings = world.get_settings()
        settings.tile_stream_distance = 2000
        world.apply_settings(settings)
        """

        # Load layered map for Town 01 with minimum layout plus buildings and parked vehicles
        # world = client.load_world('Town10_Opt', carla.MapLayer.Buildings | carla.MapLayer.ParkedVehicles)
        # Toggle all buildings off
        # world.unload_map_layer(carla.MapLayer.Buildings)

        # Getting the world and
        world = client.get_world()
        self.original_settings = world.get_settings()

        # Weather Presets:ClearNoon, CloudyNoon, WetNoon, WetCloudyNoon, SoftRainNoon, MidRainyNoon, HardRainNoon,
        #  ClearSunset, CloudySunset, WetSunset, WetCloudySunset, SoftRainSunset, MidRainSunset, HardRainSunset.
        world.set_weather(get_weather_preset(config.weather_preset))

        # Set up the simulator in synchronous mode
        settings = world.get_settings()
        settings.no_rendering_mode = config.no_rendering_mode
        settings.synchronous_mode = True  # Enables synchronous mode
        settings.fixed_delta_seconds = config.fixed_delta_seconds
        world.apply_settings(settings)

        # Set up the TM in synchronous mode
        traffic_manager = client.get_trafficmanager()
        traffic_manager.set_synchronous_mode(True)

        # Set a seed so behaviour can be repeated if necessary
        traffic_manager.set_random_device_seed(0)
        random.seed(0)
        self.traffic_manager = traffic_manager

        # Print list of available vehicles
        # vehicle_blueprints = world.get_blueprint_library().filter('vehicle')
        # for car_bp in vehicle_blueprints:
        #    print (car_bp)

        vehicle_tag = config.vehicle_tag

        # Instanciating te vehicle to which we attached the sensors
        bp = world.get_blueprint_library().filter(vehicle_tag)[0]
        bp.set_attribute("role_name", "hero")
        ego_vehicle = world.spawn_actor(
            bp, random.choice(world.get_map().get_spawn_points())
        )
        self.vehicle_list.append(ego_vehicle)
        ego_vehicle.set_autopilot(config.autopilot)
        self.ego_vehicle = ego_vehicle

        if config.enable_mirror_view:
            # Find the blueprint of the sensor.
            mirror_blueprint = world.get_blueprint_library().find("sensor.camera.rgb")
            # Modify the attributes of the blueprint to set image resolution and field of view.
            mirror_blueprint.set_attribute(
                "image_size_x", str(config.rear_window_res[0])
            )
            mirror_blueprint.set_attribute(
                "image_size_y", str(config.rear_window_res[1])
            )
            mirror_blueprint.set_attribute("fov", "120")

        # Find the blueprint of the sensor. driver's view
        car_blueprint = world.get_blueprint_library().find("sensor.camera.rgb")
        # Modify the attributes of the blueprint to set image resolution and field of view.
        car_blueprint.set_attribute("image_size_x", str(config.front_window_res[0]))
        car_blueprint.set_attribute("image_size_y", str(config.front_window_res[1]))
        car_blueprint.set_attribute("fov", "140")

        dashcam_blueprint = world.get_blueprint_library().find("sensor.camera.rgb")
        dashcam_blueprint.set_attribute("image_size_x", str(config.dashcam_res[0]))
        dashcam_blueprint.set_attribute("image_size_y", str(config.dashcam_res[1]))
        dashcam_blueprint.set_attribute("fov", str(config.dashcam_fov))

        # Set the time in seconds between sensor captures
        # blueprint.set_attribute('sensor_tick', '1')

        # lookup pre-defined mirror locations based on vehicle tag
        if config.enable_mirror_view:
            lx, ly, lz = config.mirror_location[vehicle_tag]["left"]
            rx, ry, rz = config.mirror_location[vehicle_tag]["right"]
            left_mirror_transform = carla.Transform(
                carla.Location(x=lx, y=ly, z=lz),
                carla.Rotation(pitch=self.mp.left_pitch, yaw=self.mp.left_yaw),
            )
            right_mirror_transform = carla.Transform(
                carla.Location(x=rx, y=ry, z=rz),
                carla.Rotation(pitch=self.mp.right_pitch, yaw=self.mp.right_yaw),
            )
        # Driver's view

        if config.god_view_mode:
            fx, fy, fz = config.god_view_location[vehicle_tag]
            pitch, yaw, roll = config.god_view_rotation[vehicle_tag]
            front_view_transform = carla.Transform(
                carla.Location(x=fx, y=fy, z=fz),
                carla.Rotation(pitch=pitch, yaw=yaw, roll=roll),
            )
        else:
            fx, fy, fz = config.front_view_location[vehicle_tag]
            front_view_transform = carla.Transform(carla.Location(x=fx, y=fy, z=fz))

        # Dashcam view
        dashcam_location = config.dashcam_location[vehicle_tag]
        dashcam_rotation = config.dashcam_rotation
        dashcam_transform = carla.Transform(
            carla.Location(
                x=dashcam_location[0], y=dashcam_location[1], z=dashcam_location[2]
            ),
            carla.Rotation(
                pitch=dashcam_rotation[1],
                yaw=dashcam_rotation[2],
                roll=dashcam_rotation[0],
            ),
        )

        # Tell the world to spawn the sensor, don't forget to attach it to your vehicle actor.
        fv_sensor = world.spawn_actor(
            car_blueprint, front_view_transform, attach_to=self.ego_vehicle
        )
        dc_sensor = world.spawn_actor(
            dashcam_blueprint, dashcam_transform, attach_to=self.ego_vehicle
        )
        if config.enable_mirror_view:
            lmv_sensor = world.spawn_actor(
                mirror_blueprint, left_mirror_transform, attach_to=self.ego_vehicle
            )
            rmv_sensor = world.spawn_actor(
                mirror_blueprint, right_mirror_transform, attach_to=self.ego_vehicle
            )
        # Get camera dimensions
        image_w = car_blueprint.get_attribute("image_size_x").as_int()
        image_h = car_blueprint.get_attribute("image_size_y").as_int()
        self.renderObject = RenderObject(image_w, image_h)

        # Subscribe to the sensor stream by providing a callback function, this function is
        # called each time a new image is generated by the sensor.
        fv_sensor.listen(lambda data: pygame_callback(data, self.renderObject))
        dc_sensor.listen(lambda data: process_image_data(data, "dashcam_view", False))
        self.sensors_list = [fv_sensor, dc_sensor]
        if config.enable_mirror_view:
            rmv_sensor.listen(
                lambda data: process_image_data(data, "right_mirror_view", True)
            )
            lmv_sensor.listen(
                lambda data: process_image_data(data, "left_mirror_view", True)
            )
            self.sensors_list.extend([rmv_sensor, lmv_sensor])

        pygame.init()
        pygame.display.set_caption("Carla Simulator - InADAS")
        display = pygame.display.set_mode(
            config.front_window_res,
            pygame.HWSURFACE | pygame.DOUBLEBUF,
            display=0,
            vsync=1,
        )  # pygame.FULLSCREEN |
        # Draw black to the display
        display.fill((0, 0, 0))
        display.blit(self.renderObject.surface, (0, 0))
        self.display = display
        pygame.display.flip()

        self.my_controller = controller(self.ego_vehicle, config)

        if config.autopilot:
            set_ego_autopilot_args(self.ego_vehicle, self.traffic_manager)

        bbox = ego_vehicle.bounding_box
        extent = bbox.extent  # Vector3D(x, y, z)
        print(f"Bounding box extent of ego vehicle: {extent}")
        lock.acquire()
        self.smd["data_fifo"] = []
        lock.release()

        self.cam_locx = config.dashcam_location[vehicle_tag][0]

        save_metadata(
            config.world,
            config.weather_preset,
            self.ego_vehicle,
            config.dashcam_res[1],
            config.dashcam_res[0],
            config.dashcam_fov,
            config.dashcam_location[vehicle_tag],
            config.dashcam_rotation,
            self.config.fps,
            self.config.recorder_task_name,
        )

    def run_sim(self):
        # Game loop
        crashed = False
        clock = pygame.time.Clock()
        first_start_tm = 0
        first_fid = 0
        print("Running simulation")
        while not crashed:
            start_time = time.time()
            # Advance the simulation time
            fid = self.world.tick()
            lock.acquire()
            self.smd["frame_count"] = fid
            seconds = fid / self.config.fps
            self.smd["seconds"] = seconds
            self.smd["fps"] = self.config.fps
            lock.release()
            ego_vehicle_info = get_vehicle_info(self.ego_vehicle, cal_spd=True)
            lock.acquire()
            self.smd["ego_veh_info"] = ego_vehicle_info
            lanes_data = gather_lane_data(self.world, self.ego_vehicle, self.cam_locx)
            self.smd["lanes_data"] = lanes_data
            lock.release()
            # push data of current frame to fifo
            frame_data = {
                "frame_id": fid,
                "ego_veh_info": ego_vehicle_info,
                "lanes_data": lanes_data,
            }
            if "dashcam_view" in self.smd.keys():
                frame_data["dashcam_img"] = self.smd["dashcam_view"]
                # this block should be locked
                self.lock.acquire()
                data_fifo = self.smd["data_fifo"]
                data_fifo.append(frame_data)
                self.smd["data_fifo"] = data_fifo
                self.lock.release()
                if fid % 20 == 0:
                    print(f"Data fifo length: {len(data_fifo)}")
            else:
                print("!!!! No dashcam image found, ignore this frame, frame_id: ", fid)

            if self.config.autopilot:
                set_ego_autopilot_args(self.ego_vehicle, self.traffic_manager)
            else:
                self.my_controller.parse_vehicle_wheel()
                self.my_controller._control.reverse = (
                    self.my_controller._control.gear < 0
                )

            # Update the display
            self.display.blit(self.renderObject.surface, (0, 0))
            pygame.display.flip()

            for event in pygame.event.get():
                # If the window is closed, break the while loop
                if event.type == pygame.QUIT:
                    crashed = True

                if event.type == pygame.JOYBUTTONDOWN and not self.config.autopilot:
                    if event.button == self.my_controller._reverse_idx:
                        self.my_controller._control.gear = (
                            1 if self.my_controller._control.reverse else -1
                        )
                        print("Reverse", self.my_controller._control.gear)

            end_time = time.time()
            # sleep_time(start_time, end_time, 0.05)
            if fid % 20 == 0:
                print(
                    f"Frame ID: {fid}, loop time: {round(end_time - start_time, 2) * 1000}ms; total frames: {fid - first_fid}"
                )
            if True:
                expected_tm = first_start_tm + (
                    (fid - first_fid) * self.config.fixed_delta_seconds
                )
                # clock.tick(10)
                
                if self.config.real_time_mode:
                    clock.tick_busy_loop(self.config.fps)
                this_tm = time.time()
                if first_start_tm == 0 and "dashcam_view" in self.smd.keys():
                    first_start_tm = this_tm
                    first_fid = fid
                if self.config.real_time_mode:
                    if fid % 20 == 0:
                        print(
                            f"  Expected time: {expected_tm}, current time: {this_tm}, offset: {int((this_tm - expected_tm) * 1000)}ms;"
                        )

            if first_fid > 0 and self.config.sim_frames > 0 and fid >= (self.config.sim_frames + first_fid):
                crashed = True
                print(f"Sim frames: {fid - first_fid}, expected frames: {self.config.sim_frames}; Exit the loop")

        print("Shutting Down...")
        self.stop_sensors(self.sensors_list)
        # close_all_videos()
        print("Destroying actors...")
        self.client.apply_batch(
            [carla.command.DestroyActor(x) for x in self.vehicle_list]
        )
        print("Restoring original settings...")
        self.world.apply_settings(self.original_settings)
        print("Quitting PyGame display...")
        pygame.display.quit()  # 明确退出显示模块
        # pygame.mixer.quit()
        print("Quitting PyGame...")

        self.smd.shm.close()
        self.smd.shm.unlink()
        del self.smd
        pygame.quit()
        print("Done")
        sys.exit()
        pass


def run_sim_process(config, smd, lock):
    sim = Simulator(config, smd, lock)

    sim.init_sim()
    sim.run_sim()


def main():
    global data_processor_loop
    # create a new process to run the sim
    main_process = multiprocessing.Process(
        target=run_sim_process, args=(config, smd, lock)
    )
    data_process = multiprocessing.Process(target=process_data, args=(smd, lock))

    main_process.start()
    # create a new process to process the data
    time.sleep(5)
    data_process.start()

    main_process.join()
    print("Main process end, wait for 3 seconds to send signal to the other processes")

    time.sleep(3)
    # 发送信号给指定进程
    target_pid = data_process.pid
    os.kill(target_pid, signal.SIGUSR1)
    data_process.join()


if __name__ == "__main__":
    main()
