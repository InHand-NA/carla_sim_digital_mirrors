from ruamel.yaml import YAML
from pathlib import Path


class mirror_parameters:
    def __init__(self):
        self.left_yaw = -150
        self.left_pitch = 0
        self.right_yaw = 150
        self.right_pitch = 0


class Config(object):
    def __init__(self, config_path="config.yaml"):
        configfile = Path("config.yaml")
        self.yaml = YAML(typ="safe")
        with open(configfile, "r") as f:
            self.config = self.yaml.load(f)

        # carla config
        self.host = self.config["carla"]["server"]
        self.port = self.config["carla"]["port"]
        self.autopilot = self.config["carla"]["autopilot"]
        self.timeout = int(self.config["carla"]["timeout"])
        self.no_rendering_mode = self.config["carla"]["no_rendering_mode"]
        self.world = self.config["carla"]["world"]
        self.vehicle_tag = self.config["carla"]["vehicle_tag"]
        self.steering_dead_zone = float(self.config["carla"]["steering_dead_zone"])
        self.fps = int(self.config["carla"]["fps"])
        self.fixed_delta_seconds = 1.0 / self.fps
        self.weather_preset = self.config["carla"]["weather_preset"]

        # recorder config
        self.recorder_save_debug_images = self.config["recorder"]["save_debug_images"]
        self.recorder_task_name = self.config["recorder"]["task_name"]

        # sim config
        self.default_control = self.config["sim"]["default_control"]
        self.windows = self.config["sim"]["windows"]
        self.enable_mirror_view = self.config["sim"]["enable_mirror_view"]
        self.front_window_res = self.config["sim"]["windows"]["front_res"]
        self.rear_window_res = self.config["sim"]["windows"]["mirror_res"]
        self.dashcam_res = self.config["sim"]["windows"]["dashcam_res"]

        self.real_time_mode = self.config["sim"]["real_time_mode"]
        self.god_view_mode = self.config["sim"]["god_view_mode"]

        self.god_view_location = self.config["sim"]["god_view_location"]
        self.god_view_rotation = self.config["sim"]["god_view_rotation"]
        self.front_view_location = self.config["sim"]["front_view_location"]
        self.dashcam_fov = self.config["sim"]["dashcam_fov"]
        self.dashcam_location = self.config["sim"]["dashcam_location"]
        self.dashcam_rotation = self.config["sim"]["dashcam_rotation"]

        self.mirror_location = self.config["sim"]["mirror_location"]

        self.controls = self.config["sim"]["controls"]

    def get_carla_config(self):
        return self.config["carla"]
