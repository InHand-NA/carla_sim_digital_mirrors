import os
import carla
from carla import ColorConverter as cc
import random
import time
import cv2
import numpy as np
import math
import pygame
import threading
import queue
from ruamel.yaml import YAML
from pathlib import Path
from shared_memory_dict import SharedMemoryDict
from PIL import Image

from pygame.locals import K_a
from pygame.locals import K_w
from pygame.locals import K_d
from pygame.locals import K_x
from pygame.locals import K_f
from pygame.locals import K_t
from pygame.locals import K_h
from pygame.locals import K_b

# Define shared memory object and size in bytes
smd = SharedMemoryDict(name='tokens', size=50000000)

# Read Config File
configfile=Path("config.yaml")
_config = YAML(typ='safe').load(configfile)

control_device=_config['sim']['default_control']

class mirror_parameters:
    def __init__(self):
        self.left_yaw=-150
        self.left_pitch=0
        self.right_yaw=150
        self.right_pitch=0


# Render object to keep and pass the PyGame surface
class RenderObject(object):
    def __init__(self, width, height):
        init_image = np.random.randint(0,255,(height,width,3),dtype='uint8')
        self.surface = pygame.surfarray.make_surface(init_image.swapaxes(0,1))

run_num = 0
frame_count = 0
frame_skip = 2
base_dir = "/home/inhand/carla/outputs"

existing = [d for d in os.listdir(base_dir) if d.startswith("run") and d[3:].isdigit()]
run_ids = [int(d[3:]) for d in existing]
run_num = max(run_ids, default=-1) + 1

output_dir = os.path.join(base_dir, f"run{run_num}")
os.makedirs(output_dir, exist_ok=True)

save_img_queue = queue.Queue()

def save_img_worker():
    while True:
        image = save_img_queue.get()
        if image is None:
            save_img_queue.task_done()
            break
        img_data, img_id = image
        filename = os.path.join(output_dir, f"frame_{img_id:06d}.jpg")
        cv2.imwrite(filename, img_data, [cv2.IMWRITE_JPEG_QUALITY, 90])
        save_img_queue.task_done()

threading.Thread(target=save_img_worker, daemon=True).start()


# Dashcam view sensor callback, saves frames as images based on chosen frame rate
# As well as display to its own window when dashcam_view.py is run
def dashcam_callback(data, view_id):
    global frame_count

    img = np.reshape(np.copy(data.raw_data), (data.height, data.width, 4))

    display_img = img[:, :, :3][:, :, ::-1]
    try:
        if smd._memory_block is not None:
            smd[view_id] = img
    except Exception as e:
        print(f"[SharedMemory Error] {e}")

    if frame_count % frame_skip == 0:
        saved_img = img[:, :, :3]
        save_img_queue.put((saved_img.copy(), data.frame))

    frame_count += 1

# Driver view sensor callback, reshapes raw data from camera into 2D RGB and applies to PyGame surface
def driver_callback(data, obj):
    img = np.reshape(np.copy(data.raw_data), (data.height, data.width, 4))
    save_img = img[:, :, :3][:, :, ::-1]
    obj.surface = pygame.surfarray.make_surface(save_img.swapaxes(0, 1))


# Camera sensor callback, reshapes raw data from camera into 2D RGB and applies to PyGame surface
def process_image_data(image_data, view_id, flip=False):
    img = np.reshape(np.copy(image_data.raw_data), (image_data.height, image_data.width, 4))
    img = img[:,:,:3]
    img = img[:, :, ::-1]
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)	
    if flip:
        img=cv2.flip(img, 1)
    try:
        if smd._memory_block is not None:
            smd[view_id] = img
    except Exception as e:
        print(f"[SharedMemory Error] {e}")

def save_vid():
    input_dir = output_dir
    output_video = "output_video.mp4"
    fps = 10
    ext = ".jpg" 

    images = sorted(
        [img for img in os.listdir(input_dir) if img.endswith(ext)]
    )

    if not images:
        raise ValueError("No images found!")

    # === Get image size from first frame ===
    first_image = cv2.imread(os.path.join(input_dir, images[0]))
    height, width, _ = first_image.shape

    # === Create VideoWriter ===
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")  # or 'XVID' for AVI
    video = cv2.VideoWriter(output_video, fourcc, fps, (width, height))

    # === Write each frame ===
    for img_name in images:
        img_path = os.path.join(input_dir, img_name)
        frame = cv2.imread(img_path)
        video.write(frame)

    video.release()
    print(f"Saved video to {output_video}")

class controller ():
    def __init__(self, vehicle):
        self._control = carla.VehicleControl()
        self._steer_cache = 0.0

        try:

            # initialize steering wheel
            pygame.joystick.init()

            joystick_count = pygame.joystick.get_count()
            #if joystick_count > 1:
            #    raise ValueError("Please Connect one Joystick")
            if joystick_count ==0 :
                raise ValueError("No joystick connected")

            if control_device=='fanatec':
                self._joystick = pygame.joystick.Joystick(1)
            else:
                self._joystick = pygame.joystick.Joystick(0)

            self._joystick.init()

            self._steer_idx = _config['sim']['controls'][control_device]['steering_wheel']
            self._throttle_idx = _config['sim']['controls'][control_device]['throttle']
            self._accel_idx = _config['sim']['controls'][control_device]['accel']
            self._brake_idx = _config['sim']['controls'][control_device]['brake']
            self._reverse_idx = _config['sim']['controls'][control_device]['reverse']
            self._handbrake_idx = _config['sim']['controls'][control_device]['handbrake']
            self.vehicle=vehicle
        except Exception as e: 
            print(e)
            print('Shutting Down')
            client.apply_batch([carla.command.DestroyActor(x) for x in vehicle_list])
            world.apply_settings(original_settings)
            pygame.quit()
            print ("Done")
            exit (0)


    def parse_vehicle_wheel(self):
        numAxes = self._joystick.get_numaxes()
        jsInputs = [float(self._joystick.get_axis(i)) for i in range(numAxes)]
        # print (jsInputs)
        jsButtons = [float(self._joystick.get_button(i)) for i in
                        range(self._joystick.get_numbuttons())]

        # Invert Control Signals if needed
        if control_device=='fanatec':
            ic=-1
        else:
            ic=1

        # Custom function to map range of inputs [1, -1] to outputs [0, 1] i.e 1 from inputs means nothing is pressed
        # For the steering, it seems fine as it is
        K1 = 0.15  # 0.55
        raw_steer = jsInputs[self._steer_idx]
        steerCmd = K1 * math.tan(1.1 * (abs(raw_steer) ** 0.7)) * math.copysign(1, raw_steer)
        steer_dead_zone=_config['carla']['steering_dead_zone']
        '''
        K2 = 1.6  # 1.6
        throttleCmd = 0

        # Read the raw throttle input from the joystick
        throttle_input_raw = jsInputs[self._throttle_idx]

        # Compute the transformed input that goes into the log10()
        log_input = -0.7 * ic * throttle_input_raw + 1.4

        # Only compute the throttle if the log_input is valid (> 0)
        if log_input > 0:
            throttleCmd = K2 + (2.05 * math.log10(log_input) - 1.2) / 0.92

            # Clamp the output to [0, 1]
            throttleCmd = max(0.0, min(1.0, throttleCmd))

            # Invert and smooth the response curve
            throttleCmd = abs(1 - throttleCmd)
        else:
            throttleCmd = 0.0
        '''
        accel_input = 1
        if jsButtons[self._accel_idx]:
            accel_input = 1.5
        
        K2 = 1 #+ accel_input * (1.6 - 0.5)# reduce to slow down acceleration (play with values like 0.3–0.6)

        # Normalize the throttle input from [-1, 1] to [0, 1]
        raw_throttle = -jsInputs[self._throttle_idx]
        throttle_input = max(0.0, min((raw_throttle + 1) / 2.0, 1.0))
        #throttle_input = (jsInputs[self._throttle_idx] + 1) / 2

        # Apply power of 2 to create non-linear function: smooth out acceleration
        throttleCmd = K2 * (throttle_input ** 1.3) * accel_input  # square gives gentler slope
        throttleCmd = min(throttleCmd, 1.0)

        '''
        for i in range(self._joystick.get_numaxes()):
            print(f"Axis {i}: {self._joystick.get_axis(i):.3f}")

        
'''
        # print(f"raw throttle: {raw_throttle}, throttle input: {throttle_input}, throttleCMD: {throttleCmd}")
        

        # Normalize the brake input from [1.0 → -1.0] to [0.0 → 1.0]
        raw_brake = -jsInputs[self._brake_idx]
        brake_input = max(0.0, min((raw_brake + 1) / 2.0, 1.0))

        # Optional: apply smooth non-linear curve (square = gentle start, hard end)
        brakeCmd = (brake_input ** 2)
        if brakeCmd < 0.05:
            brakeCmd = 0

        # print(f"raw brake: {raw_brake}, brake input: {brake_input}, brakeCMD: {brakeCmd}")


        self._control.steer = steerCmd
        self._control.brake = brakeCmd
        self._control.throttle = throttleCmd

        self._control.hand_brake = bool(jsButtons[self._handbrake_idx])

        self.vehicle.apply_control(self._control)
 
host=_config['carla']['server']
port=_config['carla']['port']
front_window_size=_config['sim']['windows']['front_res']
mirror_window_size=_config['sim']['windows']['mirror_res']
autopilot=_config['carla']['autopilot']

vehicle_list=[]
mp=mirror_parameters()

# Connect to the client 
client = carla.Client(host, port)
client.set_timeout(_config['carla']['timeout'])

world = client.load_world(_config['carla']['world'])

'''
# Large World Loading
world = client.load_world('Town11')
settings = world.get_settings()
settings.tile_stream_distance = 2000
world.apply_settings(settings)
'''

# Load layered map for Town 01 with minimum layout plus buildings and parked vehicles
#world = client.load_world('Town10_Opt', carla.MapLayer.Buildings | carla.MapLayer.ParkedVehicles)
# Toggle all buildings off
#world.unload_map_layer(carla.MapLayer.Buildings)


# Getting the world and
world = client.get_world()
original_settings = world.get_settings()

# weather
weather = world.get_weather()
weather.sun_azimuth_angle = 344
weather.sun_altitude_angle = 45
weather.precipitation = 0
weather.precipitation_deposits = 0 # puddles
world.set_weather(weather)

# Set up the simulator in synchronous mode
settings = world.get_settings()
settings.no_rendering_mode=_config['carla']['no_rendering_mode']
settings.synchronous_mode = True # Enables synchronous mode
settings.fixed_delta_seconds = 0.05
world.apply_settings(settings)

# Set up the TM in synchronous mode
traffic_manager = client.get_trafficmanager()
traffic_manager.set_synchronous_mode(True)

# Set a seed so behaviour can be repeated if necessary
traffic_manager.set_random_device_seed(0)
random.seed(0)

'''
# Print list of available vehicles
vehicle_blueprints = world.get_blueprint_library().filter('vehicle')
for car_bp in vehicle_blueprints:
    print (car_bp)
'''

vehicle_tag=_config['carla']['vehicle_tag']

# Instanciating te vehicle to which we attached the sensors
bp = world.get_blueprint_library().filter(vehicle_tag)[0]
bp.set_attribute('role_name', 'hero' )
vehicle = world.spawn_actor(bp, random.choice(world.get_map().get_spawn_points()))
vehicle_list.append(vehicle)
vehicle.set_autopilot(autopilot)

# Find the blueprint of the sensor.
mirror_blueprint = world.get_blueprint_library().find('sensor.camera.rgb')
# Modify the attributes of the blueprint to set image resolution and field of view.
mirror_blueprint.set_attribute('image_size_x', str(mirror_window_size[0]))
mirror_blueprint.set_attribute('image_size_y', str(mirror_window_size[0]))
mirror_blueprint.set_attribute('fov', '110')


# Find the blueprint of the sensor.
car_blueprint = world.get_blueprint_library().find('sensor.camera.rgb')
# Modify the attributes of the blueprint to set image resolution and field of view.
car_blueprint.set_attribute('image_size_x', str(front_window_size[0]))
car_blueprint.set_attribute('image_size_y', str(front_window_size[1]))
car_blueprint.set_attribute('fov', '120')

# Find the blueprint of the sensor.
dash_blueprint = world.get_blueprint_library().find('sensor.camera.rgb')
# Modify the attributes of the blueprint to set image resolution and field of view.
dash_blueprint.set_attribute('image_size_x', str(front_window_size[0]))
dash_blueprint.set_attribute('image_size_y', str(front_window_size[1]))
dash_blueprint.set_attribute('fov', '110')


# Set the time in seconds between sensor captures
#blueprint.set_attribute('sensor_tick', '1')

# Provide the position of the sensor relative to the vehicle.
#left_mirror_transform = carla.Transform(carla.Location(x=.7, y=-1, z=1.2), carla.Rotation(yaw=-150))
#right_mirror_transform = carla.Transform(carla.Location(x=.7, y=1, z=1.2), carla.Rotation(yaw=150))


# lookup pre-defined mirror locations based on vehicle tag

lx, ly,lz = _config['sim']['mirror_location'][vehicle_tag]['left']
rx, ry, rz = _config['sim']['mirror_location'][vehicle_tag]['right']
left_mirror_transform = carla.Transform(carla.Location(x=lx, y=ly, z=lz), carla.Rotation(pitch=mp.left_pitch, yaw=mp.left_yaw))
right_mirror_transform = carla.Transform(carla.Location(x=rx, y=ry, z=rz), carla.Rotation(pitch=mp.right_pitch,yaw=mp.right_yaw))
dashcam_view_transform = carla.Transform(carla.Location(x=0.8, z=1.7))
driver_view_transform = carla.Transform(carla.Location(x=0.25, y=-0.4, z=1.25), carla.Rotation(pitch=0,yaw=0))

# Tell the world to spawn the sensor, don't forget to attach it to your vehicle actor.
lmv_sensor = world.spawn_actor(mirror_blueprint, left_mirror_transform, attach_to=vehicle_list[0])
rmv_sensor = world.spawn_actor(mirror_blueprint, right_mirror_transform, attach_to=vehicle_list[0])
driver_sensor = world.spawn_actor(car_blueprint, driver_view_transform, attach_to=vehicle_list[0])
dashcam_sensor = world.spawn_actor(dash_blueprint, dashcam_view_transform, attach_to=vehicle_list[0])

# Subscribe to the sensor stream by providing a callback function, this function is
# called each time a new image is generated by the sensor.
driver_sensor.listen(lambda data: driver_callback(data, renderObject))
dashcam_sensor.listen(lambda data: dashcam_callback(data, "dashcam_view"))
rmv_sensor.listen(lambda data: process_image_data(data, "right_mirror_view", True))
lmv_sensor.listen(lambda data: process_image_data(data, "left_mirror_view", True))

# Game loop
crashed = False

# Get camera dimensions
image_w = car_blueprint.get_attribute("image_size_x").as_int()
image_h = car_blueprint.get_attribute("image_size_y").as_int()
renderObject = RenderObject(image_w, image_h)

pygame.init()
display = pygame.display.set_mode(front_window_size,  pygame.HWSURFACE | pygame.DOUBLEBUF, display=0 , vsync=1)  # pygame.FULLSCREEN |
# Draw black to the display
display.fill((0,0,0))
display.blit(renderObject.surface, (0,0))
pygame.display.flip()


my_controller=controller(vehicle)

frame_count = 0

try: 
    while not crashed:
        # Advance the simulation time
        world.tick()

        my_controller.parse_vehicle_wheel()
        steer = my_controller._control.steer  # range: -1.0 to 1.0
        torque_strength = abs(steer)           # use magnitude only for now


        # Check reverse button (hold to go into reverse)
        reverse_pressed = my_controller._joystick.get_button(my_controller._reverse_idx)

        if reverse_pressed:
            my_controller._control.reverse = True
            my_controller._control.gear = -1
        else:
            my_controller._control.reverse = False
            my_controller._control.gear = 1

        # Apply control to the vehicle
        my_controller.vehicle.apply_control(my_controller._control)

        # Update the display
        display.blit(renderObject.surface, (0, 0))
        pygame.display.flip()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                crashed = True

            

            # Process Mirror Adjustments
            if event.type == pygame.KEYDOWN:
                if event.key == K_a:
                        mp.left_yaw += 1
                elif event.key == K_d:
                        mp.left_yaw += -1
                elif event.key == K_w:
                        mp.left_pitch += 1
                elif event.key == K_x:
                        mp.left_pitch += -1

                elif event.key == K_f:
                        mp.right_yaw += 1
                elif event.key == K_h:
                        mp.right_yaw += -1
                elif event.key == K_t:
                        mp.right_pitch += 1
                elif event.key == K_b:
                        mp.right_pitch += -1

                left_mirror_transform = carla.Transform(carla.Location(x=lx, y=ly, z=lz), carla.Rotation(pitch=mp.left_pitch, yaw=mp.left_yaw))
                right_mirror_transform = carla.Transform(carla.Location(x=rx, y=ry, z=rz), carla.Rotation(pitch=mp.right_pitch,yaw=mp.right_yaw))
                lmv_sensor.set_transform(left_mirror_transform)
                rmv_sensor.set_transform(right_mirror_transform)


            # TODO - Get vehicle telemetry and post to shared memory
            '''
            #v = vehicle.get_velocity()
            #Speed = (3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2))
            #print (Speed)
            '''
except (carla.TimeoutException, RuntimeError, Exception) as e:
    print(f"[Client Error] Lost connection to CARLA: {e}")
    crashed = True
finally:
    print('Shutting Down')
    
    try:
        smd.shm.close()
        smd.shm.unlink()
    except Exception as e:
        print(f"Shared memory cleanup error: {e}")

    client.apply_batch([carla.command.DestroyActor(x) for x in vehicle_list])
    world.apply_settings(original_settings)
    #save_vid()
    pygame.quit()
    print ("Done")