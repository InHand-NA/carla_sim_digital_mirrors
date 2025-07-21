import cv2
import numpy as np
from shared_memory_dict import SharedMemoryDict
from ruamel.yaml import YAML
from pathlib import Path
from screeninfo import get_monitors
import os
import json

DATA_DIR = "logs/"

VEH_MOTION_DATA_FILE = DATA_DIR + "veh_motion_data.csv"
LANES_DATA_FILE = DATA_DIR + "lanes_data.json"

##smd = SharedMemoryDict(name='tokens', size=10000000)


def store_veh_motion_data(frame_id, veh_mot_info):
    if not os.path.exists(VEH_MOTION_DATA_FILE):
        with open(VEH_MOTION_DATA_FILE, 'w') as f:
            f.write("frame_id, locx, locy, locz, vx, vy, vz, accx, accy, accz, angvelx, angvely, angvelz, spd_kmh\n")
    with open(VEH_MOTION_DATA_FILE, 'a') as f:
        f.write(f"""{frame_id},"""
                f"""{veh_mot_info['location'][0]},{veh_mot_info['location'][1]},{veh_mot_info['location'][2]},"""
                f"""{veh_mot_info['velocity'][0]},{veh_mot_info['velocity'][1]},{veh_mot_info['velocity'][2]},"""
                f"""{veh_mot_info['acceleration'][0]},{veh_mot_info['acceleration'][1]},{veh_mot_info['acceleration'][2]},"""
                f"""{veh_mot_info['angular_velocity'][0]},{veh_mot_info['angular_velocity'][1]},{veh_mot_info['angular_velocity'][2]},"""
                f"""{veh_mot_info['speed']}\n""")

def store_lanes_data(frame_id, lanes_data):
    data = {
        "frame_id": frame_id,
        "lanes_data": lanes_data
    }
    data_str = json.dumps(data)
    if not os.path.exists(LANES_DATA_FILE):
        with open(LANES_DATA_FILE, 'w') as f:
            f.write(data_str)
            f.write("\n")
    else:
        with open(LANES_DATA_FILE, 'a') as f:
            f.write(data_str)
            f.write("\n")

def process_data(smd, lock):
    # Read Config File
    configfile=Path("config.yaml")
    _config = YAML(typ='safe').load(configfile)

    mirror_window_size=_config['sim']['windows']['mirror_res']
    dashcam_window_size=_config['sim']['windows']['dashcam_res']
    monitor=get_monitors()[0]
    print (str(monitor))

    no_img = np.zeros(shape=[mirror_window_size[1], mirror_window_size[0], 3], dtype=np.uint8)
    dc_no_img = np.zeros(shape=[dashcam_window_size[1], dashcam_window_size[0], 3], dtype=np.uint8)

    if os.path.exists(VEH_MOTION_DATA_FILE):
        os.remove(VEH_MOTION_DATA_FILE)
    if os.path.exists(LANES_DATA_FILE):
        os.remove(LANES_DATA_FILE)

    while True:
        # Left Mirror
        lock.acquire()
        if 'left_mirror_view' in smd.keys():
            img = smd['left_mirror_view']
        else:
            img = no_img
        lock.release()
        cv2.imshow("left Mirror", img)
        #cv2.moveWindow("left Mirror", monitor.x, monitor.y)

        # Right Mirror
        lock.acquire()
        if 'right_mirror_view' in smd.keys():
            img = smd['right_mirror_view']
        else:
            img = no_img
        lock.release()
        cv2.imshow("Right Mirror", img)
        #cv2.moveWindow("Right Mirror", (monitor.width-mirror_window_size[0]-10),monitor.y)

        # Dashcam
        lock.acquire()
        if 'dashcam_view' in smd.keys():
            img = smd['dashcam_view']
        else:
            img = dc_no_img
        if 'frame_count' in smd.keys():
            frame_count = smd['frame_count']
            seconds = smd['seconds']
        else:
            frame_count = -1
            seconds = -1

        if 'ego_veh_info' in smd.keys():
            ego_veh_info = smd['ego_veh_info']
            spd_kmh = ego_veh_info['speed']
            yaw_velocity = ego_veh_info['yaw_velocity']
        else:
            spd_kmh = -1
            yaw_velocity = -1

        lock.release()
        cv2.putText(img, f"Fid: {frame_count} | {int(seconds)}s", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(img, f"Spd: {int(spd_kmh)} km/h | Wyaw: {round(yaw_velocity, 2)} deg/s", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.imshow("Dashcam", img)
        #cv2.moveWindow("Dashcam", monitor.x, monitor.height-dashcam_window_size[1]-10)

        lock.acquire()
        if 'data_fifo' in smd.keys():
            data_fifo = smd['data_fifo']
            while len(data_fifo) > 0:
                #print(f"Processing frame {data_fifo[0]['frame_id']}")
                # TODO: this block should be locked
                frame_data = data_fifo.pop(0)
                smd['data_fifo'] = data_fifo # update the db
                store_veh_motion_data(frame_data['frame_id'], frame_data['ego_veh_info'])
                store_lanes_data(frame_data['frame_id'], frame_data['lanes_data'])
                # todo: store dashcam image

        lock.release()

        k = cv2.waitKey(20)
        if k == 'q':
            break




if __name__ == "__main__":
    from multiprocessing import Lock

    smd = SharedMemoryDict(name='tokens', size=10000000)
    lock = Lock()
    process_data(smd, lock)

    print("Closing shared memory")  
    #smd.shm.close()
    print("Done")

