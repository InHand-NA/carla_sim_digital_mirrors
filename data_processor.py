import cv2
import numpy as np
from shared_memory_dict import SharedMemoryDict
from ruamel.yaml import YAML
from pathlib import Path
from screeninfo import get_monitors
import os
import json
import signal
import sys
from camera_geometry import CameraGeometry

MB = 1000 * 1000

LANES_COLOR = [
    (0, 0, 255),
    (0, 255, 0),
    (255, 0, 0),
    (0, 255, 255),
]

DATA_DIR = "logs/"

VEH_MOTION_DATA_FILE = DATA_DIR + "veh_motion_data.csv"
LANES_DATA_FILE = DATA_DIR + "lanes_data.json"

##smd = SharedMemoryDict(name='tokens', size=10000000)

data_processor_loop = True
# 使用字典来管理多个视频写入器
video_writers = {}

frame_count = 0


def get_data_dir(task_name: str):
    data_dir = os.path.join(DATA_DIR, task_name)
    if not os.path.exists(data_dir):
        os.makedirs(data_dir)
    return data_dir


def save_video(img, view_id, fps, task_name: str):
    global video_writers
    global frame_count

    data_dir = get_data_dir(task_name)

    # 为每个 view_id 创建独立的视频写入器
    if view_id not in video_writers:
        video_writers[view_id] = cv2.VideoWriter(
            os.path.join(data_dir, f"{view_id}.mp4"),
            cv2.VideoWriter_fourcc(*"mp4v"),
            fps,
            (img.shape[1], img.shape[0]),
        )
    video_writers[view_id].write(img)
    frame_count += 1


def close_all_videos():
    """关闭所有视频写入器"""
    global video_writers
    for writer in video_writers.values():
        if writer is not None:
            writer.release()
    video_writers.clear()


def store_veh_motion_data(frame_id, veh_mot_info, task_name: str):
    data_dir = get_data_dir(task_name)
    veh_motion_data_file = os.path.join(data_dir, "veh_motion_data.csv")
    if not os.path.exists(veh_motion_data_file):
        with open(veh_motion_data_file, "w") as f:
            f.write(
                "frame_id, locx, locy, locz, vx, vy, vz, accx, accy, accz, angvelx, angvely, angvelz, spd_kmh\n"
            )
    with open(veh_motion_data_file, "a") as f:
        f.write(
            f"""{frame_id},"""
            f"""{veh_mot_info["location"][0]},{veh_mot_info["location"][1]},{veh_mot_info["location"][2]},"""
            f"""{veh_mot_info["velocity"][0]},{veh_mot_info["velocity"][1]},{veh_mot_info["velocity"][2]},"""
            f"""{veh_mot_info["acceleration"][0]},{veh_mot_info["acceleration"][1]},{veh_mot_info["acceleration"][2]},"""
            f"""{veh_mot_info["angular_velocity"][0]},{veh_mot_info["angular_velocity"][1]},{veh_mot_info["angular_velocity"][2]},"""
            f"""{veh_mot_info["speed"]}\n"""
        )


def get_2d_lanes_data(cam_geo, frame_id, lanes_data, img_h=720, img_w=1280):
    lanes_data_2d_list = []
    for lane in lanes_data:
        lane_data_2d = []
        v_min = img_h
        v_list = []

        if lane is None:
            lanes_data_2d_list.append([])
            continue
        for point in lane:
            u, v = cam_geo.roadXYZ_roadframe_iso8855_to_uv(point[0], point[1], point[2])
            if u > 0 and u < img_w and v > 0 and v < img_h:
                u = int(round(u))
                v = int(round(v))
                if v in v_list:
                    continue
                if v > v_min:
                    continue
                lane_data_2d.append([u, v])
                v_list.append(v)
                v_min = min(v_min, v)
        lanes_data_2d_list.append(lane_data_2d)
    return lanes_data_2d_list


def store_lanes_data(cam_geo, frame_id, lanes_data, lanes_data_2d, task_name: str, img_h=720, img_w=1280):
    #lanes_data_2d_list = get_2d_lanes_data(cam_geo, frame_id, lanes_data, img_h, img_w)
    lanes_data_2d_list = lanes_data_2d
    data = {
        "frame_id": frame_id,
        "lanes_3d": lanes_data,
        "lanes_2d": lanes_data_2d_list,
    }
    data_str = json.dumps(data)

    data_dir = get_data_dir(task_name)
    lanes_data_file = os.path.join(data_dir, "lanes_data.json")

    if not os.path.exists(lanes_data_file):
        with open(lanes_data_file, "w") as f:
            f.write(data_str)
            f.write("\n")
    else:
        with open(lanes_data_file, "a") as f:
            f.write(data_str)
            f.write("\n")

    return lanes_data_2d_list


def signal_handler(signum, frame):
    """SIGUSR1 信号处理器"""
    global data_processor_loop
    print(f"Received signal {signum}, processing data...")
    # 在这里添加你想要在收到信号时执行的代码
    # 例如：保存当前数据、刷新缓存等
    data_processor_loop = False
    pass


def init_data_dir(task_name: str):
    data_dir = get_data_dir(task_name)
    if not os.path.exists(data_dir):
        os.makedirs(data_dir)

    # rm *.mp4
    cmd = f"rm -rf {data_dir}/*.mp4"
    os.system(cmd)
    # rm images/
    cmd = f"rm -rf {data_dir}/images"
    os.system(cmd)
    # rm veh_motion_data.csv
    cmd = f"rm -rf {data_dir}/veh_motion_data.csv"
    os.system(cmd)
    # rm lanes_data.json
    cmd = f"rm -rf {data_dir}/lanes_data.json"
    os.system(cmd)

    # create images/
    os.makedirs(os.path.join(data_dir, "images"))
        
    if not os.path.exists(os.path.join(data_dir, "veh_motion_data.csv")):
        with open(os.path.join(data_dir, "veh_motion_data.csv"), "w") as f:
            f.write("frame_id, locx, locy, locz, vx, vy, vz, accx, accy, accz, angvelx, angvely, angvelz, spd_kmh\n")

    return data_dir

def process_data(smd, lock):
    # 在程序退出时调用
    global data_processor_loop

    signal.signal(signal.SIGUSR1, signal_handler)
    print("SIGUSR1 signal handler registered")
    # Read Config File
    configfile = Path("config.yaml")
    _config = YAML(typ="safe").load(configfile)

    vehicle_tag = _config["carla"]["vehicle_tag"]

    mirror_window_size = _config["sim"]["windows"]["mirror_res"]
    dashcam_window_size = _config["sim"]["windows"]["dashcam_res"]
    dashcam_locx = _config["sim"]["dashcam_location"][vehicle_tag][0]
    dashcam_locy = _config["sim"]["dashcam_location"][vehicle_tag][1]
    dashcam_locz = _config["sim"]["dashcam_location"][vehicle_tag][2]
    dashcam_roll = _config["sim"]["dashcam_rotation"][0]
    dashcam_pitch = _config["sim"]["dashcam_rotation"][1]
    dashcam_yaw = _config["sim"]["dashcam_rotation"][2]
    dashcam_fov = _config["sim"]["dashcam_fov"]
    save_debug_images = _config["recorder"]["save_debug_images"]
    task_name = _config["recorder"]["task_name"]

    init_data_dir(task_name)

    monitor = get_monitors()[0]
    print(str(monitor))

    cam_geo = CameraGeometry(
        height=dashcam_locz,
        yaw_deg=dashcam_yaw,
        pitch_deg=dashcam_pitch,
        roll_deg=dashcam_roll,
        field_of_view_deg=dashcam_fov,
        image_width=dashcam_window_size[0],
        image_height=dashcam_window_size[1],
    )

    no_img = np.zeros(
        shape=[mirror_window_size[1], mirror_window_size[0], 3], dtype=np.uint8
    )
    dc_no_img = np.zeros(
        shape=[dashcam_window_size[1], dashcam_window_size[0], 3], dtype=np.uint8
    )

    last_frame_id = -1

    while data_processor_loop:
        # Left Mirror
        lock.acquire()
        if "left_mirror_view" in smd.keys():
            img = smd["left_mirror_view"]
        else:
            # img = no_img
            img = None
        lock.release()
        if img is not None:
            cv2.imshow("left Mirror", img)
            # cv2.moveWindow("left Mirror", monitor.x, monitor.y)

        # Right Mirror
        lock.acquire()
        if "right_mirror_view" in smd.keys():
            img = smd["right_mirror_view"]
        else:
            # img = no_img
            img = None
        lock.release()
        if img is not None:
            cv2.imshow("Right Mirror", img)
            # cv2.moveWindow("Right Mirror", (monitor.width-mirror_window_size[0]-10),monitor.y)

        # Dashcam
        lock.acquire()
        if "dashcam_view" in smd.keys():
            dc_img = smd["dashcam_view"]
            fps = smd["fps"]
        else:
            dc_img = dc_no_img
            fps = -1
        if "frame_count" in smd.keys():
            frame_count = smd["frame_count"]
            seconds = smd["seconds"]
        else:
            print(f"!!!! data_processor: No frame_count in smd")
            frame_count = -1
            seconds = -1

        if "ego_veh_info" in smd.keys():
            ego_veh_info = smd["ego_veh_info"]
            spd_kmh = ego_veh_info["speed"]
            yaw_velocity = ego_veh_info["yaw_velocity"]
        else:
            spd_kmh = -1
            yaw_velocity = -1

        lock.release()
        cv2.putText(
            dc_img,
            f"Fid: {frame_count} | {int(seconds)}s",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 0, 255),
            2,
        )
        cv2.putText(
            dc_img,
            f"Spd: {int(spd_kmh)} km/h | Wyaw: {round(yaw_velocity, 2)} deg/s",
            (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 0, 255),
            2,
        )

        lane_found = False
        if "lanes_data" in smd.keys():
            lock.acquire()
            #lanes_2d = get_2d_lanes_data(cam_geo, frame_count, smd["lanes_data"])
            lanes_2d = smd["lanes_data_2d"]
            lock.release()
            for i, lane_2d in enumerate(lanes_2d):
                for point in lane_2d:
                    cv2.circle(
                        dc_img, (int(point[0]), int(point[1])), 2, LANES_COLOR[i], -1
                    )
                    lane_found = True

        cv2.imshow("Dashcam", dc_img)
        if frame_count % 1 == 0 and save_debug_images:
            data_dir = get_data_dir(task_name)
            image_dir = os.path.join(data_dir, "images")
            if not os.path.exists(image_dir):
                os.makedirs(image_dir)
            cv2.imwrite(os.path.join(image_dir, f"{frame_count}.jpg"), dc_img)
        if not lane_found:
            print(f"No lane found, frame_id: {frame_count}")

        lock.acquire()
        if "data_fifo" in smd.keys():
            data_fifo = smd["data_fifo"]
            while len(data_fifo)  > 0:
                if len(data_fifo) > 1:
                    #print(f"Data fifo length: {len(data_fifo)}")
                    pass
                frame_data = data_fifo.pop(0)
                smd["data_fifo"] = data_fifo  # update the db
                frame_id = frame_data["frame_id"]
                if last_frame_id > 0 and frame_id - last_frame_id > 1:
                    print(f"!!!! data_processor: Frame id gap: {frame_id - last_frame_id}")
                if "dashcam_img" in frame_data.keys():
                    store_veh_motion_data(
                        frame_data["frame_id"], frame_data["ego_veh_info"], task_name
                    )
                    store_lanes_data(
                        cam_geo, frame_data["frame_id"], frame_data["lanes_data"], frame_data["lanes_data_2d"], task_name
                    )
                    save_video(frame_data["dashcam_img"], "dashcam", fps, task_name)
                    last_frame_id = frame_id
                else:
                    print(
                        "!!!! No dashcam image found, ignore this frame, frame_id: ",
                        frame_data["frame_id"],
                    )
                # todo: store dashcam image

        lock.release()

        cv2.waitKey(10)

    # exit the loop
    print("Exiting data processor loop")
    close_all_videos()
    print("All videos closed")


if __name__ == "__main__":
    from multiprocessing import Lock

    smd = SharedMemoryDict(name="tokens", size=10000000)
    lock = Lock()
    process_data(smd, lock)

    print("Closing shared memory")
    # smd.shm.close()
    print("Done")
