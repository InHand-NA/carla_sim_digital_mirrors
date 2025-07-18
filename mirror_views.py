import cv2
import numpy as np
from shared_memory_dict import SharedMemoryDict
from ruamel.yaml import YAML
from pathlib import Path
from screeninfo import get_monitors

smd = SharedMemoryDict(name='tokens', size=10000000)

# Read Config File
configfile=Path("config.yaml")
_config = YAML(typ='safe').load(configfile)

mirror_window_size=_config['sim']['windows']['mirror_res']
dashcam_window_size=_config['sim']['windows']['dashcam_res']
monitor=get_monitors()[0]
print (str(monitor))

no_img = np.zeros(shape=[mirror_window_size[1], mirror_window_size[0], 3], dtype=np.uint8)
dc_no_img = np.zeros(shape=[dashcam_window_size[1], dashcam_window_size[0], 3], dtype=np.uint8)

while True:
    # Left Mirror
    if 'left_mirror_view' in smd.keys():
        img = smd['left_mirror_view']
    else:
        img = no_img
    cv2.imshow("left Mirror", img)
    #cv2.moveWindow("left Mirror", monitor.x, monitor.y)

    # Right Mirror
    if 'right_mirror_view' in smd.keys():
        img = smd['right_mirror_view']
    else:
        img = no_img
    cv2.imshow("Right Mirror", img)
    #cv2.moveWindow("Right Mirror", (monitor.width-mirror_window_size[0]-10),monitor.y)

    # Dashcam
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
    cv2.putText(img, f"Frame: {frame_count} - {seconds}s", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    cv2.imshow("Dashcam", img)
    #cv2.moveWindow("Dashcam", monitor.x, monitor.height-dashcam_window_size[1]-10)

    cv2.waitKey(20)

smd.shm.close()
