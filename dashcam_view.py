import cv2
import numpy as np
from shared_memory_dict import SharedMemoryDict
from ruamel.yaml import YAML
from pathlib import Path

smd = SharedMemoryDict(name='tokens', size=10000000)

# Read Config File
configfile=Path("config.yaml")
_config = YAML(typ='safe').load(configfile)

dashcam_window_size=_config['sim']['windows']['dashcam_res']

no_img = np.zeros(shape=[dashcam_window_size[1], dashcam_window_size[0], 3], dtype=np.uint8)

while True:
    if 'frame_count' in smd.keys():
        frame_count = smd['frame_count']
    else:
        frame_count = -1
    if 'dashcam_view' in smd.keys():
        img = smd['dashcam_view']
    else:
        img = no_img
    cv2.putText(img, f"Frame: {frame_count}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    cv2.imshow("Dashcam", img)
    cv2.waitKey(10)