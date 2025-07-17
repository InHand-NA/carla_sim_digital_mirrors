import cv2
import numpy as np
from shared_memory_dict import SharedMemoryDict
from ruamel.yaml import YAML
from pathlib import Path
from screeninfo import get_monitors

smd = SharedMemoryDict(name='tokens', size=50000000)

# Read Config File
configfile=Path("config.yaml")
_config = YAML(typ='safe').load(configfile)

dashcam_window_size=_config['sim']['windows']['dashcam_res']

monitor=get_monitors()[0]
print (str(monitor))

no_img = np.zeros(shape=[dashcam_window_size[1], dashcam_window_size[0], 3], dtype=np.uint8)

cv2.namedWindow("dashcam", cv2.WINDOW_NORMAL)
cv2.resizeWindow("dashcam", dashcam_window_size[0], dashcam_window_size[1])

while True:
    if 'dashcam_view' in smd.keys():
        img = smd['dashcam_view']
    else:
        img = no_img
    cv2.imshow("dashcam", img)
    x = (monitor.width - dashcam_window_size[0]) // 2
    cv2.moveWindow("dashcam", x, 0)

    cv2.waitKey(1)
