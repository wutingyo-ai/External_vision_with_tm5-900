import os
import sys
import cv2
from openni import openni2
import numpy as np

WINDOW_NAME = "Depth_data"
depthMat = None

def onMouseMove(event, x, y, flags, param):
    global depthMat
    if event == cv2.EVENT_MOUSEMOVE and depthMat is not None:
        depth_value = depthMat[y, x]
        print(f"Depth value at ({x},{y}) = {depth_value}")


openni2.initialize(os.environ['OPENNI2_REDIST'])
uris = openni2.Device.enumerate_uris()

if not uris:
    print('Camera not found')
    sys.exit(0)

device = openni2.Device.open_file(uris[0])

depth = device.create_depth_stream()
depth.start()

cv2.namedWindow(WINDOW_NAME)
cv2.setMouseCallback(WINDOW_NAME, onMouseMove)

while True:
    depthFrame = depth.read_frame()
    depthMat = np.frombuffer(depthFrame.get_buffer_as_uint16(), dtype=np.uint16).reshape(depthFrame.height, depthFrame.width, 1)
    depthMatDisplay = cv2.convertScaleAbs(depthMat, alpha=255.0 / 1024.0)
    depthMatDisplay = cv2.applyColorMap(depthMatDisplay, cv2.COLORMAP_JET)
    cv2.imshow(WINDOW_NAME, depthMatDisplay)

    input = cv2.waitKey(1)
    if input == ord('q'):
        break

cv2.destroyAllWindows()
depth.stop()
device.close()
openni2.unload()