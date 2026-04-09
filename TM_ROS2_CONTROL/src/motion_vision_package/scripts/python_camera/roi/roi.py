import os
import sys
import cv2
from openni import openni2
import numpy as np

openni2.initialize(os.environ['OPENNI2_REDIST'])
uris = openni2.Device.enumerate_uris()

if not uris:
    print('Camera not found')
    sys.exit(0)

device = openni2.Device.open_file(uris[0])

depth = device.create_depth_stream()
depth.start()

# ROI
x, y, w, h = 160, 80, 320, 240

while True:
    depthFrame = depth.read_frame()
    depthMat = np.frombuffer(depthFrame.get_buffer_as_uint16(), dtype=np.uint16).reshape(depthFrame.height, depthFrame.width, 1)
    depthMat = cv2.convertScaleAbs(depthMat, alpha=255.0 / 1024.0)
    depthColorMap = cv2.applyColorMap(depthMat, cv2.COLORMAP_JET)
    cv2.imshow('Depth', depthColorMap)

    roiMat = depthColorMap[y:y+h, x:x+w]
    cv2.imshow('ROI', roiMat)

    input = cv2.waitKey(1)
    if input == ord('q'):
        break

cv2.destroyAllWindows()
depth.stop()
device.close()
openni2.unload()