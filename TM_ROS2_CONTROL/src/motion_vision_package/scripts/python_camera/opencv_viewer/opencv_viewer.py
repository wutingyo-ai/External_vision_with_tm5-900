import os
import sys
import cv2
from openni import openni2
import numpy as np

openni2.initialize('/home/lips/TM_ROS2_CONTROL/Camera_diver/AE430_AE470/Redist')
uris = openni2.Device.enumerate_uris()

if not uris:
    print('Camera not found')
    sys.exit(0)

device = openni2.Device.open_file(uris[0])

color = device.create_color_stream()
color.start() # type: ignore

depth = device.create_depth_stream()
depth.start() # type: ignore

ir = device.create_ir_stream()
ir.start() # type: ignore

while True:
    rgbFrame = color.read_frame() # type: ignore
    rgbMat = np.frombuffer(rgbFrame.get_buffer_as_uint8(), dtype=np.uint8).reshape(rgbFrame.height, rgbFrame.width, 3) # type: ignore
    rgbMat = cv2.cvtColor(rgbMat, cv2.COLOR_BGR2RGB)
    cv2.imshow('RGB', rgbMat)

    depthFrame = depth.read_frame() # type: ignore
    depthMat = np.frombuffer(depthFrame.get_buffer_as_uint16(), dtype=np.uint16).reshape(depthFrame.height, depthFrame.width, 1) # type: ignore
    depthMat = cv2.convertScaleAbs(depthMat, alpha=255.0 / 1024.0)
    depthMat = cv2.applyColorMap(depthMat, cv2.COLORMAP_JET)
    cv2.imshow('Depth', depthMat)

    # irFrame = ir.read_frame()
    # irMat = np.frombuffer(irFrame.get_buffer_as_uint16(), dtype=np.uint16).reshape(irFrame.height, irFrame.width, 1)
    # irMat = cv2.convertScaleAbs(irMat, alpha=255.0 / 1024.0)
    # cv2.imshow('IR', irMat)

    input = cv2.waitKey(1)
    if input == ord('q'):
        break

cv2.destroyAllWindows()
color.stop() # type: ignore
depth.stop() # type: ignore
# ir.stop()
device.close()
openni2.unload()