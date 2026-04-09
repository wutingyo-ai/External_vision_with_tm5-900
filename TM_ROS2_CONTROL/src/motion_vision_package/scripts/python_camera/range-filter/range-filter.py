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

color = device.create_color_stream()
color.start()

depth = device.create_depth_stream()
depth.start()

thresholdValueUp = 1500
thresholdValueDown = 500

while True:
    rgbFrame = color.read_frame()
    rgbMat = np.frombuffer(rgbFrame.get_buffer_as_uint8(), dtype=np.uint8).reshape(rgbFrame.height, rgbFrame.width, 3)
    rgbMat = cv2.cvtColor(rgbMat, cv2.COLOR_BGR2RGB)
    cv2.imshow('RGB', rgbMat)

    depthFrame = depth.read_frame()
    depthMat = np.frombuffer(depthFrame.get_buffer_as_uint16(), dtype=np.uint16).reshape(depthFrame.height, depthFrame.width, 1)

    thres = depthMat.copy()
    _, thres = cv2.threshold(thres, thresholdValueDown, 1024, cv2.THRESH_TOZERO)
    _, thres = cv2.threshold(thres, thresholdValueUp, 1024, cv2.THRESH_TOZERO_INV)
    _, thres = cv2.threshold(thres, 1, 1024, cv2.THRESH_BINARY_INV)
    thres = cv2.convertScaleAbs(thres, alpha=255.0 / 1024.0)
    cv2.imshow("Threshold", thres)

    depthMat = cv2.convertScaleAbs(depthMat, alpha=255.0 / 1024.0)
    cv2.imshow('Depth', depthMat)
    
    input = cv2.waitKey(1)
    if input == ord('q'):
        break

cv2.destroyAllWindows()
color.stop()
depth.stop()
device.close()
openni2.unload()