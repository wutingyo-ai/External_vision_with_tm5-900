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
device.set_image_registration_mode(openni2.IMAGE_REGISTRATION_DEPTH_TO_COLOR)

color = device.create_color_stream()
color.start()

depth = device.create_depth_stream()
depth.start()

thresholdValueUp = 530
thresholdValueDown = 385

while True:
    rgbFrame = color.read_frame()
    rgbMat = np.frombuffer(rgbFrame.get_buffer_as_uint8(), dtype=np.uint8).reshape(rgbFrame.height, rgbFrame.width, 3)
    rgbMat = cv2.cvtColor(rgbMat, cv2.COLOR_BGR2RGB)
    cv2.imshow('Origin', rgbMat)

    depthFrame = depth.read_frame()
    depthMat = np.frombuffer(depthFrame.get_buffer_as_uint16(), dtype=np.uint16).reshape(depthFrame.height, depthFrame.width, 1)

    if depthMat.shape != rgbMat.shape[:2]:
            rgbMat = rgbMat[:depthMat.shape[0], :depthMat.shape[1]]

    thres = depthMat.copy()
    _, thres = cv2.threshold(thres, thresholdValueDown, 1024, cv2.THRESH_TOZERO)
    _, thres = cv2.threshold(thres, thresholdValueUp, 1024, cv2.THRESH_TOZERO_INV)
    _, thres = cv2.threshold(thres, 1, 1024, cv2.THRESH_BINARY)
    thres = cv2.convertScaleAbs(thres, alpha=255.0 / 1024.0)
    cv2.imshow("Thres", thres)

    res = cv2.bitwise_and(rgbMat, rgbMat, mask=thres)
    cv2.imshow('Color', res)

    
    input = cv2.waitKey(1)
    if input == ord('p'):
        if thresholdValueUp + 5 <= 2000:
            thresholdValueUp += 5
        else:
            thresholdValueUp = 2000
        print(f"Threshold Value (Up): {thresholdValueUp} (Down): {thresholdValueDown}")

    elif input == ord('o'):
        if thresholdValueUp - 5 >= 0 and thresholdValueUp - 5 >= thresholdValueDown:
            thresholdValueUp -= 5
        print(f"Threshold Value (Up): {thresholdValueUp} (Down): {thresholdValueDown}")

    elif input == ord('x'):
        if thresholdValueDown + 5 <= 2000 and thresholdValueDown + 5 <= thresholdValueUp:
            thresholdValueDown += 5
        print(f"Threshold Value (Up): {thresholdValueUp} (Down): {thresholdValueDown}")

    elif input == ord('z'):
        if thresholdValueDown - 5 >= 0:
            thresholdValueDown -= 5
        else:
            thresholdValueDown = 0
        print(f"Threshold Value (Up): {thresholdValueUp} (Down): {thresholdValueDown}")

    elif input == ord('q'):
        break

cv2.destroyAllWindows()
color.stop()
depth.stop()
device.close()
openni2.unload()