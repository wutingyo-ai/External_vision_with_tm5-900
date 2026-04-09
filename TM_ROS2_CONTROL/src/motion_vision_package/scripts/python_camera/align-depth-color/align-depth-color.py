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

overlappingMat = None
enableRegistration = False

while True:
    rgbFrame = color.read_frame()
    rgbMat = np.frombuffer(rgbFrame.get_buffer_as_uint8(), dtype=np.uint8).reshape(rgbFrame.height, rgbFrame.width, 3)
    rgbMat = cv2.cvtColor(rgbMat, cv2.COLOR_BGR2RGB)

    depthFrame = depth.read_frame()
    depthMat = np.frombuffer(depthFrame.get_buffer_as_uint16(), dtype=np.uint16).reshape(depthFrame.height, depthFrame.width, 1)
    depthMat = cv2.convertScaleAbs(depthMat, alpha=255.0 / 1024.0)
    depthMat = cv2.applyColorMap(depthMat, cv2.COLORMAP_JET)

    if depthMat.shape != rgbMat.shape[:2]:
        rgbMat = rgbMat[:depthMat.shape[0], :depthMat.shape[1]]

    overlappingMat = cv2.addWeighted(rgbMat, 0.8, depthMat, 0.2, 0.0)
    

    cv2.imshow("Align", overlappingMat)

    input = cv2.waitKey(1)
    if input == ord('q'):
        break
    elif input == ord('a'):
        enableRegistration = not enableRegistration
        device.set_image_registration_mode(openni2.IMAGE_REGISTRATION_DEPTH_TO_COLOR if enableRegistration else openni2.IMAGE_REGISTRATION_OFF)
        print(f"Image Registration: {'Enable' if enableRegistration else 'Disable'}")

cv2.destroyAllWindows()
color.stop()
depth.stop()
device.close()
openni2.unload()