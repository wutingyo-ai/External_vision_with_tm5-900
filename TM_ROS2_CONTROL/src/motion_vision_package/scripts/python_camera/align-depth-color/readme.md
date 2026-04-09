# Align Depth and Color Frame

## Overview

The depth sensor and the RGB sensor are positioned differently on the camera, resulting in a slight displacement between the images captured by each sensor. To align the depth and RGB images, we need to enable the "Registration" setting.

## Expect Output

- Registration disabled
  ![](../../.asset/align_disable.png)

- Registration enabled
  ![](../../.asset/align_enable.png)

## Prerequisite

- [OpenCV Viewer](../opencv_viewer/)

## Tutorial

Modify from [opencv_viewer](../opencv_viewer/README.md). After getting color and depth frame, we use opencv `addWeighted` function to overlap these two images.

```python
overlappingMat = cv2.addWeighted(rgbMat, 0.8, depthMat, 0.2, 0.0)
```

To turn on/off registration, Use `setImageRegistrationMode` in SDK and give proper parameter.

```python
// Enable
device.set_image_registration_mode(openni2.IMAGE_REGISTRATION_DEPTH_TO_COLOR)

//Disable
device.set_image_registration_mode(openni2.openni2.IMAGE_REGISTRATION_OFF)
```

Use variable `enableRegistration` to determinate whether to enable registration. When press 'a' on keyboard, turn on/off registration.

```python
 if input == ord('a'):
    enableRegistration = not enableRegistration
    device.set_image_registration_mode(openni2.IMAGE_REGISTRATION_DEPTH_TO_COLOR if enableRegistration else openni2.IMAGE_REGISTRATION_OFF)
    print(f"Image Registration: {'Enable' if enableRegistration else 'Disable'}")
```

## Full code

[align-depth-color.py](https://github.com/HedgeHao/LIPSedgeSDK_Tutorial/blob/master/python/align-depth-color/align-depth-color.py)
