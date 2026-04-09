# OpenCV Viewer

## Overview

This article shows how to convert OpenNI2 video frames to OpenCV Mat format. Once you have frame in Mat format, you can leverage all OpenCV functions.

## Expect Output

![](../../.asset/opencv_viewer.png)

## Tutorial

- After connect to camera and start each sensor (color, depth and IR). Read the frame into OpenNI frame

```python
color = device.create_color_stream()
color.start()

depth = device.create_depth_stream()
depth.start()

ir = device.create_ir_stream()
ir.start()

rgbFrame = color.read_frame()
depthFrame = depth.read_frame()
irFrame = ir.read_frame()
```

- Convert OpenNI frame to numpy array

```python
rgbMat = np.frombuffer(rgbFrame.get_buffer_as_uint8(), dtype=np.uint8).reshape(rgbFrame.height, rgbFrame.width, 3)

depthMat = np.frombuffer(depthFrame.get_buffer_as_uint16(), dtype=np.uint16).reshape(depthFrame.height, depthFrame.width, 1)

irMat = np.frombuffer(irFrame.get_buffer_as_uint16(), dtype=np.uint16).reshape(irFrame.height, irFrame.width, 1)

```

- For better visualization, you can do the following conversion

```python
// For color frame
rgbMat = cv2.cvtColor(rgbMat, cv2.COLOR_BGR2RGB)

// For depth frame
depthMat = cv2.convertScaleAbs(depthMat, alpha=255.0 / 1024.0)
depthMat = cv2.applyColorMap(depthMat, cv2.COLORMAP_JET)

// For IR frame
 irMat = np.frombuffer(irFrame.get_buffer_as_uint16(), dtype=np.uint16).reshape(irFrame.height, irFrame.width, 1)
irMat = cv2.convertScaleAbs(irMat, alpha=255.0 / 1024.0)
```

## Full code

[opencv_viewer.py](https://github.com/HedgeHao/LIPSedgeSDK_Tutorial/blob/master/python/opencv_viewer/opencv_viewer.py)
