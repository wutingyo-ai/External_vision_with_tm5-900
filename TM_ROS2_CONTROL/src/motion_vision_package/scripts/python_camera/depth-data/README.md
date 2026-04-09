# Depth Data

## Overview

The most important thing while using 3D cameras is to get the depth value. This demo shows how to get the depth data from camera frame. Move the mouse in an OpenCV window and obtain the depth value of the mouse cursor position.

## Expect Output

![](../../.asset/depth_data.gif)

## Prerequisite

- [OpenCV Viewer](../opencv_viewer/)

## Tutorial

- By adding mouse callback function on OpenCV window, we can get the (x,y) coordinate when mouse move over the window.

```python
def onMouseMove(event, x, y, flags, param):
    global depthMat
    if event == cv2.EVENT_MOUSEMOVE and depthMat is not None:
        depth_value = depthMat[y, x]
        print(f"Depth value at ({x},{y}) = {depth_value}")
```

- After converting depthFrame to numpy format, we can retrieve the the data directly.

```python
depth_value = depthMat[y, x]
```

## Full code

[depth_data.cpp](https://github.com/HedgeHao/LIPSedgeSDK_Tutorial/blob/master/python/depth_data/depth-data.py)
