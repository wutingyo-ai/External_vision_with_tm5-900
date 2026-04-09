# Region Of Interest (ROI)

## Overview

This example demonstrate how get the depth information with ROI using openCV function.

## Expect Output

![](../../.asset/roi.png)

## Prerequisite

- [Hello LIPSEdge SDK](../hello-lipsedge-sdk/)
- [OpenCV Viewer](../opencv_viewer/)

## Tutorial

- After getting the depth frame and convert to OpenCV Mat, we can easily set an ROI by using OpenCV Rectangle.

```python
x, y, w, h = 160, 80, 320, 240
```

- Then apply to the depth frame

```python
depthFrame = depth.read_frame()
depthMat = np.frombuffer(depthFrame.get_buffer_as_uint16(), dtype=np.uint16).reshape(depthFrame.height, depthFrame.width, 1)
roiMat = depthColorMap[y:y+h, x:x+w]
```

- Finally display the two frames

```python
cv2.imshow('Depth', depthColorMap)
cv2.imshow('ROI', roiMat)
```

## Full code

[roi.py](https://github.com/HedgeHao/LIPSedgeSDK_Tutorial/blob/master/python/roi/roi.py)
