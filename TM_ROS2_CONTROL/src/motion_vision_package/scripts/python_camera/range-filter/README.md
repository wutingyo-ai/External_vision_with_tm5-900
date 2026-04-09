# Range filter

## Overview

Since depth camera can measure the distance for each pixel. We can easily build a range filter, focusing of specific distance of data that we want are interesting in.

## Expect Output

![](../../.asset/range-filter.png)

## Prerequisite

- [OpenCV Viewer](../opencv_viewer/)

## Tutorial

Modify from [opencv_viewer](../opencv_viewer/README.md). After getting color and depth frame, we use opencv `threshold` function to filter out distance less than `threasholdValueDown` and greater than `thresholdValueUp`.

The distance units used by the SDK vary depending on the camera. In this example, we use the LIPSEdge L215 camera, which uses millimeters (mm) as the unit. So, here we set the distance limit from 500mm to 1500mm.

```python
thresholdValueUp = 1500
thresholdValueDown = 500

...

_, thres = cv2.threshold(thres, thresholdValueDown, 1024, cv2.THRESH_TOZERO)
_, thres = cv2.threshold(thres, thresholdValueUp, 1024, cv2.THRESH_TOZERO_INV)
```

Then we do the threshold again, to convert all the remaining depth value to 1, and the other value to 0. This create a region of interest (white area) base on distance.

```python
_, thres = cv2.threshold(thres, 1, 1024, cv2.THRESH_BINARY_INV)
```

Finally, convert `depth` and `thres` to gray scale value then display all of it

```python
depthMat = cv2.convertScaleAbs(depthMat, alpha=255.0 / 1024.0)
thres = cv2.convertScaleAbs(thres, alpha=255.0 / 1024.0)

cv2.imshow("Threshold", thres)
cv2.imshow('Depth', depthMat)
```

## Full code

[range-filter](https://github.com/HedgeHao/LIPSedgeSDK_Tutorial/blob/master/python/range-filter/range-filter.py)
