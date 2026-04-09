# Remove Background

## Overview

This example shows you how to remove the background of an image using depth camera with range filter.

## Expect Output

![](../../.asset/remove-background.png)

## Prerequisite

- [OpenCV Viewer](../opencv_viewer/)
- [Align Depth & Color](../align-depth-color/)
- [Range Filter](../range-filter/)

## Tutorial

Modify from [range-filter](../range-filter/), we add two variables to determinate the desired distance. Then add two keyboard shorcut to set these values.

```python
thresholdValueUp = 530
thresholdValueDown = 385

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
```

Using these two values to create range filter mask image like we did before.

```python
 _, thres = cv2.threshold(thres, thresholdValueDown, 1024, cv2.THRESH_TOZERO)
_, thres = cv2.threshold(thres, thresholdValueUp, 1024, cv2.THRESH_TOZERO_INV)
_, thres = cv2.threshold(thres, 1, 1024, cv2.THRESH_BINARY)
```

Then we use OpenCV `bitwise_and` operation on the color image and the mask. We also need to convert the mask to `CV_8UC1` because that the format needed by `bitwise_and`. Finally we will get the result image that remove the background.

```python
thres = cv2.convertScaleAbs(thres, alpha=255.0 / 1024.0)
res = cv2.bitwise_and(rgbMat, rgbMat, mask=thres)
```

## Full code

[remove-background.py](https://github.com/HedgeHao/LIPSedgeSDK_Tutorial/blob/master/python/opencv_viewer/remove-background.py)
