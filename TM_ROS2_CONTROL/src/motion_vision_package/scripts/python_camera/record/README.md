# Remove Background

## Overview

This example shows you how to remove the background of an image using depth camera with range filter.

## Expect Output

After recording, we will get a `record.oni` file. Using `NiViewer`, you can replay the recording frames.

![](../../.asset/record-2.png)

![](../../.asset/record-1.png)



## Prerequisite

- [Hello LIPSEdge SDK](../hello-lipsedge-sdk/)

## Tutorial

Create a `openni::Recorder` object to control the recording.
```python
recorder = openni2.Recorder("record.oni".encode('utf-8'))
```

You can choose which stream you want to attach. Here we attach all 3 streams.

```python
recorder.attach(color)
recorder.attach(depth)
recorder.attach(ir)
```

Then we use `start()` to record and `stop()` to stop.
```python
recorder.start()

recorder.stop()
```

## Full code

[record.py](https://github.com/HedgeHao/LIPSedgeSDK_Tutorial/blob/master/python/record/record.py)