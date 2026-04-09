# Hello LIPSEdge SDK

## Overview

This example shows how to connect to camera and get device info.

## Expect Output

```
Name:TS800
Uri:USB#VID_2DF2&PID_0213&REV_0007&BUS_02&ADR_10
USB Product ID:531
USB Vendor ID:11762
Vendor:LIPS
```

## Tutorial

- Import openni library 

```python
from openni import openni2
```

- Initialize SDK with `REDIST` folder path

```python
openni2.initialize(os.environ['OPENNI2_REDIST'])
```

- Search for connected devices

```python
uris = openni2.Device.enumerate_uris()
```

- Connect to camera and get camera device info

> Or you can use `open(openni::ANY_DEVICE)` to connect to the first found device.

```python
device = openni2.Device.open_file(uris[0])
deviceInfo = device.get_device_info()
print(f'Name: {deviceInfo.name}')
print(f'Uri: {deviceInfo.uri}')
print(f'USB Product ID: {deviceInfo.usbProductId}')
print(f'USB Vendor ID: {deviceInfo.usbVendorId}')
print(f'Vendor: {deviceInfo.vendor}')
```

- Close camera and deconstruct OpenNI SDK

```python
device.close()
openni2.unload()
```

## Full code

[hello-lipsedge-sdk.py](https://github.com/HedgeHao/LIPSedgeSDK_Tutorial/blob/master/python/hello-lipsedge-sdk/hello-lipsedge-sdk.py)
