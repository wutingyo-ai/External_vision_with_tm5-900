# Environment Setup

## Overview
This example help you make sure all the development environment is set up correctly

## Expect Output
* It's okay if the OpenNI Version is 0.0.0.0
```
OpenNI Version:0.0.0.0
OpenCV Version:4.2.0.4.2.0
```

## Tutorial
* Import library
```python
import cv2
from openni import openni2
openni2.initialize(os.environ['OPENNI2_REDIST'])
```

* This program simply include OpenCV and OpenNI headers then print the library version
```python
openniVersion = openni2.get_version()
print(f'''OpenNI Version: {openniVersion.build}.{openniVersion.major}.{openniVersion.minor}.{openniVersion.maintenance}''')
print(cv2.version.opencv_version)
```


## Full code

[environment_setup.py](https://github.com/HedgeHao/LIPSedgeSDK_Tutorial/blob/master/python/environment_setup/environment_setup.py)