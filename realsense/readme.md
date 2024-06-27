# RealSense



## Installation



[doc](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages)

[Github](https://github.com/IntelRealSense/realsense-ros?tab=readme-ov-file)



Python

```bash

pip install pyrealsense2

pip install opencv-python

```



## Code examples

[Color and Depth](https://github.com/IntelRealSense/librealsense/blob/master/wrappers/python/examples/opencv_viewer_example.py) </br>

[Pointcloud](https://github.com/IntelRealSense/librealsense/blob/development/wrappers/python/examples/opencv_pointcloud_viewer.py)


## USe


```bash

cd ~/ros2_ws
source install/setup.bash
ros2 run my_relasense talker

ros2 run my_relasense listener_rgb
ros2 run my_relasense listener_depth
ros2 run my_relasense listener_pointcloud

```



## Troubleshooting

- 'RuntimeError: Frame didn't arrive within 5000' </br>

  - Solution: Unplug and plug the camera again. change USB cable </br>
