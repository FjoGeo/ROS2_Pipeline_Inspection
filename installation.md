# Installation and setup guide
The setup is required to use `WitMotion HW T9053-485`, `RPLIDAR S2`  and `RealSense D345i` with ROS2 and Python.
We use ROS2 [Humble](https://docs.ros.org/en/humble/index.html)

## Table of content

- [RPLIDAR](#1-rplidar)
- [WitMotion](#2-witmotion-imu)
- [RealSense](#3-realsense)
- [Launch File](#launch-file)


---

## 1. RPLIDAR

- in the terminal

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_node_name
```


- update and set
  - [package.xml](https://github.com/FjoGeo/ROS_Tutotrial/blob/master/rplidar/rp_test/package.xml)
  - [setup.py](https://github.com/FjoGeo/ROS_Tutotrial/blob/master/rplidar/rp_test/setup.py)
  - [setup.cfg](https://github.com/FjoGeo/ROS_Tutotrial/blob/master/rplidar/rp_test/setup.cfg)



- install missing libraries:
```bash
pip install pyrplidar
pip install pyserial
```

- create a publisher and subscriber in `node_dir/node_dir/`
- build the Node `colcon build --packages-select my_node_name`


### Troubleshooting
- device not detected: `sudo apt remove brltty` , then unplug and replug it
- grant permission: `sudo chmod 777 /dev/ttyUSB*`
- check the permission: `ls -l /dev/ttyUSB`


### Documentation
- Links:


    [ROS2](https://github.com/Slamtec/rplidar_ros/blob/ros2/launch/rplidar_s2_launch.py)

    [Python](https://github.com/Hyun-je/pyrplidar)

    [Documentation](http://bucket.download.slamtec.com/ccb3c2fc1e66bb00bd4370e208b670217c8b55fa/LR001_SLAMTEC_rplidar_protocol_v2.1_en.pdf)


### Starting the node
```
source install/setup.bash
ros2 run node_name python_file.py
```


###  Returned values by the LiDAR:
* Measured values
    * quality - Quality of the current measurement sample - Related the reflected laser pulse strength
    * angle_q6 - The measurement heading angle related to RPLIDAR’s heading. In degree unit, [0-360) Stored using fix point numb - Refer to the below figure for details. Actual heading = angle_q6/64.0 Degre
    * distance_q2 - Measured object distance related to RPLIDAR’s rotation center. In millimeter (mm) unit. Represents using fix point. Set to 0 when the measurement is invalid - Actual Distance = distance_q2/4.0 m

---

## 2. Witmotion IMU

- in the terminal

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_node_name
```

- update and set
  - [package.xml](https://github.com/FjoGeo/ROS_Tutotrial/blob/master/WitMotion/package.xml)
  - [setup.py](https://github.com/FjoGeo/ROS_Tutotrial/blob/master/WitMotion/setup.py)
  - [setup.cfg](https://github.com/FjoGeo/ROS_Tutotrial/blob/master/WitMotion/setup.cfg)

- create a publisher and subscriber in `node_dir/node_dir/`
- build the Node `colcon build --packages-select my_node_name`

### Troubleshooting
- device not detected: `sudo apt remove brltty` , then unplug and replug it
- grant permission: `sudo chmod 777 /dev/ttyUSB*`
- check the permission: `ls -l /dev/ttyUSB*`

---

## 3. RealSense

### Required installations and documentations

[doc](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages)

[Github](https://github.com/IntelRealSense/realsense-ros?tab=readme-ov-file)


```bash

pip install pyrealsense2

pip install opencv-python

```

- update and set
  - [package.xml](https://github.com/FjoGeo/ROS_Tutotrial/blob/master/realsense/my_realsense/package.xml)
  - [setup.cfg](https://github.com/FjoGeo/ROS_Tutotrial/blob/master/realsense/my_realsense/setup.cfg)
  - [setup.py](https://github.com/FjoGeo/ROS_Tutotrial/blob/master/realsense/my_realsense/setup.py)

---

## Launch File

- navigate to the directory where the [file](https://github.com/FjoGeo/ROS_Tutotrial/blob/master/launch/test_launch.py) is located and launch it

```
source install/setup.bash
ros2 launch ./<name_of_the_file.py>
```
