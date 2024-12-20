

# Introduction
This repository for the projects [**3D-OLE**](https://www.hcu-hamburg.de/en/geomatik/harald-sternberg/3d-ole) and **5G AKI** contains a set of ROS2 drivers, installation instructions and scripts to collect and extract data.

This project integrates and manages various sensors to facilitate efficient inspection processes. The primary sensors used are:

- **WitMotion HW T9053-485**
- **RPLIDAR S2**
- **RealSense D345i**

All sensors are connected to an **Intel NUC 13 Pro**.  

<div style="text-align: center;">
  <img title="5G AKI" src="/images/platform.jpg" width="400" height="300">
</div>


## Table of contents
- [Introduction](#introduction)
- [Installation](#installation)
  - [LiDAR](#lidar)
    - [Troubleshooting LiDAR](#troubleshooting)
    - [Documentation](#documentation)
    - [Start LiDAR](#starting-the-lidar)
  - [IMU](#imu)


## Installation
The setup is required to use `WitMotion HW T9053-485`, `RPLIDAR S2`  and `RealSense D345i` with ROS2 and Python on `Ubuntu 22.04` and `ROS2` [Humble](https://docs.ros.org/en/humble/index.html).


Note: The workspace is named `ros2_ws`, the LiDAR package `rp_test`, the camera package `my_realsense` and the IMU package `witmotion_imu`.

### LiDAR 
1. Open the terminal and create a package:
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python rp_test
```

2. Update the setup and config files to include the necessary Python libraries:
  - [package.xml](https://github.com/FjoGeo/ROS_Tutotrial/blob/master/rplidar/rp_test/package.xml)
  - [setup.py](https://github.com/FjoGeo/ROS_Tutotrial/blob/master/rplidar/rp_test/setup.py)
  - [setup.cfg](https://github.com/FjoGeo/ROS_Tutotrial/blob/master/rplidar/rp_test/setup.cfg)

3. Install missing libraries:
```bash
pip install rplidar
pip install pyserial
```

4. Create a publisher and subscriber in `rp_test/rp_test/`
5. Build the Node `colcon build --packages-select rp_test`


#### Troubleshooting
- device not detected: `sudo apt remove brltty` , then unplug and replug it
- grant permission: `sudo chmod 777 /dev/ttyUSB*`
- check the permission: `ls -l /dev/ttyUSB*`


#### Documentation

[ROS2](https://github.com/Slamtec/rplidar_ros/blob/ros2/launch/rplidar_s2_launch.py)

[Python](https://github.com/SkoltechRobotics/rplidar)

[Documentation](http://bucket.download.slamtec.com/ccb3c2fc1e66bb00bd4370e208b670217c8b55fa/LR001_SLAMTEC_rplidar_protocol_v2.1_en.pdf)


#### Starting the LiDAR
This code is for starting a single LiDAR node. Check if the port is correct and open before using it.

```
cd ~/ros2_ws
source install/setup.bash
ros2 run rp_test talker_single
```


### IMU
test







## QuickStart

Connect the LiDAR first and then the IMU!

- open ports
  - ```sudo chmod 777 /dev/ttyUSB*```
- navigate into ROS2 directory
  - ```cd ~/ros2_ws/```
- source
  - ``` source install/setup.bash ```
- launch all sensors
  - ``` cd launch/ ```
  - ``` ros2 launch launch_all.py ```
- or launching sensors separately
  - ```cd ~/ros2_ws/```
  - ```ros2 run rp_test talker```
  - ```ros2 run my_realsense talker```
  - ```ros2 run witmotion_imu talker```

  
## Instructions

- All about the recorded [bag file](https://github.com/FjoGeo/ROS_Tutotrial/blob/master/bag%20file%20info.md)
- [Script](https://github.com/FjoGeo/ROS_Tutotrial/blob/master/launch/launch_all_sensors.py) to launch all sensors at the same time
- Script to display the [Metadata](https://github.com/FjoGeo/ROS_Tutotrial/blob/master/read%20and%20display%20data/display_metadata.py) 
