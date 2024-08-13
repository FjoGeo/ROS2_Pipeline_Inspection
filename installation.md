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


- [official github](https://github.com/WITMOTION/WitHighModbus_HWT9073485)
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

### Retuned Values
- `Acc`: Acceleration - These values measure the acceleration forces in different directions
- `As`: Gyroscope - These values measure the rate of rotation around each axis
- `H`: Magnetometer - These values measure the magnetic field strength in different directions
- `Ang`: Euler Angles - These values describe the orientation of the sensor in space

---

## 3. RealSense

### Required installations and documentations

[doc](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages)

[Github](https://github.com/IntelRealSense/realsense-ros?tab=readme-ov-file)


```bash

pip install pyrealsense2

pip install opencv-python

```

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_realsense
```


- update and set in src/my_realsense
  - [package.xml](https://github.com/FjoGeo/ROS_Tutotrial/blob/master/realsense/my_realsense/package.xml)
  - [setup.cfg](https://github.com/FjoGeo/ROS_Tutotrial/blob/master/realsense/my_realsense/setup.cfg)
  - [setup.py](https://github.com/FjoGeo/ROS_Tutotrial/blob/master/realsense/my_realsense/setup.py)

copy the talker from     ROS_Tutotrial/realsense/my_realsense/my_realsense/ to ros2_ws/src/my_realsense/my_realsense

to build the package:
```bash
cd ~/ros2_ws
colcon build
```

---

## Launch File

- navigate to the directory where the [file](https://github.com/FjoGeo/ROS_Tutotrial/blob/master/launch/launch_all_sensors.py) is located and launch it

```
source install/setup.bash
ros2 launch ./<name_of_the_file.py>
```
---

## Display (ZenScreen)

Disable Secure Boot in the BIOS, then reboot, and run the following:
```
# Make this directory if it doesn't exist, and cd into it
mkdir -p ~/Downloads/Install_Files/DisplayLink
cd ~/Downloads/Install_Files/DisplayLink

# Download the Ubuntu APT package provided by Synaptics.com, the official 
# makers of DisplayLink
wget https://www.synaptics.com/sites/default/files/Ubuntu/pool/stable/main/all/synaptics-repository-keyring.deb

# Install the DisplayLink APT package keyring
sudo apt install ./synaptics-repository-keyring.deb

# Update your APT package cache
sudo apt update

# Install the DisplayLink driver provided by Synaptics.com, the official source
sudo apt install displaylink-driver

# Reboot. 
# Now it is plug-and-play. Plug in your DisplayLink adapter and it just works.
# It may take up to 5~10 seconds to recognize a monitor. 
```
