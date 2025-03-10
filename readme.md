

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
  - [LiDAR (RPLIDAR S2)](#lidar)
    - [Troubleshooting LiDAR](#troubleshooting)
    - [Documentation](#documentation)
    - [Start LiDAR](#starting-the-lidar)
    - [Retuned values](#returned-values-by-the-lidar)
  - [IMU (WitMotion HW T9053-485)](#imu)
    - [Documentation](#documentation-imu)
    - [Troubleshooting](#troubleshooting-imu)
    - [Startingthe IMU](#starting-the-imu)
    - [Returned Values](#retuned-values-imu)
  - [Camera (RealSense D435i)](#camera)
    - [Documentation](#documentation-camera)
  - [External Monitor (Asus Zenscreen)](#external-monitor)
- [Quickstart](#quickstart)
  - [Bash scripts](#bash-scripts)
- [Data extraction](#data-extraction)
  - [Setup](#setup-data-extraction)
  - [Python scripts for extraction](#python-script-for-extraction)
  - [Content](#data-content)
  - [Deserialization](#data-deserialization)
- [Visual Odometry](#visual-odometry)
- [Next steps](#next-steps)


## Installation
The setup is required to use `WitMotion HW T9053-485`, `RPLIDAR S2`  and `RealSense D345i` with ROS2 and Python on `Ubuntu 22.04` and [ROS2 Humble](https://docs.ros.org/en/humble/index.html).


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


####  Returned values by the LiDAR:
* Measured values
    * quality - Quality of the current measurement sample 
    * angle_q6 - The measurement heading angle related to RPLIDAR’s heading. In degree unit, (0°-360°) Stored using fix point numb 
    * distance_q2 - Measured object distance related to RPLIDAR’s rotation center. In millimeter (mm) unit. Represents using fix point. Set to 0 when the measurement is invalid 


### IMU

1. Open the terminal and create a package:
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python witmotion_imu
```


2. Update the setup and config files to include the necessary Python libraries:
  - [package.xml](https://github.com/FjoGeo/ROS_Tutotrial/blob/master/WitMotion/package.xml)
  - [setup.py](https://github.com/FjoGeo/ROS_Tutotrial/blob/master/WitMotion/setup.py)
  - [setup.cfg](https://github.com/FjoGeo/ROS_Tutotrial/blob/master/WitMotion/setup.cfg)

3. Install missing libraries:
```bash
pip install pyserial
```

4. Create a publisher and subscriber in `~/ros2_ws/src/witmotion_imu/witmotion_imu/`
5. Build the Node `colcon build --packages-select witmotion_imu`


#### Troubleshooting IMU
- device not detected: `sudo apt remove brltty` , then unplug and replug it
- grant permission: `sudo chmod 777 /dev/ttyUSB*`
- check the permission: `ls -l /dev/ttyUSB*`


#### Documentation IMU
[Official github repo](https://github.com/WITMOTION/WitHighModbus_HWT9073485)


#### Starting the IMU
First check ports, then run node.
```
cd ~/ros2_ws
source install/setup.bash
ros2 run witmotion_imu talker
```


#### Retuned Values IMU
- `Acc`: Acceleration - These values measure the acceleration forces in different directions
- `As`: Gyroscope - These values measure the rate of rotation around each axis
- `H`: Magnetometer - These values measure the magnetic field strength in different directions
- `Ang`: Euler Angles - These values describe the orientation of the sensor in space


### Camera

1. Open the terminal and create a package:
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_realsense
```

2. Update the setup and config files to include the necessary Python libraries:
 - [package.xml](https://github.com/FjoGeo/ROS_Tutotrial/blob/master/realsense/my_realsense/package.xml)
  - [setup.cfg](https://github.com/FjoGeo/ROS_Tutotrial/blob/master/realsense/my_realsense/setup.cfg)
  - [setup.py](https://github.com/FjoGeo/ROS_Tutotrial/blob/master/realsense/my_realsense/setup.py)

3. Install missing libraries:
```bash
pip install pyrealsense2
pip install opencv-python
```

4. Install librealsense2:  https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md#prerequisites


 Clone the librealsense2 repo
```bash
git clone https://github.com/IntelRealSense/librealsense.git
```
Run Intel Realsense permissions script from librealsense2 root directory:

```bash
cd librealsense
./scripts/setup_udev_rules.sh
```

4. Create a publisher and subscriber in `~/ros2_ws/src/my_realsense/my_realsense`
5. Build the Node `colcon build --packages-select my_realsense`


#### Documentation Camera
[doc](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages)

[Github](https://github.com/IntelRealSense/realsense-ros?tab=readme-ov-file)

### scanControl (micro-epsilon)
asd23



### External Monitor
Steps to install the Asus Zenscreen:

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


## QuickStart

Check the ports and the sequence you connect the device to the PC.

- open ports
  - ```sudo chmod 777 /dev/ttyUSB*```
- navigate into ROS2 directory
  - ```cd ~/ros2_ws/```
- source
  - ``` source install/setup.bash ```
- use a launch file
  - ``` cd launch/ ```
  - ``` ros2 launch <launch_file.py> ```
-  launching sensors separately
  - ```cd ~/ros2_ws/```
  - ```ros2 run rp_test talker_single```
  - ```ros2 run my_realsense talker_rgb```
  - ```ros2 run witmotion_imu talker```

  
Using a launch file will record the data in a bag file.


### Bash scripts
To automate the commands above, it is possible to use one of the scripts in the `Bash` directory. 


## Data extraction

### Setup data extraction

To read a recorded bag  with Python you need multiple libraries:

                sudo apt-get install python-is-python3
                pip install pandas
                pip install catkin_pkg

- rclpy: https://index.ros.org/r/rclpy/
  
                sudo apt install python3-sphinx python3-pip
                sudo -H pip3 install sphinx_autodoc_typehints
Source your ROS 2 installation, then:

                mkdir -p rclpy_ws/src
                cd rclpy_ws/src
                git clone https://github.com/ros2/rclpy.git
                cd ..
                colcon build --symlink-install
                
Source workspace and build docs:

                source install/setup.bash
                cd src/rclpy/rclpy/docs
                make html


### Python script for extraction
To extract all RGB images, pointclouds and IMU/Gyroscope data from a recorded bag-file, use the `extract_all.py` file.
Other extraction files are located under the directory `read and display data`.


### Data content

The recorded file consists of 4 tables:

        
            "SELECT name FROM sqlite_master WHERE type='table';"
        

- schema
    - version of ROS and schema
- metadata
    - empty
- topics
    - list of all recorded topics, type, serialization format and info
- messages
    - contains the serialized data, the topic and the time frame

### Data deserialization

To deserialize the data you need to start rclpy and create a connection to the database.


## Visual Odometry
To calculate a trajectory based on the recorded RGB images, use the [python script](https://github.com/FjoGeo/ROS2_Pipeline_And_Crane_Inspection/blob/master/visual_odometry/odo.py). The script will calculate a trajectory based on sequential images from one camera and create a .csv withthe estimated coordinates.


## Next steps
- Next step will be the adition of a LiDAR based odometry.
- Script for camera calibration
- ...

🔹🚀 If you want better scale estimation, try optical flow-based scale recovery or deep learning-based VO (e.g., DeepVO, SfMLearner).
