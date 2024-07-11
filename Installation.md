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


## 3. RealSense

### Required installations and documentations

[doc](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages)

[Github](https://github.com/IntelRealSense/realsense-ros?tab=readme-ov-file)


```bash

pip install pyrealsense2

pip install opencv-python

```


- package.xml

```
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_realsense</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>TODO: License declaration</license>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>pyserial</exec_depend>
  <exec_depend>time</exec_depend>
  <exec_depend>threading</exec_depend>
  <exec_depend>numpy</exec_depend>
  <exec_depend>cv2</exec_depend>
  <exec_depend>pyrealsense2</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>



  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```


- setup.cfg
```
[develop]
script_dir=$base/lib/my_realsense
[install]
install_scripts=$base/lib/my_realsense
[optional]
install_require=pyrealsense2
```

- setup.py

```
from setuptools import find_packages, setup

package_name = 'my_realsense'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','rclpy', 'sensor_msgs', 'cv_bridge', 'numpy', 'pyrealsense2'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = my_realsense.talker:main',
            'listener_rgb = my_realsense.sub_rgb:main',
            'listener_depth = my_realsense.sub_depth:main',
            'listener_pointcloud = my_realsense.sub_pc:main',
        ],
    },
)
```


## Launch File

- Start all Nodes and store the data in a `.bag` file

```
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rp_test',
            executable='talker',
            name='talker_node'
        ),
        Node(
            package='witmotion_imu',
            executable='talker',
            name='talker_node2'

        ),
        Node(
            package='my_realsense',
            executable='talker',
            name='talker_node3'
        ),

        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', 'my_bag', '/lidar_scan', '/serial_data', 'realsense/depth', 'realsense/pointcloud', 'realsense/rgb'],
            output='screen'
        )
 
    ])
```
