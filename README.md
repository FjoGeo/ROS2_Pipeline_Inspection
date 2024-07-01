# Installation and setup guide
The setup is required to use `WitMotion HW T9053-485`, `RPLIDAR S2`  and `RealSense D345i` with ROS2 and Python.



## Creating a Node for ROS2

### 1. RPLIDAR

- in the terminal

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_node_name
```


- change package.xml

```
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>rp_test</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>TODO: License declaration</license>


  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>pyserial</exec_depend>
  <exec_depend>time</exec_depend>
  <exec_depend>threading</exec_depend>
  <exec_depend>pyrplidar</exec_depend>
  

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```


- setup.py

```
from setuptools import find_packages, setup

package_name = 'rp_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial', 'pyrplidar'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = rp_test.rp_publisher:main',
            'listener = rp_test.rp_subscriber:main',
        ],
    },
)
```


- setup.cfg
```
[develop]
script_dir=$base/lib/rp_test
[install]
install_scripts=$base/lib/rp_test
[options]
install_requires=pyrplidar
```


- install missing libraries:
```bash
pip install pyrplidar
pip install pyserial
```

- create a publisher and subscriber in `node_dir/node_dir/`
- build the Node `colcon build --packages-select my_node_name`


#### Troubleshooting
- device not detected: `sudo apt remove brltty` , then unplug and replug it
- grant permission: `sudo chmod 777 /dev/ttyUSB*`
- check the permission: `ls -l /dev/ttyUSB`


#### Documentation
- Links:


    [ROS2](https://github.com/Slamtec/rplidar_ros/blob/ros2/launch/rplidar_s2_launch.py)

    [Python](https://github.com/Hyun-je/pyrplidar)

    [Documentation](http://bucket.download.slamtec.com/ccb3c2fc1e66bb00bd4370e208b670217c8b55fa/LR001_SLAMTEC_rplidar_protocol_v2.1_en.pdf)


#### Starting the node
```
source install/setup.bash
ros2 run node_name python_file
```


####  Returned values by the LiDAR:
* Measured values
    * quality - Quality of the current measurement sample - Related the reflected laser pulse strength
    * angle_q6 - The measurement heading angle related to RPLIDAR’s heading. In degree unit, [0-360) Stored using fix point numb - Refer to the below figure for details. Actual heading = angle_q6/64.0 Degre
    * distance_q2 - Measured object distance related to RPLIDAR’s rotation center. In millimeter (mm) unit. Represents using fix point. Set to 0 when the measurement is invalid - Actual Distance = distance_q2/4.0 m


### 2. Witmotion IMU

- in the terminal

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_node_name
```

- change package.xml

```
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>witmotion_imu</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>TODO: License declaration</license>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>pyserial</exec_depend>
  <exec_depend>time</exec_depend>
  <exec_depend>threading</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

- change setup.py

```
from setuptools import find_packages, setup

package_name = 'witmotion_imu'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = witmotion_imu.witmotion_publisher:main',
            'listener = witmotion_imu.witmotioin_subscriber:main',
        ],
    },
)
```

- setup.cfg

```
[develop]
script_dir=$base/lib/witmotion_imu
[install]
install_scripts=$base/lib/witmotion_imu
[options]
install_requires=pyserial
```