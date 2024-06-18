# RPLIDAR S2

- Links:


    [ROS2](https://github.com/Slamtec/rplidar_ros/blob/ros2/launch/rplidar_s2_launch.py)

    [Python](https://github.com/Hyun-je/pyrplidar)

    [Documentation](http://bucket.download.slamtec.com/ccb3c2fc1e66bb00bd4370e208b670217c8b55fa/LR001_SLAMTEC_rplidar_protocol_v2.1_en.pdf)

useful commands
  
      python3 -m venv .venv
      source .venv/bin/activate
      sudo chmod 777 /dev/ttyUSB0


- Port: "COM7" on Windows and "/dev/ttyUSB0" on Linux
- Baudrate: 1000000
- Mode: 0 or 1


Start rplidar in ROS2

    ros2 launch rplidar_ros view_rplidar_s2_launch.py


## Install own package

        cd ~/ros2_ws/src
        ros2 pkg create --build-type ament_python <name_of_package>
        colcon build --packages-select <name_of_pkg>
        ros2 run <name_of_package> talker


##  Returned values by the LiDAR:
* Measured values
    * quality - Quality of the current measurement sample - Related the reflected laser pulse strength
    * angle_q6 - The measurement heading angle related to RPLIDAR’s heading. In degree unit, [0-360) Stored using fix point numb - Refer to the below figure for details. Actual heading = angle_q6/64.0 Degre
    * distance_q2 - Measured object distance related to RPLIDAR’s rotation center. In millimeter (mm) unit. Represents using fix point. Set to 0 when the measurement is invalid - Actual Distance = distance_q2/4.0 m
