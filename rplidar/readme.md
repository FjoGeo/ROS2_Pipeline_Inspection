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
