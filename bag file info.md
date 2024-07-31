# Reading ROS2 bag files

To read a recorded bag (which is a sqlite database) with Python you need multiple libraries:

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
                
                


## Content of the bag file

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



## Deserialization of the messages

To deserialize the data you need to start rclpy and create a connection to the database.


## Current topic list


To see the list of topics and tables use this [file](https://github.com/FjoGeo/ROS_Tutotrial/blob/master/read%20and%20display%20data/display_metadata.py) </br>
`Note!` Add 1 to the id when using SQL. 


            0         /serial_data/HY         std_msgs/msg/Float32
            1         /serial_data/HX         std_msgs/msg/Float32
            2        /serial_data/AsZ         std_msgs/msg/Float32
            3        /serial_data/AsX         std_msgs/msg/Float32
            4       /serial_data/AngZ         std_msgs/msg/Float32
            5         /serial_data/HZ         std_msgs/msg/Float32
            6       /serial_data/AngY         std_msgs/msg/Float32
            7        /serial_data/AsY         std_msgs/msg/Float32
            8       /serial_data/AccY         std_msgs/msg/Float32
            9       /serial_data/AngX         std_msgs/msg/Float32
            10      /serial_data/AccZ         std_msgs/msg/Float32
            11      /serial_data/AccX         std_msgs/msg/Float32
            12         /lidar_quality           std_msgs/msg/Int32
            13        /lidar_distance         std_msgs/msg/Float32
            14           /lidar_angle         std_msgs/msg/Float32
            15         /realsense/rgb        sensor_msgs/msg/Image
            16  /realsense/pointcloud  sensor_msgs/msg/PointCloud2
            17       /realsense/depth        sensor_msgs/msg/Image
