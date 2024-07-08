# Reading ROS2 bag files

To read a recorded bag with Python you need multiple libraries:
- pandas
- sqlite3
- rclpy
- std_msgs

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


            from rclpy.serialization import deserialize_message
            from std_msgs.msg import Int32, Float32, String


            # Function to deserialize data
            def deserialize_data(data, msg_type):
                return deserialize_message(data, msg_type)


            # Apply the deserialization to the "data" column
            df['deserialized_data'] = df['data'].apply(lambda x: deserialize_data(x, Float32))

            # Extract the actual data from the deserialized messages
            df['deserialized_data'] = df['deserialized_data'].apply(lambda msg: msg.data)


## Current topic list

`Note!` Add 1 to the id when using SQL. PointCloud2 has the id 17.


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
