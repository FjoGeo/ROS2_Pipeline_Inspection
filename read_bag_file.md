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
