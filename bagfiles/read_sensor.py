import sqlite3
from rclpy.serialization import deserialize_message
from std_msgs.msg import Int32, Float32, String
from sensor_msgs.msg import Image, PointCloud2, PointField
import rclpy
import pandas as pd
import numpy as np


# Initialize rclpy
rclpy.init()

bag_file = "my_bag_1/my_bag_0.db3"
conn = sqlite3.connect(bag_file)
cur = conn.cursor()

df = pd.read_sql_query("SELECT * FROM messages WHERE topic_id = 1 LIMIT 100", conn)

# Function to deserialize data
def deserialize_data(data, msg_type):
    return deserialize_message(data, msg_type)

# Apply the deserialization to the "data" column
df['deserialized_data'] = df['data'].apply(lambda x: deserialize_data(x, PointCloud2))

# Extract the actual data from the deserialized messages
df['deserialized_data'] = df['deserialized_data'].apply(lambda msg: msg.data)

# Display the deserialized data
print(df[['timestamp', 'deserialized_data']])

# Close the connection
conn.close()

# Shutdown rclpy
rclpy.shutdown()
