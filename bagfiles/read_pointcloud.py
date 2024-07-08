import sqlite3
import numpy as np
import pandas as pd
from sensor_msgs.msg import PointCloud2
from rclpy.serialization import deserialize_message
import rclpy

# Function to deserialize PointCloud2 message
def deserialize_pointcloud2(data):
    # Use rclpy's deserialize_message to convert binary data to PointCloud2
    msg = deserialize_message(data, PointCloud2)
    return msg

# Path to your .db3 bag file
db_path = "my_bag_1/my_bag_0.db3"

# Connect to the database
conn = sqlite3.connect(db_path)
cursor = conn.cursor()

# Query to get messages with topic_id=16 (adjust query as needed)
cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id=17 LIMIT 5")
rows = cursor.fetchall()

# Close the connection
conn.close()

rclpy.init()

# Extract the binary data and timestamps
timestamps = [row[0] for row in rows]
data = [row[1] for row in rows]

# Deserialize all messages
point_clouds = [deserialize_pointcloud2(d) for d in data]

def pointcloud2_to_dataframe(point_cloud):
    # Extract x, y, z coordinates from PointCloud2 message
    points = []
    for point in np.ndindex((point_cloud.height, point_cloud.width)):
        # Assuming point_cloud.data is a list of bytes where each point consists of 3 float values (x, y, z)
        point_data = point_cloud.data[point[0] * point_cloud.row_step + point[1] * point_cloud.point_step:
                                     point[0] * point_cloud.row_step + point[1] * point_cloud.point_step + 12]
        x = np.frombuffer(bytes(point_data[0:4]), dtype=np.float32)
        y = np.frombuffer(bytes(point_data[4:8]), dtype=np.float32)
        z = np.frombuffer(bytes(point_data[8:12]), dtype=np.float32)
        points.append([x[0], y[0], z[0]])
    
    # Create DataFrame
    df = pd.DataFrame(points, columns=['x', 'y', 'z'])
    
    return df

# Convert each point cloud to a DataFrame
dataframes = [pointcloud2_to_dataframe(pc) for pc in point_clouds]

# concat the dataframes
df = pd.concat(dataframes)

print(df.head())
print(df.tail())
print(len(df))

# # Example: Accessing the first DataFrame
# first_dataframe = dataframes[0]
# print(first_dataframe.head())
# print(len(dataframes))


rclpy.shutdown()
