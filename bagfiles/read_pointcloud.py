import sqlite3
import numpy as np
import pandas as pd
from sensor_msgs.msg import PointCloud2
from rclpy.serialization import deserialize_message
import rclpy


def deserialize_pointcloud2(data):
    msg = deserialize_message(data, PointCloud2)
    return msg


db_path = "my_bag_1/my_bag_0.db3"
conn = sqlite3.connect(db_path)
cursor = conn.cursor()

cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id=17 LIMIT 1")
rows = cursor.fetchall()

conn.close()


rclpy.init()

timestamps = [row[0] for row in rows]
data = [row[1] for row in rows]


point_clouds = [deserialize_pointcloud2(d) for d in data]


def pointcloud2_to_dataframe(point_cloud, timestamp):
    points = []
    for point in np.ndindex((point_cloud.height, point_cloud.width)):
        
        point_start_index = point[0] * point_cloud.row_step + point[1] * point_cloud.point_step
        point_data = point_cloud.data[point_start_index:point_start_index + 16]
        
        # Extract the data for each field
        x = np.frombuffer(point_data[0:4], dtype=np.float32)[0]
        y = np.frombuffer(point_data[4:8], dtype=np.float32)[0]
        z = np.frombuffer(point_data[8:12], dtype=np.float32)[0]
        r = np.frombuffer(point_data[12:13], dtype=np.uint8)[0]
        g = np.frombuffer(point_data[13:14], dtype=np.uint8)[0]
        b = np.frombuffer(point_data[14:15], dtype=np.uint8)[0]
        # points.append((x, y, z, r, g, b))
        points.append((timestamp, x, y, z, r, g, b))
    
    # Create DataFrame
    df = pd.DataFrame(points, columns=['time','x', 'y', 'z', 'r', 'g', 'b'])
    
    return df

# Convert each point cloud to a DataFrame
#dataframes = [pointcloud2_to_dataframe(pc) for pc in point_clouds]
dataframes = [pointcloud2_to_dataframe(pc, t) for pc,t in zip(point_clouds, timestamps)]

# concat the dataframes
df = pd.concat(dataframes)

print(df.head())
print(df.tail())
print(len(df))



rclpy.shutdown()
