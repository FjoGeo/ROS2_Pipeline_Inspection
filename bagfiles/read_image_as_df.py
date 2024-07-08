import sqlite3
import pandas as pd
from sensor_msgs.msg import Image
from rclpy.serialization import deserialize_message
import rclpy
import cv2
from cv_bridge import CvBridge

# Function to deserialize Image message
def deserialize_image(data):
    # Use rclpy's deserialize_message to convert binary data to Image
    msg = deserialize_message(data, Image)
    return msg

# Path to your .db3 bag file
db_path = "my_bag_1/my_bag_0.db3"

# Connect to the database
conn = sqlite3.connect(db_path)
cursor = conn.cursor()

# Query to get messages with topic_id=16 (adjust query as needed)
cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id=16 LIMIT 5")
rows = cursor.fetchall()  # Fetch multiple rows

# Close the connection
conn.close()

rclpy.init()

# Initialize the DataFrame
df = pd.DataFrame(columns=['timestamp', 'image'])

if rows:
    bridge = CvBridge()
    data_list = []  # Temporary list to store data before concatenating
    
    for row in rows:
        timestamp, data = row

        # Deserialize image message
        image = deserialize_image(data)

        # Convert the ROS Image message to an OpenCV image
        cv_image = bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')

        # Add the image and timestamp to the temporary list
        data_list.append({'timestamp': timestamp, 'image': cv_image})
    
    # Concatenate the temporary list to the DataFrame
    df = pd.concat([df, pd.DataFrame(data_list)], ignore_index=True)

# Shutdown ROS
rclpy.shutdown()

# Display the first image as an example
# if not df.empty:
#     cv2.imshow('Image', df.loc[0, 'image'])
#     cv2.waitKey(0)
#     cv2.destroyAllWindows()

# Example of accessing the DataFrame
print(df.head())
