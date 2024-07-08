import sqlite3
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

if rows:
    bridge = CvBridge()
    
    for row in rows:
        timestamp, data = row

        # Deserialize image message
        image = deserialize_image(data)

        # Convert the ROS Image message to an OpenCV image
        cv_image = bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')

        # Display the image
        cv2.imshow('Image', cv_image)
        cv2.waitKey(0)  # Wait indefinitely until a key is pressed
        cv2.destroyAllWindows()

else:
    print("No data found in the database for topic_id=16")

rclpy.shutdown()
