import sqlite3
from sensor_msgs.msg import Image
from rclpy.serialization import deserialize_message
import rclpy
import cv2
from cv_bridge import CvBridge


class ImageReader:
    def __init__(self, dbPath, topicID):
        self.dbPath = dbPath
        self.conn = sqlite3.connect(dbPath)
        self.cursor = self.conn.cursor()
        self.rclpy = rclpy
        self.rclpy.init()
        self.topicID = topicID
        self.images = []
        self.timestamps = None
        self.deserializeDataFrame()


    @staticmethod
    def deserializeImage(data):
        msg = deserialize_message(data, Image)
        return msg


    def getDataFromDB(self):
        self.cursor.execute(f"SELECT timestamp, data FROM messages WHERE topic_id = {self.topicID}")
        rows = self.cursor.fetchall()
        self.conn.close()

        return rows
    

    def deserializeDataFrame(self):
        rows = self.getDataFromDB()
        timestamps = []
        for row in rows:
            timestamp, data = row
            image = self.deserializeImage(data)
            self.images.append(image)
            timestamps.append(timestamp)
        self.timestamps = timestamps
        
    

    def displayImages(self):
        bridge = CvBridge()
        for image in self.images:
            cv_image = bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")
            cv2.imshow("Image", cv_image)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        self.rclpy.shutdown()

    def saveImages(self, path):
        bridge = CvBridge()
        for i, (image, timestamp) in enumerate(zip(self.images, self.timestamps)):
            cv_image = bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")
            cv2.imwrite(f"{path}/{timestamp}.jpg", cv_image)
        self.rclpy.shutdown()


if __name__ == "__main__":
    dbPath = "my_bag_1/my_bag_0.db3"
    topicID = 18
    imageReader = ImageReader(dbPath, topicID)
    # imageReader.displayImages()
    imageReader.saveImages("images")
