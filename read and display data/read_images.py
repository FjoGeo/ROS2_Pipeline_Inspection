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
        self.dataframe = None
        self.topicID = topicID
        self.images = []
        self.deserializeDataFrame()


    @staticmethod
    def deserializeImage(data):
        msg = deserialize_message(data, Image)
        return msg


    def getDataFromDB(self):
        self.cursor.execute(f"SELECT timestamp, data FROM messages WHERE topic_id = {self.topicID} LIMIT 2")
        rows = self.cursor.fetchall()
        self.conn.close()

        return rows
    

    def deserializeDataFrame(self):
        rows = self.getDataFromDB()
        for row in rows:
            _, data = row
            image = self.deserializeImage(data)
            self.images.append(image)
    

    def displayImages(self):
        bridge = CvBridge()
        for image in self.images:
            cv_image = bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")
            cv2.imshow("Image", cv_image)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        self.rclpy.shutdown()


if __name__ == "__main__":
    dbPath = "my_bag_1/my_bag_0.db3"
    topicID = 16
    imageReader = ImageReader(dbPath, topicID)
    imageReader.displayImages()
