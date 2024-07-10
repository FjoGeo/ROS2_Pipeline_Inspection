import sqlite3
import pandas as pd
from sensor_msgs.msg import Image
from rclpy.serialization import deserialize_message
import rclpy
from cv_bridge import CvBridge


class ImageToDataFrame:
    def __init__(self, dbPath, topicID):
        self.dbPath = dbPath
        self.conn = sqlite3.connect(dbPath)
        self.cursor = self.conn.cursor()
        self.rclpy = rclpy
        self.rclpy.init()
        self.dataframe = None
        self.topicID = topicID
        self.imageToCVImage()
        


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
        images = []
        timestamps = []
        for row in rows:
            timestamp, data = row
            image = self.deserializeImage(data)
            images.append(image)
            timestamps.append(timestamp)

        return images, timestamps


    def imageToCVImage(self):
        dataList = []
        bridge = CvBridge()
        images, timestamps = self.deserializeDataFrame()
        for image, timestamp in zip(images, timestamps):
            cv_image = bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")
            dataList.append({'timestamp':timestamp, 'data':cv_image})
        self.dataframe = pd.concat([self.dataframe, pd.DataFrame(dataList)], ignore_index=True)
        self.rclpy.shutdown()


    def displayDataFrame(self):
        print(self.dataframe.head())
        print(self.dataframe.tail())
        # print(self.dataframe.info())
        # print(self.dataframe.describe())
        # print(self.dataframe.columns)
        # print(self.dataframe.shape)
        
    
if __name__ == "__main__":
    dbPath = "my_bag_1/my_bag_0.db3"
    topicID = 16
    imageDF = ImageToDataFrame(dbPath, topicID)
    imageDF.displayDataFrame()
