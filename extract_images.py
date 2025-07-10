"""
Export Images from 4 RealSense Cameras in separate directories.
Images will have their timestamp in the file name.
Before running set name of the bag file and delete previous export directory! 
"""

#####
PATH_TO_BAG_FILE = "./4camera.db3"  ##### CHANGE THIS
EXPORT_PATH = "export_mexport"         ##### CHANGE THIS
####

import sqlite3, rclpy, os, cv2
import pandas as pd
import numpy as np
from rclpy.serialization import deserialize_message
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge



class ExtractAll:
    def __init__(self, dbPath: str, exportPath: str):
        self.dbPath = dbPath
        self.exportPath = exportPath
        self.list_of_topics = []
        self.images = []
        self.timestamps = None


    def load_bag_file(self):
        self.conn = sqlite3.connect(self.dbPath)
        self.cur = self.conn.cursor()


    def start_rclpy(self):
        self.rclpy = rclpy
        self.rclpy.init()
    

    def createFolder(self):
        try:
            os.makedirs(self.exportPath, exist_ok=True)
            for num in range(4):
                rgbPath = self.exportPath + f"/rgb{num}"
                os.makedirs(rgbPath, exist_ok=True)
        except FileExistsError:
            pass


    def getTopics(self, sensor: str):
        try:
            df = pd.read_sql_query(f"""
                                   SELECT id FROM topics 
                                    WHERE name LIKE '%realsense1/{sensor}%' 
                                   OR name LIKE '%realsense2/{sensor}%'
                                   OR name LIKE '%realsense3/{sensor}%'
                                   OR name LIKE '%realsense4/{sensor}%'
                                    """, self.conn)
               
            self.list_of_topics = df['id'].tolist()
        except Exception as e:
            print(f"Error: {e}")
            print(f"No topics found for {sensor}")
            pass


    def getData(self):
        try:
            list_of_df = []
            for topic in self.list_of_topics:

                df = pd.read_sql_query(f"""SELECT * FROM messages 
                                        WHERE 
                                            topic_id = {str(topic)}""",  self.conn)

                df['deserialized_data'] = df['data'].apply(lambda x: deserialize_message(x, Float32MultiArray))
                df['deserialized_data'] = df['deserialized_data'].apply(lambda msg: msg.data)
                df['converted_timestamp'] = pd.to_datetime(df['timestamp'], unit='ns')
                df[['X', 'Y', 'Z']] = pd.DataFrame(df['deserialized_data'].tolist(), index=df.index)
                df.drop(['data', 'topic_id', 'id', 'deserialized_data'], axis=1, inplace=True)

                list_of_df.append(df)
            return list_of_df
        
        except Exception as e:
            print(f"Error: {e}")
            print("No data found")
            pass


    ## RGB
    def storeImages(self):
        for index,topic in enumerate(self.list_of_topics):
            self.cur.execute(f"SELECT timestamp, data FROM messages WHERE topic_id = {topic}")
            rows = self.cur.fetchall()
            timestamps = []
            for row in rows:
                timestamp, data = row
                image = self.deserializeImage(data)
                self.images.append(image)
                timestamps.append(timestamp)
            self.timestamps = timestamps

            # save images
            bridge = CvBridge()
            for _, (image, timestamp) in enumerate(zip(self.images, self.timestamps)):
                cv_image = bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")
                image_path = self.exportPath + f"/rgb{index}/image_{timestamp}.jpg"
                cv2.imwrite(image_path, cv_image)
            # delete timestamps and images
            self.images = []
            self.timestamps = None


    ## Pointcloud
    def getDataAndTime(self, topic: int):
        self.cur.execute(f"SELECT timestamp, data FROM messages WHERE topic_id={topic}")
        rows = self.cur.fetchall()
        timestamps = [row[0] for row in rows]
        data = [row[1] for row in rows]
        return timestamps, data



    def getAll(self,sensor: list):
        if sensor == 'rgb':
            try:
                self.getTopics(sensor)
                self.storeImages()
                print(f"Data exported for RealSense {sensor} \n")
            except Exception as e:
                print(f"Error: {e}")
                print(f"No data found for RealSense {sensor}")
                pass


    def run_all(self):
        self.load_bag_file()
        self.start_rclpy()
        self.createFolder()
        self.getAll('rgb')


    @staticmethod
    def deserializeImage(data):
        msg = deserialize_message(data, Image)
        return msg


if __name__ == "__main__":
    display = ExtractAll(PATH_TO_BAG_FILE, EXPORT_PATH)
    display.run_all()
