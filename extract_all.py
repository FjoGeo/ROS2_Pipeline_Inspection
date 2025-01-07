import sqlite3
import rclpy
import pandas as pd
from rclpy.serialization import deserialize_message
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image, PointCloud2
import os
import cv2
from cv_bridge import CvBridge


class ExtractAll:
    def __init__(self, dbPath: str):
        self.dbPath = dbPath
        self.list_of_topics = []
        self.images = []
        self.timestamps = None
        self.conn = sqlite3.connect(dbPath)
        self.cur = self.conn.cursor()
        self.rclpy = rclpy
        self.rclpy.init()
        self.createFolder()
        self.getAll(['accel', 'gyro', 'rgb'])
        self.conn.close()
        self.rclpy.shutdown()
    

    def createFolder(self):
        try:
            os.mkdir('data_export')
            os.mkdir('data_export/images1')
            os.mkdir('data_export/images2')
        except FileExistsError:
            pass


    def getTopics(self, sensor: str):
        try:
            df = pd.read_sql_query(f"""SELECT id FROM topics WHERE name LIKE '%realsense1/{sensor}%' OR name LIKE '%realsense2/{sensor}%'
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
            # self.conn.close()
            timestamps = []
            for row in rows:
                timestamp, data = row
                image = self.deserializeImage(data)
                self.images.append(image)
                timestamps.append(timestamp)
            self.timestamps = timestamps

            # save images
            bridge = CvBridge()
            for i, (image, timestamp) in enumerate(zip(self.images, self.timestamps)):
                cv_image = bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")
                cv2.imwrite(f"data_export/images{index+1}/image_{timestamp}.jpg", cv_image)
            # delete timestamps and images
            self.images = []
            self.timestamps = None


    def getAll(self,sensors: list):
        for sensor in sensors:
            if sensor != 'rgb':
                try:
                    self.getTopics(sensor)
                    list_of_df = self.getData()
                    for i,df in enumerate(list_of_df):
                        df.to_csv(f"data_export/realsense_{sensor}_{i}_data.csv", index=False)
                    print(f"Data exported for RealSense {sensor} \n")

                except Exception as e:
                    print(f"Error: {e}")
                    print(f"No data found for RealSense {sensor}")
                    pass
            if sensor == 'rgb':
                try:
                    self.getTopics(sensor)
                    self.storeImages()
                    print(f"Data exported for RealSense {sensor} \n")
                except Exception as e:
                    print(f"Error: {e}")
                    print(f"No data found for RealSense {sensor}")
                    pass


    @staticmethod
    def deserializeImage(data):
        msg = deserialize_message(data, Image)
        return msg
    
    
    @staticmethod
    def deserializePointcloud2(data):
        msg = deserialize_message(data, PointCloud2)
        return msg



if __name__ == "__main__":
    dbPath = "my_bag/bagfile.db3"
    display = ExtractAll(dbPath)