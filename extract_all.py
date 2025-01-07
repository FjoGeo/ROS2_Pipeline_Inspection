import sqlite3, rclpy, os, cv2
import pandas as pd
import numpy as np
from rclpy.serialization import deserialize_message
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image, PointCloud2
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
        self.getAll(['accel', 'gyro', 'rgb', 'pointcloud'])
        self.conn.close()
        self.rclpy.shutdown()
    

    def createFolder(self):
        try:
            os.mkdir('data_export')
            os.mkdir(f'data_export/rgb1')
            os.mkdir(f'data_export/rgb0')
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
                cv2.imwrite(f"data_export/rgb{index}/image_{timestamp}.jpg", cv_image)
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


    def deserializeDataIntoPointCloud(self, topic: int):
        timestamps, data = self.getDataAndTime(topic)
        pointclouds = [self.deserializePointcloud2(d) for d in data]
        return pointclouds, timestamps
    

    def pointcloud2ToDataFrame(self, pointcloud, timestamp):
        points = []
        for point in np.ndindex((pointcloud.height, pointcloud.width)):
            point_start_index = point[0] * pointcloud.row_step + point[1] * pointcloud.point_step
            point_data = pointcloud.data[point_start_index:point_start_index + 16]
            x = np.frombuffer(point_data[0:4], dtype=np.float32)[0]
            y = np.frombuffer(point_data[4:8], dtype=np.float32)[0]
            z = np.frombuffer(point_data[8:12], dtype=np.float32)[0]
            r = np.frombuffer(point_data[12:13], dtype=np.uint8)[0]
            g = np.frombuffer(point_data[13:14], dtype=np.uint8)[0]
            b = np.frombuffer(point_data[14:15], dtype=np.uint8)[0]
            points.append((timestamp, x, y, z, r, g, b))
        df = pd.DataFrame(points, columns=['time', 'x', 'y', 'z', 'r', 'g', 'b'])
        
        return df


    def storePointcloud(self):
        for index, topic in enumerate(self.list_of_topics):
            pointclouds, timestamps = self.deserializeDataIntoPointCloud(topic)
            dataframes = [self.pointcloud2ToDataFrame(pc, t) for pc, t in zip(pointclouds, timestamps)]
            df = pd.concat(dataframes)
            df.to_csv(f"data_export/pointcloud{index}_data.csv", index=False)
            df, dataframes, pointclouds, timestamps = None, None, None, None


    def getAll(self,sensors: list):
        for sensor in sensors:
            if sensor == 'rgb':
                try:
                    self.getTopics(sensor)
                    self.storeImages()
                    print(f"Data exported for RealSense {sensor} \n")
                except Exception as e:
                    print(f"Error: {e}")
                    print(f"No data found for RealSense {sensor}")
                    pass

            if sensor == 'pointcloud':
                try:
                    self.getTopics(sensor)
                    self.storePointcloud()
                    print(f"Data exported for RealSense {sensor} \n")
                except Exception as e:
                    print(f"Error: {e}")
                    print(f"No data found for RealSense {sensor}")
                    pass

            if sensor == 'accel' or sensor == 'gyro':
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