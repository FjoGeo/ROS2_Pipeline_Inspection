import sqlite3
import numpy as np
import pandas as pd
from sensor_msgs.msg import PointCloud2
from rclpy.serialization import deserialize_message
import rclpy


class ReadPC:
    def __init__(self, db_path):
        self.db_path = db_path
        self.conn = sqlite3.connect(db_path)
        self.cursor = self.conn.cursor()
        self.rclpy = rclpy
        self.rclpy.init()
        self.df = None
        self.getDataFrames()


    @staticmethod
    def deserializePointcloud2(data):
        msg = deserialize_message(data, PointCloud2)
        return msg


    def getDataAndTime(self):
        self.cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id=17 LIMIT 3")
        rows = self.cursor.fetchall()
        self.conn.close()
        timestamps = [row[0] for row in rows]
        data = [row[1] for row in rows]

        return timestamps, data


    def deserializeDataIntoPointCloud(self):
        timestamps, data = self.getDataAndTime()
        pointClouds = [self.deserializePointcloud2(d) for d in data]

        return pointClouds, timestamps


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
    

    def getDataFrames(self):
        pointClouds, timestamps = self.deserializeDataIntoPointCloud()
        dataframes = [self.pointcloud2ToDataFrame(pc, t) for pc, t in zip(pointClouds, timestamps)]
        self.df = pd.concat(dataframes)
        self.rclpy.shutdown()
    

    def printDataframes(self):
        print(self.df.head())
        print(self.df.tail())



if __name__ == "__main__":
    db_path = "my_bag_1/my_bag_0.db3"
    read_pc = ReadPC(db_path)
    read_pc.printDataframes()
