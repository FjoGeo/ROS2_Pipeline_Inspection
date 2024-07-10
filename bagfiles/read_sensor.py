import sqlite3
from rclpy.serialization import deserialize_message
from std_msgs.msg import Int32, Float32
import rclpy
import pandas as pd


class SensorReader:
    def __init__(self, dbPath, topicID):
        self.dbPath = dbPath
        self.conn = sqlite3.connect(dbPath)
        self.cursor = self.conn.cursor()
        self.rclpy = rclpy
        self.rclpy.init()
        self.dataframe = None
        self.topicID = topicID
        self.deserializeDataFrame()
        

    def getDataFrame(self):
        df = pd.read_sql_query(f"SELECT * FROM messages WHERE topic_id = {self.topicID}", self.conn)
        self.conn.close()
        return df


    def deserializeDataFrame(self):
        df = self.getDataFrame()
        df['deserialized_data'] = df['data'].apply(lambda x: deserialize_message(x, Float32))
        df['deserialized_data'] = df['deserialized_data'].apply(lambda msg: msg.data)
        self.dataframe = df[['timestamp', 'deserialized_data']]
        self.dataframe.columns = ['timestamp', 'data']
        self.rclpy.shutdown()


    def printDataFrame(self):
        print(self.dataframe.head())
        print(self.dataframe.tail())


if __name__ == "__main__":
    dbPath = "my_bag_1/my_bag_0.db3"
    topicID = 1
    sensor = SensorReader(dbPath, topicID)
    sensor.printDataFrame()
