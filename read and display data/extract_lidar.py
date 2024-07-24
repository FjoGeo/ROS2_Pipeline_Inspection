import sqlite3
from rclpy.serialization import deserialize_message
from std_msgs.msg import Int32, Float32
import rclpy
import pandas as pd


class IMUReader:
    def __init__(self, dbPath):
        self.dbPath = dbPath
        self.conn = sqlite3.connect(dbPath)
        self.cursor = self.conn.cursor()
        self.rclpy = rclpy
        self.rclpy.init()
        self.dfTopics = None
        self.dataframe = None
        self.getTopics()
        self.deserializeDataFrame()
        self.mergeDataFrames()
        self.pivotDataFrame()
        self.fillNaNValues()
        self.convertTimestamps()
        

    def getDataFrame(self):
        topic_ids = ','.join(map(str, self.dfTopics['topic_id'].tolist()))
        query = f"SELECT * FROM messages WHERE topic_id IN ({topic_ids})"
        df = pd.read_sql_query(query, self.conn)
        return df
    

    def deserializeDataFrame(self):
        df = self.getDataFrame()
        df['deserialized_data'] = df['data'].apply(lambda x: deserialize_message(x, Float32))
        df['deserialized_data'] = df['deserialized_data'].apply(lambda msg: msg.data)
        self.dataframe = df[['timestamp', 'topic_id','deserialized_data']]
        self.dataframe.columns = ['timestamp','topic_id', 'data']
        # print(self.dataframe.head(12))
        self.rclpy.shutdown()

    def getTopics(self):
        df = pd.read_sql_query("""SELECT 
                                        id, name, type 
                                    FROM topics 
                                        WHERE 
                                    name LIKE '%lidar%'
                                    """, self.conn)
        df['name'] = df['name'].str.rsplit('/').str[-1]
        df.columns = ['topic_id', 'topic_name', 'type']
        self.dfTopics = df
        # print(df.head(12))
        

    def mergeDataFrames(self):
        self.dataframe = pd.merge(self.dataframe, self.dfTopics, on='topic_id')
        self.conn.close()
    
    def pivotDataFrame(self):
        self.dataframe = self.dataframe.pivot_table(index='timestamp', columns='topic_name', values='data', aggfunc='first')
        self.dataframe.reset_index(inplace=True)

    def fillNaNValues(self):
        self.dataframe.ffill(inplace=True)
        self.dataframe.bfill(inplace=True)

    def convertTimestamps(self):
        self.dataframe['converted_timestamp'] = pd.to_datetime(self.dataframe['timestamp'], unit='ns')

    def printDataFrame(self):
        print(self.dataframe.head(12))

    def saveDataFrame(self, path):
        self.dataframe.to_csv(path, index=False)


if __name__ == "__main__":
    dbPath = "my_bag_1/my_bag_0.db3"
    imu = IMUReader(dbPath)
    # imu.printDataFrame()
    imu.saveDataFrame("lidar_data.csv")
