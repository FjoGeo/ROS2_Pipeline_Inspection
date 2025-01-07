import sqlite3
import rclpy
import pandas as pd
from rclpy.serialization import deserialize_message
from std_msgs.msg import Float32MultiArray
import os


class DisplayMetaData:
    def __init__(self, dbPath):
        self.dbPath = dbPath
        self.conn = sqlite3.connect(dbPath)
        self.cur = self.conn.cursor()
        self.rclpy = rclpy
        self.rclpy.init()
        # self.getAllTables()
        # self.getAllTopics()
        self.getRealSenseAcc()
        self.conn.close()
        self.rclpy.shutdown()


    # remove later or comment out
    def getAllTables(self):
        self.cur.execute("SELECT name FROM sqlite_master WHERE type='table';")
        print("Tables in the database: ")
        print(f"{self.cur.fetchall()} \n")


    def getAllTopics(self):
        df = pd.read_sql_query("""
                               SELECT id, name, type FROM topics 
                                    """, self.conn)

        print(f"All recorded topics: \n {df} \n")
    

    def getRealSenseAcc(self):

        try:
            # get topics
            df = pd.read_sql_query("""SELECT
                                    id
                                FROM topics
                                    WHERE
                                name LIKE '%realsense1/accel%' OR name LIKE '%realsense2/accel%'
                                """, self.conn)
            
            list_of_topics = df['id'].tolist()

            # get data
            df = pd.read_sql_query(f"""SELECT * FROM messages 
                                WHERE topic_id IN ({','.join(map(str, list_of_topics))})""", 
                                self.conn)

            df['deserialized_data'] = df['data'].apply(lambda x: deserialize_message(x, Float32MultiArray))
            df['deserialized_data'] = df['deserialized_data'].apply(lambda msg: msg.data)
            df['converted_timestamp'] = pd.to_datetime(df['timestamp'], unit='ns')
            df[['X', 'Y', 'Z']] = pd.DataFrame(df['deserialized_data'].tolist(), index=df.index)
            df.drop(['data', 'topic_id', 'id', 'deserialized_data'], axis=1, inplace=True)

            # create a folder to store the data
            folder = "data_export"
            try:
                os.mkdir(folder)
            except FileExistsError:
                pass

            # save the data to a csv file
            df.to_csv(f"{folder}/realsense_accel_data.csv", index=False)

        except Exception as e:
            print(f"Error: {e}")
            print("No data found for RealSense Accel")
            pass



if __name__ == "__main__":
    dbPath = "my_bag/bagfile.db3"
    display = DisplayMetaData(dbPath)