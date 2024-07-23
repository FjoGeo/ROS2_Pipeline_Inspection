import sqlite3
import rclpy
import pandas as pd



class DisplayMetaData:
    def __init__(self, dbPath):
        self.dbPath = dbPath
        self.conn = sqlite3.connect(dbPath)
        self.cur = self.conn.cursor()
        self.rclpy = rclpy
        self.rclpy.init()
        self.getAllTables()
        self.getAllTopics()
        self.conn.close()
        self.rclpy.shutdown()


    def getAllTables(self):
        self.cur.execute("SELECT name FROM sqlite_master WHERE type='table';")
        print("Tables in the database: ")
        print(self.cur.fetchall())


    def getAllTopics(self):
        print("\n\n")
        df = pd.read_sql_query("""SELECT 
                                        id, name, type 
                                    FROM topics 
                                        where 
                                    name LIKE '%HZ' OR name LIKE '%HY' OR name LIKE '%HX' 
                                    OR name LIKE '%AccX' OR name LIKE '%AccY' OR name LIKE '%AccZ'
                                    OR name LIKE '%AsY' OR name LIKE '%AsX' OR name LIKE '%AsZ'
                                    OR name LIKE '%AngX' OR name LIKE '%AngY' OR name LIKE '%AngZ'
                                    """, self.conn)

        df['name'] = df['name'].str.rsplit('/').str[-1]
        print(df)



if __name__ == "__main__":
    dbPath = "my_bag_1/my_bag_0.db3"
    display = DisplayMetaData(dbPath)
