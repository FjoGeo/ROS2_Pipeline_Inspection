import sqlite3
import pandas as pd
from helper_functions import create_connection, extract_imu_from_db, extract_lidar_from_db

database_directory = 'bagfiles\my_bag_0.db3'
conn, cur = create_connection(database_directory)

# # Get the list of tables in the database
# cur.execute("SELECT name FROM sqlite_master WHERE type='table';")
# allTables = cur.fetchall()
# for table in allTables:
#     print(table[0])


# # display column names
# cur.execute("PRAGMA table_info(messages)")
# allColumns = cur.fetchall()
# for column in allColumns:
#     print(column)


# convert IMU signals to pandas dataframe
# read IMU
imu_topic_id = 1
df_final = extract_imu_from_db(conn, imu_topic_id)
print(df_final.head())

# read LiDAR
imu_topic_id = 2
df_final = extract_lidar_from_db(conn, imu_topic_id)
print(df_final.head())


conn.close()