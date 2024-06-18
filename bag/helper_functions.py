import sqlite3
import pandas as pd
import ast
import re


# Function to extract and convert the dictionary part of the byte string
def extract_and_convert(byte_str:bytes):
    # Decode the byte string
    decoded_str = byte_str.decode('latin-1')
    
    # Extract the dictionary part using a regular expression
    match = re.search(r"\{.*\}", decoded_str)
    if match:
        dict_str = match.group(0)
        # Convert the string representation of the dictionary to an actual dictionary
        return ast.literal_eval(dict_str)
    else:
        return None
    

def transform_imu_data(byte_str:bytes):

    decoded_str = byte_str.decode('latin-1')
    cleaned_string = ''.join(c for c in decoded_str if c.isprintable())
    numeric_values = re.findall(r'[-+]?\d*\.\d+|\d+', cleaned_string)
    numeric_values = [int(num) if num.isdigit() else float(num) for num in numeric_values]

    return numeric_values # quality, angle, distance

    

def create_connection(directory:str):
    conn = sqlite3.connect(directory)
    cur = conn.cursor() 
    
    return conn, cur


def extract_imu_from_db(conn, topic_id:int):
    df = pd.read_sql_query(f"SELECT * FROM messages WHERE topic_id = {topic_id}", conn)
    df["data_dict"] = df["data"].apply(extract_and_convert)
    df_expanded = pd.json_normalize(df['data_dict'].dropna())
    df_final = df.drop(columns=['data', 'data_dict']).join(df_expanded)

    return df_final


def extract_lidar_from_db(conn, topic_id:int):
    df = pd.read_sql_query(f"SELECT * FROM messages WHERE topic_id = {topic_id}", conn)
    df["data_dict"] = df["data"].apply(transform_imu_data)
    df[["quality", "angle", "distance"]] = pd.DataFrame(df["data_dict"].tolist(), index=df.index)
    df.drop(columns=["data_dict", "data"], inplace=True)

    return df
    
