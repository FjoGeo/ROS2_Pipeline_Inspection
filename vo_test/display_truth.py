import pandas as pd


df = pd.read_csv(r'./data/Ground truth distances.csv', header=None)
df = df.set_index(0).T 
df.columns = df.iloc[0]
df = df[1:]
df = df.set_index("Frame")
df.index = df.index.astype(int)
df = df.reset_index(drop=True)


# cumsum
df_sum = df.cumsum()


# to csv
df.to_csv(r'./data/distance.csv')
df_sum.to_csv(r'./data/distance_sum.csv')