import pandas as pd

CSV_PATH = "robot_recording_with_predictions.csv"

df = pd.read_csv(CSV_PATH, sep=";")

# Filter rows where either 'direction' or 'yaw_angle' is present (not NaN/empty)
has_direction = df["direction"].notna() & (df["direction"].astype(str).str.strip() != "")
has_yaw = df["yaw_angle"].notna()

filtered_df = df[has_direction | has_yaw]

print(filtered_df.head())