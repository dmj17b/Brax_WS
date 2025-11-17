import pandas as pd
import numpy as np
import tensorflow as tf
from keras import regularizers
import matplotlib.pyplot as plt


# Load datasets
random_data = pd.read_csv('random_data_noise.csv')
joystick_data = pd.read_csv('joystick_data_noise.csv')

print(f"Random data shape: {random_data.shape}")
print(f"Joystick data shape: {joystick_data.shape}")

# Check if column names match
random_cols = set(random_data.columns)
joystick_cols = set(joystick_data.columns)

print(f"\nColumns only in random_data: {random_cols - joystick_cols}")
print(f"Columns only in joystick_data: {joystick_cols - random_cols}")
print(f"Common columns: {len(random_cols & joystick_cols)}")

# Combine datasets
data = pd.concat([random_data, joystick_data], ignore_index=True)
print(f"\nCombined data shape: {data.shape}")  # This will show 124 columns if names don't match
