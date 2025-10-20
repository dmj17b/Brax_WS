import pandas as pd
import numpy as np
import tensorflow as tf
from keras import regularizers

data = pd.read_csv('simulation_data.csv')


# Preprocess data - separate wheel contacts from the rest of data:
x = data.drop(columns=[col for col in data.columns if 'wheel_contact' in col]).values
y = data[[col for col in data.columns if 'wheel_contact' in col]].values

print(x.shape, y.shape)

initializer = tf.keras.initializers.HeNormal()

model = tf.keras.Sequential()

# Create input layer based on input feature size (32 features for 16 joint states and 16 joint targets)
model.add(tf.keras.layers.InputLayer(input_shape=(x.shape[1],)))

# Normalize input layer:
model.add(tf.keras.layers.Normalization(axis=-1))

# Add hidden layers:
model.add(tf.keras.layers.Dense(128, activation='relu', kernel_initializer = initializer))
model.add(tf.keras.layers.Dense(128, activation='relu', kernel_initializer = initializer))
model.add(tf.keras.layers.Dense(128, activation='relu', kernel_initializer = initializer))
model.add(tf.keras.layers.Dense(128, activation='relu', kernel_initializer = initializer))
model.add(tf.keras.layers.Dense(64, activation='relu', kernel_initializer = initializer))

# Add output layer with 8 outputs (one for each wheel contact):
model.add(tf.keras.layers.Dense(y.shape[1], activation='sigmoid', kernel_initializer = initializer))

model.summary()

# Compile the model
optimizer = tf.keras.optimizers.Adam()
model.compile(optimizer=optimizer, loss='binary_crossentropy', metrics=['accuracy'])


history = model.fit(x, y, epochs=10000, batch_size=1024)

model.save('contact_predictor_model.keras')

# Plot training accuracy and loss
import matplotlib.pyplot as plt

plt.plot(history.history['accuracy'], label='accuracy')
plt.plot(history.history['loss'], label='loss')
plt.xlabel('Epoch')
plt.ylabel('Value')
plt.title('Training Accuracy and Loss')
plt.legend()
plt.show()