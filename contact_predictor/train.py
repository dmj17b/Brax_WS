import pandas as pd
import numpy as np
import tensorflow as tf
from keras import regularizers

data = pd.read_csv('test_data.csv')


# Preprocess data - separate wheel contacts from the rest of data:
x = data.drop(columns=[col for col in data.columns if 'wheel_contact' in col]).values
y = data[[col for col in data.columns if 'wheel_contact' in col]].values


# Preprocess validation data:
validation_data = pd.read_csv('validation_data.csv')
x_val = validation_data.drop(columns=[col for col in validation_data.columns if 'wheel_contact' in col]).values
y_val = validation_data[[col for col in validation_data.columns if 'wheel_contact' in col]].values

print(x.shape, y.shape)

initializer = tf.keras.initializers.GlorotUniform()

model = tf.keras.Sequential()

# Create input layer based on input feature size (32 features for 16 joint states and 16 joint targets)
model.add(tf.keras.layers.InputLayer(input_shape=(x.shape[1],)))

# Normalize input layer:
model.add(tf.keras.layers.Normalization(axis=-1))

# Add hidden layers:
# model.add(tf.keras.layers.Dense(64,
#                                  activation='tanh',
#                                   kernel_initializer=initializer,
#                                   ))

model.add(tf.keras.layers.Dense(128,
                                 activation='tanh',
                                  kernel_initializer=initializer,
                                  ))

model.add(tf.keras.layers.Dense(256,
                                 activation='tanh',
                                  kernel_initializer=initializer,
                                  ))
model.add(tf.keras.layers.Dense(128,
                                 activation='tanh',
                                  kernel_initializer=initializer,
                                 ))
# model.add(tf.keras.layers.Dense(64,
#                                  activation='tanh',
#                                   kernel_initializer=initializer,
#                                   kernel_regularizer=regularizers.l2(1e-5)))

# Add output layer with 8 outputs (one for each wheel contact):
model.add(tf.keras.layers.Dense(y.shape[1], activation='sigmoid', kernel_initializer=initializer))

model.summary()

# Callback to stop training if training accuracy does not improve for 20 consecutive epochs
early_stopping = tf.keras.callbacks.EarlyStopping(monitor='accuracy', min_delta=0.005, patience=250, restore_best_weights=True)

# Callback to reduce learning rate if training accuracy does not improve for 15 consecutive epochs
reduce_lr = tf.keras.callbacks.ReduceLROnPlateau(monitor='accuracy', min_delta=0.005, factor=0.8, patience=30, min_lr=1e-6, restore_best_weights=True)

# Compile the model
optimizer = tf.keras.optimizers.Adam()
model.compile(optimizer=optimizer, loss='binary_crossentropy', metrics=['accuracy'])


history = model.fit(x, y, validation_data=(x_val, y_val), epochs=5000, batch_size=64, callbacks=[reduce_lr])

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