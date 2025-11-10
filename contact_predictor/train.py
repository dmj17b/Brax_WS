import pandas as pd
import numpy as np
import tensorflow as tf
from keras import regularizers
import matplotlib.pyplot as plt


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
normalizer = tf.keras.layers.Normalization(axis=-1)
normalizer.adapt(x)  # This computes mean and std from your training data
model.add(normalizer)


# Add hidden layers:

# Layer 1
model.add(tf.keras.layers.Dense(64,
                                  kernel_initializer=initializer,
                                  ))
model.add(tf.keras.layers.BatchNormalization())
model.add(tf.keras.layers.ReLU())

# Layer 2
model.add(tf.keras.layers.Dense(128,
                                  kernel_initializer=initializer,
                                  ))
model.add(tf.keras.layers.BatchNormalization())
model.add(tf.keras.layers.ReLU())
model.add(tf.keras.layers.Dropout(0.2))

model.add(tf.keras.layers.Dense(256,
                                  kernel_initializer=initializer,
                                  ))
model.add(tf.keras.layers.BatchNormalization())
model.add(tf.keras.layers.ReLU())
model.add(tf.keras.layers.Dropout(0.2))

# Layer 3
model.add(tf.keras.layers.Dense(128,
                                  kernel_initializer=initializer,
                                  ))
model.add(tf.keras.layers.BatchNormalization())
model.add(tf.keras.layers.ReLU())
model.add(tf.keras.layers.Dropout(0.2))

# Layer 4
model.add(tf.keras.layers.Dense(64,
                                  kernel_initializer=initializer,
                                 )) 
model.add(tf.keras.layers.BatchNormalization())
model.add(tf.keras.layers.ReLU())


# Add output layer with 8 outputs (one for each wheel contact):
model.add(tf.keras.layers.Dense(y.shape[1], kernel_initializer=initializer))

model.summary()


# Callback to reduce learning rate if training accuracy does not improve
reduce_lr = tf.keras.callbacks.ReduceLROnPlateau(monitor='val_loss', min_delta=0.005, factor=0.8, patience=10, min_lr=1e-7, restore_best_weights=True)

# Callback to save the best model during training
best_model = tf.keras.callbacks.ModelCheckpoint('best_contact_predictor_model.keras', monitor='val_loss', save_best_only=True)

# Callback to stop training early if validation loss does not improve
early_stopping = tf.keras.callbacks.EarlyStopping(monitor='val_loss', min_delta=0.01, patience=60, restore_best_weights=True)

# Optimizer
optimizer = tf.keras.optimizers.Adam(learning_rate=0.001)

# M
binary_accuracy_metric = tf.keras.metrics.BinaryAccuracy(threshold=0.5, name='binary_acc')
pr_auc_metric = tf.keras.metrics.AUC(curve='PR', name='pr_auc', multi_label=True)


loss_fn = tf.keras.losses.BinaryCrossentropy(from_logits = True, label_smoothing=0.05)

# Compile the model
model.compile(optimizer=optimizer, loss=loss_fn, metrics=['accuracy', binary_accuracy_metric, pr_auc_metric])

# Train the model
history = model.fit(x, y, validation_data=(x_val, y_val), epochs=500, batch_size=64, callbacks=[reduce_lr, best_model, early_stopping], shuffle=True)

model.save('contact_predictor_model.keras')

# Plot training accuracy and loss
plt.plot(history.history['binary_acc'], label='Binary Accuracy')
plt.plot(history.history['loss'], label='loss')
plt.plot(history.history['val_binary_acc'], label='Val_Binary_Accuracy')
plt.plot(history.history['val_loss'], label='val_loss')
plt.xlabel('Epoch')
plt.ylabel('Value')
plt.title('Training Accuracy and Loss')
plt.legend()
plt.show()