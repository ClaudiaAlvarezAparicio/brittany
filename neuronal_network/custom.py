from __future__ import print_function
import keras
from keras.datasets import mnist
from keras.models import Sequential
from keras.layers import Dense, Dropout, Flatten
from keras.layers import Conv2D, MaxPooling2D
from keras.preprocessing.image import ImageDataGenerator
from keras import backend as K
import numpy as np
from keras.callbacks import ModelCheckpoint, CSVLogger
from keras.utils import multi_gpu_model
from sklearn.model_selection import train_test_split

batch_size = 128
num_classes = 5
epochs = 100

# input image dimensions
img_rows, img_cols = 256, 256

print("+++++++++++++++++++++++++++++++++++++++++++++++")
print("+++++++++++++++++ custom_x_y +++++++++++++++++")
print("+++++++++++++++++++++++++++++++++++++++++++++++")

data_raw = np.load('../concat_x_y/npy_total_train_raw.npy')
data_label = np.load('../concat_x_y/npy_total_train_label.npy')

X_train, X_test, y_train, y_test = train_test_split(
    data_raw,
    data_label,
    test_size=0.1,
    shuffle=True,
    random_state=42,
)

if K.image_data_format() == 'channels_first':
    X_train = X_train.reshape(X_train.shape[0], 1, img_rows, img_cols)
    X_test = X_test.reshape(X_test.shape[0], 1, img_rows, img_cols)
    input_shape = (1, img_rows, img_cols)
else:
    X_train = X_train.reshape(X_train.shape[0], img_rows, img_cols, 1)
    X_test = X_test.reshape(X_test.shape[0], img_rows, img_cols, 1)
    input_shape = (img_rows, img_cols, 1)

X_train = X_train.astype('float32')
X_test = X_test.astype('float32')

# convert class vectors to binary class matrices
y_train = keras.utils.to_categorical(y_train, num_classes)
y_test = keras.utils.to_categorical(y_test, num_classes)


## Create model
model = Sequential()
model.add(Conv2D(32, kernel_size=(9, 9),
                 activation='relu',
                 input_shape=input_shape))

model.add(Conv2D(64, (9, 9), activation='relu'))
model.add(MaxPooling2D(pool_size=(8, 8)))
model.add(Dropout(0.85))

model.add(Conv2D(64, (9, 9), activation='relu'))
model.add(Conv2D(64, (9, 9), activation='relu'))

model.add(MaxPooling2D(pool_size=(4, 4)))
model.add(Dropout(0.5))

model.add(Flatten())
model.add(Dense(128, activation='relu'))
model.add(Dropout(0.90))

model.add(Dense(num_classes, activation='softmax'))

model.compile(loss=keras.losses.categorical_crossentropy,
              optimizer=keras.optimizers.Adadelta(),
              metrics=['accuracy'])

model_checkpoint = ModelCheckpoint('../custom_x_y.h5', monitor='loss', save_best_only=True)
history_cb = CSVLogger('../custom_x_y.csv', separator=",", append=False)

# Training
hist=model.fit(X_train, y_train,
          batch_size=batch_size,
          epochs=epochs,
          verbose=1,
          validation_data=(X_test, y_test),
	  callbacks=[model_checkpoint, history_cb])

model.summary()


