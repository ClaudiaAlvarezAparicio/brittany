from __future__ import print_function
import keras
from keras.datasets import mnist
from keras.models import Sequential
from keras.layers import Dense, Dropout, Flatten
from keras.layers import Conv2D, MaxPooling2D
from keras.preprocessing.image import ImageDataGenerator
from keras import backend as K
import numpy as np
from keras.callbacks import ModelCheckpoint
from keras.utils import multi_gpu_model

batch_size = 128
num_classes = 5
epochs = 100

# input image dimensions
img_rows, img_cols = 256, 256

# Load data
x_train = np.load('./npy_total_raw_train.npy')
y_train = np.load('./npy_total_label_train.npy')

x_test = np.load('./npy_total_raw_test.npy')
y_test = np.load('./npy_total_label_test.npy')



if K.image_data_format() == 'channels_first':
    x_train = x_train.reshape(x_train.shape[0], 1, img_rows, img_cols)
    x_test = x_test.reshape(x_test.shape[0], 1, img_rows, img_cols)
    input_shape = (1, img_rows, img_cols)
else:
    x_train = x_train.reshape(x_train.shape[0], img_rows, img_cols, 1)
    x_test = x_test.reshape(x_test.shape[0], img_rows, img_cols, 1)
    input_shape = (img_rows, img_cols, 1)

x_train = x_train.astype('float32')
x_test = x_test.astype('float32')


# convert class vectors to binary class matrices
y_train = keras.utils.to_categorical(y_train, num_classes)
y_test = keras.utils.to_categorical(y_test, num_classes)


# Layer definition
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


parallel_model = model
parallel_model.compile(loss=keras.losses.categorical_crossentropy,
              optimizer=keras.optimizers.Adadelta(),
              metrics=['accuracy'])

model_checkpoint = ModelCheckpoint('./model.h5', monitor='loss', save_best_only=True)

# Training
parallel_model.fit(x_train, y_train,
          batch_size=batch_size,
          epochs=epochs,
          verbose=1,
          validation_data=(x_test, y_test),
	  callbacks=[model_checkpoint])


parallel_model.load_weights('./model.h5')

# Evaluation
score = parallel_model.evaluate(x_test, y_test, verbose=0)
print('Test loss:', score[0])
print('Test accuracy:', score[1])

