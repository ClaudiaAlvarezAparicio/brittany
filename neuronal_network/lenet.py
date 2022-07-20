import keras,os
from keras.models import Sequential
from keras.layers import Dense, Conv2D, MaxPool2D , Flatten, MaxPooling2D
from keras.preprocessing.image import ImageDataGenerator
import numpy as np
from keras.optimizers import Adam
from keras.callbacks import ModelCheckpoint, EarlyStopping, CSVLogger

# Mios
from sklearn.model_selection import train_test_split
from keras import backend as K

print("+++++++++++++++++++++++++++++++++++++++++++++++")
print("+++++++++++++++++ lenet_x_y +++++++++++++++++")
print("+++++++++++++++++++++++++++++++++++++++++++++++")

batch_size = 32
num_classes = 5
epochs = 10

# input image dimensions
img_rows, img_cols = 256, 256

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


model = Sequential()
#Layer 1
#Conv Layer 1
model.add(Conv2D(filters = 6, 
                 kernel_size = 5, 
                 strides = 1, 
                 activation = 'relu', 
                 input_shape = input_shape))
#Pooling layer 1
model.add(MaxPooling2D(pool_size = 2, strides = 2))
#Layer 2
#Conv Layer 2
model.add(Conv2D(filters = 16, 
                 kernel_size = 5,
                 strides = 1,
                 activation = 'relu',
                 input_shape = (14,14,6)))
#Pooling Layer 2
model.add(MaxPooling2D(pool_size = 2, strides = 2))
#Flatten
model.add(Flatten())
#Layer 3
#Fully connected layer 1
model.add(Dense(units = 120, activation = 'relu'))
#Layer 4
#Fully connected layer 2
model.add(Dense(units = 84, activation = 'relu'))
#Layer 5
#Output Layer
model.add(Dense(units = num_classes, activation = 'softmax'))
model.compile(optimizer = 'adam', loss = 'categorical_crossentropy', metrics = ['accuracy'])


checkpoint = ModelCheckpoint("../lenet_x_y.h5",  monitor='loss', save_best_only=True)
history_cb = CSVLogger('../lenet_x_y.csv', separator=",", append=False)



hist = model.fit(X_train, y_train,
          batch_size=batch_size,
          epochs=epochs,
          verbose=1,
          validation_data=(X_test, y_test),
	  callbacks=[checkpoint, history_cb])

model.summary()