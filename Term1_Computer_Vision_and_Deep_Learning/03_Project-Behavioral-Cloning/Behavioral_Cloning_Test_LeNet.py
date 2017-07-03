import csv
import cv2
import numpy as np
import tensorflow as tf

## Load data from 'csv' file
lines =[]
with open('./data_sample/driving_log.csv') as csvfile:
    reader = csv.reader(csvfile)
    for line in reader :
        lines.append(line)

## split data X_train : images , y_train : steering value at that moment
images =[]
measurements = []
data_dir = './data_sample/IMG/'
for line in lines[1:] :
# for line in lines[1:6] :
    source_path = line[0]
    filename = source_path.split('/')[-1]
    current_path = data_dir + filename
#     print(current_path)
    image = cv2.imread(current_path)
    images.append(image)
    measurement = float(line[3])
#     print(measurement)
    measurements.append(measurement)

## augment image(flip)
augmented_images, augmented_measurements = [] , []
for image , measurement in zip(images,measurements):
    augmented_images.append(image)
    augmented_measurements.append(measurement)
    augmented_images.append(cv2.flip(image,1))  ## cv2.flip : http://docs.opencv.org/2.4/modules/core/doc/operations_on_arrays.html#flip
    augmented_measurements.append(-measurement)

X_train = np.array(images)
y_train = np.array(measurements)

# import matplotlib.pyplot as plt
# # Visualizations will be shown in the notebook.
# # %matplotlib inline
# print(X_train[0])
# plt.imshow(X_train[0])

## implement CNN
from keras.models import Sequential
from keras.layers import Flatten, Dense, Activation, Lambda
from keras.layers import Convolution2D , MaxPooling2D ,Dropout
from keras import backend as K

input_shape = X_train[0].shape
model = Sequential()
# model.add(Lambda(lambda img : tf.image.resize_images(img ,resized_shape),
#                 input_shape=input_shape))
## Note : above code gives error , NameError: name 'tf' is not defined
## when executed as 'python drive.py xxx.h5' , presume Keras problem

model.add(Lambda(lambda x : (x/255.0)-0.5 , input_shape=input_shape))

model.add(Convolution2D(6,5,5,activation ='relu'))
model.add(MaxPooling2D())

model.add(Convolution2D(6,5,5,activation ='relu'))
model.add(MaxPooling2D())

# model.add(Dropout(0.25))

# model.add(Conv2D(64, (3, 3), padding='same'))
# model.add(Activation('relu'))
# model.add(Conv2D(64, (3, 3)))
# model.add(Activation('relu'))
# model.add(MaxPooling2D(pool_size=(2, 2)))
# model.add(Dropout(0.25))

# model.add(Activation('relu'))
# model.add(Dropout(0.5))
# model.add(Dense(1))
# model.add(Activation('softmax'))

model.add(Flatten())
model.add(Dense(120))
model.add(Dense(84))
model.add(Dense(1))
#######################################################################

model.compile(loss = 'mse', optimizer = 'adam')
model.fit(X_train, y_train, validation_split=0.2, shuffle=True, nb_epoch=3)

model.save('model_LeNet.h5')
