import cv2
import os
import numpy as np
import batch_generate
import sklearn
import matplotlib.pyplot as plt
import matplotlib

## hyper-parameters
validation_size = 0.2
batch_size = 256
epochs = 4
use_saved_model = True ## use this if you want to load & training saved model&weight
# use_saved_model = False ## use this when first training

input_shape = (160,320,3)  ## original size
# input_shape = (64,64,3)  ## cropped & resized
saved_model_name = 'model.h5'

## Load log data from 'drive_log.csv' file
samples = batch_generate.read_csv_file()

## split up log data ,train and validation
from sklearn.model_selection import train_test_split
train_samples, validation_samples = train_test_split(samples, test_size=validation_size)

# Get data batches using generator
train_iterator = batch_generate.batch_generator(train_samples, batch_size)
validation_iterator = batch_generate.batch_generator(validation_samples, batch_size)


# ## debug
# train_images,train_angles = next(train_iterator)
# print('augmented angle = {}'.format(train_angles[1]))
# plt.imshow(train_images[1])
# plt.show()


# for image in train_images :

    # image = cv2.imread("test.jpg")
    # cv2.imshow("Image", image)
    # cv2.waitKey(0)
## debug end




## implement Neuralnetwork
from keras.models import Sequential , load_model
from keras.layers import Flatten, Dense, Activation, Lambda
from keras.layers import Convolution2D , MaxPooling2D ,Dropout
from keras.optimizers import Adam

model = Sequential()
model.add(Lambda(lambda x : (x/255.0)-0.5 , input_shape=input_shape))
############################################################################

## Convolutional and maxpooling layers
model.add(Convolution2D(24,5,5,border_mode='valid', activation='relu', subsample=(2,2)))
model.add(Convolution2D(36,5,5,border_mode='valid', activation='relu', subsample=(2,2)))
model.add(Convolution2D(48,5,5,border_mode='valid', activation='relu', subsample=(2,2)))
# model.add(Dropout(0.5))
model.add(Convolution2D(64,3,3,border_mode='valid', activation='relu', subsample=(1,1)))
# model.add(Dropout(0.5))
model.add(Convolution2D(64,3,3,border_mode='valid', activation='relu', subsample=(1,1)))
# model.add(Dropout(0.5))
model.add(Flatten())
model.add(Dense(1164, activation='relu'))
model.add(Dropout(0.0))
model.add(Dense(100, activation='relu'))
model.add(Dropout(0.0))
model.add(Dense(50, activation='relu'))
model.add(Dropout(0.0))
model.add(Dense(10, activation='relu'))
model.add(Dense(1))

######################################################################
model.summary()

### Run model

if use_saved_model == True :
    del model  # deletes the existing model

    ## returns a compiled model
    ## identical to the previous one
    print()
    print("----- Model Load = " + saved_model_name)
    model = load_model(saved_model_name)

print()
print("----- Trainning Start ----- ")
adam = Adam(lr = 0.0001)
model.compile(optimizer= adam, loss='mse', metrics=['accuracy'])
history =   model.fit_generator(train_iterator,
                                samples_per_epoch = len(train_samples) ,
                                validation_data = validation_iterator,
                                nb_val_samples = len(validation_samples),
                                nb_epoch= epochs,
                                verbose = 1)

model.save(saved_model_name)
print()
print ("(----- Training Done. Model saved as = " + saved_model_name)
