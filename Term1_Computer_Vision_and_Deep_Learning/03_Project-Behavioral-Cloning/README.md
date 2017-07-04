
# Project : **Behavioral Cloning**
### Self-Driving Car Engineer Nanodegree
---
<br>
<img src="./center_sample.JPG" width="480" alt="Combined Image" />
</br>

### Predicted Steering Angle  : +0.03 [rad]
---

## Overview

### 1. Objective
  * Let the car learn how to drive the track watching how human drivers
  * After learning, car simply sees the image and decide the steering angle of handle to drive inside the track
  <br><br>
  _Note : This project was done in simulator provided by Udacity._

### 2. Applied Techniques
* Keras/Tensorflow for building neural network
* Deep Learning Network(CNN , FCN)
* Image Augmentation for Training Data
* Using `Batch Generator` to deal with large size of training data

### 3. The goals / steps of this project
* Use the simulator to collect data of good driving behavior
* Build, a deep learning network in Keras that predicts steering angles from images
* Train and validate the model with a training and validation set
* Test that the model successfully drives around track one without leaving the road

### 4. Result

#### Result Video
[VIDEO : Car dirves automatically aroud the track](https://youtu.be/-jkfagvO4VQ)<br>
_Note : There is some dangerous situation when the car try to turn a curve_

#### Final Model Architecture
The overall strategy for deriving a model architecture was to use NVIDIA Autopilot model for starting point and training network with augmented data.


|Layer (type)                     |Output Shape          |Param #     |Connected to                 |    
|---------------------------------|----------------------|------------|-----------------------------|
|lambda_1 (Lambda)                |(None, 160, 320, 3)   |0           |lambda_input_1[0][0]            |
|convolution2d_1 (Convolution2D)  |(None, 78, 158, 24)   |1824       | lambda_1[0][0]                  |
|convolution2d_2 (Convolution2D)  |(None, 37, 77, 36)    |21636      | convolution2d_1[0][0]     |       
|convolution2d_3 (Convolution2D)  |(None, 17, 37, 48)    |43248       |convolution2d_2[0][0]      |      
|convolution2d_4 (Convolution2D)  |(None, 15, 35, 64)    |27712       |convolution2d_3[0][0]      |      
|convolution2d_5 (Convolution2D)  |(None, 13, 33, 64)    |36928       |convolution2d_4[0][0]      |      
|flatten_1 (Flatten)              |(None, 27456)         |0           |convolution2d_5[0][0]      |      
|dense_1 (Dense)                  |(None, 1164)          |31959948    |flatten_1[0][0]            |      
|dropout_1 (Dropout)              |(None, 1164)          |0           |dense_1[0][0]              |      
|dense_2 (Dense)                  |(None, 100)           |116500      |dropout_1[0][0]            |      
|dropout_2 (Dropout)              |(None, 100)           |0           |dense_2[0][0]              |      
|dense_3 (Dense)                  |(None, 50)            |5050        |dropout_2[0][0]            |      
|dropout_3 (Dropout)              |(None, 50)            |0           |dense_3[0][0]              |      
|dense_4 (Dense)                  |(None, 10)            |510         |dropout_3[0][0]            |      
|dense_5 (Dense)                  |(None, 1)             |11          |dense_4[0][0]              |
---

## Reflection

### 1. Result Analyze
* It's basically linear regression. Car get an image and match it to steering angle.
* This model need to be traind again if track is changed.
* I've trained model using image augmentation (e.g : random darkness)
  But I didn't tested it under variation of brightness.
* I hope to check if there is a noise in input image (e.g. : snow falling) car could drive well.

### 2. Improvements
* For human, driving need history of a few seconds ago. This project used CNN & FCN.But I think it's not so adequate in this point of view.<br>
  I'll touch this project again using RNN/LSTM architecture.

---
## For More Detaied Step-by-Step write-up is [here](https://github.com/ksjgh/SDCND/blob/master/Term1_Computer_Vision_and_Deep_Learning/03_Project-Behavioral-Cloning/writeup_Behavioral_Cloning.md)<br>

## Code Impementation is [here](https://github.com/ksjgh/SDCND/blob/master/Term1_Computer_Vision_and_Deep_Learning/03_Project-Behavioral-Cloning/model.py)
