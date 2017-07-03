import cv2
import csv
import numpy as np
import sklearn

## data DIR setting
# data_DIR = './data_sample/'
data_DIR = './data/'
IMG_DIR = 'IMG/'
log_file_name = 'driving_log.csv'

## Load saved  log data from 'csv' file
def read_csv_file() :
    samples = []
    with open(data_DIR + log_file_name) as csvfile:
        reader = csv.reader(csvfile)
        for line in reader:
            samples.append(line)
        del samples[0]

    return samples

### random BGR to RGB Convert
def cvt_BGR2RGB(image):

    import random

    prob_thresh = 0.5
    prob = random.random()

    if prob > prob_thresh :
        aug_image = cv2.cvtColor(image , cv2.COLOR_BGR2RGB )
        return  aug_image
    else :
        return image

### Crop & Resize
def crop_resize(image):
    cropped = cv2.resize(image[60:140,:], (64,64))
    return cropped

### Flip vertical
def random_flip_vertical(image , angle):

    import random

    prob_thresh = 0.5
    prob = random.random()

    if prob > prob_thresh :
        aug_image = cv2.flip(image,1)
        aug_angle = angle*(-1.0)
        return aug_image, aug_angle
    else :
        return image , angle


### Flip Horizontal
def random_flip_horizontal(image , angle):

    import random

    prob_thresh = 0.5
    prob = random.random()

    if prob > prob_thresh :
        aug_image = cv2.flip(image,0)
        aug_angle = angle*(1.0)
        return aug_image, aug_angle
    else :
        return image , angle

## delete upper region of image to remove unnecessary scenery
def random_delete_upper(image):

    import random

    prob_thresh = 0.5
    prob = random.random()

    if prob > prob_thresh :
        aug_img = np.copy(image)
        aug_img[0:50,:] = 255
        return aug_img
    else :
        return image

## randomly change brightness of image
def random_brightness(image):
    import random

    prob_thresh = 0
    prob = random.random()

    if prob > prob_thresh :
        #Convert 2 HSV colorspace from RGB colorspace
        hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

        #Generate new random brightness
        rand = random.uniform(0.3,1.0)
        hsv[:,:,2] = rand*hsv[:,:,2]

        #Convert back to RGB colorspace
        new_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)
        return new_img
    else :
        return image

## randomly augment input image
def random_augment(image , angle):
    augmented_image = np.copy(image)
    augmented_angle = angle

    ## image augment pipeline
    ## randomly convert BGR2RGB
    augmented_image = cvt_BGR2RGB(augmented_image)

    ## randomly delele upper side
    augmented_image = random_delete_upper(augmented_image)

    ## randomly flip vertically
    augmented_image , augmented_angle = random_flip_vertical(augmented_image , augmented_angle)

    ## randomly flip horizontally
    augmented_image , augmented_angle = random_flip_horizontal(augmented_image , augmented_angle)

    ## randomly change brightness
    augmented_image = random_brightness(augmented_image)

    return augmented_image, augmented_angle

## generate training & validation batch data
def batch_generator(samples, batch_size):
    num_samples = len(samples)
    while True: # Loop forever untill the generator is exhausted
        sklearn.utils.shuffle(samples)
        # print('     progress ={} % '.format(offset/num_samples*100.0))
        for offset in range(0, num_samples, batch_size):
            batch_start = offset
            batch_end = offset+batch_size
            batch_samples = samples[batch_start:batch_end]

            # if batch_end < num_samples :
            #     batch_samples = samples[batch_start:batch_end]
            # else :
            #     batch_samples = samples[batch_start:]

            batch_images = []
            batch_angles = []
            for batch_sample in batch_samples:
                img_name = data_DIR + IMG_DIR + batch_sample[0].split('\\')[-1]
                center_image = cv2.imread(img_name)
                center_angle = float(batch_sample[3])
                augmented_image , augmented_angle = random_augment(center_image,center_angle)
                batch_images.append(augmented_image)
                batch_angles.append(augmented_angle)

            ## covert to numpy array
            batch_X_train = np.array(batch_images)
            batch_y_train = np.array(batch_angles)
            yield sklearn.utils.shuffle(batch_X_train, batch_y_train)
