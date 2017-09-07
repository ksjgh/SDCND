
# Project : **Advanced Lane Finding**
---
<br>
<img src="./output_images/output_36_1.png" width="960" alt="Combined Image" />

## Find drivable lane area from road image
* Input : Video stream
* Output : Founded lane area video

#### Result Video Link - [Advanced Lane Finding](https://www.youtube.com/watch?v=gUkVRg-zaJU)<br>
---

## Overview

### 1. Objective
  * Find lane line from road image and apply this algorithm on video

### 2. Applied Techniques
* Camera calibration(distortion correction)
* Color transform on image (HLS,YUV,HSV...)
* Applying gradients on image (OpenCV Sobel Operator)
* Applying a perspective transform on image
* Binary thresholding on image
* Sliding windows to find lane line
* Polynomial fitting
* Python 3.5 , OpenCV

### 3. The goals / steps of this project
* Compute the camera calibration matrix and distortion coefficients given a set of chessboard images.
* Apply a distortion correction to raw images.
* Use color transforms, gradients, etc., to create a thresholded binary image.
* Apply a perspective transform to rectify binary image ("birds-eye view").
* Detect lane pixels and fit to find the lane boundary.
* Determine the curvature of the lane and vehicle position with respect to center.
* Warp the detected lane boundaries back onto the original image.
* Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.

### 4. Result

#### Result Video
[VIDEO : Lane Line Finding](https://www.youtube.com/watch?v=gUkVRg-zaJU)<br>

## Reflection

### 1. Brief Review
* My model has some problem in curve. I need to tweak threshold of gradient , color space and
  structure of binary lane extraction.
* My pipeline seems to have trouble in variation of light , rain or snow or scenery.
* I think cropping image in interested region is very useful if this combined with sensor fusion.<br>
  Measuring slop of road using gyro sensor and gravity sensor can help find cropping region.

### 2. Improvements
* I'll touch this project again using `semantic segmentation`.

---
## More Detailed Step-by-Step Write-up is [here](https://github.com/ksjgh/SDCND/blob/master/Term1_Computer_Vision_and_Deep_Learning/04_Project-Advanced_Lane_Finding/Writeup_project_Advanced_Lane_Finding_submit.md)<br>

---
## Code Impementation is [here](https://github.com/ksjgh/SDCND/blob/master/Term1_Computer_Vision_and_Deep_Learning/04_Project-Advanced_Lane_Finding/Advanced_Lane_Finding-submit.ipynb)
---
