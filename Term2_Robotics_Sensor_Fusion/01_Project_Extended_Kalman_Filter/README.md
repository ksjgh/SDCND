
# Project : **Extended Kalman Filter**
---
<img src="./report_data/EKF.PNG" width="960" alt="Combined Image" />

## Estimate vehicle's position(state) using EKF
![#f03c15](https://placehold.it/15/f03c15/000000?text=+) Red dots : Noisy LiDAR data , input to EKF<br>
![#1589F0](https://placehold.it/15/1589F0/000000?text=+) Blue dots : Noisy Radar data , input to EKF<br>
![#c5f015](https://placehold.it/15/c5f015/000000?text=+) Green triangles : Estimated position , output of EKF

#### Result Video Link - [Extended Kalman Filter](https://youtu.be/AxB2kRkKnU4)<br>
---

## Overview

### 1. Goals
  * Utilize EKF(extended kalman filter) to estimate the state of a moving object with noisy lidar and radar measurements.<br>
  * Obtain low RMSE(Root Mean Square Error) values.<br>


  *Note :<br>
  The simulator provide program <br>
  1) Noisy Rada sensor data.<br>
  2) Noisy LiDAR sensor data.*<br>

  *Program estimate<br>
  1) current position : x, y<br>
  2) current velocity : vx, vy<br>
  3) RSME of x, y, vx, vy*<br>

### 2. Applied Techniques
* Linearization of Radar sensor data for kalman fiter
* Sensor data coordinate transform
* C++ , Eigen Library(for vector, matrix manipulation)

### 3. Result
* RSME of x ,  y < 0.1[m]  
* RSME of vx, vy < 0.5[m/s]
