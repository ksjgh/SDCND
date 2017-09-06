
# Project : **Unscented Kalman Filter**
### Self-Driving Car Engineer Nanodegree
---
<img src="./report_data/UKF.PNG" width="480" alt="Combined Image" />

#### Result Video Link  [Unscented Kalman Filter](https://youtu.be/Rwtg0_52t1c)<br>

![#f03c15](https://placehold.it/15/f03c15/000000?text=+) Red dots : Noisy LiDAR data , input to UKF<br>
![#1589F0](https://placehold.it/15/1589F0/000000?text=+) Blue dots : Noisy Radar data , input to UKF<br>
![#c5f015](https://placehold.it/15/c5f015/000000?text=+) Green dots : Estimated position , output of UKF<br>

---

## Overview

### 1. Goals
  * Utilize UKF(Unscented kalman filter) to estimate the state of a moving object with noisy lidar and radar measurements.<br>
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
* Motion Models : constant turn rate and velocity magnitude model (CTRV) of car is used
* Unscented Transformation & Extended Kalman Filter
* C++ , Eigen Library(for vector, matrix manipulation)

### 3. Result
* Improved compared to using EKF
* RSME of x ,  y < 0.09[m]  
* RSME of vx, vy < 0.4[m/s]

_Note: EKF Result <br>
RSME of x ,  y < 0.1[m]  
RSME of vx, vy < 0.5[m/s]_
