
# Project : **Extended Kalman Filter**
### Self-Driving Car Engineer Nanodegree
---
<br>
<img src="./report_data/EKF.png" width="480" alt="Combined Image" />

#### Result Video Link  [Extended Kalman Filter](https://youtu.be/AxB2kRkKnU4)<br>

 #### <span style="color:blue"> *Blue dots* </span> : Noisy Radar data , input to EKF

 #### <span style="color:red"> *Red dots* </span> : Noisy LiDAR data , input to EKF

 #### <span style="color:green"> *Green dots* </span> : Estimated position , output of EKF

---

## Overview

### 1. Goals
  * Utilize EKF(extended kalman filter) to estimate the state of a moving object with noisy lidar and radar measurements.<br>
  * Obtain low RMSE(Root Mean Square Error) values.<br>

  _Note :<br>
  The simulator provide program <br>
  1) Noisy Rada sensor data.<br>
  2) Noisy LiDAR sensor data.<br>_

  _Program estimate<br>
  1) current position : x, y<br>
  2) current velocity : vx, vy<br>
  3) RSME of x, y, vx, vy_<br>

### 2. Applied Techniques
* Linearization of Rada sensor data
* Sensor data coordinate transform
* Extended Kalman Filter
* C++ , Eigen Library(for vector, matrix manipulation)

### 3. Result
* RSME of x ,  y < 0.1[m]  
* RSME of vx, vy < 0.5[m/s]
