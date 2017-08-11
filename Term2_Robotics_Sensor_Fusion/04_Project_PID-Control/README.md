
# Project : **PID Control**
### Self-Driving Car Engineer Nanodegree
---
<br>
<img src="PID_Control.jpg" width="480" alt="Combined Image" />

---

## Overview

### 1. Objective
  * Keep the car in the center of the track using PID control

### 2. Applied Techniques
* C++
* PID Controller design an gain tuning.

### 3. The goals / steps of this project
* Implement PID controller using C++
* Tune the gain of controller by manual , SGD or other method to keep the car in the center of the track.

### 4. Result

#### Result Video
[VIDEO : PID control of the car](https://youtu.be/2lOEfyVJfe4)<br>

## Reflection

### 1. PID Gain tuning method
* I tuned the gain  manually
* Kp : proportional gain<br>
  Ki : integration gain<br>
  Kd : derivertive gain<br>

#### 1) Tune the gain in the straight course
* Kp = -0.2 , Ki =0 , Kd =0<br>
  : Car tries to track the center but off the track quickly with oscillation increasing(divergence).<br>

* Kp = -0.2 , Ki =0 , Kd =-1.0<br>
  : Added and tuned Kd cause it improves reducing oscillation(damping).<br>

* Kp = -0.2 , Ki =-0.0001 , Kd =-1.5<br>
  : Added and tuned Ki cause it improves offset from center.<br>

* Showed good result in the straight course.<br>

#### 2) Tune the gain in the curve.
* Kp = -0.2 , Ki =-0.0001 , Kd =-1.5<br>
  : used gain in step 1). Very unstable in the curve with large oscillation and overshoot.<br>

* Kp = -0.15 , Ki =-0.002 , Kd =-1.3<br>
  : Decrease Kp , Kd to reduce oscillation and overshoot.<br>
  : Increase Ki to reduce offset from the center.<br>
  : Iterate this step.

* Kp = -0.12 , Ki =-0.0015 , Kd =-1.0<br>
  : Showed good result in all course.

### 2. Improvements
* Need to control `throttle` to achieve more stability in the curve.<br>
  But to do this, I need `predicted path data` from the simulator. Human get this data by watching the road and guessing the curvature.
* Before using SGD or twiddle to find the best PID gain , we have to consider about `goodness` function.<br>
In `goodness` function, we have to include `stability` or `comfortability`
because control gain that gives large overshoot from the center is not good for human although it gives minimum `cross track error` on average.

---

---
## Code Impementation is [here](https://github.com/ksjgh/SDCND/tree/master/Term2_Robotics_Sensor_Fusion/04_Project_PID-Control)
---
