
# Project : **PID Control Reflection**
### Self-Driving Car Engineer Nanodegree
---

### 0. Result Video<br>
[VIDEO : PID control of the car](https://youtu.be/2lOEfyVJfe4)<br>

---

### 1. PID Gain tuning method

### 0) PID Gain Parameters Description
* Kp : proportional gain , adjust increase of `steer_value` proportional to CTE.<br>
* Ki : integration gain , adjust integration rate of CTE and increase `steer_value` if there is static bias of CTE. <br>
I tuned this to reduce static CTE at the corner.<br>
* Kd : derivertive gain, adjust change rate of CTE and increase `steer_value` to reduce CTE if CTE is about to increase<br>
I tuned this to reduce oscillation.

#### 1) Tune the gain in the straight course
* I tuned the gain manually<br>

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

### 2. Further Improvements
* Need to control `throttle` to achieve more stability in the curve.<br>
  But to do this, I need `predicted path data` from the simulator. Human get this data by watching the road and guessing the curvature.
* Before using SGD or twiddle to find the best PID gain , we have to consider about `goodness` function.<br>
In `goodness` function, we have to include `stability` or `comfortability`
because control gain that gives large overshoot from the center is not good for human although it gives minimum `cross track error` on average.

---
## Code Implementation is [here](https://github.com/ksjgh/SDCND/tree/master/Term2_Robotics_Sensor_Fusion/04_Project_PID-Control)
---
