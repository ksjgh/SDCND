
# Project : **Path Planning**
### Self-Driving Car Engineer Nanodegree
---

<br>
<img src="./reflection_img/MPC_Project.PNG" width="480" alt="Combined Image" />

---
---

### 1. Goal
* The code compiles correctly.<br>
* The car is able to drive at least 4.32 miles without  incident.<br>
* The car drives according to the speed limit.<br>
* Max Acceleration and Jerk are not Exceeded.<br>
* Car does not have collisions.<br>
* The car stays in its lane, except for the time between changing lanes.<br>
* The car is able to change lanes.<br>
* There is a reflection on how to generate paths.<br>

### 2. Path Planning Method
- Predict my car's position at behavior horizon. Code lines 292-296.<br>
-  Read other car's current state, predict state at behavior horizon , code lines 301-318.<br>
- Check if other car is too close or not, left/right lane is safe for lane chage. code lines 326-336.<br>
- If other car is in front of my car too close, then slow down and change lane if possible. code lines 340-354.<br>
- If lane is clear , increase speed. code lines 355-360.<br>

### 3. Generate trajectory
- Set ref_x_prev,ref_y_prev,ref_x,ref_y as starting point for trajectory. code lines 370-386.<br>
- Add some far points to make smooth spline. Code lines 395-406.<br>
- Make spline using points in 'ptsx , ptsy'. code lines 417-431.<br>
- Get path points by sampling spline. Code lines 433-467.<br>

### 4. reflection
- I've tried to solve this problem using potential field method.<br>
But it was harder than I thought with limiting time.<br>
I got some favorable result using finite state method and this gave me deep understanding about path planning.<br>
I would approach this project again using potential field method. 
