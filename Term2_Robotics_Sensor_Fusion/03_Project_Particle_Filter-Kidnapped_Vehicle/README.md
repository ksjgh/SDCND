
# Project : **Particle Filter**
---
<img src="./report_data/particle_filter.PNG" width="960" alt="Combined Image" />

#### Result Video Link - [Particle Filter](https://youtu.be/mXSa4lwoU3Q)<br>

![#000000](https://placehold.it/15/000000/000000?text=+) Black Circles : Landmarks in the given map , input to particle filter<br>
![#c5f015](https://placehold.it/15/c5f015/000000?text=+) Green line : show detected landmarks by particle<br>
![#1589F0](https://placehold.it/15/1589F0/000000?text=+) Blue Circle : Estimated position of vehicle by particle filter , output<br>

---

## Overview

### 1. Goals
  * Utilize particle filter to localize a moving object from blind start.<br>
  * Obtain low RMSE(Root Mean Square Error) of vehicle position(x,y) and heading direction(yaw).<br>

  _Note :<br>
  The simulator provide program <br>
  1) Initial GPS measurement.<br>
  2) Observed distance to each landmarks.<br>
  3) Vehicle control data_<br>

  _Program compute<br>
  1) current position : x, y<br>
  2) current direction : yaw<br>
  3) RSME of x, y, yaw_<br>

### 2. Applied Techniques
* Motion Models : constant turn rate and velocity magnitude model (CTRV) of vehicle is used for motion update process.
* Multi-variable gaussian to evaluate each particle.
* Implemented using C++

### 3. Result
* RSME of x ,  y < 0.12[m]  
* RSME of yaw < 0.04[rad]
