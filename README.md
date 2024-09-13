# Task Description

### Overview

#### General Approach

The general approach for this task was to implement a simple **PI controller** for both linear and angular velocities. The steps of the algorithm are 

1. Calculate & align with goal heading using angular PI controller
2. Using linear PI controller to move towards heading
3. Re-calculate heading with every step, adjust as needed with PI Controller
4. Once Arrived, Reorient to match goal pose


![My First Board](https://github.com/user-attachments/assets/a1a3bd3e-7885-4c92-9823-080d9974e3ba)

#### Process & Considerations

**Input Parameters:** The exposed input parameters are an important considertaion when looking to design a control scheme. For our case, the robot was quite simple, with only import parameters being linear velocity (${v}$) and angular velocity (${w}$)

**Drive-System:** An important consideration to this approach was the drive & steering system of this robot. The robot was holonomically driven (instead of a differential drive). This allowed us to treat the robot as a **rigid body** and leverage rigid body kinematics instead relative to something like the bicycle model for differential drive robots.

**Localization & Odometry:**
The robot localization happened through the `/odom` topic. The `/odom` topic publishes Pose data through the a local coordinate frame (the robot frame of reference) where as the goal points are published/specified in the global reference frame. This is something that needed to be considered when interpreting Pose data from `/odom` and parsing through the PI controller. The robot was also intially spawned with a yaw offset of ${\approx}0.30rad$. This was hard-coded as robot-to-world transform. The `/odom` topic is continous (as EKF algorithms have a hard time with discontinuities) but also susceptible to drift. The drift can be addressed by leveraging a greater sensor suite and using sensor fusion to get a better idea of the robot location.

**Software Modularity:**
An additional `PIDController` class was spun up to effectively increase modularity of the written code. This was done by decoupling the computational functionality from the node itself allowing the computational functions to be plug-and-play with other potential nodes and reducing debug time. This maintains code readability, organization, and makes state-management a lot nicer all while making the code more extensible.

### Results

The results matched & surpassed the minimum specified thresholds with **Euclidian Distance Error <= 3cm** and **Pose Error < 0.1 rad** all while maintaing non-turtle speed 😊

![output(1)](https://github.com/user-attachments/assets/c536724b-b570-429f-9e0d-5de9a8256f3f)
![output](https://github.com/user-attachments/assets/6cc9b754-e562-4004-87ed-596197915c38)

#### Demo Video

https://github.com/user-attachments/assets/7a936c04-a984-42ab-978f-43c978672d21


### Areas For Improvement

#### Software

* Goal Input As ROS Services:
  - Instead of specifying the goal points in the node file, it would be nice to make a ROS Service so that we can continously change points without having to rebuild and potentially even use scripts to automate the process of giving input and extracting output from the simulations
    
* Logging Class
  - Instead of having scattered logs right now, more time could be spent creating a custom logging wrapper to enhance logging capabilites to allow us to capture a diverse set of logs and data. 

#### Control

* Robust PID Tuning & Gain Scheduling
  - From the graphs, it is evident that the Robot, while meeting the requirements, is less precise when trying to navigate situations with sharp/short turns. This could be improved by by leveraging state-models and software (e.x. simulink) to effectively tune the PID for car instead of observablility based trial and error. Moreover, we can look to adapt dynamic PID techniques such as Gain scheduling to fine-tune PID constants for specific scenarios and swap them in and out as needed.
  
* Sensor Suite & Localization
  - Instead of having hardcoded transformations, we can look to develop a Global frame of reference and use **tf2** library to develop more robot `/odom` to world frame translations for the simulation. However, since autonomous vehicles in real life don't have that oppurtunity, we can always move towards better locationalization techniques by introducing things like sensor fusion.

* AntiWindUp & Derivate Control
  - Since we are using integral to get rid of the steady state error and provide orientation intertia, we are susceptible to integral/error windup which can cause oscillations. In order to reduce the chances of such oscillations we can use anti-windup techniques like clamping to prevent overcompenstation. Morever, we can also introduce derivative control to further dampen our control system.

* Security
  - It might be beneficial to add security feature to the car like max linear velocity, max angular velocity, max rotation and etc as "safety" percuations
 

### Software Structure
```
- docker -- Where the Dockerfile lives.
- scripts -- Where necessary external scripts live.
- workspace -- Where all the packages live.
```

### Build the simulator

```bash
./scripts/build/sim.sh
```

### Run the simulator

```bash
./scripts/deploy/devel.sh # To enter the docker container
ros2 launch limo_simulation limo.launch.py # To launch the simulator
```

1. This will not work with docker desktop, please do not use it, use the default engine.

Feel free to modify anything else if it does not work as expected.
