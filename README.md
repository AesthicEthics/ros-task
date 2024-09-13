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

#### Other Potential Approaches

### Results

The results matched & surpassed the minimum specified thresholds with **Euclidian Distance Error <= 3cm** and **Pose Error < 0.1 rad** all while maintaing non-turtle speed 😊

![output(1)](https://github.com/user-attachments/assets/c536724b-b570-429f-9e0d-5de9a8256f3f)
![output](https://github.com/user-attachments/assets/6cc9b754-e562-4004-87ed-596197915c38)

#### Demo Video
https://youtu.be/Y3LloVwmLyc

### Areas For Improvement

#### Software

**Input As ROS Services**
**Logging Class**

#### Control

**Robust Kinematics Based Tuning**

**Sensor Suite & Localization**

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

### What do I edit?

1. Modify the package `limo_control` in the workspace directory for adding your c++ controller program.
2. Make a launch file that can launch everything (Controller and Simualation).
3. Modify `scripts/deploy/app.sh` such that, when `scripts/deploy/start.sh` is run, the task is executed automatically.

### Known Issues

1. This will not work with docker desktop, please do not use it, use the default engine.

Feel free to modify anything else if it does not work as expected.
