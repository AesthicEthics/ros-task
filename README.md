Here is the corrected markdown with spelling mistakes fixed:

# Task Description

### Overview

#### General Approach

The general approach for this task was to implement a simple **PI controller** for both linear and angular velocities. The steps of the algorithm are:

1. Calculate & align with goal heading using angular PI controller
2. Use linear PI controller to move towards heading
3. Re-calculate heading with every step, adjust as needed with PI Controller
4. Once arrived, reorient to match goal pose

![My First Board](https://github.com/user-attachments/assets/a1a3bd3e-7885-4c92-9823-080d9974e3ba)

#### Process & Considerations

**Input Parameters:** The exposed input parameters are an important consideration when designing a control scheme. In our case, the robot was quite simple, with the only important parameters being linear velocity (${v}$) and angular velocity (${w}$).

**Drive-System:** An important consideration in this approach was the drive & steering system of this robot. The robot was holonomically driven (instead of a differential drive). This allowed us to treat the robot as a **rigid body** and leverage rigid body kinematics, instead of something like the bicycle model for differential drive robots.

**Localization & Odometry:**  
The robot localization happened through the `/odom` topic. The `/odom` topic publishes Pose data through a local coordinate frame (the robot frame of reference), whereas the goal points are published/specified in the global reference frame. This needed to be considered when interpreting Pose data from `/odom` and parsing it through the PI controller. The robot was also initially spawned with a yaw offset of ${\approx}0.30rad$. This was hard-coded as the robot-to-world transform. The `/odom` topic is continuous (as EKF algorithms have a hard time with discontinuities) but is also susceptible to drift. The drift can be addressed by leveraging a better sensor suite and using sensor fusion to get a more accurate idea of the robot’s location.

**Software Modularity:**  
An additional `PIDController` class was spun up to effectively increase the modularity of the written code. This was done by decoupling the computational functionality from the node itself, allowing the computational functions to be plug-and-play with other potential nodes and reducing debug time. This maintains code readability and organization and makes state management easier, all while making the code more extensible.

### Results

The results matched and surpassed the minimum specified thresholds, with **Euclidean Distance Error <= 3cm** and **Pose Error < 0.1 rad**, all while maintaining non-turtle speed 😊

![output(1)](https://github.com/user-attachments/assets/c536724b-b570-429f-9e0d-5de9a8256f3f)
![output](https://github.com/user-attachments/assets/6cc9b754-e562-4004-87ed-596197915c38)
![output(3)](https://github.com/user-attachments/assets/1bdadeea-8afa-4e9a-a5a9-212828ebe44a)

**Note:** Last few Epochs were taken for the Orientation error plot as that is when the robot with reorient itself to the match to goal pose. 

#### Demo Video

https://github.com/user-attachments/assets/7a936c04-a984-42ab-978f-43c978672d21

### Areas for Improvement

#### Software

* **Goal Input as ROS Services**:
  - Instead of specifying the goal points in the node file, it would be nice to make a ROS Service so that we can continuously change points without having to rebuild. We could also use scripts to automate the process of providing input and extracting output from the simulations.
    
* **Logging Class**:
  - Instead of having scattered logs, more time could be spent creating a custom logging wrapper to enhance logging capabilities. This would allow us to capture a diverse set of logs and data. 

#### Control

* **Robust PID Tuning & Gain Scheduling**:
  - From the graphs, it is evident that the robot, while meeting the requirements, is less precise when navigating situations with sharp/short turns. This could be improved by leveraging state-models and software (e.g., Simulink) to effectively tune the PID for the car instead of relying on observability-based trial and error. Moreover, we can look to adapt dynamic PID techniques such as gain scheduling to fine-tune PID constants for specific scenarios and swap them in and out as needed.
  
* **Sensor Suite & Localization**:
  - Instead of having hardcoded transformations, we could develop a global frame of reference and use the **tf2** library to create more accurate robot `/odom` to world frame translations for the simulation. However, since autonomous vehicles in real life don’t have that opportunity, we could move towards better localization techniques by introducing sensor fusion.

* **Anti-Windup & Derivative Control**:
  - Since we are using integral control to eliminate the steady-state error and provide orientation inertia, we are susceptible to integral/error windup, which can cause oscillations. To reduce the chances of such oscillations, we can use anti-windup techniques like clamping to prevent overcompensation. Moreover, we could introduce derivative control to further dampen our control system.

* **Security**:
  - It might be beneficial to add security features to the car, such as max linear velocity, max angular velocity, and max rotation, as "safety" precautions.

### Software Structure
```
- docker -- Where the Dockerfile lives.
- scripts -- Where necessary external scripts live.
- workspace -- Where all the packages live.
```

### Build the Simulator

```bash
./scripts/build/sim.sh
```

### Run the Simulator

```bash
./scripts/deploy/devel.sh # To enter the docker container
ros2 launch limo_simulation limo.launch.py # To launch the simulator
```
