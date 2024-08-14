# Delta_Robot_Control

## Overview
This repository hosts the control system for a Delta Robot, optimized for precise end effector manipulation and movement in both simulated and real-world settings.

## Features
- **Customized URDF**: Enhanced and adapted from an existing model to meet our Delta Robot's specific requirements. Thanks to the original contributors of the [ros_delta_robot repository](https://github.com/serrauvic/ros_delta_robot) for the URDF file.
- **Controller Development**: Scripts and launch files developed to enable precise control over the robot’s joints.
- **Implemented Inverse Kinematics**: Applied inverse kinematic theories to manage the position of the end effector with high accuracy.

## Future Works
- **End Effector Straight Line Movement**: To implement algorithms that will allow the end effector to move in a straight line, which is essential for executing precision tasks.
- **Real Robot Application**: To adapt theoretical algorithms for use in a real Delta Robot, aiming to confirm the system’s effectiveness in actual use scenarios.

## Installation
Clone this repository to your local machine using:

```
git clone https://github.com/Makizy/Delta_Robot_Control.git
```

## Usage
Navigate to the project directory and launch the simulation with:

```
roslaunch delta_robot_gazebo.launch
```

## Demonstration
A video demonstration of the robot in Gazebo, showing the end effector moving to the desired point with joint torque inputs, can be viewed here:


https://github.com/user-attachments/assets/4880e58a-f23b-4990-856d-1744584fd150


## Acknowledgements
Special thanks to the [ros_delta_robot project](https://github.com/serrauvic/ros_delta_robot) for providing the base URDF file, which was crucial for our adaptations and initial project setup.

## License
This project is licensed under the MIT License.
