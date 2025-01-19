# Mundo Project

## Overview

This project is a ROS-based robotic simulation using Stage. It includes a robot that can navigate a simulated environment, avoid obstacles using either a simple avoidance algorithm or potential fields, and follow a leader robot. The project is structured to include various components such as movement control, goal setting, and sensor data processing.

## Components

### Source Code

- **moveRobot.cpp**: Contains the main logic for robot movement, obstacle avoidance, and leader-following behavior.
- **talkerGoals.cpp**: Publishes goal points for the robot to follow.

### Launch Files

- **launch_stage.launch**: Launches the Stage simulator with the specified world and robot configurations.

### World Files

- **mundo.world**: Defines the simulated environment including obstacles and robot initial positions.
- **map.inc**: Includes map-related configurations.
- **objects.inc**: Defines various environmental objects.
- **pioneer.inc**: Contains definitions for the Pioneer robot models.
- **sick.inc**: Defines the Sick laser sensor configurations.

## Dependencies

- ROS (Robot Operating System)
- Stage (2D robot simulator)
- geometry_msgs
- nav_msgs
- roscpp
- std_msgs
- tf

## Building the Project

To build the project, run the following command:

```sh
catkin_make
```

Running the Simulation
To run the simulation, use the following command:

```sh
roslaunch mundo launch_stage.launch
```

## Configuration

- `ID_ROBOT`: Robot ID
- `ROBOT_ROL`: Role of the robot (LEADER or FOLLOWER)
- `ID_LEADER`: ID of the leader robot
- `DIST_LEADER`: Distance from the follower to the leader
- `ALGOR`: Algorithm number for obstacle avoidance
- `CRIT_DIST`: Critical distance for obstacle detection
- `D_OBJ`: Distance to the object
- `V_MAX_DES`: Desired maximum velocity
- `V_MAX_ROT`: Maximum rotation velocity
- `K_ROT_MIN`: Minimum rotation constant
- `K_ROT_MAX`: Maximum rotation constant
- `ORI_ERROR`: Orientation error
- `T_AVOID_OBS`: Time to avoid obstacle
- `W_1`: Weight of go-to-target
- `W_2`: Weight of avoid obstacles
- `T_WAIT`: Time to wait for potential fields algorithm