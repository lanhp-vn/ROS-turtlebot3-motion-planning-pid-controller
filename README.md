# Motion Planning Using ROS

## Project Overview

This project implements a ROS-based motion planning system using PID controllers for a TurtleBot3 Waffle-Pi robot in Gazebo simulation. The system controls a differential drive robot to navigate from its current position to a target pose using two different control modes.

## System Architecture

The project consists of two main ROS nodes:

### 1. PID Controller Node (`pid_controller.py`)
- **Purpose**: Implements dual PID controllers (linear and angular velocity control)
- **Subscriptions**:
  - `/reference_pose` (Float64MultiArray): Target position [x, y, θ, mode]
  - `/odom` (Odometry): Current robot pose from Gazebo
- **Publications**:
  - `/cmd_vel` (Twist): Linear and angular velocity commands
  - `/goal_reached` (Bool): Status flag when target is reached

### 2. Motion Planner Node (`motion_planner.py`)
- **Purpose**: User interface for setting target goals and monitoring completion
- **Publications**:
  - `/reference_pose` (Float64MultiArray): Publishes user-defined targets
- **Subscriptions**:
  - `/goal_reached` (Bool): Monitors when robot reaches target

## Control Modes

### Mode 0 (Sequential Control)
1. **Rotate to Face Target**: Turn robot toward target position
2. **Move Forward**: Travel in straight line to target position  
3. **Final Orientation**: Rotate to achieve desired final angle

### Mode 1 (Simultaneous Control)
- **Concurrent Control**: Simultaneously control position and orientation
- **Final Adjustment**: Fine-tune orientation after reaching target position

## Prerequisites

### Required Software
- **VMware Workstation** (as specified in lab requirements)
- **Ubuntu 18.04/20.04** with ROS Melodic/Noetic
- **ROS Packages**:
  - `turtlebot3`
  - `turtlebot3_simulations`
  - `gazebo_ros`

### ROS Dependencies
The following ROS packages are required (already configured in `package.xml`):
- `rospy`
- `std_msgs`
- `geometry_msgs`
- `nav_msgs`

## Installation and Setup

### 1. Clone/Extract Project

### 2. Build the Workspace
```bash
# Build all packages in the workspace
catkin_make

# Source the workspace
source devel/setup.bash
```

### 3. Environment Setup
```bash
# Set TurtleBot3 model (required for Gazebo simulation)
export TURTLEBOT3_MODEL=waffle_pi

# Add to ~/.bashrc for persistence
echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc
```

## Usage Instructions

### Step 1: Launch Gazebo Simulation
Open **Terminal 1** and run:
```bash
source devel/setup.bash
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
```
This opens Gazebo with the TurtleBot3 in an empty world environment.

### Step 2: Start PID Controller
Open **Terminal 2** and run:
```bash
source devel/setup.bash
rosrun pid_controller pid_controller.py
```

### Step 3: Start Motion Planner
Open **Terminal 3** and run:
```bash
source devel/setup.bash
rosrun pid_controller motion_planner.py
```

### Step 4: Set Target Goals
The motion planner will prompt for user input:
```
Enter target X: 2.0
Enter target Y: 1.5
Enter target orientation theta (rad): 1.57
Enter mode (0 for sequential, 1 for simultaneous): 0
```

## PID Controller Tuning

### Current Parameters
```python
# Linear velocity PID gains
Kp_lin = 0.4    # Proportional gain
Ki_lin = 0.0001 # Integral gain  
Kd_lin = 0.0001 # Derivative gain

# Angular velocity PID gains
Kp_ang = 1      # Proportional gain
Ki_ang = 0.0001 # Integral gain
Kd_ang = 0.0001 # Derivative gain
```

### Tuning Guidelines
1. **Start with Ki and Kd at zero**
2. **Increase Kp until system oscillates**
3. **Adjust Ki to reduce steady-state error**
4. **Adjust Kd to reduce overshoot**

### Performance Tolerances
- **Position Error**: ≤ 0.1 meters
- **Orientation Error**: ≤ 0.1 radians (~5.7°)

## Key Features

### PID Controller Implementation
- **Dual-loop Control**: Separate PID controllers for linear and angular motion
- **Mode-dependent Logic**: Different control strategies based on user selection
- **Error Handling**: Proper angle wrapping and error normalization
- **Velocity Clamping**: Safety limits on maximum velocities

### Motion Planner Interface
- **Interactive Input**: Command-line interface for goal specification
- **Goal Monitoring**: Automatic detection when targets are reached
- **Continuous Operation**: Supports multiple sequential goals