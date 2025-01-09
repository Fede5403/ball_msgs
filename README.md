# DARwIn-OP Robot Arm Control via Motor Map

## Overview

This project implements automated arm movement control for the DARwIn-OP robot. Initially designed with PlayStation 4 joystick control, the system now features autonomous control through a motor map, enabling the robot to automatically adjust its arms to balance a ball within a water-filled tube.

## Key Features

* **ROS op2_motormap Node**: Manages arm movement automation
* **Custom Message Type (ball_msgs/ball)**: Contains ball distance and velocity data from the robot's vision system
* **Vision System Integration**: Real-time vision data processing for calculating arm joint angles using the motor map algorithm

## Project Structure

### Custom Message Definition

A custom ROS message `ball_msgs/ball` facilitates communication between the vision system and motor map node:

```
float32 distance
float32 velocity
```

### Data Publication and ROS Integration

The processed data (ball distance from center and velocity) is encapsulated in the custom `ball_msgs/ball` ROS message. The vision system publishes this message to the `/ball_topic` at a rate synchronized with frame capture. This ensures real-time data availability for other nodes in the ROS network, particularly the motor map control system which subscribes to this topic for sensor data.

## Requirements

* **ROS (Robot Operating System)**: Compatible version with developed node
* **DARwIn-OP Robot**: Configured and operational
* **Integrated Vision System**: For ball detection
* **PlayStation 4 Controller**: Required only for initial manual control phase

## Installation

1. **Clone the Repository**
   ```bash
   git clone https://github.com/alessandonicosia/darwin-op-motormap-control.git
   ```

2. **Navigate to Project Directory**
   ```bash
   cd darwin-op-motormap-control
   ```

3. **Install Dependencies**
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

4. **Build the Project**
   ```bash
   catkin_make
   ```

5. **Source the Workspace**
   ```bash
   source devel/setup.bash
   ```

## Usage

1. **Start the Vision System**
   * Ensure the vision system is active and correctly publishing `ball_msgs/ball` messages to `/ball_topic`

2. **Launch the Motor Map Node**
   ```bash
   rosrun op2_motormap op2_motormap_node
   ```

3. **Monitor Arm Movement**
   * The op2_motormap node will process incoming data and automatically control the robot's arms to balance the ball within the tube

## Author

* Alessandro Nicosia

## Notes

This README was generated with the assistance of ChatGPT, a language model developed by OpenAI.

For further information, issues, or contributions, please visit the GitHub repository.
