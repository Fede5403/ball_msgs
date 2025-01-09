# ball_msgs

## Overview

`ball_msgs` is a ROS message package that defines custom messages for ball tracking in the DARwIn-OP robot control system. This package facilitates communication between the vision system and the motor map control node.

## Message Types

### ball.msg

This message type contains information about the ball's position and movement:

```
float32 distance  # Distance of the ball from the center
float32 velocity  # Current velocity of the ball
```

## Usage

### Publishing Messages

```cpp
#include <ball_msgs/ball.h>

// Create and publish the message
ball_msgs::ball ball_data;
ball_data.distance = current_distance;
ball_data.velocity = current_velocity;
publisher.publish(ball_data);
```

### Subscribing to Messages

```cpp
#include <ball_msgs/ball.h>

void ballCallback(const ball_msgs::ball::ConstPtr& msg)
{
    float distance = msg->distance;
    float velocity = msg->velocity;
    // Process the data
}

// In your node's main function
ros::Subscriber sub = nh.subscribe("/ball_topic", 1, ballCallback);
```

## Installation

1. **Add to Your Workspace**
   ```bash
   cd ~/catkin_ws/src
   git clone <repository_url>/ball_msgs.git
   ```

2. **Build the Package**
   ```bash
   cd ~/catkin_ws
   catkin_make
   ```

3. **Source Your Workspace**
   ```bash
   source ~/catkin_ws/devel/setup.bash
   ```

## Dependencies

* `message_generation`
* `message_runtime`
* `std_msgs`

## Package Structure

```
ball_msgs/
├── CMakeLists.txt
├── package.xml
└── msg/
    └── ball.msg
```

## Integration with the Vision System

The vision system publishes ball data using this message type to the `/ball_topic` topic. The motor map control system subscribes to this topic to receive real-time updates about the ball's position and velocity for automated arm control.

## Build Configuration

### CMakeLists.txt

Ensure your CMakeLists.txt includes:

```cmake
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_message_files(
  FILES
  ball.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)
```

### package.xml

Required dependencies in package.xml:

```xml
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
<depend>std_msgs</depend>
```
