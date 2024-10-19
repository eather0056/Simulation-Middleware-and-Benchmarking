# Robot Trajectory in ROS2

This project implements a ROS2 node that controls a robot to move in a square trajectory using the TurtleBot3 simulation environment. The project is built using `rclcpp` and publishes velocity commands to the `/cmd_vel` topic in the form of `geometry_msgs::msg::Twist` messages.

## Table of Contents
- [Installation](#installation)
- [Setup](#setup)
- [Running the Node](#running-the-node)
- [Simulation](#simulation)
- [Troubleshooting](#troubleshooting)
- [License](#license)

## Installation

Follow these steps to install and set up the project:

1. **Clone the Repository:**

   ```bash
   git clone <repository-url>
   cd <repository-directory>
   ```

2. **Create a ROS2 Workspace:**

   ```bash
   mkdir -p ~/tb3_ws/src
   cd ~/tb3_ws/src
   ```

3. **Move the Package:**

   Move the `robot_trajectory` package into the `~/tb3_ws/src` directory.

4. **Install ROS2 and TurtleBot3:**

   If ROS2 and TurtleBot3 are not installed, follow these guides to set them up:
   - [Install ROS2](https://docs.ros.org/en/foxy/Installation.html)
   - [Install TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)

5. **Build the Workspace:**

   After placing the package into the workspace, build the workspace:

   ```bash
   cd ~/tb3_ws
   colcon build
   ```

6. **Source the Workspace:**

   Once the build is complete, source the workspace setup file:

   ```bash
   source install/setup.bash
   ```

## Setup

This project includes:
- A ROS2 package named `robot_trajectory`.
- A ROS2 node (`square.cpp`) that commands the robot to move in a square trajectory.
  
### Dependencies

The package depends on the following ROS2 libraries:
- `rclcpp`
- `geometry_msgs`

Make sure the following lines are included in the `package.xml` file:

```xml
<depend>rclcpp</depend>
<depend>geometry_msgs</depend>
```

The `CMakeLists.txt` file should properly link these libraries for compilation.

## Running the Node

To run the node and make the robot move in a square trajectory:

1. Launch a TurtleBot3 simulation in Gazebo:

   ```bash
   export TURTLEBOT3_MODEL=burger
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```

2. Run the `robot_trajectory` node:

   ```bash
   source ~/tb3_ws/install/setup.bash
   ros2 run robot_trajectory square
   ```

The robot should start moving in a square trajectory, making four 90º turns.

## Simulation

You can test the robot's movement in Gazebo or Webots. Follow these steps:

1. **Gazebo Simulator:**
   
   Make sure to have Gazebo installed. You can launch the simulation using:

   ```bash
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```

2. **Webots Simulator (Optional):**

   If you want to use Webots for simulation, download and install Webots:

   ```bash
   wget https://github.com/cyberbotics/webots/releases/download/R2023b/webots-R2023b-x86-64.tar.bz2
   tar xjf webots-R2023b-x86-64.tar.bz2
   ```

   Then, follow the instructions to launch Webots with the robot.

## Troubleshooting

If you encounter errors while building or running the node, try the following:

- **Build errors (`undefined reference to 'main'`):** 
  Ensure that `square.cpp` contains the `main()` function and that the `CMakeLists.txt` file correctly specifies the source file for compilation.

- **Node not publishing to `/cmd_vel`:**
  Make sure that your `square.cpp` file is correctly publishing `geometry_msgs::msg::Twist` messages to the `/cmd_vel` topic and that the robot is subscribed to this topic.

- **Robot not moving:**
  Check if the simulation is running properly and the `/cmd_vel` topic is receiving velocity commands.


