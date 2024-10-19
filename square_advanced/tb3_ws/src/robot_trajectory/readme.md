# TurtleSim Square Trajectory in ROS2

This ROS2 package controls a TurtleSim robot to move in a square trajectory. The linear and angular speeds of the turtle are adjustable via ROS2 parameters. The project leverages the `rclcpp` and `geometry_msgs` libraries to control the turtle's movements.

## Table of Contents
- [Installation](#installation)
- [Package Overview](#package-overview)
- [Usage](#usage)
- [Custom Parameters](#custom-parameters)
- [How it Works](#how-it-works)
- [License](#license)

## Installation

### Prerequisites

Ensure you have ROS2 (Foxy or later) and the `turtlesim` package installed. If not, you can install `turtlesim` as follows:

```bash
sudo apt install ros-<ros2-distro>-turtlesim
```

Replace `<ros2-distro>` with your ROS2 distribution, e.g., `foxy` or `galactic`.

### Cloning the Repository

1. Create a ROS2 workspace if you don't already have one:

   ```bash
   mkdir -p ~/tb3_ws/src
   cd ~/tb3_ws/src
   ```

2. Clone this repository into the `src` directory of your workspace:

   ```bash
   git clone <repository-url> robot_trajectory
   ```

3. Build the workspace:

   ```bash
   cd ~/tb3_ws
   colcon build
   ```

4. Source your workspace:

   ```bash
   source install/setup.bash
   ```

## Package Overview

This package contains the following key components:

- **C++ Node**: `square.cpp`, which controls the turtle to move in a square.
- **ROS2 Parameters**: Customizable `linear_speed` and `angular_speed` parameters to control the movement of the turtle.
- **Dependencies**: The project depends on `rclcpp`, `geometry_msgs`, and `turtlesim`.

## Usage

### Launch Turtlesim

Before running the node, you need to launch the `turtlesim_node` to visualize the turtle in the simulator:

```bash
ros2 run turtlesim turtlesim_node
```

### Run the Square Trajectory Node

Now, run the `square_trajectory` node to start moving the turtle in a square:

```bash
ros2 run robot_trajectory square
```

This will run the turtle with the default linear and angular speeds (0.1 m/s for linear speed, and π/20 rad/s for angular speed).

### Custom Parameters

You can adjust the turtle's movement by specifying custom parameters for `linear_speed` and `angular_speed`:

```bash
ros2 run robot_trajectory square --ros-args --param linear_speed:=0.5 --param angular_speed:=0.78
```

In this example:
- The turtle will move forward at `0.5 m/s`.
- The turtle will rotate at `0.78 rad/s`.

You can also remap the `/cmd_vel` topic as needed:

```bash
ros2 run robot_trajectory square --ros-args --remap /cmd_vel:=/turtle1/cmd_vel --param linear_speed:=0.5 --param angular_speed:=0.78 --param distance:=2.0
```

## How it Works

### Parameters

- **linear_speed**: This parameter controls the forward movement speed of the turtle. The default value is `0.1 m/s`.
- **angular_speed**: This parameter controls the rotation speed of the turtle during the 90-degree turn. The default value is `π/20 rad/s`.
- **distance**: This parameter controls how long the bot will cover. The default value is `1.0 m`.

### Logic

The turtle moves in a square trajectory by:
1. Moving forward for a certain number of iterations (calculated based on `linear_speed`).
2. Rotating 90 degrees (π/2 radians) for a certain number of iterations (calculated based on `angular_speed`).
3. Repeating the above two steps four times to complete the square.

### Code Overview

1. The node declares two ROS2 parameters: `linear_speed` and `angular_speed`.
2. These parameters are fetched and used to compute the number of iterations required for both linear movement and turning.
3. The turtle alternates between moving straight and rotating 90 degrees to complete the square.
