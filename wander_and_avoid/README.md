# Multi Turtlebot Simulation with Obstacle Avoidance and Follower Behavior

This repository provides a ROS2-based multi-robot simulation featuring two Turtlebot3 robots. The robots exhibit two primary behaviors:
1. **Obstacle Avoidance**: Independently navigate around detected obstacles to prevent collisions.
2. **Follower Behavior**: `Robot2` follows `Robot1` using real-time transformations (tf) while maintaining a safe distance.

The simulation environment includes many closely spaced obstacles, which may cause occasional collisions due to the constraints in sensing and movement. 

## Theoretical Logic

### Obstacle Avoidance

The obstacle avoidance mechanism relies on **LIDAR data** from each robot's laser scanner. The robot continuously scans a 60-degree sector directly in front and flags an obstacle if any detected object is within a threshold distance (0.4 meters for `Robot1` and 0.5 meters for `Robot2`). Upon detecting an obstacle:
- The robot randomly selects a turn direction (left or right).
- It continues turning until the obstacle is cleared, at which point it resumes forward motion.

This basic obstacle avoidance technique works well in open spaces but may struggle in environments with closely spaced obstacles.

### Follower Behavior

`Robot2` uses a **tf-based tracking** approach to follow `Robot1`:
- `Robot2` retrieves the positional transformation of `Robot1` using ROS2 `tf2` and adjusts its orientation toward `Robot1`.
- It moves forward until within a specified distance (0.5 meters), then stops.
- This behavior resumes once the path is clear of obstacles.

## Setup Instructions

### 1. Create a ROS2 Workspace, Clone the Repository, and Build the Package

```bash
mkdir -p wf_ws/src
cd wf_ws/src
git clone https://github.com/RobInLabUJI/multi_turtlebot_sim
cd ..
source /opt/ros/humble/setup.bash
colcon build
```

### 2. Launch the Standalone Simulation Environment

The following command launches the simulation environment without spawning any robots:

```bash
source /opt/ros/humble/setup.bash
source wf_ws/install/setup.bash
ros2 launch multi_turtlebot_sim standalone_world.launch.py
ros2 launch multi_turtlebot_sim standalone_world.launch.py world_name:=turtlebot3_world.world
```

> **Note**: The default environment is an empty world. To use a predefined world with obstacles, specify `world_name:=turtlebot3_world.world`.

### 3. Spawn Robots

#### Spawn `Robot1` at a Given Position

```bash
source /opt/ros/humble/setup.bash
source wf_ws/install/setup.bash
ros2 launch multi_turtlebot_sim spawn_turtlebot3.launch.py robot_prefix:=robot1 x_pose:=0.5 y_pose:=0.5
```

The `robot_prefix` parameter sets both the ROS namespace and tf prefix for `Robot1`.

#### Spawn `Robot2` at a Different Position

```bash
source /opt/ros/humble/setup.bash
source wf_ws/install/setup.bash
ros2 launch multi_turtlebot_sim spawn_turtlebot3.launch.py robot_prefix:=robot2 x_pose:=-0.5 y_pose:=-0.5
```

#### Launch `wander_and_avoid` with `tf`-based Following

This command launches the obstacle avoidance and follower behaviors for `Robot1` and `Robot2`:

```bash
ros2 launch wander_and_avoid robots_with_world_tf.launch.py
```

## Limitations

Due to the high density of obstacles in the `turtlebot3_world`, the robots may occasionally collide with objects. This is an inherent limitation of the basic obstacle avoidance algorithm, as the robots only consider a limited sector in front for obstacle detection and may struggle to maneuver around closely spaced objects.
