# Simulation, Middleware and Benchmarking (2024-2025)

## Course Overview
**SJO038 - MIR@UJI SJO038 - 2425: Simulation, Middleware, and Benchmarking** is a comprehensive course that introduces students to the architectures of robot systems, middleware technologies, and simulation techniques. The course is structured to include both theoretical knowledge and practical assignments, focusing on developing skills in C++, Git, ROS2, and other relevant tools.

### Course Instructors
- **Prof. Antonio Morales**  
  [More info](#) | [Office Hours](#)
  
- **Dr. Jorge Sales**  
  [More info](#)

## Table of Contents
- [Course Description](#course-description)
- [Architectures for Robot Systems](#architectures-for-robot-systems)
  - [Key Topics and Readings](#key-topics-and-readings)
- [Activities](#activities)
  - [Activity 1: Hello World!](#activity-1-hello-world)
  - [Activity 2: C++ Recap](#activity-2-c-recap)
  - [Activity 3: Hello ROS](#activity-3-hello-ros)
  - [Activity 4: ROS Sum](#activity-4-ros-sum)
  - [Activity 5: ROS Stats](#activity-5-ros-stats)
- [ROS2 Tutorials](#ros2-tutorials)
- [Lab Environment](#lab-environment)

## Course Description
This course provides an in-depth understanding of robot system architectures, middleware solutions, and benchmarking techniques. It emphasizes practical implementations in ROS2 and C++ with a focus on building scalable and efficient robotic software systems.

Key objectives of the course include:
- Understanding robotic system architectures.
- Developing proficiency in ROS2.
- Applying middleware solutions in simulation environments.
- Benchmarking robotic applications.

## Architectures for Robot Systems

### Key Topics and Readings
- **Robotics Systems Architectures and Programming**
  - Kortekamp, Simmons, Brugali: *Springer Handbook of Robotics* (Chapter 12), 2016.
  
- **Behavior-Based Systems**
  - Michaud and Nicolescu: *Springer Handbook of Robotics* (Chapter 13), 2016.

- **Software Organization of Autonomy**
  - Murphy: *Introduction to AI Robotics*, MIT Press, 2019.

- **Layered Control System**
  - Brooks: *A robust layered control system for a mobile robot*, IEEE Journal, 1986.

## Activities

### Activity 1: Hello World!
This initial task introduces students to the Git and C++ development environment. Students are required to set up a GitHub repository and submit the "Hello World!" C++ program.

- [GitHub Repository Link](#)
- [Assignment Submission](#)

### Activity 2: C++ Recap
Recap of C++ basics, focusing on solving mathematical problems programmatically.

- **Homework:**
  - [sum v5](#)
  - [sum_ab](#)
  - [Quadratic Equation](#)
  - [Submit a Math Problem](#)
  - [Solve Your Math Problem](#)

### Activity 3: Hello ROS
Introduction to ROS2. Students develop a basic ROS2 node to understand nodes, topics, services, and actions.

- [ROS Tutorial](https://docs.ros.org/en/humble/)
- [Assignment Submission](#)

### Activity 4: ROS Sum
Students implement a ROS2 node that takes input from topics, processes it, and publishes the sum of two integers.

- [Assignment Submission](#)

### Activity 5: ROS Stats
Using ROS bag files, students develop a system to calculate statistics (mean, median, mode) for a stream of numerical data.

- **Data File:** [numbers.bag.zip](#)
- [Assignment Submission](#)

## ROS2 Tutorials
Students were required to complete the following ROS2 tutorials:
- Configuring the environment.
- Using `turtlesim`, `ros2`, and `rqt`.
- Understanding nodes, topics, services, and actions.
- Using `rqt_console` for debugging and monitoring.
- Launching nodes and managing parameters.

Additional reading materials include:
- [ROS2 Cheatsheet](#)
- [ROS2 Package Development](#)

## Lab Environment

If the lab computers become slow or take time to compile, you can run the following command to stop the antivirus daemon:

```bash
sudo /etc/init.d/clamav-daemon stop
```

## Course Resources
- [Official Slides - Basic Concepts](#)
- [Hints for Visual Studio Code Users](#)

## Additional Projects

### ROS2 Advanced Packages
1. **Square Advanced - Gazebo Simulation**
   - [Simulation Link](#)

2. **Square Advanced - Turtlesim**
   - [Simulation Link](#)
   
## How to Use This Repository

1. Clone the repository:
   ```bash
   git clone https://github.com/eather0056/Simulation-Middleware-and-Benchmarking.git
   ```

2. Navigate to the specific activity folder:
   ```bash
   cd Simulation-Middleware-and-Benchmarking/activity_X
   ```

3. Compile and run:
   ```bash
   mkdir build
   cd build
   cmake ..
   make
   ./your_program
   ```
