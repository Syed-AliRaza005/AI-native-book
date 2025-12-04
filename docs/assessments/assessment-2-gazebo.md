---
title: "Assessment 2: Gazebo Simulation and Control"
---

### Assessment 2: Gazebo Simulation and Control

**Objective:** To create a simulated robot and environment in Gazebo, demonstrating an understanding of robot modeling and physics simulation.

**Description:** You will take a provided URDF model of a simple mobile robot and enhance it. You will be required to add a sensor plugin (e.g., a LiDAR or camera) and a differential drive plugin. You will then create a custom Gazebo "world" file with obstacles. The final part of the assessment is to write a simple ROS 2 node that subscribes to the sensor data and publishes velocity commands to make the robot navigate your world without colliding with obstacles.

**Deliverables:**
1.  Your modified URDF file containing the necessary plugin configurations.
2.  Your custom `.world` file.
3.  The Python ROS 2 node for your obstacle avoidance logic.
4.  A launch file that starts Gazebo, spawns your robot in your world, and starts your control node.
5.  A video demonstrating your robot successfully navigating the environment.

**Learning Outcomes Assessed:**
*   Ability to describe a robot's physical properties using URDF.
*   Understanding of Gazebo plugins for sensor and actuator simulation.
*   Practical application of the Sense-Plan-Act cycle in a simulated environment.
