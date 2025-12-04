---
title: "1. Gazebo: Setup and ROS Integration"
---

## Gazebo: Setup and ROS Integration

In robotics, the adage "bits are cheaper than atoms" holds profound truth. Before a physical robot ever moves, its digital counterpart will have lived a thousand lives, crashed a thousand times, and learned from every failure in the safety of a simulated world. This module is dedicated to the theory and practice of creating these "digital twins" using Gazebo, the de facto open-source physics simulator for the ROS ecosystem.

### Gazebo and the ROS Ecosystem

It is essential to understand that ROS and Gazebo are separate, powerful projects that have been integrated to work together. Gazebo's role is to be the "world"—it handles the physics, the rendering, and the sensor data generation. ROS's role is to be the robot's "brain"—it runs the software that makes decisions.

#### The Gazebo-ROS Bridge

The communication between these two systems is handled by a set of specialized nodes provided by the `gazebo_ros` package. These nodes form a **bridge** between Gazebo's internal data transport layer and the ROS 2 DDS network. For example, when you add a camera plugin to your simulated robot, that plugin publishes image data on a Gazebo topic. A bridge node then subscribes to this Gazebo topic and republishes the data onto a ROS 2 topic, making it available to the rest of your ROS system.

#### Simulation Time

A crucial concept in any simulation is **time**. A complex physics simulation may not be able to run in real-time; it might run slower. Conversely, a simple simulation might run much faster than real-time.

For deterministic testing, it is vital that your ROS nodes operate on the simulation's clock, not the "wall clock" of your computer. Gazebo publishes the current simulation time on the `/clock` topic. By setting the `use_sim_time` parameter to `true` on your ROS nodes, you instruct them to subscribe to the `/clock` topic and use its timestamps instead of the system clock. This ensures that even if the simulation slows down, the logic of your system remains consistent.

### Basic Gazebo Commands

*   **Launching Gazebo:** You can launch Gazebo with a pre-built world. For example, to launch an empty world:
    ```bash
    ros2 launch gazebo_ros gazebo.launch.py
    ```
*   **Spawning a Robot:** Once Gazebo is running, you can spawn a robot model into the simulation. This is typically done with a ROS 2 node that reads a URDF or SDF file and calls the `/spawn_entity` service in Gazebo.
    ```bash
    ros2 run gazebo_ros spawn_entity.py -entity my_robot -file my_robot.urdf -x 0 -y 0 -z 1
    ```
