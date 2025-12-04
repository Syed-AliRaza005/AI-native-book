---
title: "Assessment 1: ROS 2 Package Development"
---

### Assessment 1: ROS 2 Package Development Project

**Objective:** To demonstrate mastery of the ROS 2 framework by developing a robust, well-documented software package that performs a defined robotics task.

**Description:** You will be required to design and implement a ROS 2 package from the ground up in Python. The package must contain at least two nodes that communicate with each other using a combination of topics, services, and/or actions. You will also write a comprehensive launch file to start and configure your system.

**Potential Project Ideas:**
*   A "purity patrol" system where one node simulates a wandering robot and publishes its position, and a second "guardian" node determines if the robot has entered a "forbidden zone" and, if so, uses a service to call it back.
*   An "emergency stop" system where a subscriber node listens for high-velocity commands on a topic and, if a command exceeds a safety threshold, it activates a "stop" service on the motor controller node.

**Deliverables:**
1.  A fully functional ROS 2 package in a Git repository.
2.  A `README.md` file explaining the purpose of the package, how to build it, and how to run it using your launch file.
3.  A short video (2-3 minutes) demonstrating your system in action (e.g., using `ros2 topic echo` and `ros2 service call` to show the interaction).
4.  The code must be well-commented and adhere to standard Python and ROS 2 conventions.

**Learning Outcomes Assessed:**
*   Proficiency in creating and building ROS 2 packages.
*   Deep understanding of the primary ROS 2 communication mechanisms.
*   Ability to manage a multi-node system using launch files.
