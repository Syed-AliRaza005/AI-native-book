---
title: "Assessment 3: Isaac-based Perception Pipeline"
---

### Assessment 3: Isaac-based Perception Pipeline

**Objective:** To leverage the power of the NVIDIA Isaac ecosystem to build a hardware-accelerated perception pipeline.

**Description:** This project will be completed using Isaac Sim. You will be given a simulated environment containing various objects. Your task is to build a perception pipeline that uses the simulated camera feed to identify a specific target object (e.g., a red cube) and determine its 3D position relative to the robot. You will use Isaac ROS packages for this, such as those for image rectification and object detection.

**Deliverables:**
1.  A ROS 2 package containing a launch file that starts the Isaac Sim environment and your perception pipeline.
2.  A Python node that subscribes to the output of your perception pipeline and prints the 3D coordinates of the detected object.
3.  A written report (1-2 pages) explaining the architecture of your pipeline, the roles of the different Isaac ROS nodes you used, and the format of the data as it flows through the system.

**Learning Outcomes Assessed:**
*   Familiarity with the Isaac Sim environment.
*   Ability to integrate and use hardware-accelerated Isaac ROS perception packages.
*   Understanding of data-flow in a complex perception system.
