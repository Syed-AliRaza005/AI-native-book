---
title: "2. Describing Robots: URDF to XACRO"
---

## Describing Robots: From URDF to XACRO

Simulating a robot effectively begins with a precise and comprehensive description of its physical characteristics. The Robotic Operating System (ROS) ecosystem predominantly uses the Unified Robot Description Format (URDF) for this purpose. However, for complex, real-world robots, raw URDF often proves cumbersome. This section delves into URDF's capabilities and introduces XACRO, an indispensable tool for managing complex robot descriptions.

### URDF: The Foundational Robot Description Format

URDF is an XML-based file format used in ROS to describe all elements of a robot. It defines the robot's kinematic and dynamic properties, its visual appearance, and its collision characteristics.

*   **Links:** Represent rigid bodies of the robot (e.g., chassis, wheel, arm segment). Each link has properties such as:
    *   `<visual>`: Describes the visual appearance (e.g., geometry, color, mesh file).
    *   `<collision>`: Defines the collision geometry, which is often a simplified shape for faster physics calculations.
    *   `<inertial>`: Specifies the mass, center of mass, and inertia tensor, critical for accurate dynamic simulation.
*   **Joints:** Define the kinematic and dynamic connection between two links (a `parent` and a `child`). Key joint properties include:
    *   `type`: (e.g., `continuous`, `revolute`, `prismatic`, `fixed`).
    *   `origin`: The transform from the parent link to the child link.
    *   `axis`: The axis of rotation or translation for the joint.
    *   `limit`: For revolute/prismatic joints, defines the range of motion and velocity/effort limits.

#### Example: A Simple Two-Wheeled Robot in URDF

Let's examine a simplified URDF for a differential drive robot to illustrate these concepts:

```xml
<!-- A simple two-wheeled robot URDF -->
<robot name="simple_diff_drive_robot">

  <!-- Base Link (Chassis of the robot) -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.4 0.2 0.1"/> <!-- Length, Width, Height -->
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/> <!-- Blue color -->
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.4 0.2 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <!-- Mass of the base link -->
      <mass value="5.0"/>
      <!-- Center of mass (relative to link origin) -->
      <origin xyz="0 0 0"/>
      <!-- Inertia tensor - crucial for physics simulation -->
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Right Wheel Link -->
  <link name="right_wheel_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.04"/> <!-- Radius, Length -->
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Joint connecting Base to Right Wheel -->
  <joint name="base_to_right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel_link"/>
    <!-- Position and orientation relative to parent link -->
    <origin xyz="0 -0.12 0" rpy="1.5707 0 0"/> <!-- Rotate to align cylinder axis -->
    <axis xyz="0 1 0"/> <!-- Axis of rotation (Y-axis) -->
  </joint>

  <!-- Left Wheel Link and Joint (similar structure, omitted for brevity) -->

</robot>
```
**`visual` vs. `collision`:** It is a best practice to define separate geometries for `visual` and `collision`. The `visual` geometry can be highly detailed (e.g., a complex mesh from Blender), while the `collision` geometry should be simplified (e.g., a primitive box or cylinder) to reduce the computational load on the physics engine.
**`inertial` properties:** Accurate `mass`, `origin` (center of mass), and `inertia` tensor are paramount for realistic dynamic simulation. Incorrect values can lead to unstable or physically implausible robot behavior.

### XACRO: Making URDF Manageable

For any robot beyond the simplest configurations, directly writing URDF becomes tedious and error-prone due to repetition. **XACRO (XML Macros)** is a preprocessor that allows you to use macros, constants, and basic arithmetic to generate URDF from a more concise and maintainable source.

#### The Advantages of XACRO:
*   **Modularity:** Break down a complex robot into smaller, reusable components (e.g., define a `wheel` macro once, instantiate it multiple times).
*   **Parameterization:** Use variables for dimensions, masses, etc., making it easy to adjust robot parameters without searching through the entire file.
*   **Readability:** Reduces redundancy and improves the clarity of the robot description.
*   **Inclusion:** Allows you to include other XACRO files, promoting reuse across different robot projects.

#### Example: Refactoring with XACRO

Let's refactor the previous URDF example using XACRO:

```xml
<?xml version="1.0"?>
<!-- simple_bot.urdf.xacro -->
<robot name="simple_diff_drive_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- 1. Define Constants and Materials -->
  <xacro:property name="M_PI" value="3.141592653589793" />
  <xacro:property name="base_length" value="0.4" />
  <xacro:property name="base_width" value="0.2" />
  <xacro:property name="base_height" value="0.1" />
  <xacro:property name="base_mass" value="5.0" />

  <xacro:property name="wheel_radius" value="0.05" />
  <xacro:property name="wheel_width" value="0.04" />
  <xacro:property name="wheel_mass" value="0.5" />
  <xacro:property name="wheel_offset_y" value="${base_width/2 + wheel_width/2}" /> <!-- Position wheels slightly outside base -->

  <material name="blue">
    <color rgba="0 0 0.8 1"/>
  </material>
  <material name="black">
    <color rgba="0 0 0 1"/>
  </material>

  <!-- 2. Define a Macro for an Inertial Element (reusable) -->
  <xacro:macro name="default_inertial" params="mass">
    <inertial>
      <mass value="${mass}"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="${(mass / 12.0) * (base_height*base_height + base_width*base_width)}" ixy="0.0" ixz="0.0"
               iyy="${(mass / 12.0) * (base_length*base_length + base_height*base_height)}" iyz="0.0"
               izz="${(mass / 12.0) * (base_length*base_length + base_width*base_width)}"/>
    </inertial>
  </xacro:macro>

  <!-- 3. Base Link -->
  <link name="base_link">
    <visual>
      <geometry><box size="${base_length} ${base_width} ${base_height}"/></geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry><box size="${base_length} ${base_width} ${base_height}"/></geometry>
    </collision>
    <xacro:default_inertial mass="${base_mass}"/> <!-- Use the inertial macro -->
  </link>

  <!-- 4. Define a Macro for a Wheel (Link + Joint) -->
  <xacro:macro name="diff_drive_wheel" params="prefix parent_link x_offset y_offset z_offset">
    <link name="${prefix}_wheel_link">
      <visual>
        <origin xyz="0 0 0" rpy="${M_PI/2} 0 0"/> <!-- Rotate visual to align cylinder -->
        <geometry><cylinder radius="${wheel_radius}" length="${wheel_width}"/></geometry>
        <material name="black"/>
      </visual>
      <collision>
        <origin xyz="0 0 0" rpy="${M_PI/2} 0 0"/>
        <geometry><cylinder radius="${wheel_radius}" length="${wheel_width}"/></geometry>
      </collision>
      <xacro:default_inertial mass="${wheel_mass}"/>
    </link>

    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="${parent_link}"/>
      <child link="${prefix}_wheel_link"/>
      <origin xyz="${x_offset} ${y_offset} ${z_offset}" rpy="0 0 0"/>
      <axis xyz="0 1 0"/> <!-- Wheel rotates around its own Y axis -->
    </joint>
  </xacro:macro>

  <!-- 5. Instantiate Wheel Macros -->
  <xacro:diff_drive_wheel prefix="right" parent_link="base_link" x_offset="0" y_offset="${-wheel_offset_y}" z_offset="0"/>
  <xacro:diff_drive_wheel prefix="left" parent_link="base_link" x_offset="0" y_offset="${wheel_offset_y}" z_offset="0"/>

</robot>
```

Before this `.xacro` file can be used by tools that expect URDF (like the `robot_state_publisher` or `spawn_entity` in Gazebo), it must be processed by the `xacro` parser. This is typically done using the `xacro` command-line tool:

```bash
ros2 run xacro xacro simple_bot.urdf.xacro > simple_bot.urdf
```
This command expands all macros, evaluates properties, and outputs a standard URDF file.
The use of XACRO significantly streamlines the process of defining complex robots, making the descriptions more modular, readable, and less error-prone. It is an indispensable tool for serious ROS 2 development.
