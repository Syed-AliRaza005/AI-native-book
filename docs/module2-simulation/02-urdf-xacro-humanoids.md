---
title: "2. URDF and XACRO for Humanoid Robots"
---

## 2. URDF and XACRO for Humanoid Robots in ROS 2

Describing your robot accurately is the first critical step towards simulating and controlling it. In ROS 2, the **Unified Robot Description Format (URDF)** and its extension, **XACRO (XML Macros)**, are the standard tools for this task. For humanoid robots, which have many degrees of freedom and complex geometries, XACRO becomes indispensable for managing complexity.

### 2.1 Understanding URDF and XACRO

*   **URDF:** An XML-based file format used to describe all aspects of a robot. It defines the robot's kinematic and dynamic properties, visual appearance, and collision geometry.
    *   **Links:** Represent the rigid parts of the robot (e.g., torso, head, upper arm, hand).
    *   **Joints:** Define the connections between links, specifying their type (revolute, prismatic, fixed), axis of rotation/translation, and limits.
*   **XACRO:** An XML macro language that allows you to write more modular, readable, and parameterizable URDF files. XACRO uses macros, properties, and mathematical expressions to avoid repetition and simplify complex robot descriptions. You'll write `.xacro` files that are processed into a standard `.urdf` file before use.

### 2.2 Designing Your Humanoid Robot (Conceptualization)

Before writing any code, it's beneficial to conceptually design your humanoid robot's structure:

1.  **Identify Links:** Break down the human body into rigid segments (e.g., pelvis, chest, head, upper_arm, forearm, hand, thigh, calf, foot).
2.  **Define Joints:** Determine how these links connect. For a humanoid, you'll need many joints (e.g., neck_pitch, shoulder_roll, elbow_flex, hip_yaw, knee_flex, ankle_pitch). For each joint, consider:
    *   **Type:** `revolute` (rotating), `prismatic` (sliding), `fixed` (rigid connection).
    *   **Origin:** The position and orientation of the joint relative to its parent link.
    *   **Axis:** The axis of rotation for revolute joints.
    *   **Limits:** The minimum and maximum range of motion.
3.  **Mass Properties:** Estimate the mass, inertia matrix, and center of mass for each link. These are crucial for realistic physics simulation in Gazebo.
4.  **Visuals & Collisions:**
    *   **Visuals:** How the robot *looks*. Often uses 3D mesh files (`.stl`, `.dae`, `.obj`).
    *   **Collisions:** How the robot *interacts physically* with the environment. Often uses simplified mesh files or primitive shapes (box, cylinder, sphere) to improve simulation performance.

### 2.3 Creating XACRO Files: A Modular Approach

For humanoid robots, organizing your XACRO files into a main robot description and several component files (one per body part or common utility) is highly recommended.

Let's assume you have a ROS 2 package named `humanoid_description` in your workspace. Your XACRO files will reside in `humanoid_description/urdf/`.

#### `humanoid_description/urdf/humanoid.xacro` (Main Robot Description)

This file acts as the central hub, including other XACRO files and defining the overall structure.

```xml
<?xml version="1.0"?>
<robot name="humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">

    <!-- Import common macros and properties -->
    <xacro:include filename="$(find humanoid_description)/urdf/common_macros.xacro" />
    <xacro:include filename="$(find humanoid_description)/urdf/materials.xacro" />
    <xacro:include filename="$(find humanoid_description)/urdf/properties.xacro" /> <!-- For mass/inertia props -->

    <!-- Define the root link of the robot -->
    <link name="base_link">
        <visual>
            <geometry>
                <box size="0.05 0.05 0.05"/>
            </geometry>
            <material name="black"/>
        </visual>
        <collision>
            <geometry>
                <box size="0.05 0.05 0.05"/>
            </geometry>
        </collision>
        <inertial>
            <mass value="0.01"/>
            <origin xyz="0 0 0"/>
            <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
        </inertial>
    </link>

    <!-- Include body parts -->
    <xacro:include filename="$(find humanoid_description)/urdf/torso.xacro" />
    <xacro:include filename="$(find humanoid_description)/urdf/head.xacro" />
    <xacro:include filename="$(find humanoid_description)/urdf/left_arm.xacro" />
    <xacro:include filename="$(find humanoid_description)/urdf/right_arm.xacro" />
    <xacro:include filename="$(find humanoid_description)/urdf/left_leg.xacro" />
    <xacro:include filename="$(find humanoid_description)/urdf/right_leg.xacro" />

    <!-- Gazebo specific settings -->
    <xacro:include filename="$(find humanoid_description)/urdf/humanoid.gazebo" />

</robot>
```

#### `humanoid_description/urdf/common_macros.xacro` (Reusable Utilities)

This file can contain macros for commonly used elements, like default inertial properties.

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

    <xacro:macro name="default_inertial" params="mass ixx iyy izz">
        <inertial>
            <mass value="${mass}" />
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <inertia ixx="${ixx}" ixy="0.0" ixz="0.0" iyy="${iyy}" iyz="0.0" izz="${izz}" />
        </inertial>
    </xacro:macro>

</robot>
```

#### `humanoid_description/urdf/materials.xacro` (Color Definitions)

Define reusable material colors.

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
    <material name="white"> <color rgba="1 1 1 1"/> </material>
    <material name="black"> <color rgba="0 0 0 1"/> </material>
    <material name="blue"> <color rgba="0 0 1 1"/> </material>
    <material name="red"> <color rgba="1 0 0 1"/> </material>
</robot>
```

#### `humanoid_description/urdf/properties.xacro` (Robot-specific Parameters)

Define properties like link lengths, masses, or joint limits as XACRO properties for easy modification.

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
    <xacro:property name="torso_length" value="0.3" />
    <xacro:property name="torso_radius" value="0.08" />
    <xacro:property name="torso_mass" value="5.0" />

    <xacro:property name="upper_arm_length" value="0.2" />
    <xacro:property name="upper_arm_radius" value="0.03" />
    <xacro:property name="upper_arm_mass" value="1.0" />
</robot>
```

#### Example: `humanoid_description/urdf/torso.xacro` (A Body Part Definition)

This is where you define the specific links and joints for a body segment.

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

    <!-- Torso Link -->
    <link name="torso_link">
        <visual>
            <geometry>
                <cylinder radius="${torso_radius}" length="${torso_length}"/>
            </geometry>
            <material name="blue"/>
        </visual>
        <collision>
            <geometry>
                <cylinder radius="${torso_radius}" length="${torso_length}"/>
            </geometry>
        </collision>
        <xacro:default_inertial mass="${torso_mass}" ixx="0.01" iyy="0.01" izz="0.01"/>
    </link>

    <!-- Joint connecting base_link to torso_link -->
    <joint name="torso_joint" type="fixed">
        <parent link="base_link"/>
        <child link="torso_link"/>
        <origin xyz="0 0 ${torso_length/2 + 0.025}" rpy="0 0 0"/> <!-- Position above base_link -->
    </joint>

    <!-- Attach other body parts (e.g., head, arms) to torso_link -->
    <!-- Example: Neck Joint and Head Link (would be in head.xacro) -->
    <!-- <joint name="neck_joint" type="revolute"> ... </joint> -->
    <!-- <link name="head_link"> ... </link> -->

</robot>
```

Repeat this modular approach for `head.xacro`, `left_arm.xacro`, `right_arm.xacro`, `left_leg.xacro`, and `right_leg.xacro`, ensuring each part correctly connects to its parent link with an appropriate joint.

### 2.4 Adding Visuals and Collisions with Meshes

For realistic humanoid robot models, 3D mesh files (e.g., `.stl`, `.dae`, `.obj`) are typically used for the `visual` and `collision` elements.

*   Create a `humanoid_description/meshes` directory to store your mesh files.
*   Update your XACRO files to reference these meshes:

    ```xml
    <visual>
        <geometry>
            <!-- Use package:// for relative paths to package resources -->
            <mesh filename="package://humanoid_description/meshes/torso_visual.stl" scale="1 1 1"/>
        </geometry>
        <material name="blue"/>
    </visual>
    <collision>
        <geometry>
            <!-- Collision meshes are often simplified for better simulation performance -->
            <mesh filename="package://humanoid_description/meshes/torso_collision.stl" scale="1 1 1"/>
        </geometry>
    </collision>
    ```
    **Tip:** Ensure your mesh files are correctly scaled and oriented during creation. Simplified collision meshes are crucial for improving simulation performance and stability.

### 2.5 Integrating Gazebo Elements (Plugins and Controllers)

For your URDF/XACRO description to be fully functional in Gazebo, you often need to add Gazebo-specific elements using `<gazebo>` tags. These tags are usually placed within the `humanoid.xacro` file or a dedicated `humanoid.gazebo` XACRO file.

#### `humanoid_description/urdf/humanoid.gazebo` (Gazebo-specific additions)

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

    <!-- Gazebo reference for each link (optional but good practice) -->
    <gazebo reference="base_link">
        <material>Gazebo/Black</material>
    </gazebo>
    <gazebo reference="torso_link">
        <material>Gazebo/Blue</material>
    </gazebo>
    <!-- Add more for other links -->

    <!-- Example: Adding a Gazebo plugin for a differential drive robot (for wheeled base) -->
    <!-- For humanoids, you'll use ros2_control plugins for joint control -->
    <!--
    <gazebo>
        <plugin name="gazebo_ros_control" filename="libgazebo_ros2_control.so">
            <robot_sim_type>gazebo_ros2_control/GazeboSystem</robot_sim_type>
            <parameters>$(find my_robot_controller)/config/robot_controllers.yaml</parameters>
        </plugin>
    </gazebo>
    -->

</robot>
```

### 2.6 ROS 2 Control Integration: Bridging Simulation and Hardware

For humanoid robots, **`ros2_control`** is the recommended framework for managing joint actuators, sensors, and controllers. It provides a standardized interface that works seamlessly with both simulated robots (in Gazebo) and real hardware.

The process typically involves:

1.  **Adding `<ros2_control>` tags to your XACRO:** These tags define the hardware interfaces (e.g., position, velocity, effort) for each joint.
2.  **Creating Controller Configuration Files (YAML):** These YAML files define the specific controllers (e.g., `JointTrajectoryController`, `JointStateBroadcaster`) and their gains.
3.  **Writing Launch Files:** To load the `ros2_control` hardware interface and start the controllers in both simulation and on real hardware.

This unified approach allows you to develop and test your control algorithms in Gazebo and then deploy them directly to your physical humanoid robot with minimal changes.

### 2.7 Building and Visualizing Your Humanoid

#### 1. Update `humanoid_description/package.xml`

Ensure your `package.xml` includes necessary dependencies for XACRO processing, `robot_state_publisher`, `joint_state_publisher_gui` (for visualization), and Gazebo integration.

```xml
<?xml version="1.0"?>
<package format="3">
  <name>humanoid_description</name>
  <version>0.0.0</version>
  <description>URDF and XACRO description for a humanoid robot</description>
  <maintainer email="user@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>ament_cmake_auto</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>urdf</depend>
  <depend>xacro</depend>
  <depend>robot_state_publisher</depend>
  <depend>joint_state_publisher_gui</depend>
  <depend>rviz2</depend>
  <depend>gazebo_ros</depend>
  <depend>ros2_control</depend>
  <depend>ros2_controllers</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

#### 2. Update `humanoid_description/CMakeLists.txt`

Add installation rules for your URDF, mesh, and launch files.

```cmake
# ... (existing CMake code)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_cmake_export_dependencies()

install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_PROJECT_NAME}
)
install(
  DIRECTORY meshes
  DESTINATION share/${PROJECT_PROJECT_NAME}
)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_PROJECT_NAME}
)

ament_auto_package()
```

#### 3. Build Your Workspace

Navigate to your workspace root (`~/ros2_humanoid_ws`) and build your package:

```bash
cd ~/ros2_humanoid_ws
colcon build --packages-select humanoid_description
source install/setup.bash # Re-source after building new packages
```

#### 4. Visualize in RViz2

Use a launch file to display your robot model in RViz2, which is excellent for verifying your URDF/XACRO definition.

*   Create `humanoid_description/launch/display.launch.py`:

    ```python
    import os
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument
    from launch.substitutions import LaunchConfiguration, Command
    from launch_ros.actions import Node

    def generate_launch_description():
        pkg_path = get_package_share_directory('humanoid_description')
        xacro_file_path = os.path.join(pkg_path, 'urdf', 'humanoid.xacro')

        # Use xacro to process the XACRO file into URDF
        robot_description_content = Command(['xacro', ' ', xacro_file_path])
        
        # Robot State Publisher Node
        robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}],
        )

        # Joint State Publisher GUI Node (for manipulating joints with sliders)
        joint_state_publisher_gui_node = Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        )

        # RViz2 Node
        rviz_config_path = os.path.join(pkg_path, 'rviz', 'display.rviz')
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
        )

        return LaunchDescription([
            robot_state_publisher_node,
            joint_state_publisher_gui_node, # Optional, useful for debugging joint limits
            rviz_node
        ])
    ```
    Remember to create a basic `humanoid_description/rviz/display.rviz` by running `rviz2`, adding a `RobotModel` display, setting its `Description Topic` to `/robot_description`, and saving the configuration.

#### 5. Simulate in Gazebo

Once your URDF/XACRO is defined, you can spawn your humanoid in Gazebo. This typically involves:

1.  **Gazebo-specific plugins:** Add `gazebo` tags to your XACRO for things like sensor plugins (e.g., cameras, IMUs), and importantly, `libgazebo_ros2_control.so` for hardware-agnostic control.
2.  **`ros2_control` setup:** Configure `ros2_control` to manage the joints. This involves defining `hardware_interfaces` and `joint_limits` in your XACRO, and creating controller configuration files (YAML).
3.  **Launch files:** Create a launch file to:
    *   Start Gazebo.
    *   Spawn your robot model.
    *   Load the `ros2_control` framework.
    *   Start your joint controllers.

This process allows you to perform realistic physics simulations, test control algorithms, and gather sensor data in a virtual environment before deploying to expensive physical hardware.
