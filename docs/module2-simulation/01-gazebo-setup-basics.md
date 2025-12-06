---
title: "1. Gazebo Simulation Environment Setup"
---

## 1. Gazebo Simulation Environment Setup with ROS 2

Setting up a Gazebo simulation environment with ROS 2 is a crucial step for developing and testing robotic applications without the need for physical hardware. This section will guide you through the process, covering installation, workspace creation, and launching a basic simulation.

### 1.1 Understanding the Core Components

Before diving into the setup, it's essential to understand the roles of the main components:

*   **ROS 2 (Robot Operating System 2):** This is your robotic middleware, providing a collection of tools, libraries, and conventions that simplify the task of creating complex robot behaviors. Your robot's "brain" (nodes) will run on ROS 2.
*   **Gazebo:** A powerful 3D robot simulator. Gazebo accurately simulates rigid body dynamics, sensors (like cameras, LiDAR, IMUs), and environmental conditions. It allows you to develop and test algorithms in a realistic virtual world.
*   **`ros_gz` (ROS Gazebo Bridge):** This critical package provides the necessary interfaces to enable communication between your ROS 2 nodes and the Gazebo simulation. It translates messages between ROS 2 topics/services and Gazebo topics, allowing your ROS 2 controllers to send commands to the simulated robot and receive sensor data back.

### 1.2 Prerequisites

To ensure a smooth installation process, make sure your system meets the following requirements:

*   **Operating System:** **Ubuntu 22.04 LTS (Jammy Jellyfish)** is highly recommended, as it offers the best support for recent ROS 2 distributions and Gazebo versions. Other Linux distributions might work but may require additional troubleshooting.
*   **ROS 2 Distribution:** This guide assumes you are using **ROS 2 Humble Hawksbill** for Ubuntu 22.04. If you are using a different ROS 2 distribution (e.g., Iron Irwini), adjust the installation commands accordingly.

### 1.3 Installation Steps

#### a. Install ROS 2 Humble Hawksbill

If you haven't already, install ROS 2 Humble by following the official documentation. Here's a condensed version for Ubuntu 22.04:

```bash
# Update and install locales
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble Desktop (includes Gazebo Classic)
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop

# Source ROS 2 setup script automatically
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
**Important:** After installation, open a new terminal or run `source ~/.bashrc` to ensure your environment variables are set correctly.

#### b. Install Gazebo Classic and ROS Gazebo Packages

For ROS 2 Humble, **Gazebo Classic (version 11)** is the well-supported simulator. The `ros-humble-gazebo-ros-pkgs` metapackage includes `gazebo_ros` which provides the necessary integration.

```bash
sudo apt update
sudo apt install gazebo11 ros-humble-gazebo-ros-pkgs
```
This command installs Gazebo Classic (version 11) and the ROS 2 packages required for its integration with Humble.

### 1.4 Create a ROS 2 Workspace

A ROS 2 workspace is a collection of ROS 2 packages that you are developing or modifying.

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/
colcon build
# Add workspace sourcing to your bashrc for convenience
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
**Important:** Always remember to `source install/setup.bash` (or your shell's equivalent) in *each new terminal* where you want to use packages from your workspace.

### 1.5 Create a Gazebo Package for Your Robot and World

Now, let's create a dedicated ROS 2 package to store your robot's description (URDF/SDF), Gazebo world files, and launch files for your simulations.

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake my_robot_description
```

#### Example: A Simple Robot (URDF/XACRO)

Create a `urdf` directory inside `my_robot_description` and add a basic robot description, e.g., `my_robot_description/urdf/my_robot.urdf.xacro`:

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.2 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.2 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>
</robot>
```

#### Example: An Empty Gazebo World

Create a `worlds` directory inside `my_robot_description` and add a simple world file, e.g., `my_robot_description/worlds/empty.world`:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="default">
    <light name="sun" type="directional">
      <cast_shadows>1</cast_shadows>
      <pose>0 0 10 0 -0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1.0</mu>
                <mu2>1.0</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

#### Example: Launching Your Robot in Gazebo

Create a `launch` directory inside `my_robot_description` and add a launch file, e.g., `my_robot_description/launch/my_robot_spawn.launch.py`:

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'my_robot_description'
    pkg_share_dir = get_package_share_directory(pkg_name)
    urdf_file = os.path.join(pkg_share_dir, 'urdf', 'my_robot.urdf.xacro')
    world_file = os.path.join(pkg_share_dir, 'worlds', 'empty.world')

    # Start Gazebo sim
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_file}.items(),
    )

    # Robot State Publisher reads the URDF and publishes the robot's joint states
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_file, 'r').read()}],
    )

    # Spawn entity in Gazebo
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-entity', 'my_robot',
                                   '-topic', 'robot_description',
                                   '-x', '0.0', '-y', '0.0', '-z', '0.1'], # Initial position
                        output='screen')

    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher_node,
        spawn_entity
    ])
```

#### 1.5.1 Update `CMakeLists.txt` and `package.xml`

To ensure your URDF, world, and launch files are installed correctly, you need to modify `my_robot_description/CMakeLists.txt` and `my_robot_description/package.xml`.

**`my_robot_description/CMakeLists.txt` (add installation rules):**

```cmake
# ... (existing CMake code)

install(
  DIRECTORY urdf launch worlds
  DESTINATION share/${PROJECT_NAME}
)
```

**`my_robot_description/package.xml` (add dependencies):**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_description</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="user@example.com">Your Name</maintainer>
  <license>TODO: License declaration</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclpy</depend>
  <depend>gazebo_ros</depend>
  <depend>robot_state_publisher</depend>
  <depend>xacro</depend>
  
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### 1.6 Build Your Workspace

Navigate back to the root of your ROS 2 workspace (`~/ros2_ws`) and build your new package:

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_description
source install/setup.bash # Re-source after building new packages
```
It's crucial to re-source your `setup.bash` (or equivalent) after building any new packages to make them visible to your ROS 2 environment.

### 1.7 Launch Your Simulation

Finally, you can launch your robot in Gazebo:

```bash
ros2 launch my_robot_description my_robot_spawn.launch.py
```
This command should open Gazebo with your simple box robot spawned in the empty world.

### 1.8 Basic ROS 2 / Gazebo Communication

Once your robot is in Gazebo, you can start interacting with it using ROS 2. The `ros_gz` bridge (which `gazebo_ros` utilizes) handles the communication under the hood.

*   **List Gazebo topics:** In a separate terminal (after sourcing ROS 2), you can list topics published directly by Gazebo (if `ros_gz_sim` is installed and sourced):
    ```bash
    gz topic list
    ```
*   **List ROS 2 topics:** In another terminal, you can see the corresponding ROS 2 topics:
    ```bash
    ros2 topic list
    ```
*   **Bridge Gazebo to ROS 2 (Manual Example):** For custom Gazebo topics that aren't automatically bridged, you might need to use the `ros_gz_bridge` manually. For example, to bridge a Gazebo clock topic to a ROS 2 clock topic:
    ```bash
    ros2 run ros_gz_bridge bridge /clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock
    ```
    Typically, for a fully integrated robot, these bridges are configured within your launch files or through dedicated bridge configuration files.

This comprehensive guide should provide a solid foundation for setting up and understanding Gazebo simulations with ROS 2.
