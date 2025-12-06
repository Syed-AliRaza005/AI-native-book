---
title: "4. Advanced Launch Files, Parameter Management, and System Orchestration"
---

## Advanced Launch Files, Parameter Management, and System Orchestration

As robotic systems grow in complexity, managing the startup and configuration of numerous interdependent nodes becomes a significant challenge. **Launch files** in ROS 2 provide a powerful, programmatic solution for orchestrating the entire system. They allow you to define what executables to run, how to configure them, and how they should interact, all from a single entry point.

### 4.1 Python-based Launch Files: The Power of Programmatic Control

ROS 2 launch files are written in Python. This is a deliberate design choice that offers immense flexibility compared to the XML-based launch files of ROS 1. With Python, you gain:

*   **Conditional Logic:** Start nodes based on arguments, environmental variables, or other conditions.
*   **Looping:** Launch multiple similar nodes with varying configurations (e.g., multiple camera drivers).
*   **External Integration:** Load parameters from YAML files, integrate with external scripts, or query system information dynamically.

#### Structure of a Python Launch File

A ROS 2 Python launch file typically defines a `generate_launch_description()` function that returns a `LaunchDescription` object. This object is a container for various actions that describe how your system should start.

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution

def generate_launch_description():
    # 1. Declare Launch Arguments (optional parameters for the launch file)
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='my_robot',
        description='Name of the robot to launch'
    )

    # 2. Load Configuration Files (e.g., YAML parameters) - Explained in detail below
    params_file = os.path.join(
        get_package_share_directory('my_package'),
        'config',
        'robot_params.yaml'
    )

    # 3. Define Node Actions
    # A simple node, passing parameters
    sensor_node = Node(
        package='my_package',
        executable='sensor_publisher',
        name='my_sensor_node',
        parameters=[params_file, {'frame_id': LaunchConfiguration('robot_name')}], # Example of parameters
        output='screen' # Directs node output to the console
    )

    # A node with topic remapping
    controller_node = Node(
        package='my_package',
        executable='motor_controller',
        name='motor_controller_node',
        remappings=[
            ('/cmd_vel', '/{}/cmd_vel'.format(LaunchConfiguration('robot_name'))), # Remap cmd_vel topic
            ('/odom', '/{}/odom'.format(LaunchConfiguration('robot_name'))),       # Remap odometry
        ],
        arguments=['--ros-args', '--log-level', 'info'] # Pass command-line arguments
    )

    # 4. Include other launch files (for modularity)
    another_subsystem_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('another_package'), 'launch', 'subsystem.launch.py')
        ]),
        launch_arguments={'enable_logging': 'true'}.items()
    )

    # 5. Group actions (e.g., for applying specific parameters to a set of nodes)
    robot_group = GroupAction([
        Node(package='robot_state_publisher', executable='robot_state_publisher', name='rsp'),
        Node(package='joint_state_publisher', executable='joint_state_publisher', name='jsp')
    ])

    return LaunchDescription([
        robot_name_arg,
        sensor_node,
        controller_node,
        another_subsystem_launch,
        robot_group
    ])
```

To run this launch file, you would use the command:
`ros2 launch my_package my_robot.launch.py robot_name:=my_fancy_robot`

### 4.2 Parameter Management in Launch Files

Parameters in ROS 2 are critical for configuring node behavior without modifying source code. Launch files provide robust mechanisms for declaring, loading, and overriding these parameters.

#### 1. Declaring Parameters

Parameters can be defined directly within the launch file or loaded from external YAML files.

**a) Direct Declaration (Python Launch File):**

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='my_node',
            name='my_node_instance',
            parameters=[
                {'my_parameter': 'value'},
                {'another_parameter': 123},
            ]
        ),
    ])
```

**b) Loading from YAML File (Python Launch File):**
This is a common and recommended approach for managing a larger set of parameters.

First, create a YAML file (e.g., `config/my_params.yaml`):
```yaml
my_node_instance:
  ros__parameters:
    my_parameter: "value_from_yaml"
    another_parameter: 456
    nested:
      param: true
```

Then, in your Python launch file:
```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to your config file
    config_dir = os.path.join(
        get_package_share_directory('my_package'),
        'config'
    )
    param_file = os.path.join(config_dir, 'my_params.yaml')

    return LaunchDescription([
        Node(
            package='my_package',
            executable='my_node',
            name='my_node_instance',
            parameters=[param_file] # Pass the path to the YAML file
        ),
    ])
```
You can also pass multiple YAML files or a mix of YAML files and dictionaries to the `parameters` argument. Later parameters in the list will override earlier ones if keys conflict.

**c) XML Launch File (using `<param>` and `<param from="">`):**

```xml
<launch>
    <node pkg="my_package" exec="my_node" name="my_node_instance">
        <!-- Direct parameter declaration -->
        <param name="my_parameter" value="value" type="str"/>
        <param name="another_parameter" value="123" type="int"/>

        <!-- Load parameters from a YAML file -->
        <param from="$(find-pkg-share my_package)/config/my_params.yaml"/>
    </node>
</launch>
```

#### 2. Parameter Scope and Namespaces

Parameters can be defined at different scopes:

*   **Node-specific:** Parameters declared directly within a `Node` or loaded for a specific node are scoped to that node.
*   **Global/Group:** Parameters can be applied to a group of nodes using `<group>` in XML or `GroupAction` in Python. They can also be set globally for all nodes launched in a description (less common for specific parameters, more for general settings).

**Example with Group (Python Launch File):**

```python
from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node

def generate_launch_description():
    my_node_parameters = [
        {'shared_param': 'group_value'}
    ]

    return LaunchDescription([
        GroupAction(
            actions=[
                Node(
                    package='my_package',
                    executable='node_a',
                    name='node_a_instance',
                    parameters=my_node_parameters
                ),
                Node(
                    package='my_package',
                    executable='node_b',
                    name='node_b_instance',
                    parameters=my_node_parameters
                ),
            ]
        )
    ])
```

#### 3. Overriding Parameters

ROS 2 provides mechanisms to override parameters, which is crucial for testing and deployment in different environments.

**a) Command-line Overrides:**
You can override parameters when running a launch file directly from the command line:

```bash
ros2 launch my_package my_launch_file.launch.py my_node_instance.my_parameter:=new_value
```
Or for nodes loaded from YAML:
```bash
ros2 launch my_package my_launch_file.launch.py my_node_instance.ros__parameters.my_parameter:=new_value
```

**b) Parameter Reclaring in Launch Files:**
If you declare a parameter multiple times within the same launch context, the last declaration takes precedence. This is why YAML files passed later in the `parameters` list can override earlier ones.

#### 4. Special Parameters

*   **`use_sim_time`**: A common boolean parameter (`true`/`false`) that tells nodes to use the `/clock` topic for time instead of system time. Often set globally.
*   **`ros__parameters` prefix in YAML**: When loading parameters from YAML files, ROS 2 expects the node's parameters to be under a `ros__parameters` key if the YAML file might contain other configuration for the node. If the YAML only contains parameters, this prefix is optional. However, it's good practice to include it for clarity and consistency.

#### Best Practices:

*   **External YAML for complex configurations:** For nodes with many parameters, always use external YAML files for better organization and readability.
*   **`get_package_share_directory`:** Use this function in Python launch files to create package-agnostic paths to your configuration files.
*   **Namespacing:** Use namespaces for nodes and parameters, especially in complex systems, to avoid conflicts and improve clarity.
*   **Clear documentation:** Document your parameters, their purpose, valid ranges, and default values.

### 4.3 Event Handlers: Building Resilient Systems

Python launch files enable the creation of robust, self-healing robotic systems through **Event Handlers**. These allow your launch system to react dynamically to events that occur during the execution of your nodes.

```python
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart, OnShutdown
from launch.actions import LogInfo

# ... inside generate_launch_description()
my_sensor_node = Node(package='my_package', executable='sensor_publisher', name='my_sensor_node')

ld.add_action(my_sensor_node)

# When my_sensor_node exits, log a message
ld.add_action(RegisterEventHandler(
    event_handler=OnProcessExit(
        target_action=my_sensor_node,
        on_exit=[
            LogInfo(msg='Sensor node has exited!'),
            # You could add actions here to restart the node, or trigger a system shutdown
            # ExecuteProcess(cmd=['ros2', 'launch', 'my_package', 'recovery.launch.py']),
        ]
    )
))

# When the entire launch system is shutting down, perform cleanup
ld.add_action(RegisterEventHandler(
    event_handler=OnShutdown(
        on_shutdown=[
            LogInfo(msg='ROS 2 system is shutting down. Performing cleanup...'),
            ExecuteProcess(cmd=['python3', 'cleanup_script.py'])
        ]
    )
))
```

### 4.4 Composable Nodes and Node Containers: Performance Optimization

For applications demanding high data throughput and low latency, inter-process communication overhead can be a significant bottleneck. ROS 2's **composable nodes** and **node containers** provide an elegant solution by enabling **intra-process communication (IPC)**.

*   **Composable Nodes:** Instead of writing an independent executable for each ROS node, you write your node as a C++ class that adheres to the `rclcpp::Node` interface (or Python equivalent components). These classes can then be dynamically loaded.
*   **Node Containers:** A node container is a special ROS 2 executable (`ros2 run rclcpp_components component_container`) that acts as a single process. It can load multiple composable nodes into its own address space.

![Diagram: A comparison showing two architectural patterns. On the left: three distinct ROS nodes (e.g., Camera, ImageProcessor, ObjectDetector) running as separate processes, communicating via DDS (with arrows showing data flow). On the right: A single process labeled "Component Container" with the three components (CameraComponent, ImageProcessorComponent, ObjectDetectorComponent) inside it, with arrows showing direct in-process memory sharing.](/img/ALf.png)

When multiple composable nodes are loaded into the same node container, their communication occurs directly via shared memory pointers. This bypasses the entire DDS serialization/deserialization and network stack, resulting in **zero-copy data transfer** and **near-zero latency**. This is a critical optimization technique for building high-performance, real-time perception and control pipelines.
