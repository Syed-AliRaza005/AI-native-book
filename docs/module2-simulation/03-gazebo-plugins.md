---
title: "3. Gazebo Plugins: Extending Simulation Capabilities for ROS 2 Humanoids"
---

## 3. Gazebo Plugins: Extending Simulation Capabilities for ROS 2 Humanoids

Gazebo's power in realistic robotic simulation stems from its flexible **plugin architecture**. Plugins are dynamically loaded shared libraries (typically written in C++) that allow you to extend Gazebo's core functionality, enabling deep customization of your virtual environment, models, and interactions. For ROS 2 humanoid robots, plugins are crucial for bridging the gap between simulation and the ROS 2 ecosystem.

### 3.1 The Role of Plugins in ROS 2 Robotics

Plugins serve as the interface between Gazebo's physics engine and your ROS 2 control and perception systems. They allow you to:

*   **Simulate Advanced Sensors:** Generate realistic data from cameras, LiDAR, IMUs, force/torque sensors, depth cameras, etc., and publish this data to ROS 2 topics.
*   **Implement Custom Actuator Control:** Read control commands (e.g., joint positions, velocities, efforts) from ROS 2 topics and apply them to your robot's joints or other actuators within the simulation.
*   **Bridge Gazebo and ROS 2:** Facilitate seamless communication, mapping Gazebo's internal messages and events to ROS 2 topics, services, and actions.
*   **Manipulate the Environment:** Create dynamic environmental interactions or custom physics behaviors.
*   **Integrate `ros2_control`:** Crucially, `libgazebo_ros2_control` plugin allows `ros2_control` to interface with the simulated robot, enabling hardware-agnostic controllers to work in both simulation and on physical hardware.

### 3.2 Types of Gazebo Plugins

Gazebo offers several categories of plugins, each designed to hook into a specific level of the simulation:

*   **World Plugins (`WorldPlugin`):** Affect the entire simulation world (e.g., custom gravity, weather effects, global event handling).
*   **Model Plugins (`ModelPlugin`):** Attach to a specific model (like your humanoid robot, an obstacle, or a gripper) to control its behavior or query its state. These are most commonly used for robot-specific logic.
*   **Sensor Plugins (`SensorPlugin`):** Attach directly to specific sensors within a model, extending their capabilities or simulating specialized sensor types. Many `gazebo_ros` packages provide pre-built sensor plugins.
*   **System Plugins (`SystemPlugin`):** Load when Gazebo starts and can interact with the simulator at a fundamental level (less common for robot-specific tasks).

For humanoid robot development, you will primarily work with **Model Plugins** (for robot control and ROS 2 integration) and **Sensor Plugins** (for detailed sensor simulation).

### 3.3 Developing a Gazebo-ROS 2 Plugin (C++)

Creating a custom plugin involves writing C++ code that interacts with both Gazebo's API and ROS 2's client libraries (`rclcpp`).

#### Step 1: Create a ROS 2 Package

Start by creating a standard ROS 2 `ament_cmake` package to house your plugin. This manages dependencies and compilation.

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake my_humanoid_gazebo_plugins --dependencies rclcpp gazebo_ros gazebo_dev
```
*   `rclcpp`: The C++ client library for ROS 2 communication.
*   `gazebo_ros`: Provides utilities and base classes for integrating ROS 2 with Gazebo.
*   `gazebo_dev`: Provides the necessary Gazebo headers and libraries for plugin development.

#### Step 2: Write the Plugin Code (C++)

Create a C++ file (e.g., `src/my_humanoid_control_plugin.cpp`) inside your new package. Your plugin class will inherit from `gazebo::ModelPlugin` (for model-level control).

```c++
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh> // For ConnectWorldUpdateBegin
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <functional> // For std::bind

namespace my_humanoid_plugins
{

class MyHumanoidControlPlugin : public gazebo::ModelPlugin
{
public:
    // Load method: Called when the plugin is loaded by Gazebo
    void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override
    {
        this->model_ = _model; // Store a pointer to the model this plugin is attached to

        // Initialize ROS 2 node (scoped to the plugin)
        // It's good practice to ensure the ROS 2 context is initialized
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
        this->node_ = rclcpp::Node::make_shared("humanoid_control_plugin_node");

        // Example: Get a joint by name
        this->main_joint_ = this->model_->GetJoint("main_joint_name"); // Replace with actual joint name
        if (!this->main_joint_) {
            RCLCPP_ERROR(this->node_->get_logger(), "Joint 'main_joint_name' not found in model '%s'.", _model->GetName().c_str());
            return;
        }

        // Example: Create a subscriber for joint commands
        this->joint_command_sub_ = this->node_->create_subscription<std_msgs::msg::Float64>(
            "/humanoid/main_joint_controller/command",
            10, // QoS history depth
            std::bind(&MyHumanoidControlPlugin::OnJointCommand, this, std::placeholders::_1)
        );

        // Connect to Gazebo's world update event for periodic updates
        this->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
            std::bind(&MyHumanoidControlPlugin::OnUpdate, this)
        );

        RCLCPP_INFO(this->node_->get_logger(), "MyHumanoidControlPlugin for model '%s' loaded!", _model->GetName().c_str());
    }

    // OnUpdate method: Called at the beginning of each simulation step
    void OnUpdate()
    {
        // Spin the ROS 2 node to process callbacks
        rclcpp::spin_some(this->node_);

        // Example: Apply control to the joint
        if (this->main_joint_) {
            // This is a simple position control. More complex controllers would use PID.
            // Note: Gazebo's SetPosition directly sets position, usually you apply forces/torques.
            this->main_joint_->SetPosition(0, this->commanded_position_);
            // Alternatively, apply a force/torque:
            // this->main_joint_->SetForce(0, 10.0); // Apply 10 Nm torque
        }

        // Example: Publish sensor data (e.g., joint state)
        // std_msgs::msg::Float64 joint_state_msg;
        // joint_state_msg.data = this->main_joint_->Position(0);
        // this->joint_state_pub_->publish(joint_state_msg);
    }

    // Callback for receiving joint commands
    void OnJointCommand(const std_msgs::msg::Float64::SharedPtr msg)
    {
        this->commanded_position_ = msg->data;
        RCLCPP_DEBUG(this->node_->get_logger(), "Received joint command: %f", this->commanded_position_);
    }

private:
    gazebo::physics::ModelPtr model_;
    gazebo::physics::JointPtr main_joint_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr joint_command_sub_;
    gazebo::event::ConnectionPtr update_connection_;
    double commanded_position_ = 0.0; // Desired joint position
};

// Register the plugin with Gazebo
GZ_REGISTER_MODEL_PLUGIN(MyHumanoidControlPlugin)

} // namespace my_humanoid_plugins
```

#### Step 3: Configure `CMakeLists.txt`

You need to tell CMake how to build your plugin and link it against Gazebo and ROS 2 libraries. In your `my_humanoid_gazebo_plugins/CMakeLists.txt`:

```cmake
# ... (existing CMake code)

find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED) # Provides useful Gazebo-ROS integration utilities
find_package(gazebo_dev REQUIRED) # Provides Gazebo's development headers and libraries
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED) # If you use std_msgs

# Add your plugin as a shared library
add_library(my_humanoid_control_plugin SHARED
  src/my_humanoid_control_plugin.cpp
)
# Make sure it compiles with C++17 or higher
target_compile_features(my_humanoid_control_plugin PUBLIC cxx_std_17)

# Link against necessary libraries
ament_target_dependencies(my_humanoid_control_plugin
  rclcpp
  std_msgs
  gazebo_ros
)

# Link against Gazebo libraries
target_link_libraries(my_humanoid_control_plugin
  ${GAZEBO_LIBRARIES}
)

# Install the compiled plugin
install(TARGETS
  my_humanoid_control_plugin
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_libraries(my_humanoid_control_plugin)
```

#### Step 4: Update `package.xml`

Ensure all necessary dependencies are declared in `my_humanoid_gazebo_plugins/package.xml`.

```xml
<?xml version="1.0"?>
<package format="3">
  <name>my_humanoid_gazebo_plugins</name>
  <version>0.0.0</version>
  <description>Gazebo plugins for humanoid robot control and simulation</description>
  <maintainer email="user@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>gazebo_ros</depend>
  <depend>gazebo_dev</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

#### Step 5: Modify Your Robot's URDF/SDF

To load your custom plugin, you must reference it within your robot's URDF (which is converted to SDF for Gazebo) or directly in an SDF file. This is done using a `<plugin>` tag.

```xml
<!-- In your humanoid's XACRO/URDF file, often within a <gazebo> tag that applies to the entire model -->
<gazebo>
    <plugin name="my_humanoid_control_plugin" filename="libmy_humanoid_control_plugin.so">
        <ros>
            <namespace>/my_robot</namespace> <!-- ROS 2 namespace for the plugin's internal node -->
            <argument>--ros-args --log-level INFO</argument> <!-- Example: pass ROS arguments -->
        </ros>
        <!-- Custom parameters for your plugin, if defined in its SDF parsing logic -->
        <update_rate>100.0</update_rate> <!-- Example: parameter to control update frequency -->
        <joint_name>main_joint_name</joint_name> <!-- Example: pass joint name to plugin -->
    </plugin>
</gazebo>
```
*   `filename`: Must match `lib<your_library_name>.so` from `add_library` in `CMakeLists.txt`.
*   `<ros><namespace>`: Defines the ROS 2 namespace for any nodes created by the plugin.

#### Step 6: Build and Run

1.  **Build Your Workspace:**
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select my_humanoid_gazebo_plugins
    ```
2.  **Source Your Workspace:**
    ```bash
    source install/setup.bash # Important after building new packages
    ```
3.  **Launch Gazebo:** Launch your robot in Gazebo using its launch file. Your plugin should now load automatically, and its ROS 2 node (e.g., `/my_robot/humanoid_control_plugin_node`) should appear in your ROS 2 graph (`ros2 node list`).

### 3.4 Key Concepts for Plugin Development

*   **ROS 2 and Gazebo Integration (through `gazebo_ros`):** The `gazebo_ros` package provides many useful base classes and utilities for common integration tasks, simplifying the creation of ROS-aware Gazebo plugins.
*   **`rclcpp::spin_some()` in `OnUpdate`:** When processing ROS 2 callbacks within the Gazebo update loop, use `rclcpp::spin_some()` in your plugin's `OnUpdate` method. This processes pending ROS 2 messages without blocking the simulation. Avoid `rclcpp::spin()` as it's blocking.
*   **URDF vs. SDF:** Remember that Gazebo internally uses SDF (Simulation Description Format). URDF files are typically converted to SDF before loading. Plugins are ultimately defined within the SDF structure.
*   **Joint and Link Manipulation:** Gazebo's `physics::ModelPtr`, `physics::JointPtr`, and `physics::LinkPtr` provide extensive APIs to read states (position, velocity, force) and apply commands (forces, torques, positions).
*   **Sensor Data Access:** Accessing simulated sensor data is done via `gazebo::sensors::SensorPtr` and its derived classes (e.g., `gazebo::sensors::CameraSensorPtr`).
*   **Debugging:** Utilize `RCLCPP_INFO`, `RCLCPP_ERROR` for ROS 2 logging. For Gazebo-specific messages, `gzmsg`, `gzerr`, `gzwarn` are available. Running Gazebo in verbose mode (`gazebo -v 4`) can also provide crucial information about plugin loading and execution.

By mastering Gazebo plugins, you gain fine-grained control over your simulated humanoid robot, enabling realistic testing and development within the ROS 2 framework.
