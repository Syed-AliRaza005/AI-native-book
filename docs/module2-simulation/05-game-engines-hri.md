---
title: "4. High-Fidelity Simulation with Game Engines: Unity & ROS 2"
---

## 4. High-Fidelity Simulation with Game Engines: Unity & ROS 2

While Gazebo excels in physics-accurate simulation and tight integration with ROS, game engines like **Unity** offer unparalleled capabilities for high-fidelity rendering, realistic visual environments, and rich human-robot interaction (HRI) experiences. This makes Unity an attractive option for tasks requiring advanced visualization, virtual reality (VR)/augmented reality (AR) interfaces, or simulating complex human-robot workspaces.

### 4.1 Why Use Unity for Robotics?

*   **Photorealistic Rendering:** Unity's advanced graphics pipeline allows for highly realistic environments and robot models, crucial for tasks like perception model training where visual fidelity impacts generalization.
*   **Rich Human-Robot Interaction (HRI):** Unity's robust UI system and support for various input devices make it ideal for developing intuitive interfaces for controlling robots, visualizing their internal states, and creating immersive teleoperation experiences.
*   **Asset Store:** Access to a vast marketplace of 3D models, environments, and tools significantly accelerates development.
*   **Cross-Platform Deployment:** Unity allows you to deploy your simulations to various platforms, including desktop, web, VR/AR headsets.
*   **Extensible Physics:** While Gazebo's physics engine is purpose-built, Unity's physics engine is highly capable and can be extended for specific robotic needs.

### 4.2 ROS 2 and Unity Integration Overview

The primary method for connecting ROS 2 with Unity is through the **Unity Robotics Hub**, specifically using the **ROS 2 TCP Endpoint** package. This package facilitates communication between Unity (C#) and ROS 2 (Python/C++) nodes over TCP/IP, allowing you to:

*   Publish sensor data from Unity to ROS 2.
*   Subscribe to control commands from ROS 2 to actuate robots in Unity.
*   Call and provide ROS 2 services.
*   Send and receive ROS 2 actions (though often handled at a higher level).

### 4.3 Setting Up a Unity Project for ROS 2 Communication

#### Step 1: Create a New Unity 3D Project

1.  Open **Unity Hub**.
2.  Click **"New Project"**.
3.  Select a **3D (Core)** or **3D (URP)** template.
4.  Give your project a name (e.g., `ROS2_Humanoid_Sim`) and choose a location.
5.  Click **"Create Project"**.

#### Step 2: Install Unity Robotics ROS 2 TCP Endpoint

This package enables the communication bridge.

1.  In Unity, go to **Window > Package Manager**.
2.  Click the **"+"** icon in the top-left corner and select **"Add package from git URL..."**.
3.  Enter the URL: `https://github.com/Unity-Technologies/Unity-Robotics-ROS2-TCP-Endpoint.git`
4.  Click **"Add"**.
5.  Alternatively, you can add it via the `manifest.json` file in your `Packages` folder:
    ```json
    {
      "dependencies": {
        "com.unity.robotics.ros2-tcp-endpoint": "https://github.com/Unity-Technologies/Unity-Robotics-ROS2-TCP-Endpoint.git#main",
        // ... other packages
      }
    }
    ```

#### Step 3: Configure `ROSConnection` in Your Scene

The `ROSConnection` prefab manages the TCP/IP connection to your ROS 2 environment.

1.  In your Unity project, navigate to `Packages/ROS 2 TCP Endpoint/Runtime/Prefabs`.
2.  Drag the `ROSConnection` prefab into your scene hierarchy.
3.  Select the `ROSConnection` object in the Hierarchy. In the Inspector window, you can configure:
    *   **ROS IP Address:** The IP address of your ROS 2 master (usually your machine's IP, or `127.0.0.1` for local setup).
    *   **ROS Port:** The port for communication (default is `10000`).

### 4.4 Robot Model Integration in Unity

You can import robot models into Unity using various methods:

*   **URDF Importer (Unity Robotics Hub):** The Unity Robotics Hub also provides a URDF Importer package that allows you to directly import URDF files, converting them into Unity GameObjects with Rigidbody and Joint components.
    1.  Install the `Unity.Robotics.URDF-Importer` package from the Unity Robotics Hub repository.
    2.  Go to **Robotics > URDF Importer > Import URDF**.
    3.  Select your robot's URDF file.
*   **Custom 3D Models:** Import `FBX`, `OBJ`, `GLB` files (exported from CAD software) directly into Unity. You will then manually add `Rigidbody` and `Collider` components and configure `Joint` components (e.g., `HingeJoint`, `ConfigurableJoint`) to match your robot's kinematics.

For humanoid robots, ensure your model has a proper kinematic chain with `Rigidbody` components for each link and `Joint` components connecting them, configured to match the joint types (revolute, prismatic) and limits from your URDF/XACRO.

### 4.5 ROS 2 Communication in Unity (C#)

Unity scripts will handle publishing commands and subscribing to sensor data.

#### Example: Publishing Joint Commands from Unity

This C# script might send target joint positions to a ROS 2 controller.

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std; // Example: using standard Float64 message type

public class JointCommandPublisher : MonoBehaviour
{
    public string topicName = "/humanoid/joint_controller/command";
    public string jointName = "my_robot_joint";
    public float targetPosition = 0.0f; // Example target

    ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.Get  Instance();
        ros.RegisterPublisher<Float64Msg>(topicName);
    }

    void Update()
    {
        // Example: Send command when a key is pressed
        if (Input.GetKeyDown(KeyCode.Space))
        {
            Float64Msg jointCmd = new Float64Msg(targetPosition);
            ros.Publish(topicName, jointCmd);
            Debug.Log($"Published joint command {targetPosition} to {topicName}");
        }
    }

    public void SetTargetPosition(float position)
    {
        targetPosition = position;
    }
}
```

#### Example: Subscribing to Sensor Data in Unity

This script might receive joint state feedback from ROS 2 to update the Unity model.

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor; // Example: using JointState message type

public class JointStateSubscriber : MonoBehaviour
{
    public string topicName = "/joint_states";
    public GameObject jointVisual; // Reference to the GameObject representing the joint

    ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.Get  Instance();
        ros.Subscribe<JointStateMsg>(topicName, JointStateCallback);
    }

    void JointStateCallback(JointStateMsg msg)
    {
        // Find the specific joint's position from the message
        // For simplicity, assuming the first position corresponds to the jointVisual
        if (msg.position.Length > 0)
        {
            float angle = (float)msg.position[0];
            // Update the visual representation of the joint
            if (jointVisual != null)
            {
                // Assuming this joint rotates around Z-axis, adjust as per your robot's design
                jointVisual.transform.localRotation = Quaternion.Euler(0, 0, angle * Mathf.Rad2Deg);
            }
        }
    }
}
```

### 4.6 Basic Human-Robot Interaction (HRI) in Unity

Unity's UI system (Canvas, Buttons, Sliders) makes it straightforward to create interactive control panels.

1.  **Create a UI Canvas:** Go to **GameObject > UI > Canvas**.
2.  **Add UI Elements:** Add Buttons for discrete commands (e.g., "Move Forward," "Stop"), Sliders for continuous control (e.g., joint angles, speed), and Text elements for displaying robot status.
3.  **Connect UI to Scripts:** Use Unity's event system to connect UI element events (e.g., button click) to methods in your C# scripts that publish ROS 2 messages.

### 4.7 ROS 2 Control Node (Python Example)

On the ROS 2 side, you'll have Python (or C++) nodes that process commands from Unity and send feedback.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState # For feedback

class UnityRobotCommander(Node):
    def __init__(self):
        super().__init__('unity_robot_commander')
        # Subscriber for commands from Unity
        self.cmd_sub_ = self.create_subscription(
            Float64,
            '/humanoid/joint_controller/command',
            self.command_callback,
            10
        )
        # Publisher for joint state feedback to Unity
        self.joint_state_pub_ = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_state) # Publish every 0.1s

        self.current_joint_position = 0.0
        self.get_logger().info('Unity Robot Commander node started.')

    def command_callback(self, msg):
        self.get_logger().info(f'Received command from Unity: {msg.data}')
        self.current_joint_position = msg.data # Update internal state

    def publish_joint_state(self,):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['my_robot_joint'] # Must match joint name in Unity
        msg.position = [self.current_joint_position]
        self.joint_state_pub_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    unity_robot_commander = UnityRobotCommander()
    rclpy.spin(unity_robot_commander)
    unity_robot_commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
This Python node acts as the "brain" for your Unity simulated robot, receiving high-level commands and sending back detailed state information.

### 4.8 Unity vs. Gazebo: When to Choose Which?

| Feature                      | Gazebo                                   | Unity                                           |
| :--------------------------- | :--------------------------------------- | :---------------------------------------------- |
| **Physics Accuracy**         | Highly accurate, purpose-built physics   | Capable, but may require more configuration for complex robotics |
| **Visual Fidelity**          | Functional, but generally lower fidelity | High-fidelity, photorealistic rendering         |
| **Human-Robot Interaction**  | Limited UI tools, primarily programmatic | Excellent UI/UX tools, VR/AR integration        |
| **ROS Integration**          | Native and deeply integrated             | Requires external packages (`ROS TCP Endpoint`) |
| **Sensor Simulation**        | Strong, wide range of plugins            | Requires custom scripting or specialized assets |
| **Asset Ecosystem**          | Limited (SDF models)                     | Vast Asset Store (3D models, environments, tools) |
| **Use Case**                 | Control algorithm testing, sensor integration, large-scale multi-robot scenarios | HRI research, virtual commissioning, operator training, AI perception training (synthetic data) |

In summary, for detailed physics-based robot control and sensor integration with ROS, Gazebo often remains the default. However, for visually rich simulations, complex human-robot interfaces, and scenarios benefiting from advanced graphics, Unity provides a powerful alternative and complement.
