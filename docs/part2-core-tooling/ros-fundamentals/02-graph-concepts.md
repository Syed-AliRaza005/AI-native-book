---
title: "2. The ROS 2 Graph: Communication Concepts"
---

## The ROS 2 Graph: Communication Concepts

The **ROS 2 graph** is the logical network of communicating processes that make up your robot's software system. Understanding its fundamental elements—nodes, topics, services, and actions—is paramount to designing robust and scalable robotic applications. These elements facilitate the flow of data and control signals throughout the robot's "nervous system."

### Nodes: The Modular Building Blocks
A **node** is the smallest executable unit in ROS 2. Conceptually, a node is a single-purpose program. Rather than creating one monolithic program for your robot, ROS encourages a modular approach where specific functionalities are encapsulated within separate nodes.

*   **Examples:**
    *   A `camera_driver_node` to interface with a physical camera and publish image data.
    *   A `lidar_processor_node` to take raw LiDAR data, filter it, and extract features.
    *   A `motor_controller_node` to send commands to the robot's motors.
    *   A `path_planner_node` to compute collision-free trajectories.

This modularity enhances maintainability, reusability, and debugging. If a camera driver crashes, the rest of the robot's system (e.g., motor control) can potentially continue to function.

### Communication Primitives: Topics, Services, and Actions

ROS 2 offers three primary communication mechanisms, each designed for different interaction patterns. Choosing the correct primitive for a given task is a key design decision in any ROS application.

| Communication Primitive | Pattern | Key Characteristics | Typical Use Cases |
| :---------------------- | :------ | :------------------ | :---------------- |
| **Topics**              | Asynchronous, Many-to-Many, Unidirectional | Continuous data streams. Publishers send data without expecting a response. Subscribers receive all published data. | Sensor data (images, LiDAR scans, IMU data), robot state information (odometry, joint states), velocity commands to a low-level controller. |
| **Services**            | Synchronous, One-to-One, Bidirectional (Request/Response) | Used for short-duration, blocking operations where a client sends a request and waits for a single response. | Configuration changes (e.g., `set_camera_exposure`), querying current state (e.g., `get_robot_pose`), triggering a single action (e.g., `calibrate_imu`). |
| **Actions**             | Asynchronous, One-to-One, Bidirectional (Goal/Feedback/Result) | Designed for long-running, interruptible tasks that provide continuous feedback on their progress and a final result. | Navigation to a target location, complex manipulation tasks (e.g., `pick_and_place`), executing a sequence of behaviors. |

`[Diagram: A flowchart illustrating the communication patterns. One path shows a publisher node sending data to multiple subscriber nodes via a topic (1-to-N). Another path shows a client node sending a request to a service server node and waiting for a response (1-to-1 synchronous). A third path shows an action client sending a goal to an action server, receiving periodic feedback, and eventually a result (1-to-1 asynchronous with feedback).]`

### Message Definitions: The Language of ROS 2 Data

All data exchanged via topics, services, and actions in ROS 2 must conform to a predefined structure. These structures are specified using a simplified **Interface Definition Language (IDL)** and are saved in `.msg`, `.srv`, and `.action` files.

#### 3.1 `.msg` Files: Defining Topic Data
A `.msg` file is a plain text file that defines the fields and their types for a ROS 2 message.
```
# In a file named MyCustomMessage.msg
# This is a comment
std_msgs/Header header   # Standard ROS header with timestamp and frame_id
int32 id                 # An integer identifier for the message
string name              # A string field for a name
float64[3] position      # An array of 3 double-precision floating-point numbers
bool is_valid            # A boolean flag

# You can also define constants within the message definition
int32 STATUS_OK=0
int32 STATUS_ERROR=1
```
When compiled, this `.msg` file generates code in your chosen language (e.g., Python, C++) that allows you to create, populate, and access instances of `MyCustomMessage`. The `std_msgs/Header` is an example of including another message type, facilitating complex, nested data structures.

#### 3.2 `.srv` Files: Structuring Service Requests and Responses
A `.srv` file defines both the request and response structure for a ROS 2 service. The two parts are separated by a `---` delimiter.
```
# In a file named CalculateSum.srv
int64 a
int64 b
---
int64 sum
```
Here, `a` and `b` are the input parameters for the service request, and `sum` is the single output parameter for the service response.

#### 3.3 `.action` Files: Goals, Results, and Feedback
An `.action` file is more complex, defining three distinct message types:
*   **Goal:** What the client wants the action server to achieve.
*   **Result:** The final outcome of the long-running task.
*   **Feedback:** Intermediate updates on the progress of the task.

These three parts are separated by `---` delimiters.
```
# In a file named DoDishes.action
# Goal
uint32 dishwasher_id
string[] utensils_to_clean
---
# Result
bool success
uint32 dishes_cleaned_count
string status_message
---
# Feedback
float32 percent_complete
string current_utensil_being_cleaned
```
This comprehensive structure allows actions to manage the inherent complexity of tasks that unfold over time, providing transparency and control to the client.
