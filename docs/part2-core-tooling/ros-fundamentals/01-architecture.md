---
title: "1. The ROS 2 Architecture"
---

## The ROS 2 Architecture: A DDS-Based Foundation

ROS 2 represents a fundamental paradigm shift from its predecessor, ROS 1. This shift was motivated by the need for a system that could meet the demands of real-world, commercial products—demands for reliability, security, and real-time performance. The core of this shift is the adoption of the **Data Distribution Service (DDS)** standard.

### DDS: The Industrial Middleware
DDS is not a specific software but an open standard for publish-subscribe middleware, maintained by the Object Management Group (OMG). This means ROS 2's communication layer is not a bespoke creation but is built upon a mature, industry-vetted technology used in mission-critical systems from avionics to industrial control.

*   **Vendor Implementations:** Because DDS is a standard, multiple vendors provide implementations. ROS 2 is designed to be middleware-agnostic. While the default is eProsima's **Fast DDS**, you can, with some configuration, swap it out for another implementation like RTI's **Connext DDS**. This is a powerful feature for commercial products that may have specific certification or performance requirements.
*   **DDS Security:** The DDS standard includes a comprehensive security specification. This provides a built-in mechanism for securing your robot's communication graph, featuring:
    *   **Authentication:** Verifying the identity of each node.
    *   **Access Control:** Defining which nodes are allowed to publish or subscribe to which topics.
    *   **Cryptography:** Encrypting the data "on the wire."

### Quality of Service (QoS): Engineering Your Data Flow
A profound advantage of the DDS foundation is the rich **Quality of Service (QoS)** framework. In ROS 1, all topics behaved like unreliable UDP streams. In ROS 2, you have fine-grained control over the behavior of each connection.

Consider a simple scenario: a `camera_node` publishes images, and a `vision_node` subscribes to them.
*   **The Problem:** What if the `vision_node` takes longer to process an image than the camera takes to produce one? The data will pile up.
*   **The Solution with QoS:** You can set the `history` policy to `KEEP_LAST` and the `depth` to `1`. This tells the middleware to discard any message that hasn't been processed by the time a new one arrives. The `vision_node` will always get the most recent image, ensuring it is acting on up-to-date information, while automatically preventing a memory overflow.

**A Critical Detail:** For a publisher and subscriber to connect, their QoS profiles must be compatible. A common pitfall is a publisher using a `RELIABLE` policy and a subscriber requesting `BEST_EFFORT`. They will fail to connect, and the system will silently fail. You can debug this using the tool `ros2 topic info -v /my_topic`, which will show the QoS profiles of all publishers and subscribers on that topic.
