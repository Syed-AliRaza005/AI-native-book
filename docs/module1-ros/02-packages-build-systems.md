---
title: "3. ROS 2 Packages and the Build System"
---

## ROS 2 Packages and the Build System

In ROS 2, a **package** serves as the fundamental unit of organization for software. It's a structured directory containing source code, build scripts, configuration files, message definitions, and other resources related to a specific piece of functionality. The build system, primarily `colcon`, orchestrates how these packages are compiled and made available within your ROS environment.

### 3.1 Anatomy of a ROS 2 Package

A typical Python ROS 2 package (often created using `ros2 pkg create --build-type ament_python <package_name>`) adheres to a well-defined structure:

```
my_python_pkg/
├── package.xml         # Package manifest: metadata and dependencies
├── setup.py            # Standard Python setuptools script for installation
├── setup.cfg           # Configuration for setup.py
└── my_python_pkg/      # Python module directory (same name as package)
    ├── __init__.py     # Makes it a Python package
    └── my_node.py      # Your primary Python node executable
    └── (optional) other_modules.py
    └── (optional) data/ # Data files, YAML configs, etc.
└── (optional) launch/  # Directory for Python launch files
```

#### `package.xml`: The Package Manifest

This XML file is the heart of your package. It provides essential metadata and declares dependencies, guiding `colcon` and other ROS tools.

*   **`<name>`**: The unique name of your package.
*   **`<version>`**: The package's version number.
*   **`<description>`**: A brief summary of the package's purpose.
*   **`<maintainer>`**: Contact information for the package maintainer.
*   **`<license>`**: The software license under which the package is released.
*   **Dependencies:** These are crucial for `colcon` to build your package correctly and for other packages to find your package's resources.
    *   **`<depend>`**: A general dependency that is needed for both building and execution.
        *   Example: `<depend>rclpy</depend>` (needed to build and run Python ROS nodes).
        *   Example: `<depend>std_msgs</depend>` (needed if your node uses standard ROS messages like `std_msgs/String`).
    *   **`<build_depend>`**: A dependency only needed during the build process (e.g., code generation tools).
    *   **`<exec_depend>`**: A dependency only needed during runtime (less common for Python packages as `depend` often suffices).
    *   **`<test_depend>`**: A dependency only needed for running tests.

Failing to declare all dependencies properly is a common cause of build failures in CI/CD environments or when sharing your code.

#### `setup.py`: The Python Entry Point

This standard Python script is used by `setuptools` to build and install your Python package. In a ROS 2 context, it plays a vital role in defining your executables (your nodes) and data files.

```python
# In setup.py
from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_python_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files in the 'launch' directory
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.[pxy][yem]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='A sample ROS 2 Python package.',
    license='Apache-2.0',
    tests_require=['pytest'],
    # Define your executables (your ROS 2 nodes) here
    entry_points={
        'console_scripts': [
            'my_node = my_python_pkg.my_node:main', # Maps 'my_node' command to main() function in my_node.py
            'simple_publisher = my_python_pkg.simple_publisher:main', # From previous example
        ],
    },
)
```
The `entry_points` dictionary is particularly important, as it tells `colcon` which Python functions should be exposed as executable commands (nodes) within the ROS 2 environment.

### 3.2 Building Workspaces with `colcon`

`colcon` (COLlective CONstruction) is the build orchestration tool for ROS 2. It is designed to efficiently build multiple packages in a workspace, respecting their interdependencies.

#### The `colcon build` Process:
1.  **Dependency Resolution:** `colcon` first scans your workspace (and any sourced underlays) to identify all packages and their dependencies.
2.  **Topological Sort:** It then performs a topological sort, building packages in the correct order (dependencies first).
3.  **Build Type Execution:** For each package, `colcon` invokes the appropriate build tool based on its `build_type` (e.g., `ament_python` for Python packages, `ament_cmake` for C++ packages).
4.  **Installation:** After building, artifacts (executables, libraries, Python modules, message headers) are installed into the `install/` directory of your workspace.

#### Workspace Overlays: Managing Your Environment
The ROS 2 environment is fundamentally built using an **overlay** system.
*   When you install ROS 2, it's installed into a base directory (e.g., `/opt/ros/humble/`). When you `source /opt/ros/humble/setup.bash`, you are setting up your environment to find these base packages (the **underlay**).
*   When you develop your own packages, you put them in a workspace. After building with `colcon build`, you `source install/setup.bash` from your workspace. This *overlays* your workspace's packages on top of the underlay.
*   **The Rule:** If a package exists in both the underlay and your overlay, the version in your overlay will be used. This allows you to work on modifications to existing ROS packages without rebuilding the entire ROS distribution, and to easily switch between different versions of your own packages.

### 3.3 Step-by-Step: Creating and Running a ROS 2 Python Package

Let's walk through the process of creating a simple ROS 2 Python package that publishes a "Hello, World!" message.

#### 1. Set Up Your Workspace
If you don't already have a ROS 2 workspace, create one.
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

#### 2. Create the Python Package
Use the `ros2 pkg create` command with `--build-type ament_python`.
```bash
ros2 pkg create --build-type ament_python my_ros_pkg --dependencies rclpy std_msgs
```
This command creates a directory named `my_ros_pkg` and populates it with a `package.xml` and `setup.py`, along with a `my_ros_pkg` subdirectory for your Python modules. We also declare `rclpy` and `std_msgs` as dependencies.

#### 3. Implement Your Publisher Node
Navigate into your newly created package's Python module directory and create a file, for example, `my_publisher.py`.
```bash
cd ~/ros2_ws/src/my_ros_pkg/my_ros_pkg
touch my_publisher.py
```
Open `my_publisher.py` and add the following code:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyPublisher(Node):
    def __init__(self):
        super().__init__('my_publisher_node')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.i = 0
        self.get_logger().info('MyPublisher node has been started.')

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello, ROS 2! Count: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = MyPublisher()
    rclpy.spin(node) # Keeps the node alive until Ctrl+C is pressed
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### 4. Configure `setup.py`
You need to tell `colcon` that `my_publisher.py` is an executable ROS 2 node. Open `~/ros2_ws/src/my_ros_pkg/setup.py` and modify the `entry_points` section:

```python
# ... (existing imports and package_name definition)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='A sample ROS 2 Python publisher package.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_publisher = my_ros_pkg.my_publisher:main', # Add this line
        ],
    },
)
```
Ensure you add the line `'my_publisher = my_ros_pkg.my_publisher:main'` within the `console_scripts` dictionary.

#### 5. Build Your Workspace
Navigate back to the root of your ROS 2 workspace (`~/ros2_ws`) and build your package using `colcon`.
```bash
cd ~/ros2_ws
colcon build --packages-select my_ros_pkg
```
The `--packages-select` flag ensures that only your `my_ros_pkg` (and its dependencies) are built, saving time.

#### 6. Source the Setup Files
After a successful build, you must source your workspace's `setup` file to make your new package and its executables available to your ROS environment.
```bash
source install/setup.bash
# For Zsh users: source install/setup.zsh
# For PowerShell users: . install/setup.ps1
```
You need to run this command in every new terminal where you want to use your ROS 2 packages.

#### 7. Run Your ROS 2 Node
Finally, you can run your new publisher node:
```bash
ros2 run my_ros_pkg my_publisher
```
You should see output similar to:
```
[INFO] [my_publisher_node]: MyPublisher node has been started.
[INFO] [my_publisher_node]: Publishing: "Hello, ROS 2! Count: 0"
[INFO] [my_publisher_node]: Publishing: "Hello, ROS 2! Count: 1"
# ... and so on
```
In a separate terminal, after sourcing your workspace, you can verify that messages are being published using:
```bash
ros2 topic echo /topic
```
This comprehensive example demonstrates the full lifecycle of creating, building, and running a basic ROS 2 Python package, a foundational skill for any roboticist.
