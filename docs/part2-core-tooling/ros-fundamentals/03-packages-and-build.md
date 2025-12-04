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

This modular build system is a cornerstone of efficient ROS 2 development, enabling collaborative efforts on complex robotic systems.
