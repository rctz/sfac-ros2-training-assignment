# Assignment 1: ROS2 Fundamentals (Basic)

Create your first ROS2 Python packages with publishers and subscribers.

## Learning Objectives

- Create ROS2 Python packages using standard tools
- Implement publisher and subscriber nodes
- Understand ROS2 topics and message passing
- Use custom message types
- Apply basic ROS2 launch files

## Assignment Tasks

### Task 1: Create Publisher Package

Create a package `robot_sensor_publisher` that:

- Publishes simulated sensor data every 1 second
- Uses custom message type `SensorData.msg`
- Includes temperature, humidity, and timestamp fields

### Task 2: Create Subscriber Package

Create a package `robot_data_processor` that:

- Subscribes to sensor data
- Processes and logs received data
- Calculates running averages
- Publishes processed results to new topic

### Task 3: Launch Configuration

Create launch file that:

- Starts both publisher and subscriber nodes
- Sets configurable parameters
- Includes node remapping

## Step-by-Step Instructions

### 1. Setup Workspace

```bash
mkdir -p ~/assignment_1_ws/src
cd ~/assignment_1_ws/src
```

### 2. Create Publisher Package

```bash
ros2 pkg create --build-type ament_python robot_sensor_publisher
cd robot_sensor_publisher
```

### 3. Create Robot Interfaces Package

Before creating the publisher and subscriber packages, you need to create a separate package for custom message definitions:

```bash
cd ~/assignment_1_ws/src
ros2 pkg create --build-type ament_cmake robot_interfaces
cd robot_interfaces
```

#### Creating Custom Messages

Create the message directory and define your custom message:

```bash
mkdir msg
```

Create `msg/SensorData.msg` with the following structure:

```
float32 temperature
float32 humidity
builtin_interfaces/Time timestamp
```

#### Package Configuration for Messages

Update `package.xml` to include message dependencies:

```xml
<depend>builtin_interfaces</depend>
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

Update `CMakeLists.txt` to generate messages:

```cmake
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SensorData.msg"
  DEPENDENCIES builtin_interfaces
)
```

#### 💡 Hints for Robot Interfaces:

- **Separate interfaces package**: Always create a separate package for custom messages/services/actions to promote reusability
- **Naming convention**: Use PascalCase for message names (e.g., `SensorData.msg`)
- **Field types**: Use appropriate ROS2 field types (`float32`, `int32`, `string`, etc.)
- **Timestamps**: Use `builtin_interfaces/Time` for timestamp fields
- **Build order**: Interface packages must be built before packages that use them
- **Dependencies**: Don't forget to add the interfaces package as a dependency in packages that use the messages

### 4. Update Publisher Package Dependencies

After creating the interfaces package, update your publisher package to use the custom message:

In `robot_sensor_publisher/package.xml`, add:

```xml
<depend>robot_interfaces</depend>
```

### 5. Implement Publisher Node

- Create `robot_sensor_publisher/sensor_publisher.py`
- Publish data at 1Hz frequency
- Use random values for simulation

### 6. Create Subscriber Package

```bash
cd ~/assignment_1_ws/src
ros2 pkg create --build-type ament_python robot_data_processor
```

### 7. Update Subscriber Package Dependencies

In `robot_data_processor/package.xml`, add:

```xml
<depend>robot_interfaces</depend>
```

### 8. Implement Subscriber Node

- Create `robot_data_processor/data_processor.py`
- Subscribe to sensor data topic
- Calculate and publish averages

### 9. Create Launch File

- Create `launch/robot_system.launch.py`
- Launch both nodes with parameters

### 10. Build and Test

```bash
cd ~/assignment_1_ws

# Build interfaces package first
colcon build --packages-select robot_interfaces

# Build all packages
colcon build

source install/setup.bash
ros2 launch robot_sensor_publisher robot_system.launch.py
```

#### 💡 Build Order Hints:

- **Interface packages first**: Always build interface packages before other packages that depend on them
- **Selective building**: Use `--packages-select` to build specific packages
- **Clean builds**: Use `colcon build --packages-select <package_name> --cmake-clean-cache` if you encounter build issues
- **Source setup**: Don't forget to source the setup script after building

## Expected Outcomes

After completion, you should have:

- Three functioning ROS2 packages (interfaces, publisher, subscriber)
- Custom message type for sensor data defined in separate interfaces package
- Publisher node generating simulated data using custom messages
- Subscriber node processing and republishing data
- Launch file coordinating the system
- Demonstrated understanding of ROS2 topics and custom message interfaces

### Verification Commands

```bash
# Check running nodes
ros2 node list

# Monitor topics
ros2 topic list
ros2 topic echo /sensor_data
ros2 topic echo /processed_data

# Check message rates
ros2 topic hz /sensor_data
```

## Reference Documentation

- [Creating a Package](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)
- [Writing a Simple Publisher and Subscriber (Python)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [Creating Custom Message and Service Files](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)
- [Launch Files](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)
- [Parameters](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html)

## Success Criteria

- [ ] Robot interfaces package builds successfully and generates custom messages
- [ ] All three packages (robot_interfaces, robot_sensor_publisher, robot_data_processor) build successfully with `colcon build`
- [ ] Custom SensorData message type is properly defined and accessible
- [ ] Publisher publishes data at correct frequency using custom message format
- [ ] Subscriber receives and processes data correctly
- [ ] Launch file starts both nodes
- [ ] System runs without errors for 5 minutes
- [ ] Topic communication verified with `ros2 topic echo`
- [ ] Message interface can be inspected with `ros2 interface show robot_interfaces/msg/SensorData`
