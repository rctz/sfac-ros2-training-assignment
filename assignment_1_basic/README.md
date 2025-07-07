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

### 3. Define Custom Message

- Create `msg/SensorData.msg` with required fields
- Update `package.xml` and `CMakeLists.txt` for message generation

### 4. Implement Publisher Node

- Create `robot_sensor_publisher/sensor_publisher.py`
- Publish data at 1Hz frequency
- Use random values for simulation

### 5. Create Subscriber Package

```bash
cd ~/assignment_1_ws/src
ros2 pkg create --build-type ament_python robot_data_processor
```

### 6. Implement Subscriber Node

- Create `robot_data_processor/data_processor.py`
- Subscribe to sensor data topic
- Calculate and publish averages

### 7. Create Launch File

- Create `launch/robot_system.launch.py`
- Launch both nodes with parameters

### 8. Build and Test

```bash
cd ~/assignment_1_ws
colcon build
source install/setup.bash
ros2 launch robot_sensor_publisher robot_system.launch.py
```

## Expected Outcomes

After completion, you should have:

- Two functioning ROS2 Python packages
- Custom message type for sensor data
- Publisher node generating simulated data
- Subscriber node processing and republishing data
- Launch file coordinating the system
- Demonstrated understanding of ROS2 topics

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

- [ ] Both packages build successfully with `colcon build`
- [ ] Publisher publishes data at correct frequency
- [ ] Subscriber receives and processes data correctly
- [ ] Custom message type works properly
- [ ] Launch file starts both nodes
- [ ] System runs without errors for 5 minutes
- [ ] Topic communication verified with `ros2 topic echo`
