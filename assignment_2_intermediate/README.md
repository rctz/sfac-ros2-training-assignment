# Assignment 2: ROS2 Services and Actions (Intermediate)

Implement ROS2 services, actions, and lifecycle nodes for advanced communication patterns.

## Learning Objectives

- Implement ROS2 services for request-response communication
- Create and use ROS2 actions for long-running tasks
- Understand lifecycle node management
- Apply parameter configurations
- Integrate multiple communication patterns

## Assignment Tasks

### Task 1: Robot Control Service

Create a package `robot_control_service` that:

- Provides service to control robot movement
- Accepts velocity commands and duration
- Returns status and distance traveled
- Uses custom service definition

### Task 2: Data Processing Action

Create a package `robot_processing_action` that:

- Implements action server for data processing
- Processes batch sensor data with progress feedback
- Supports cancellation and preemption
- Returns processing results and statistics

### Task 3: Lifecycle Node Manager

Create a package `robot_lifecycle_manager` that:

- Implements lifecycle node for system management
- Manages component states (inactive/active)
- Provides state transition services
- Includes parameter configuration

## Step-by-Step Instructions

### 1. Setup Workspace

```bash
mkdir -p ~/assignment_2_ws/src
cd ~/assignment_2_ws/src
```

### 2. Create Service Package

```bash
ros2 pkg create --build-type ament_python robot_control_service
cd robot_control_service
```

### 3. Define Custom Service

- Create `srv/RobotControl.srv` with request/response fields
- Update package configuration for service generation

### 4. Implement Service Server

- Create service server node
- Handle velocity control requests
- Simulate robot movement and return results

### 5. Create Action Package

```bash
cd ~/assignment_2_ws/src
ros2 pkg create --build-type ament_python robot_processing_action
```

### 6. Define Custom Action

- Create `action/ProcessData.action` with goal, result, and feedback
- Configure package for action generation

### 7. Implement Action Server

- Create action server for data processing
- Provide progress feedback during execution
- Handle goal cancellation appropriately

### 8. Create Lifecycle Package

```bash
cd ~/assignment_2_ws/src
ros2 pkg create --build-type ament_python robot_lifecycle_manager
```

### 9. Implement Lifecycle Node

- Create lifecycle node with managed states
- Implement state transition callbacks
- Add parameter management for configuration

### 10. Create Integration Launch

- Create launch file that starts all components
- Configure parameters and node relationships
- Include lifecycle management commands

### 11. Build and Test

```bash
cd ~/assignment_2_ws
colcon build
source install/setup.bash
```

## Expected Outcomes

After completion, you should have:

- Functioning service server and client
- Action server with feedback and cancellation
- Lifecycle node with proper state management
- Parameter-driven configuration system
- Integrated launch file for complete system

### Verification Commands

```bash
# Check services
ros2 service list
ros2 service call /robot_control robot_control_service/srv/RobotControl "{linear_velocity: 1.0, angular_velocity: 0.5, duration: 2.0}"

# Check actions
ros2 action list
ros2 action send_goal /process_data robot_processing_action/action/ProcessData "{data_size: 100}"

# Check lifecycle states
ros2 lifecycle list
ros2 lifecycle get /lifecycle_manager
ros2 lifecycle set /lifecycle_manager activate

# Monitor parameters
ros2 param list
ros2 param get /lifecycle_manager processing_rate
```

## Reference Documentation

- [Creating Custom Service and Action Files](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)
- [Writing a Simple Service and Client (Python)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html)
- [Writing an Action Server and Client (Python)](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html)
- [Managed Nodes](https://docs.ros.org/en/humble/Tutorials/Intermediate/Managed-Nodes.html)
- [Using Parameters in a Class (Python)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html)

## Success Criteria

- [ ] Service server responds correctly to requests
- [ ] Action server provides progress feedback
- [ ] Action server handles cancellation properly
- [ ] Lifecycle node transitions between states correctly
- [ ] Parameters are loaded and used appropriately
- [ ] All packages build without errors
- [ ] Integration launch file works properly
- [ ] System demonstrates all communication patterns

## Advanced Challenges

- Add service client that calls robot control automatically
- Implement action client with progress monitoring
- Create parameter update callbacks
- Add error handling and recovery mechanisms
- Implement multi-threaded execution
