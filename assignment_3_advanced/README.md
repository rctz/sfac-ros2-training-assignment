# Assignment 3: Navigation and SLAM (Advanced)

Implement autonomous navigation and SLAM using Navigation2 stack in Gazebo simulation.

## Learning Objectives

- Configure and use Navigation2 stack
- Implement SLAM for map building
- Control simulated robot in Gazebo
- Create complex launch file configurations
- Integrate navigation with custom behaviors

## Assignment Tasks

### Task 1: Robot Simulation Setup

Create a package `robot_simulation` that:

- Configures TurtleBot3 in Gazebo world
- Sets up sensor configurations (LiDAR, camera)
- Provides URDF and robot description
- Includes custom world with obstacles

### Task 2: SLAM Integration

Create a package `robot_slam` that:

- Implements SLAM using slam_toolbox
- Provides map saving and loading functionality
- Integrates with robot simulation
- Creates navigation-ready maps

### Task 3: Navigation System

Create a package `robot_navigation` that:

- Configures Navigation2 stack
- Implements waypoint navigation
- Provides goal management system
- Includes obstacle avoidance parameters

### Task 4: Autonomous Mission

Create a package `robot_mission` that:

- Implements autonomous mission planner
- Executes predefined patrol routes
- Handles navigation failures and recovery
- Provides mission status reporting

## Step-by-Step Instructions

### 1. Setup Workspace

```bash
mkdir -p ~/assignment_3_ws/src
cd ~/assignment_3_ws/src
```

### 2. Install Required Packages

```bash
# Install TurtleBot3 packages
sudo apt install ros-humble-turtlebot3*
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-slam-toolbox
```

### 3. Create Simulation Package

```bash
ros2 pkg create --build-type ament_python robot_simulation
cd robot_simulation
```

### 4. Configure Robot Description

- Create URDF files for robot model
- Add sensor configurations (LiDAR, IMU)
- Set up Gazebo world with obstacles

### 5. Create SLAM Package

```bash
cd ~/assignment_3_ws/src
ros2 pkg create --build-type ament_python robot_slam
```

### 6. Configure SLAM Parameters

- Create slam_toolbox configuration
- Set up map saving/loading
- Configure SLAM parameters for environment

### 7. Create Navigation Package

```bash
cd ~/assignment_3_ws/src
ros2 pkg create --build-type ament_python robot_navigation
```

### 8. Configure Navigation2

- Set up Navigation2 parameter files
- Configure global and local planners
- Set obstacle avoidance parameters

### 9. Create Mission Package

```bash
cd ~/assignment_3_ws/src
ros2 pkg create --build-type ament_python robot_mission
```

### 10. Implement Mission Planner

- Create waypoint navigation system
- Implement mission execution logic
- Add failure recovery mechanisms

### 11. Create Complete Launch System

- Create master launch file
- Include simulation, SLAM, and navigation
- Add parameter configurations

### 12. Build and Test

```bash
cd ~/assignment_3_ws
colcon build
source install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
```

## Expected Outcomes

After completion, you should have:

- Functioning robot simulation in Gazebo
- SLAM system that builds accurate maps
- Navigation2 stack for autonomous movement
- Mission planner for complex behaviors
- Integrated system for autonomous navigation

### Verification Commands

```bash
# Launch complete system
ros2 launch robot_simulation complete_system.launch.py

# Check robot status
ros2 topic echo /cmd_vel
ros2 topic echo /scan
ros2 topic echo /odom

# Navigation commands
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "..."

# SLAM commands
ros2 run nav2_map_server map_saver_cli -f ~/map

# Mission commands
ros2 service call /start_mission robot_mission/srv/StartMission "{}"
```

## Reference Documentation

- [Navigation2 Getting Started](https://navigation.ros.org/getting_started/index.html)
- [TurtleBot3 Simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [Nav2 Configuration](https://navigation.ros.org/configuration/index.html)
- [Gazebo Integration](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html)

## System Architecture

```
Gazebo Simulation
├── Robot Model (URDF)
├── Sensor Plugins (LiDAR, IMU)
└── World Environment

SLAM System
├── slam_toolbox
├── Map Building
└── Localization

Navigation2 Stack
├── Global Planner
├── Local Planner
├── Recovery Behaviors
└── Costmap Layers

Mission System
├── Waypoint Manager
├── Mission Executor
└── Status Reporter
```

## Success Criteria

- [ ] Robot spawns correctly in Gazebo simulation
- [ ] SLAM system builds accurate map of environment
- [ ] Navigation2 successfully plans and executes paths
- [ ] Robot avoids obstacles autonomously
- [ ] Mission system executes waypoint sequences
- [ ] System recovers from navigation failures
- [ ] All components integrate through launch files
- [ ] System runs stably for 30+ minutes

## Advanced Challenges

- Add dynamic obstacle detection and avoidance
- Implement multi-robot coordination
- Create custom behavior trees for complex missions
- Add semantic mapping capabilities
- Integrate with perception systems (object detection)
- Implement elevator/door interaction behaviors

## Troubleshooting Tips

- Ensure TURTLEBOT3_MODEL environment variable is set
- Check TF tree for missing transforms
- Verify all required Navigation2 packages are installed
- Monitor topics for data flow issues
- Check parameter files for configuration errors
