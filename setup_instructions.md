# ROS2 Humble Environment Setup

Quick setup guide for completing training assignments.

## Option 1: Native Installation (Recommended)

### Install ROS2 Humble

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Add ROS2 repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions -y
```

### Setup Workspace

```bash
# Source ROS2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

## Option 2: Docker Setup

### Create Dockerfile

```dockerfile
FROM ros:humble
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-pip \
    vim \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /workspace
```

### Run Container

```bash
docker build -t ros2-training .
docker run -it --rm -v $(pwd):/workspace ros2-training
```

## Verify Installation

```bash
# Test ROS2 installation
ros2 run demo_nodes_cpp talker
# In another terminal:
ros2 run demo_nodes_py listener
```

## IDE Setup

### VS Code Extensions

- ROS
- Python
- CMake Tools

### Configure VS Code settings.json

```json
{
  "ros.distro": "humble",
  "python.defaultInterpreterPath": "/usr/bin/python3"
}
```

## Common Commands

```bash
# Build workspace
colcon build

# Source workspace
source install/setup.bash

# Clean build
rm -rf build install log
colcon build

# Run specific package
ros2 run <package_name> <node_name>

# List all nodes
ros2 node list

# Check topics
ros2 topic list
```

## Troubleshooting

- **Permission denied**: Add user to dialout group: `sudo usermod -a -G dialout $USER`
- **Package not found**: Check if workspace is sourced properly
- **Build fails**: Ensure all dependencies are installed with `rosdep install --from-paths src --ignore-src -r -y`
