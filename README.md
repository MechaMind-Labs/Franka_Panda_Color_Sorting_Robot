# 🤖 Franka Panda Color Sorting Robot

A **ROS 2-based intelligent color sorting system** powered by the **Franka Emika Panda robotic arm**. This project seamlessly integrates **OpenCV computer vision**, **MoveIt 2 motion planning**, and **Gazebo simulation** to create an autonomous pick-and-place system that detects and sorts colored objects with precision.

---

## 🎥 Pick and Place Demo  

https://github.com/user-attachments/assets/0813eb4e-310d-4b68-b538-3c8a857079a4


> **At a glance:** a Franka Emika Panda (7-DOF) in Gazebo sorts red, green and blue blocks into bins. OpenCV HSV segmentation finds each block, tf2 moves its position into the robot frame, MoveIt 2 plans the pick through pymoveit2, and ros2_control executes arm and gripper trajectories. A Docker image runs it on ROS 2 Humble and Jazzy.
>
> **Watch:** [full build masterclass (2 h 45 min)](https://youtu.be/0N3B8FG0T1Y) · [demo](https://youtu.be/EM88ld4cCNk) · [demo 2](https://youtu.be/V-1uXPxmQys) · [simulation short](https://www.youtube.com/shorts/HbTO98TFiUU)
>
> **Related:** [SCARA conveyor cell](https://github.com/Curious-Utkarsh/ROS2_SCARA_WS) — the same vision-guided pick pipeline on real hardware · [Dual-Panda tic-tac-toe](https://github.com/MechaMind-Labs/ROS2_Dual_Franka_Panda_TicTacToe_AI_Vision) — two Pandas playing a game with MoveIt 2 and OpenCV · [UR5 MediaPipe pick-and-place](https://github.com/MechaMind-Labs/UR5_PICK_AND_PLACE_MEDIAPIPE_ROS2) · [3-DOF arm digital twin](https://github.com/MechaMind-Labs/ROS2_3DOF_Robotic_Arm_Digital_Twin)

---

## ✨ Features

- 🎨 **Color Detection**: Real-time OpenCV-based vision system for Red, Green, and Blue objects
- 🦾 **Motion Planning**: Advanced trajectory planning using MoveIt 2
- 🎯 **Autonomous Operation**: Complete pick-and-place automation with gripper control
- 🔄 **Dynamic Target Selection**: Switch between colors without restarting the system
- 📊 **Visual Feedback**: Integrated RViz visualization for motion planning and execution
- 🐳 **Docker Ready**: Pre-configured container for instant deployment

---

## 🎯 What You'll Learn

- Setting up a complete ROS 2 robotic system from scratch
- Integrating computer vision with robotic manipulation
- Motion planning and execution with MoveIt 2
- Using PyMoveIt2 for high-level Python control
- Building modular ROS 2 packages
- Docker containerization for robotics applications

---

## 📋 Table of Contents

1. [🐳 Quick Start with Docker (Recommended)](#-quick-start-with-docker-recommended)
2. [💻 Manual Installation on Local PC](#-manual-installation-on-local-pc)
3. [🎮 Running the Project](#-running-the-project)
4. [⚠️ Troubleshooting](#️-troubleshooting)
5. [📚 References](#-references)

---

# 🐳 Quick Start with Docker (Recommended)

Get up and running in minutes with our pre-built Docker image! This method eliminates dependency conflicts and provides a consistent, production-ready environment.

## Prerequisites

- **Docker** installed on your system
- **X11 display server** (included in most Linux distributions)
- **4GB+ free disk space** for the Docker image
- **Stable internet connection** for pulling the image

---

## 🔧 Step 1: Install Docker

### On Ubuntu/Debian Linux:

**1.1 Set up Docker's apt repository:**
```bash
# Add Docker's official GPG key:
sudo apt update
sudo apt install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add the repository to Apt sources:
sudo tee /etc/apt/sources.list.d/docker.sources <<EOF
Types: deb
URIs: https://download.docker.com/linux/ubuntu
Suites: $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}")
Components: stable
Signed-By: /etc/apt/keyrings/docker.asc
EOF

sudo apt update
```

**1.2 Install Docker packages:**
```bash
sudo apt install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin -y
```

**1.3 Post-installation steps (Run Docker without sudo):**
```bash
# Create docker group (if not exists)
sudo groupadd docker

# Add your user to the docker group
sudo usermod -aG docker $USER

# Activate the changes to groups
newgrp docker

# Log out and back in for changes to take effect
```

**1.4 Verify Docker installation:**
```bash
docker run hello-world
```
✅ **Success!** You should see "Hello from Docker!" message.

---

### On Windows:

1. Download **Docker Desktop** from: [https://www.docker.com/products/docker-desktop](https://www.docker.com/products/docker-desktop)
2. Run the installer and follow the setup wizard
3. Enable **WSL 2** integration when prompted
4. Restart your computer
5. Install **VcXsrv** or **X410** for GUI support

---

### On macOS:

1. Download **Docker Desktop** from: [https://www.docker.com/products/docker-desktop](https://www.docker.com/products/docker-desktop)
2. Drag Docker to Applications folder
3. Launch Docker Desktop
4. Install **XQuartz** for X11 support: `brew install --cask xquartz`

---

## 🖥️ Step 2: Enable GUI Support

Docker containers need permission to access your display for RViz and Gazebo visualization.

```bash
xhost +local:docker
```

**Note:** Run this command each time you restart your computer before launching the container.

---

## 🚀 Step 3: Run the Docker Container

Pull and start the pre-configured container with all dependencies:

```bash
docker run -it --rm \
  --name franka_panda_color_sorter \
  --network host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  curiousutkarsh/franka_panda_color_sorter:humble bash
```

**Command Breakdown:**
- `--name franka_panda_color_sorter` → Assign a friendly container name
- `--network host` → Share host network for seamless ROS 2 communication
- `-e DISPLAY=$DISPLAY` → Pass display variable for GUI applications
- `-v /tmp/.X11-unix:/tmp/.X11-unix:rw` → Mount X11 socket for graphics
- `curiousutkarsh/franka_panda_color_sorter:humble` → Pre-built Docker image
- `bash` → Start interactive shell

🎉 **You're now inside the Docker container!** The workspace is pre-built and ready to use.

---

## 🎮 Step 4: Launch the System

Once inside the container, open **two terminal sessions**:

### Terminal 1: Launch the Complete System

```bash
# Inside Docker container
source install/setup.bash
ros2 launch panda_bringup pick_and_place.launch.py
```

This launches:
- ✅ Gazebo simulation with Panda robot
- ✅ RViz motion planning visualization
- ✅ Camera and color detection node
- ✅ MoveIt 2 motion planning server
- ✅ Robot controllers

**Wait for all nodes to initialize** (you'll see "Ready to plan" messages)

---

### Terminal 2: Run Pick-and-Place Node

Open a new terminal and attach to the running container:

```bash
docker exec -it franka_panda_color_sorter bash
```

Inside the new terminal:

```bash
source install/setup.bash
ros2 run pymoveit2 pick_and_place.py --ros-args -p target_color:=R
```

**Color Options:**
- `target_color:=R` → Sort Red objects
- `target_color:=G` → Sort Green objects  
- `target_color:=B` → Sort Blue objects

🎯 **Watch the robot detect, pick, and place objects automatically!**

---

## 🔄 Step 5: Managing the Container

### Switch Target Color:

Stop the pick-and-place node (`Ctrl+C`) and rerun with a new color:

```bash
ros2 run pymoveit2 pick_and_place.py --ros-args -p target_color:=G
```

### Stop the Container:

```bash
# From host terminal (outside Docker)
docker stop franka_panda_color_sorter
```

### Restart Existing Container:

```bash
docker start franka_panda_color_sorter
docker exec -it franka_panda_color_sorter bash
```

### Remove Container:

```bash
docker stop franka_panda_color_sorter
docker rm franka_panda_color_sorter
```

### Pull Latest Image:

```bash
docker pull curiousutkarsh/franka_panda_color_sorter:humble
```

---

## 🛠️ Useful Docker Commands

| Command | Purpose |
|---------|---------|
| `docker ps` | List running containers |
| `docker ps -a` | List all containers (including stopped) |
| `docker images` | List downloaded images |
| `docker system prune` | Clean up unused containers/images |
| `docker logs franka_panda_color_sorter` | View container logs |
| `docker exec -it franka_panda_color_sorter bash` | Attach new terminal |

---

# 💻 Manual Installation on Local PC

If you prefer complete control or need to modify the source code, follow this comprehensive setup guide for a local installation.

## Prerequisites

- **Ubuntu 22.04 LTS** (recommended for ROS 2 Humble)
- **16GB+ RAM** (recommended for Gazebo simulation)
- **20GB+ free disk space**
- **Stable internet connection**

---

## 📦 Step 1: Install ROS 2 Humble

### 1.1 Set Locale

```bash
sudo apt update && sudo apt upgrade -y
locale  # check for UTF-8

sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### 1.2 Setup Sources

```bash
# Add ROS 2 apt repository
sudo apt install software-properties-common -y
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### 1.3 Install ROS 2 Packages

```bash
sudo apt update
sudo apt install ros-humble-desktop-full -y
```

### 1.4 Environment Setup

```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Add to bashrc for automatic sourcing
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 1.5 Install Development Tools

```bash
sudo apt install python3-rosdep python3-colcon-common-extensions python3-pip -y

# Initialize rosdep
sudo rosdep init
rosdep update
```

---

## 🔧 Step 2: Install MoveIt 2 and Dependencies

```bash
sudo apt install -y \
  ros-humble-moveit \
  ros-humble-moveit-common \
  ros-humble-moveit-ros-move-group \
  ros-humble-moveit-ros-planning \
  ros-humble-moveit-ros-planning-interface \
  ros-humble-moveit-visual-tools \
  ros-humble-moveit-configs-utils \
  ros-humble-moveit-setup-assistant \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-gazebo-ros2-control \
  ros-humble-ros-gz \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-rqt-image-view
```

---

## 📥 Step 3: Install Python Dependencies

```bash
pip3 install --no-cache-dir \
  opencv-python==4.10.0.84 \
  numpy==1.24.4 \
  transforms3d
```

---

## 🏗️ Step 4: Create and Build Workspace

### 4.1 Create Workspace

```bash
mkdir -p ~/panda_ws/src
cd ~/panda_ws
```

### 4.2 Clone the Repository

```bash
cd ~/panda_ws/src
git clone https://github.com/MechaMind-Labs/Franka_Panda_Color_Sorting_Robot.git .
```

### 4.3 Install Package Dependencies

```bash
cd ~/panda_ws
rosdep install --from-paths src --ignore-src --skip-keys=opencv_python -r -y
```

### 4.4 Build the Workspace

```bash
colcon build
```

**Note:** Initial build may take 10-15 minutes depending on your system.

### 4.5 Source the Workspace

```bash
source install/setup.bash

# Add to bashrc for convenience
echo "source ~/panda_ws/install/setup.bash" >> ~/.bashrc
```

---

## ✅ Step 5: Verify Installation

Test that all packages are properly installed:

```bash
# Check available packages
ros2 pkg list | grep panda

# Expected output:
# panda_bringup
# panda_controller
# panda_description
# panda_moveit
# panda_vision
```

---

## 🎮 Running the Project

### Complete Pick-and-Place System

Open **two terminal windows**:

#### Terminal 1: Launch System

```bash
source ~/panda_ws/install/setup.bash
ros2 launch panda_bringup pick_and_place.launch.py
```

Wait for all nodes to initialize (~30 seconds)

#### Terminal 2: Start Pick-and-Place

```bash
source ~/panda_ws/install/setup.bash
ros2 run pymoveit2 pick_and_place.py --ros-args -p target_color:=R
```

**Change colors by stopping** (`Ctrl+C`) **and rerunning with different parameter:**

```bash
# For Green objects
ros2 run pymoveit2 pick_and_place.py --ros-args -p target_color:=G

# For Blue objects
ros2 run pymoveit2 pick_and_place.py --ros-args -p target_color:=B
```

---

## 🔍 Useful ROS 2 Commands

### Monitoring and Debugging

```bash
# List all active nodes
ros2 node list

# List all topics
ros2 topic list

# View camera feed
ros2 run rqt_image_view rqt_image_view

# Echo detected colors
ros2 topic echo /detected_color

# Check robot joint states
ros2 topic echo /joint_states

# Monitor gripper state
ros2 topic echo /panda_gripper/joint_states

# View TF tree
ros2 run rqt_tf_tree rqt_tf_tree

# Visualize computation graph
ros2 run rqt_graph rqt_graph
```

### System Diagnostics

```bash
# Check ROS 2 environment
printenv | grep ROS

# Verify MoveIt installation
ros2 pkg list | grep moveit

# Test motion planning
ros2 launch panda_moveit moveit.launch.py
```

---

# ⚠️ Troubleshooting

## Common Issues and Solutions

### 🔴 Build Failures

**Error:** `Package 'xyz' not found` during build

**Solutions:**
```bash
# Update rosdep database
rosdep update

# Reinstall dependencies
cd ~/panda_ws
rosdep install --from-paths src --ignore-src -r -y

# Clean and rebuild
rm -rf build/ install/ log/
colcon build
```

---

### 🔴 Gazebo Won't Start

**Error:** `Gazebo crashes` or `Segmentation fault`

**Solutions:**
```bash
# Reset Gazebo configuration
killall gzserver gzclient
rm -rf ~/.gazebo/

# Check GPU drivers
glxinfo | grep "OpenGL"

# Try software rendering (slower but stable)
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch panda_bringup pick_and_place.launch.py
```

---

### 🔴 RViz Display Issues

**Error:** `RViz shows black screen` or crashes

**Solutions:**
```bash
# Reset RViz configuration
rm -rf ~/.rviz2/

# Check display
echo $DISPLAY

# For Docker users
xhost +local:docker
```

---

### 🔴 MoveIt Planning Failures

**Error:** `Unable to plan` or `No valid plan found`

**Solutions:**
1. Check if all controllers are active:
   ```bash
   ros2 control list_controllers
   ```

2. Verify robot state in RViz
3. Adjust planning time in configuration
4. Check for collisions in scene

---

### 🔴 Camera Not Publishing

**Error:** No images on `/camera/image_raw` topic

**Solutions:**
```bash
# Check if Gazebo camera plugin loaded
ros2 topic list | grep camera

# Verify camera in Gazebo GUI
# View → World → Models → camera

# Restart launch file
```

---

### 🔴 Color Detection Not Working

**Error:** No objects detected or wrong colors

**Solutions:**
1. Check HSV thresholds in `color_detector.py`
2. Verify lighting in Gazebo
3. Test with `rqt_image_view`:
   ```bash
   ros2 run rqt_image_view rqt_image_view
   ```
4. Adjust camera position/angle

---

### 🔴 Docker Container Issues

**Error:** `Cannot connect to display`

**Solution:**
```bash
# Before starting container
xhost +local:docker

# If still failing
xhost +
```

**Error:** `Device or resource busy`

**Solution:**
```bash
# Remove existing container
docker rm -f franka_panda_color_sorter

# Restart Docker service
sudo systemctl restart docker
```

---

## 🐛 Getting Help

If you encounter issues not covered here:

1. **Check logs:**
   ```bash
   ros2 launch panda_bringup pick_and_place.launch.py --log-level debug
   ```

2. **Search existing issues:**
   [GitHub Issues](https://github.com/MechaMind-Labs/Franka_Panda_Color_Sorting_Robot/issues)

3. **Create a new issue** with:
   - System info: `uname -a`, `ros2 --version`
   - Error messages
   - Steps to reproduce

---

# 📂 Project Structure

```
panda_ws/
├── src/
│   ├── panda_description/        # Robot URDF, meshes, visuals
│   ├── panda_controller/         # Joint/gripper controllers
│   ├── panda_moveit/            # MoveIt 2 configuration
│   ├── panda_vision/            # OpenCV color detection
│   ├── panda_bringup/           # Launch files for full system
│   └── pymoveit2/               # Python MoveIt 2 interface
│       └── examples/
│           └── pick_and_place.py  # Main pick-and-place logic
├── build/                        # Build artifacts
├── install/                      # Installed packages
└── log/                         # Build and runtime logs
```

---

# 🧩 Package Overview

| Package | Description | Key Files |
|---------|-------------|-----------|
| **panda_description** | Robot model and visualization | `panda.urdf.xacro`, meshes |
| **panda_controller** | Controller configurations | `panda_controllers.yaml` |
| **panda_moveit** | Motion planning setup | MoveIt configs, SRDF |
| **panda_vision** | Computer vision system | `color_detector.py` |
| **panda_bringup** | System launcher | `pick_and_place.launch.py` |
| **pymoveit2** | High-level control | `pick_and_place.py` |

---

# 🚀 How It Works

## System Architecture

```
Camera Feed → Color Detection → Object Localization
                                        ↓
                                 Motion Planning (MoveIt 2)
                                        ↓
                               Trajectory Execution
                                        ↓
                         Pick Object → Move to Bin → Place
```

## Detailed Workflow

1. **Vision System** continuously monitors the workspace
2. **Color detector** identifies target color and computes 3D position
3. **PyMoveIt2** receives object coordinates
4. **MoveIt 2** plans collision-free trajectory
5. **Robot controller** executes motion
6. **Gripper** closes to grasp object
7. **Motion planner** computes path to sorting bin
8. **Robot** moves to designated area
9. **Gripper** opens to release object
10. **System** returns to home position and repeats

---

# 💡 Customization Guide

## Adjust Color Thresholds

Edit `panda_vision/panda_vision/color_detector.py`:

```python
# HSV ranges for different colors
red_lower = (0, 120, 70)
red_upper = (10, 255, 255)
```

## Change Sorting Positions

Modify `pymoveit2/examples/pick_and_place.py`:

```python
# Bin positions (x, y, z)
RED_BIN = [0.5, -0.3, 0.2]
GREEN_BIN = [0.5, 0.0, 0.2]
BLUE_BIN = [0.5, 0.3, 0.2]
```

## Tune Motion Planning

Edit `panda_moveit/config/planning_python_api.yaml`:

```yaml
plan_request_params:
  planning_attempts: 1               # increase for cluttered scenes
  max_velocity_scaling_factor: 1.0   # lower = slower, safer
  max_acceleration_scaling_factor: 1.0
```

---

# 📚 References

## Official Documentation
- 🌐 [ROS 2 Humble](https://docs.ros.org/en/humble/)
- 🌐 [MoveIt 2](https://moveit.picknik.ai/humble/index.html)
- 🌐 [Franka Emika Panda](https://frankaemika.github.io/)
- 🌐 [PyMoveIt2](https://github.com/AndrejOrsula/pymoveit2)
- 🌐 [Gazebo](https://gazebosim.org/)

## Learning Resources
- 📖 [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- 📖 [MoveIt Tutorials](https://moveit.picknik.ai/humble/doc/tutorials/tutorials.html)
- 📖 [OpenCV Python](https://docs.opencv.org/4.x/d6/d00/tutorial_py_root.html)

## Related Projects
- 🔗 [franka_ros2](https://github.com/frankaemika/franka_ros2) - Official Franka ROS 2 packages
- 🔗 [moveit2_tutorials](https://github.com/moveit/moveit2_tutorials)
- 🔗 [ros2_control](https://control.ros.org/)

---

# 🤝 Contributing

We welcome contributions from the community! Here's how you can help:

### Ways to Contribute

- 🐛 **Report Bugs**: Open an issue with detailed reproduction steps
- 💡 **Suggest Features**: Share your ideas for improvements
- 📝 **Improve Documentation**: Fix typos or add clarifications
- 🔧 **Submit Pull Requests**: Add new features or fix bugs

### Development Workflow

1. Fork the repository
2. Create a feature branch: `git checkout -b feature/amazing-feature`
3. Make your changes and test thoroughly
4. Commit with clear messages: `git commit -m 'Add amazing feature'`
5. Push to your fork: `git push origin feature/amazing-feature`
6. Open a Pull Request with description

---

# 📄 License

This project is licensed under the **MIT License** - see the [LICENSE](LICENSE) file for details.

---

# 🙌 Credits & Acknowledgments

**Maintained by:** [Curious-Utkarsh](https://github.com/Curious-Utkarsh)

**Special Thanks:**
- Franka Emika for the excellent Panda robot
- MoveIt community for motion planning framework
- ROS 2 team for the middleware
- All contributors and supporters

**Inspired by:** Real-world industrial pick-and-place automation systems

**Third-party code:** [`pymoveit2`](https://github.com/AndrejOrsula/pymoveit2) by Andrej Orsula (BSD-3-Clause) is vendored in `pymoveit2/` with its own licence.

---

# ⭐ Show Your Support

If this project helped you learn robotics or build something amazing:

- ⭐ **Star** the repository
- 🐛 **Report** issues you encounter
- 💡 **Suggest** improvements
- 🤝 **Contribute** code or documentation
- 📢 **Share** with the robotics community

---

**Built with ❤️ for the robotics community**

🚀 Happy Building! 🤖
