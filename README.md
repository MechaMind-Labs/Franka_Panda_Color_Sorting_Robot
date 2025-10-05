# Franka Panda Color Sorting Robot

A **ROS 2-based color sorting system** using the **Franka Emika Panda robotic arm**, combining **OpenCV for vision**, **MoveIt 2 for motion planning**, and **Gazebo for simulation**.
This project allows the Panda robot to detect colored objects (Red, Green, or Blue) and perform **pick-and-place** actions automatically.

---

## 🚀 Quick Start — How to Use This Repository

### **1️⃣ Clone the Repository**

```bash
cd ~/panda_ws/src
git clone https://github.com/MechaMind-Labs/Franka_Panda_Color_Sorting_Robot.git
```

### **2️⃣ Install Dependencies**

Use `rosdep` to install all required ROS dependencies:

```bash
cd ~/panda_ws
sudo apt install python3-rosdep -y
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y

```

### **3️⃣ Build the Workspace**

```bash
colcon build
```

### **4️⃣ Source the Setup File**

```bash
source install/setup.bash
```

### **5️⃣ Launch the Complete System**

In **Terminal 1**, launch the bringup (simulation, MoveIt, and vision):

```bash
ros2 launch panda_bringup pick_and_place.launch.py
```

In **Terminal 2**, run the pick and place node with a target color:

```bash
ros2 run pymoveit2 pick_and_place.py --ros-args -p target_color:=R
```

👉 You can replace `R` with `G` or `B` to change the target color.
If you want to change the color, stop the node (`Ctrl+C`) and rerun it with the new color.

✅ The Panda arm will detect the selected color and automatically pick and place it in the designated area.

---

## 🧩 Project Overview

This project demonstrates a **complete robotic color-sorting pipeline** in ROS 2 Humble using simulation tools.

It includes:

* Franka Panda robot description and controllers
* MoveIt 2 configuration for motion planning
* OpenCV-based color detection
* PyMoveIt2 for high-level pick-and-place control
* Unified bringup for Gazebo + RViz + Vision

---

## 🏗️ Step-by-Step Development Process

### **Step 1 — Workspace and Robot Description**

1. Create and initialize your ROS 2 workspace:

   ```bash
   mkdir -p ~/panda_ws/src
   cd ~/panda_ws
   colcon build
   source install/setup.bash
   ```

2. Create the description package:

   ```bash
   cd src
   ros2 pkg create --build-type ament_cmake panda_description
   ```

3. Copy the Panda URDF, meshes, and config files from the [official `franka_ros2`](https://github.com/frankaemika/franka_ros2) repository.

4. Visualize in RViz:

   ```bash
   ros2 launch panda_description display.launch.py
   ```

   ✅ *You should see the Panda robot model in RViz.*

---

### **Step 2 — Controllers Setup**

1. Create `panda_controller`:

   ```bash
   ros2 pkg create --build-type ament_python panda_controller
   ```

2. Add configuration files:

   * `panda_controller/config/panda_controllers.yaml` (joint, arm, gripper)
   * `panda_controller/launch/controller.launch.py`

3. Implement a simple test controller node:
   `slider_controller.py` — controls Panda arm joints manually.

4. Test:

   ```bash
   ros2 launch panda_controller controller.launch.py
   ```

---

### **Step 3 — MoveIt 2 Setup**

1. Launch the MoveIt Setup Assistant:

   ```bash
   ros2 run moveit_setup_assistant moveit_setup_assistant
   ```

2. Load your `panda_description` URDF and create:

   * Planning groups (`panda_arm`, `panda_gripper`)
   * End-effector definitions
   * Controller connections

3. Create panda_moveit cmake pkg with config folder in it:

   ```bash
   ros2 pkg create --build-type ament_python panda_moveit
   cd panda_moveit
   mkdir config
   ```

    *then paste relevant panda_moveit_configurations, from moveit_setup_assistant in config folder of panda_moveit pkg*
   
5. Test planning:

   ```bash
   ros2 launch panda_moveit moveit.launch.py
   ```

   ✅ *Panda arm should move through planned trajectories.*

---

### **Step 4 — Vision with OpenCV**

1. Create a package:

   ```bash
   ros2 pkg create --build-type ament_python panda_vision
   ```

2. Implement `color_detector.py`:

   * Subscribes to camera images
   * Filters HSV colors (Red, Green, Blue)
   * Detects contours and centroids
   * Publishes detected color position as a ROS message

3. Create a launch file for camera + color detection node.

4. Create `panda_bringup` package:

   * Launches robot, MoveIt, vision, and controllers together.

---

### **Step 5 — Pick and Place using PyMoveIt2**

1. Install PyMoveIt2:

   ```bash
   git clone https://github.com/AndrejOrsula/pymoveit2.git
   ```

2. Create a `panda_pick_place` node in pymoveit2' examples folder.

3. Implement `pick_and_place.py`:

   * Subscribes to detected color topic
   * Plans motion using PyMoveIt2
   * Controls gripper open/close
   * Executes full pick-and-place cycle

4. Run the system:

   ```bash
   ros2 launch panda_bringup pick_and_place.launch.py
   ros2 run pymoveit2 pick_and_place.py --ros-args -p target_color:=R
   ```

---

## 📂 Package Summary

| Package                 | Description                           |
| ----------------------- | ------------------------------------- |
| **panda_description**   | URDF, meshes, and robot model         |
| **panda_controller**    | Controller configs and test nodes     |
| **panda_moveit**        | MoveIt 2 motion planning setup        |
| **panda_vision**        | OpenCV-based color detection          |
| **panda_bringup**       | Launch files to start the full system |
| **pymoveit2**           | PyMoveIt2 pick-and-place control node |

---

## 🧠 Key Technologies

| Component        | Purpose                                 |
| ---------------- | --------------------------------------- |
| **ROS 2 Humble** | Robot middleware and node communication |
| **Gazebo**       | Robot simulation                        |
| **RViz**         | Visualization of robot and motion plans |
| **MoveIt 2**     | Motion planning and control             |
| **PyMoveIt2**    | Python interface for MoveIt 2           |
| **OpenCV**       | Real-time color detection               |

---

## 🛠️ Dependencies

Ensure the following packages are installed:

```bash
sudo apt install \
  ros-humble-moveit \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-ros-gz \
  ros-humble-cv-bridge \
  python3-opencv \
  python3-colcon-common-extensions

```

---

## 📸 Demo Workflow

1. Camera detects colored objects on a table.
2. Vision node publishes color and position.
3. PyMoveIt2 receives target color and plans motion.
4. Robot moves to the object, picks it, and places it in the correct bin.
5. The process repeats for other colors.

---

## 💡 Future Enhancements

* Integration with a real Franka Panda robot
* Depth-based grasping (Intel RealSense / ZED)
* Improved multi-object sorting logic
* Dynamic color threshold calibration

---

## 📄 License

This project is licensed under the **MIT License**.

---

## 🤝 Acknowledgments

* [Franka Emika Panda ROS 2](https://github.com/frankaemika/franka_ros2)
* [MoveIt 2](https://moveit.picknik.ai/)
* [PyMoveIt2](https://github.com/AndyZe/pymoveit2)
* ROS 2 community

---

## 🙌 Credits

Maintained by **[Curious-Utkarsh](https://github.com/Curious-Utkarsh)**
Inspired by real-world **pick-and-place robotic** applications.

---
