FROM osrf/ros:humble-desktop-full

ENV ROS_WS=/root/panda_ws
ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /root

# Install base tools and MoveIt packages
RUN apt update && apt install -y \
    git \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-pip \
    ros-humble-moveit \
    ros-humble-moveit-common \
    ros-humble-moveit-ros-move-group \
    ros-humble-moveit-ros-planning \
    ros-humble-moveit-ros-planning-interface \
    ros-humble-moveit-visual-tools \
    ros-humble-moveit-configs-utils \
    ros-humble-moveit-setup-assistant \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies before building workspace
RUN pip3 install --no-cache-dir \
    opencv-python==4.10.0.84 \
    numpy==1.24.4 \
    transforms3d

# Initialize rosdep
RUN rosdep update --rosdistro=humble

# Create panda workspace and clone repo
RUN mkdir -p $ROS_WS/src
WORKDIR $ROS_WS/src
RUN git clone https://github.com/MechaMind-Labs/franka-color-sorting-robot.git .

WORKDIR $ROS_WS
RUN rosdep install --from-paths src -y --ignore-src --skip-keys=opencv_python --rosdistro=humble || \
    (apt update && rosdep install --from-paths src -y --ignore-src --skip-keys=opencv_python --rosdistro=humble)

# Build the workspace
RUN . /opt/ros/humble/setup.sh && colcon build

# Source the workspace in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source $ROS_WS/install/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]
