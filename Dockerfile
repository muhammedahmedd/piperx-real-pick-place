FROM osrf/ros:humble-desktop

# 1. Python dependencies
RUN apt update && apt install -y python3-pip

RUN pip3 install python-can scipy numpy

# 2. CAN tools
RUN apt update && apt install -y \
    can-utils \
    ethtool \
    iproute2 \
    usbutils

# 3. ROS2 dependencies
RUN apt update && apt install -y \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    ros-humble-topic-tools \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    python3-colcon-common-extensions

# 4. MoveIt dependencies
RUN apt update && apt install -y ros-$ROS_DISTRO-moveit*

RUN apt-get install -y \
    ros-$ROS_DISTRO-control* \
    ros-$ROS_DISTRO-joint-trajectory-controller \
    ros-$ROS_DISTRO-joint-state-* \
    ros-$ROS_DISTRO-gripper-controllers \
    ros-$ROS_DISTRO-trajectory-msgs
    
# 5. RealSense / perception dependencies
RUN apt update && apt install -y \
    ros-humble-realsense2-* \
    ros-humble-image-transport \
    ros-humble-cv-bridge \
    ros-humble-image-geometry \
    ros-humble-camera-info-manager \
    ros-humble-rqt-image-view \
    ros-humble-tf-transformations \
    python3-opencv \
    python3-numpy \
    libopencv-contrib-dev

WORKDIR /workspace

COPY ros_entrypoint.sh /

RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["bash"]
