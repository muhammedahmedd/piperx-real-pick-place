# Piper X Arm Robot Pick-and-Place Workspace

This repository contains the ROS 2 Humble workspace for running a real Piper X robot arm pick-and-place system.

It brings together the AgileX ROS 2 driver, MoveIt 2, a RealSense camera, ArUco marker perception, and a custom pick-and-place controller. The goal is to control the real Piper X arm, detect a tagged cube and placement markers with the camera, and execute a pick-and-place sequence using MoveIt and live robot feedback.

The project is built to run inside Docker, so the ROS 2 environment and required dependencies are handled inside the container instead of being installed directly on the host system.

---

## Workspace Layout

```text
~/
├── piperx_ws/
│   ├── Dockerfile
│   ├── compose.yaml
│   ├── ros_entrypoint.sh
│   ├── README.md
│   └── src/
│       ├── agx_arm_ros/          # External AgileX ROS 2 driver dependency
│       ├── piperx_bringup/       # Hardware bringup launch package
│       ├── piperx_perception/    # ArUco marker detection package
│       └── piperx_control/       # Pick-and-place controller package
│
└── pyAgxArm/                     # External AgileX Python SDK dependency
```

---

## Setup

### 1. Clone this repository

On the host machine:

```bash
cd ~
git clone https://github.com/muhammedahmedd/piperx-real-pick-place.git piperx_ws
cd ~/piperx_ws
```

---

### 2. Clone the external AgileX ROS 2 driver

The AgileX ROS 2 driver is used as an external dependency and should be cloned into `src/agx_arm_ros`.

```bash
cd ~/piperx_ws/src
git clone --recurse-submodules https://github.com/agilexrobotics/agx_arm_ros.git agx_arm_ros
```

---

### 3. Clone the Piper X Python SDK

The Piper X Python SDK is mounted into the Docker container and installed inside the container.

On the host machine:

```bash
cd ~
git clone https://github.com/agilexrobotics/pyAgxArm.git
cd pyAgxArm
```

---

## Docker

This project is designed to run inside Docker. The Docker container provides the ROS 2 Humble environment and the dependencies needed to build and run the workspace.

### 1. Start the container

From the host machine:

```bash
cd ~/piperx_ws
docker compose up -d
```

### 2. Enter the container

```bash
docker exec -it ros2_piperx bash
```

---

## Build and Source the ROS 2 Workspace

Inside the container:

```bash
source /opt/ros/humble/setup.bash
cd /workspace/piperx_ws
colcon build
source install/setup.bash
```

---

## Piper X Python SDK Setup

Inside the container, install the Piper X Python SDK:

```bash
cd /workspace/pyAgxArm
pip3 install .
```

---

## CAN Setup

Before launching the real robot driver, activate CAN:

```bash
cd /workspace/piperx_ws/src/agx_arm_ros/scripts
bash can_activate.sh
```

---

## Hardware Bringup

The hardware bringup launch starts the real Piper X robot system.

```bash
cd /workspace/piperx_ws
source install/setup.bash

ros2 launch piperx_bringup hardware.launch.py
```

This launch brings up:

- AgileX Piper X ROS 2 driver
- MoveIt 2
- RealSense camera
- Hardware interface for the real robot arm and gripper
- Static transform between `gripper_base` and `camera_link`
- Updated `robot_state_publisher` TF publish frequency

The static camera transform defines how the RealSense camera is mounted on the gripper. Specifically, it publishes the fixed transform between `gripper_base` and `camera_link`. Here, `camera_link` is the main RealSense camera frame used as the parent frame for the camera’s optical frames. This transform connects the camera TF tree to the robot TF tree, allowing ArUco marker poses detected by the camera to be transformed into `base_link`.

The launch also updates `robot_state_publisher` by setting:

```text
publish_frequency = 100.0

### Hardware bringup arguments

Default arguments:

```bash
can_port:=can0
speed_percent:=10
tcp_offset:="[0.0, 0.0, 0.1058, 0.0, 0.0, 0.0]"
```

Example with a custom speed:

```bash
ros2 launch piperx_bringup hardware.launch.py speed_percent:=20
```

Use this launch first. The pick-and-place controller should be started after hardware bringup is running.

---

## Pick-and-Place Launch

After hardware bringup is running, open a second terminal and enter the container:

```bash
docker exec -it ros2_piperx bash
source /opt/ros/humble/setup.bash
cd /workspace/piperx_ws
source install/setup.bash
```

Start the pick-and-place system:

```bash
ros2 launch piperx_control pick_place.launch.py
```

This launch starts:

- ArUco marker detector
- Piper X pick-and-place controller

The controller subscribes to:

```text
/aruco/cube_pose_base
/aruco/place_pose_base
/feedback/joint_states
```

The controller uses the MoveIt groups:

```text
arm
gripper
```

The end-effector link is:

```text
tcp_link
```

### Pick-and-place arguments

Default arguments:

```bash
joint_position_tolerance:=0.02
place_tcp_z:=0.040
marker_size:=0.040
```

Example with custom arguments:

```bash
ros2 launch piperx_control pick_place.launch.py \
  joint_position_tolerance:=0.01 \
  place_tcp_z:=0.040 \
  marker_size:=0.040
```

Argument meanings:

```text
joint_position_tolerance
  Joint feedback tolerance used to confirm the arm reached the planned target.

place_tcp_z
  TCP height used when placing and picking the cube.

marker_size
  ArUco marker side length in meters.
```

---

## Packages

### `piperx_bringup`

Hardware bringup package for the real Piper X robot.

Main launch file:

```bash
ros2 launch piperx_bringup hardware.launch.py
```

---

### `piperx_perception`

ArUco marker perception package using the RealSense camera.

Main executable:

```bash
ros2 run piperx_perception aruco_detector
```

Published topics:

```text
/aruco/cube_pose_base
/aruco/place_pose_base
```

Camera topics used:

```text
/camera/camera/color/image_raw
/camera/camera/color/camera_info
```

---

### `piperx_control`

Pick-and-place controller package.

Main launch file:

```bash
ros2 launch piperx_control pick_place.launch.py
```

The controller plans and executes the pick-and-place routine using MoveIt 2 and confirms arm targets using feedback from `/feedback/joint_states`.

---

## Notes

### External dependencies

The AgileX ROS 2 driver is stored in:

```text
src/agx_arm_ros/
```

The Piper X Python SDK is stored outside the ROS workspace at:

```text
~/pyAgxArm
```

Both are treated as external dependencies.

---
 
### Gripper Mounting Angle Calibration

The URDF in the external AgileX driver was adjusted to better match the real gripper mounting angle.

At the zero/home pose, the physical gripper orientation did not fully match the URDF and MoveIt model. The original `gripper_base_joint` yaw used the nominal 90-degree value:

```xml
<joint name="gripper_base_joint" type="fixed">
  <origin xyz="0 0 0.0045" rpy="0 0 1.5707963"/>
````

A mobile phone angle app was used to measure the mismatch. The gripper was calibrated to hold the phone, and the measured offset was approximately `8.5°`. The URDF yaw was updated to:

```xml
<joint name="gripper_base_joint" type="fixed">
  <origin xyz="0 0 0.0045" rpy="0 0 1.4224433404"/>
```

This makes the MoveIt model better match the real robot gripper orientation.

---

### Docker note

If the Docker container is deleted or recreated, reinstall the Piper X Python SDK inside the container:

```bash
cd /workspace/pyAgxArm
pip3 install .
```
