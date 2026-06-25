# Piper X Real Robot Pick-and-Place Workspace

This repository contains the ROS 2 Humble workspace for running the real Piper X robot arm with MoveIt 2, a RealSense camera, ArUco marker perception, and a custom pick-and-place controller.

The workspace is designed to run inside a Docker container.

## Workspace Layout

```text
piperx_ws/
├── Dockerfile
├── compose.yaml
├── ros_entrypoint.sh
├── README.md
└── src/
    ├── agx_arm_ros/          # External AgileX ROS 2 driver dependency
    ├── piperx_bringup/       # Hardware bringup launch package
    ├── piperx_perception/    # ArUco marker detection package
    └── piperx_control/       # Pick-and-place controller package
```

## Packages

### `piperx_bringup`

Hardware bringup package for the real Piper X system.

It starts:

```text
AgileX Piper X ROS 2 driver
MoveIt 2
RealSense camera
gripper_base → camera_link static transform
robot_state_publisher TF frequency update
```

Main launch file:

```bash
ros2 launch piperx_bringup hardware.launch.py
```

Default launch arguments:

```text
can_port:=can0
speed_percent:=10
tcp_offset:="[0.0, 0.0, 0.1058, 0.0, 0.0, 0.0]"
```

Example with custom speed:

```bash
ros2 launch piperx_bringup hardware.launch.py speed_percent:=20
```

### `piperx_perception`

Perception package for detecting ArUco markers using the RealSense camera.

Main executable:

```bash
ros2 run piperx_perception aruco_detector
```

Published topics:

```text
/aruco/cube_pose_base
/aruco/place_pose_base
```

The detector uses the RealSense color image and camera info topics:

```text
/camera/camera/color/image_raw
/camera/camera/color/camera_info
```

### `piperx_control`

Control package for the real Piper X pick-and-place routine.

Main executable:

```bash
ros2 run piperx_control pick_place_controller
```

Main launch file:

```bash
ros2 launch piperx_control pick_place.launch.py
```

The pick-place launch starts:

```text
ArUco detector
Piper X pick-place controller
```

Default launch arguments:

```text
settle_velocity_threshold:=0.033
place_tcp_z:=0.040
marker_size:=0.055
```

Example with custom parameters:

```bash
ros2 launch piperx_control pick_place.launch.py \
  settle_velocity_threshold:=0.033 \
  place_tcp_z:=0.040 \
  marker_size:=0.055
```

## Starting the Real Piper X System

### 1. Start the Docker container

From the host machine:

```bash
cd ~/piperx_ws
docker compose up -d
```

### 2. Enter the container

```bash
docker exec -it ros2_piperx bash
```

### 3. Source ROS 2 Humble

Inside the container:

```bash
source /opt/ros/humble/setup.bash
```

### 4. Install the Piper X Python SDK

Inside the container:

```bash
cd /workspace/pyAgxArm
pip3 install .
```

Check that it installed correctly:

```bash
python3 -c "import pyAgxArm; print('pyAgxArm import OK')"
```

Expected output:

```text
pyAgxArm import OK
```

### 5. Go to the ROS 2 workspace

```bash
cd /workspace/piperx_ws
```

### 6. Build the workspace

```bash
colcon build
source install/setup.bash
```

If the workspace has already been built, just source it:

```bash
source install/setup.bash
```

### 7. Activate CAN

```bash
cd /workspace/piperx_ws/src/agx_arm_ros/scripts
bash can_activate.sh
```

Expected output should say that `can0` was reset and activated.

You can verify CAN with:

```bash
ip link show can0
```

### 8. Start hardware bringup

```bash
cd /workspace/piperx_ws
source install/setup.bash

ros2 launch piperx_bringup hardware.launch.py
```

This starts the real robot driver, MoveIt, RealSense camera, static camera transform, and TF frequency update.

## Verifying Hardware Bringup

Open a second terminal and enter the same Docker container:

```bash
docker exec -it ros2_piperx bash
source /opt/ros/humble/setup.bash
cd /workspace/piperx_ws
source install/setup.bash
```

Check the robot state publisher frequency:

```bash
ros2 param get /robot_state_publisher publish_frequency
```

Expected value:

```text
100.0
```

Check TF frequency:

```bash
ros2 topic hz /tf
```

Check camera topics:

```bash
ros2 topic list | grep camera/camera/color
```

Check the robot-to-camera TF chain:

```bash
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

If transforms print repeatedly, the camera TF chain is working.

## Running Pick-and-Place

After hardware bringup is already running, open another terminal:

```bash
docker exec -it ros2_piperx bash
source /opt/ros/humble/setup.bash
cd /workspace/piperx_ws
source install/setup.bash
```

Then start the pick-and-place launch:

```bash
ros2 launch piperx_control pick_place.launch.py
```

This starts:

```text
aruco_detector
pick_place_controller
```

The controller subscribes to:

```text
/aruco/cube_pose_base
/aruco/place_pose_base
/feedback/joint_states
```

The controller uses MoveIt groups:

```text
arm
gripper
```

The end-effector link is:

```text
gripper_tcp
```

## Useful Commands

List active ROS nodes:

```bash
ros2 node list
```

List topics:

```bash
ros2 topic list
```

Check joint states:

```bash
ros2 topic echo /feedback/joint_states
```

Check ArUco cube pose:

```bash
ros2 topic echo /aruco/cube_pose_base
```

Check ArUco place pose:

```bash
ros2 topic echo /aruco/place_pose_base
```

Check controller parameters:

```bash
ros2 param list /pick_place_controller
```

Check a specific parameter:

```bash
ros2 param get /pick_place_controller settle_velocity_threshold
```

## Notes

The external AgileX driver is stored in:

```text
src/agx_arm_ros/
```

This folder is treated as an external dependency and should not be committed to this repository.

If the Docker container is deleted or recreated, the Python SDK may need to be reinstalled:

```bash
cd /workspace/pyAgxArm
pip3 install .
```

The old direct AgileX launch command was:

```bash
ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py \
  can_port:=can0 \
  arm_type:=piper_x \
  effector_type:=agx_gripper \
  speed_percent:=10
```

The preferred command now is:

```bash
ros2 launch piperx_bringup hardware.launch.py
```

Use `piperx_bringup` for hardware, camera, TF, and MoveIt.

Use `piperx_control` only after hardware bringup is working.
