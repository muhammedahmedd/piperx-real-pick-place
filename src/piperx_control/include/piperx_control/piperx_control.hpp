#ifndef PIPERX_CONTROL__PIPERX_CONTROL_HPP_
#define PIPERX_CONTROL__PIPERX_CONTROL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

class PiperXControl : public rclcpp::Node
{
public:

  PiperXControl();

  ~PiperXControl() = default;

  void cubePoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  void placePoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  
  void initializeMoveIt();

  void runStateMachine();

  bool moveTcpToCube();

  bool moveTcpToPlace();

  bool moveArmJoints(const std::vector<double> & joint_angles);

  void moveGripperJoints(const std::vector<double> & joint_angles);

  bool robotIsSettled();


private:

  enum class PickState
  {
    OPEN_GRIPPER,
    MOVE_TO_SCAN,
    WAIT_FOR_MARKERS,
    MOVE_TO_PICK,
    GRASP,
    LIFT,
    PLACE,
    DONE
  };

  PickState current_state_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr cube_pose_sub_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr place_pose_sub_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  geometry_msgs::msg::PoseStamped cube_pose_;

  geometry_msgs::msg::PoseStamped place_pose_;

  bool has_cube_pose_;

  bool has_place_pose_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;

  moveit::planning_interface::MoveGroupInterface::Plan arm_plan_;

  moveit::planning_interface::MoveGroupInterface::Plan gripper_plan_;

  // joint-space poses used by the pick-and-place state machine
  std::vector<double> scan_pose_joints_ = {0.0, 0.373, -1.283, 1.315, 0.0, 0.0};
  std::vector<double> lift_pose_joints_ = {0.0, 1.7628, -1.8326, 1.5708, 0.0, 0.0};

  // gripper joint targets for open and grasp positions
  std::vector<double> gripper_open_joints_ = {0.100};
  std::vector<double> gripper_grasp_joints_;

  // state-machine flags to prevent repeated motion commands
  bool scan_motion_done_;
  bool place_motion_done_;

  // robot motion-settling feedback
  bool has_joint_state_;
  bool arm_is_moving_;

  // Tunable parameters
  double settle_velocity_threshold_;
  double place_tcp_z_;
};

#endif 