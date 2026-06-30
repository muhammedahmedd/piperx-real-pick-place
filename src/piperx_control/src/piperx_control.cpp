#include "piperx_control/piperx_control.hpp"


PiperXControl::PiperXControl() : Node("pick_place_controller")
{
  RCLCPP_INFO(this->get_logger(), "Piper X pick-and-place control node started.");

  current_state_ = PickState::OPEN_GRIPPER;

  has_cube_pose_ = false;

  has_place_pose_ = false;

  // Tuned for grasping the cube (5 cm sides).
  gripper_grasp_joints_ = {0.050};

  scan_motion_done_ = false;

  place_motion_done_ = false;

  has_joint_state_ = false;

  arm_is_moving_ = true;

  // threshold used to decide when arm joints are slow enough to be considered settled
  this->declare_parameter<double>("settle_velocity_threshold", 0.033);

  settle_velocity_threshold_ =
    this->get_parameter("settle_velocity_threshold").as_double();

  // TCP release height above the table (place marker)  
  this->declare_parameter<double>("place_tcp_z", 0.050);

  place_tcp_z_ =
    this->get_parameter("place_tcp_z").as_double();

  cube_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/aruco/cube_pose_base",
    1,
    std::bind(&PiperXControl::cubePoseCallback, this, std::placeholders::_1)
  );

  place_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/aruco/place_pose_base",
    1,
    std::bind(&PiperXControl::placePoseCallback, this, std::placeholders::_1)
  );

  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/feedback/joint_states",
    1,
    std::bind(&PiperXControl::jointStateCallback, this, std::placeholders::_1)
  );
}

void PiperXControl::cubePoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (current_state_ != PickState::WAIT_FOR_MARKERS)
  {
    return;
  }

  if (has_cube_pose_)
  {
    return;
  }

  cube_pose_ = *msg;
  has_cube_pose_ = true;
}

void PiperXControl::placePoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (current_state_ != PickState::WAIT_FOR_MARKERS)
  {
    return;
  }

  if (has_place_pose_)
  {
    return;
  }

  place_pose_ = *msg;
  has_place_pose_ = true;
}

void PiperXControl::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  has_joint_state_ = true;

  arm_is_moving_ = false;

  for (int i = 0; i < msg->name.size(); ++i)
  {

    const std::string & joint_name = msg->name[i];

    if (
      joint_name == "joint1" ||
      joint_name == "joint2" ||
      joint_name == "joint3" ||
      joint_name == "joint4" ||
      joint_name == "joint5" ||
      joint_name == "joint6"
    )
    {
      double velocity = std::abs(msg->velocity[i]);

      if (velocity > settle_velocity_threshold_)
      {
        arm_is_moving_ = true;
        break;
      }
    }
  }
}

void PiperXControl::initializeMoveIt()
{
  arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    shared_from_this(), "arm");

  gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    shared_from_this(), "gripper");

  arm_group_->setMaxVelocityScalingFactor(0.1);
  arm_group_->setMaxAccelerationScalingFactor(0.1);

  arm_group_->setEndEffectorLink("tcp_link");
      
  arm_group_->setNumPlanningAttempts(1000);
  arm_group_->setPlanningTime(5.0);
}  

void PiperXControl::runStateMachine()
{
  switch (current_state_)
  {
    case PickState::OPEN_GRIPPER:
      RCLCPP_INFO(this->get_logger(), "State: OPEN_GRIPPER");

      moveGripperJoints(gripper_open_joints_);

      current_state_ = PickState::MOVE_TO_SCAN;

      break; 

    case PickState::MOVE_TO_SCAN:
      RCLCPP_INFO(this->get_logger(), "State: MOVE_TO_SCAN");

      if (!scan_motion_done_)
      {
        if (moveArmJoints(scan_pose_joints_))
        {
          scan_motion_done_ = true;
        }
        else
        {
          RCLCPP_WARN(this->get_logger(), "Scan pose failed, retrying...");
        }

        break;
      }

      if (!robotIsSettled())
      {
        RCLCPP_WARN(this->get_logger(), "Waiting for the arm to settle...");
      }
      else
      {
        current_state_ = PickState::WAIT_FOR_MARKERS;
      }

      break;
    
    case PickState::WAIT_FOR_MARKERS:

      if (has_cube_pose_ && has_place_pose_)
      {
        RCLCPP_INFO(
          this->get_logger(),
          "Cube marker pose: x=%.3f, y=%.3f, z=%.3f",
          cube_pose_.pose.position.x,
          cube_pose_.pose.position.y,
          cube_pose_.pose.position.z
        );

        RCLCPP_INFO(
          this->get_logger(),
          "Place marker pose: x=%.3f, y=%.3f, z=%.3f",
          place_pose_.pose.position.x,
          place_pose_.pose.position.y,
          place_pose_.pose.position.z
        );

        current_state_ = PickState::MOVE_TO_PICK;
      }
  
      break;
    
    case PickState::MOVE_TO_PICK:
      RCLCPP_INFO(this->get_logger(), "State: MOVE_TO_PICK");

      if (moveTcpToCube())
      {
        current_state_ = PickState::GRASP;
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Pick failed, retrying...");
      }

      break;

    case PickState::GRASP:
      RCLCPP_INFO(this->get_logger(), "State: GRASP");

      moveGripperJoints(gripper_grasp_joints_);

      current_state_ = PickState::LIFT;

      break;

    case PickState::LIFT:
      RCLCPP_INFO(this->get_logger(), "State: LIFT");

      if (moveArmJoints(lift_pose_joints_))
      {
        current_state_ = PickState::PLACE;
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Lift pose failed, retrying...");
      }

      break;

    case PickState::PLACE:
      RCLCPP_INFO(this->get_logger(), "State: PLACE");

      if (!place_motion_done_)
      {
        if (moveTcpToPlace())
        {
          place_motion_done_ = true;
        }
        else
        {
          RCLCPP_WARN(this->get_logger(), "Place failed, retrying...");
        }

        break;
      }

      if (!robotIsSettled())
      {
        RCLCPP_WARN(this->get_logger(), "Waiting for the arm to settle before opening gripper...");
      }
      else
      {
        moveGripperJoints(gripper_open_joints_);

        RCLCPP_INFO(this->get_logger(), "Object placed. Pick-and-place complete.");

        moveArmJoints(scan_pose_joints_);

        current_state_ = PickState::DONE;
      }

      break;

    case PickState::DONE:
        
      break;
  }
}

bool PiperXControl::robotIsSettled()
{
  if (!has_joint_state_)
  {
    return false;
  }

  return !arm_is_moving_;
}

bool PiperXControl::moveTcpToCube()
{
  geometry_msgs::msg::Pose target_pose;

  target_pose.position.x = cube_pose_.pose.position.x;
  target_pose.position.y = cube_pose_.pose.position.y;
  target_pose.position.z = place_tcp_z_;

  target_pose.orientation.x = 0.652;
  target_pose.orientation.y = -0.757;
  target_pose.orientation.z = 0.0;
  target_pose.orientation.w = 0.0;

  arm_group_->setStartStateToCurrentState();
  arm_group_->setPoseTarget(target_pose);

  if (arm_group_->plan(arm_plan_) == moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_INFO(this->get_logger(), "Pregrasp plan succeeded. Executing...");
    auto result = arm_group_->execute(arm_plan_);
    arm_group_->clearPoseTargets();

    if (result == moveit::core::MoveItErrorCode::SUCCESS)
    {
      return true;
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Execute failed.");
      return false;
    }
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Pregrasp plan failed.");
    arm_group_->clearPoseTargets();
    return false;
  }
}

bool PiperXControl::moveTcpToPlace()
{
  geometry_msgs::msg::Pose target_pose;

  target_pose.position.x = place_pose_.pose.position.x;
  target_pose.position.y = place_pose_.pose.position.y;

  // The place marker is on the table, so we do not move the TCP exactly to the marker surface.
  target_pose.position.z = place_tcp_z_;

  target_pose.orientation.x = 0.652;
  target_pose.orientation.y = -0.757;
  target_pose.orientation.z = 0.0;
  target_pose.orientation.w = 0.0;

  arm_group_->setStartStateToCurrentState();
  arm_group_->setPoseTarget(target_pose);

  if (arm_group_->plan(arm_plan_) == moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_INFO(this->get_logger(), "Place plan succeeded. Executing...");
    auto result = arm_group_->execute(arm_plan_);
    arm_group_->clearPoseTargets();

    if (result == moveit::core::MoveItErrorCode::SUCCESS)
    {
      return true;
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Place execute failed.");
      return false;
    }
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Place plan failed.");
    arm_group_->clearPoseTargets();
    return false;
  }
}

bool PiperXControl::moveArmJoints(const std::vector<double> & joint_angles)
{
  arm_group_->setJointValueTarget(joint_angles);

  if (arm_group_->plan(arm_plan_) == moveit::core::MoveItErrorCode::SUCCESS)
  {
    auto result = arm_group_->execute(arm_plan_);

    if (result == moveit::core::MoveItErrorCode::SUCCESS)
    {
      return true;
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Arm execute failed.");
      return false;
    }
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Arm plan failed.");
    return false;
  }
}

void PiperXControl::moveGripperJoints(const std::vector<double> & joint_angles)
{
  gripper_group_->setStartStateToCurrentState();
  gripper_group_->setJointValueTarget("gripper", joint_angles[0]);

  if (gripper_group_->plan(gripper_plan_) == moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_INFO(this->get_logger(), "Gripper plan succeeded. Executing...");

    auto result = gripper_group_->execute(gripper_plan_);

    if (result == moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_INFO(this->get_logger(), "Gripper execute succeeded.");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Gripper execute failed.");
    }
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Gripper plan failed.");
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<PiperXControl>();

  node->initializeMoveIt();

  rclcpp::Rate rate = rclcpp::Rate(30);

  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);

    node->runStateMachine();

    rate.sleep();
  }

  rclcpp::shutdown();

  return 0;
}