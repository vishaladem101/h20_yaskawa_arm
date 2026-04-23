#include <memory>
#include <vector>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose.hpp>

using moveit::planning_interface::MoveGroupInterface;

// Helper function to plan and move the arm
bool move_arm(MoveGroupInterface& move_group_interface, double x, double y, double z, double w, rclcpp::Logger logger, const std::string& pose_name) {
  // Check if we are already essentially at the target position to prevent duplicate zero-length planning errors
  geometry_msgs::msg::Pose current_pose = move_group_interface.getCurrentPose().pose;
  double dx = current_pose.position.x - x;
  double dy = current_pose.position.y - y;
  double dz = current_pose.position.z - z;
  double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
  
  if (distance < 0.005) { // within 5mm
     RCLCPP_INFO(logger, "Arm is already at %s (dist: %.3fm), skipping move.", pose_name.c_str(), distance);
     return true;
  }

  geometry_msgs::msg::Pose target_pose;
  target_pose.orientation.w = w;
  target_pose.position.x = x;
  target_pose.position.y = y;
  target_pose.position.z = z;
  move_group_interface.setPoseTarget(target_pose);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(move_group_interface.plan(plan));

  if (success) {
    auto exec_result = move_group_interface.execute(plan);
    if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(logger, "Arm reached %s", pose_name.c_str());
        return true;
    } else {
        RCLCPP_WARN(logger, "Arm failed to execute trajectory for %s", pose_name.c_str());
        return false;
    }
  } else {
    RCLCPP_WARN(logger, "Arm Planning failed for %s! Pose might be unreachable or identical. Skipping...", pose_name.c_str());
    return false;
  }
}

// Helper function to plan and move the gripper
bool move_gripper(MoveGroupInterface& gripper_group_interface, double width, rclcpp::Logger logger, const std::string& action_name) {
  gripper_group_interface.setJointValueTarget(std::vector<double>{width}); 

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(gripper_group_interface.plan(plan));

  if (success) {
    auto exec_result = gripper_group_interface.execute(plan);
    if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(logger, "Gripper %s goal reached", action_name.c_str());
        return true;
    } else {
        RCLCPP_ERROR(logger, "Gripper failed to execute trajectory for %s", action_name.c_str());
        return false;
    }
  } else {
    RCLCPP_ERROR(logger, "Gripper Planning failed for %s!", action_name.c_str());
    return false;
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // Using automatically_declare_parameters_from_overrides(true) allows loading the yaml file dynamically
  auto const node = std::make_shared<rclcpp::Node>(
    "goal_pose_executor",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Spin the node in a background thread.
  // This is strictly required for MoveIt to receive TF and joint_states callbacks!
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner_thread([&executor]() { executor.spin(); });

  auto const logger = rclcpp::get_logger("goal_pose_executor");

  auto move_group_interface = MoveGroupInterface(node, "manipulator");
  auto gripper_group_interface = MoveGroupInterface(node, "gripper");

  // Read Parameters
  bool initial_gripper_status = node->get_parameter_or("initial_gripper_status", true);
  double threshold_z = node->get_parameter_or("threshold_z", -0.05);
  double gripper_open = node->get_parameter_or("gripper_pose.open", 0.04);
  double gripper_close = node->get_parameter_or("gripper_pose.close", 0.0);

  double init_x = node->get_parameter_or("initial_pos.x", 0.3);
  double init_y = node->get_parameter_or("initial_pos.y", -0.2);
  double init_z = node->get_parameter_or("initial_pos.z", 0.1);
  double init_w = node->get_parameter_or("initial_pos.w", 1.0);

  double goal_x = node->get_parameter_or("goal_pose.x", 0.4);
  double goal_y = node->get_parameter_or("goal_pose.y", 0.0);
  double goal_z = node->get_parameter_or("goal_pose.z", 0.1);
  double goal_w = node->get_parameter_or("goal_pose.w", 1.0);

  double home_x = node->get_parameter_or("home_pos.x", 0.3);
  double home_y = node->get_parameter_or("home_pos.y", 0.2);
  double home_z = node->get_parameter_or("home_pos.z", 0.1);
  double home_w = node->get_parameter_or("home_pos.w", 1.0);

  // Wait for the robot to be initialized and joint states to be received
  RCLCPP_INFO(logger, "Waiting for MoveGroup and JointStates to be ready...");
  while (rclcpp::ok()) {
      auto current_pose = move_group_interface.getCurrentPose();
      if (!current_pose.header.frame_id.empty()) {
          RCLCPP_INFO(logger, "MoveGroup is ready. Current frame: %s", current_pose.header.frame_id.c_str());
          break;
      }
      RCLCPP_INFO(logger, "Still waiting for joint states...");
      rclcpp::sleep_for(std::chrono::milliseconds(500));
  }

  RCLCPP_INFO(logger, "Executing sequence...");

  // 0. First move to home position
  move_arm(move_group_interface, home_x, home_y, home_z, home_w, logger, "HOME (Start Position)");

  // 1. Initial Gripper execution
  if (initial_gripper_status) {
    move_gripper(gripper_group_interface, gripper_open, logger, "OPEN");
  } else {
    move_gripper(gripper_group_interface, gripper_close, logger, "CLOSE");
  }

  // 2. Move to initial position
  move_arm(move_group_interface, init_x, init_y, init_z, init_w, logger, "INITIAL POSE");

  // 3. Conditional step (if initial_gripper_status was false)
  if (!initial_gripper_status) {
    // Open gripper at initial pose
    move_gripper(gripper_group_interface, gripper_open, logger, "OPEN (Pick Prep)");
    // Move slightly inside Z
    move_arm(move_group_interface, init_x, init_y, init_z + threshold_z, init_w, logger, "INITIAL POSE THRESHOLD Z");
  }

  // 4. Move to Goal pose
  move_arm(move_group_interface, goal_x, goal_y, goal_z, goal_w, logger, "GOAL POSE");

  // 5. Close gripper
  move_gripper(gripper_group_interface, gripper_close, logger, "CLOSE (Drop/Secure)");

  // 6. Return to Home
  move_arm(move_group_interface, home_x, home_y, home_z, home_w, logger, "HOME");

  RCLCPP_INFO(logger, "Sequence Completed Successfully!");

  rclcpp::shutdown();
  spinner_thread.join();
  return 0;
}