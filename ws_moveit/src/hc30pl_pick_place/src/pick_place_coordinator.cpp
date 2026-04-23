#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "pick_place_coordinator",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Read parameters for pick and place coordinates
  double px = node->get_parameter_or("pick_x", 0.5);
  double py = node->get_parameter_or("pick_y", 0.0);
  double pz = node->get_parameter_or("pick_z", 0.5);
  
  double lx = node->get_parameter_or("place_x", 0.5);
  double ly = node->get_parameter_or("place_y", 0.3);
  double lz = node->get_parameter_or("place_z", 0.5);

  rclcpp::Logger logger = node->get_logger();

  // Create MoveGroupInterface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "manipulator");

  // Define Pick Pose
  geometry_msgs::msg::Pose pick_pose;
  pick_pose.orientation.w = 1.0; // Pointing straight down (or default)
  pick_pose.position.x = px;
  pick_pose.position.y = py;
  pick_pose.position.z = pz;

  // Define Place Pose
  geometry_msgs::msg::Pose place_pose;
  place_pose.orientation.w = 1.0;
  place_pose.position.x = lx;
  place_pose.position.y = ly;
  place_pose.position.z = lz;

  auto plan_and_execute = [&move_group_interface, &logger](const geometry_msgs::msg::Pose& target_pose, const std::string& name) {
    move_group_interface.setPoseTarget(target_pose);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    
    bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if(success) {
      RCLCPP_INFO(logger, "Planning successful for %s. Executing...", name.c_str());
      move_group_interface.execute(my_plan);
    } else {
      RCLCPP_ERROR(logger, "Planning failed for %s!", name.c_str());
    }
    return success;
  };

  // Execute sequence
  RCLCPP_INFO(logger, "Moving to PICK position: (%f, %f, %f)", px, py, pz);
  if (plan_and_execute(pick_pose, "PICK")) {
      RCLCPP_INFO(logger, "At pick position. Simulating grasping...");
      rclcpp::sleep_for(std::chrono::seconds(2));
      
      RCLCPP_INFO(logger, "Moving to PLACE position: (%f, %f, %f)", lx, ly, lz);
      if (plan_and_execute(place_pose, "PLACE")) {
          RCLCPP_INFO(logger, "At place position. Simulating release...");
          rclcpp::sleep_for(std::chrono::seconds(2));
          RCLCPP_INFO(logger, "Task completed successfully.");
      }
  }

  rclcpp::shutdown();
  return 0;
}
