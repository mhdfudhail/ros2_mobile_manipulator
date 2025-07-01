#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  auto const logger = rclcpp::get_logger("hello_moveit");

  using moveit::planning_interface::MoveGroupInterface;
  auto arm_group_interface = MoveGroupInterface(node, "manipulator");

  // Specify a planning pipeline to be used for further planning 
  arm_group_interface.setPlanningPipelineId("ompl");
  
  // Specify a planner to be used for further planning
  arm_group_interface.setPlannerId("RRTConnectkConfigDefault");  

  // Specify the maximum amount of time in seconds to use when planning
  arm_group_interface.setPlanningTime(1.0);
  
  // Set a scaling factor for optionally reducing the maximum joint velocity. Allowed values are in (0,1].
  arm_group_interface.setMaxVelocityScalingFactor(1.0);
  
  //  Set a scaling factor for optionally reducing the maximum joint acceleration. Allowed values are in (0,1].
  arm_group_interface.setMaxAccelerationScalingFactor(1.0);

  // Display helpful logging messages on the terminal
  RCLCPP_INFO(logger, "Planning pipeline: %s", arm_group_interface.getPlanningPipelineId().c_str());    
  RCLCPP_INFO(logger, "Planner ID: %s", arm_group_interface.getPlannerId().c_str());
  RCLCPP_INFO(logger, "Planning time: %.2f", arm_group_interface.getPlanningTime());

  // Read pose from command-line arguments
  if (argc < 8) {
    RCLCPP_ERROR(logger, "Usage: hello_moveit x y z qx qy qz qw");
    return 1;
  }
  double x = std::stod(argv[1]);
  double y = std::stod(argv[2]);
  double z = std::stod(argv[3]);
  double qx = std::stod(argv[4]);
  double qy = std::stod(argv[5]);
  double qz = std::stod(argv[6]);
  double qw = std::stod(argv[7]);
  
  // Log the pose
  RCLCPP_INFO(logger, "Received pose goal: x=%.3f, y=%.3f, z=%.3f, qx=%.3f, qy=%.3f, qz=%.3f, qw=%.3f", x, y, z, qx, qy, qz, qw);

  // Set a target pose for the end effector of the arm 
  
    geometry_msgs::msg::PoseStamped arm_target_pose;
    arm_target_pose.header.frame_id = "base_link";
    arm_target_pose.header.stamp = node->now(); 
    arm_target_pose.pose.position.x = x;
    arm_target_pose.pose.position.y = y;
    arm_target_pose.pose.position.z = z;
    arm_target_pose.pose.orientation.x = qx;
    arm_target_pose.pose.orientation.y = qy;
    arm_target_pose.pose.orientation.z = qz;
    arm_target_pose.pose.orientation.w = qw;
  
  arm_group_interface.setPoseTarget(arm_target_pose);

  // Create a plan to that target pose
  // This will give us two things:
  // 1. Whether the planning was successful (stored in 'success')
  // 2. The actual motion plan (stored in 'plan')
  auto const [success, plan] = [&arm_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(arm_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Try to execute the movement plan if it was created successfully
  // If the plan wasn't successful, report an error
  // Execute the plan
  if (success)
  {
    arm_group_interface.execute(plan);
  }
    else
  {
    RCLCPP_ERROR(logger, "Planning failed!");
  }
  
  // Shut down ROS 2 cleanly when we're done
  rclcpp::shutdown();
  return 0;
}

