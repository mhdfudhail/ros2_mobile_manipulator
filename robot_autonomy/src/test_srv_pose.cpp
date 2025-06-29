#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
// Add service client header - you'll need to replace this with your actual service type
#include <vision_interface/srv/get_pose.hpp>  // Replace with your actual service header

int main(int argc, char * argv[])
{
  // Start up ROS 2
  rclcpp::init(argc, argv);
  
  // Creates a node named "hello_moveit". The node is set up to automatically
  // handle any settings (parameters) we might want to change later without editing the code.
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Creates a "logger" that we can use to print out information or error messages
  // as our program runs.
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Create service client for getpose service
  auto client = node->create_client<vision_interface::srv::GetPose>("/get_pose_service");  // Replace with your actual service type

  // Wait for the service to be available
  while (!client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(logger, "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(logger, "Service not available, waiting again...");
  }

  // Create service request
  auto request = std::make_shared<vision_interface::srv::GetPose::Request>();  // Replace with your actual service type
  // Add any required fields to your request here if needed
  request->req = true;
  
  // Call the service
  auto result = client->async_send_request(request);
  
  // Wait for the result
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    auto response = result.get();
    
    // Extract x, y, z from service response
    double x = response->pose_x;  // Adjust field names based on your service definition
    double y = response->pose_y;
    double z = response->pose_z;
    
    RCLCPP_INFO(logger, "Received position from service: x=%.3f, y=%.3f, z=%.3f", x, y, z);
    
    // Read orientation from command-line arguments (since service only provides position)
    // if (argc < 5) {
    //   RCLCPP_ERROR(logger, "Usage: hello_moveit qx qy qz qw (position will be fetched from service)");
    //   return 1;
    // }
    // double qx = std::stod(argv[1]);
    // double qy = std::stod(argv[2]);
    // double qz = std::stod(argv[3]);
    // double qw = std::stod(argv[4]);
    
    // Log the complete pose
    RCLCPP_INFO(logger, "Target pose: x=%.3f, y=%.3f, z=%.3f", x, y, z);

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

    // Set a target pose for the end effector of the arm 
    geometry_msgs::msg::PoseStamped arm_target_pose;
    arm_target_pose.header.frame_id = "base_link";
    arm_target_pose.header.stamp = node->now(); 
    arm_target_pose.pose.position.x = x;  // Using x from service
    arm_target_pose.pose.position.y = y;  // Using y from service
    arm_target_pose.pose.position.z = z+0.17;  // Using z from service
    arm_target_pose.pose.orientation.x = 1.0;
    arm_target_pose.pose.orientation.y = 0.0;
    arm_target_pose.pose.orientation.z = 0.0;
    arm_target_pose.pose.orientation.w = 0.0;
    
    arm_group_interface.setPoseTarget(arm_target_pose);

    // Create a plan to that target pose
    auto const [success, plan] = [&arm_group_interface] {
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      auto const ok = static_cast<bool>(arm_group_interface.plan(msg));
      return std::make_pair(ok, msg);
    }();

    // Try to execute the movement plan if it was created successfully
    if (success) {
      arm_group_interface.execute(plan);
      RCLCPP_INFO(logger, "Plan executed successfully!");
    } else {
      RCLCPP_ERROR(logger, "Planning failed!");
    }
  } else {
    RCLCPP_ERROR(logger, "Failed to call service getpose");
    return 1;
  }
  
  // Shut down ROS 2 cleanly when we're done
  rclcpp::shutdown();
  return 0;
}