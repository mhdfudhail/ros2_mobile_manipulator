cmake_minimum_required(VERSION 3.8)
project(robot_autonomy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vision_interface REQUIRED)
find_package(linkattacher_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Function to add an executable with common settings
function(add_common_executable target_name source_file)
  add_executable(${target_name} src/${source_file})
  target_include_directories(${target_name} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_compile_features(${target_name} PUBLIC c_std_99 cxx_std_17)
  
  ament_target_dependencies(${target_name}
    moveit_msgs
    moveit_ros_planning_interface
    moveit_visual_tools
    rclcpp
    vision_interface
    linkattacher_msgs
  )
endfunction()

add_common_executable(pick_place pick_place.cpp)
add_common_executable(pick_place_object pick_place_object.cpp)
add_common_executable(test_pose test_pose.cpp)

install(TARGETS 
  pick_place_object
  pick_place
  test_pose
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
