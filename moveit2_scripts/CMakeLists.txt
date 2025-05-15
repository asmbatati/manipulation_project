cmake_minimum_required(VERSION 3.8)
project(moveit2_scripts)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)

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

# Generate the executable
add_executable(joint_space_trajectory
    src/joint_space_trajectory.cpp)
add_executable(goal_pose_trajectory
    src/goal_pose_trajectory.cpp)
add_executable(gripper_control
    src/gripper_control.cpp)
add_executable(cartesian_trajectory
    src/cartesian_trajectory.cpp)
add_executable(kinematics_plugin_trajectory
    src/kinematics_plugin_trajectory.cpp)

target_include_directories(joint_space_trajectory
    PUBLIC include)
target_include_directories(goal_pose_trajectory
    PUBLIC include)
target_include_directories(gripper_control
    PUBLIC include)
target_include_directories(cartesian_trajectory
    PUBLIC include)
target_include_directories(kinematics_plugin_trajectory
    PUBLIC include)

ament_target_dependencies(joint_space_trajectory
    ament_cmake
    rclcpp
    rclcpp_action
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
    moveit_ros_planning
    control_msgs)
ament_target_dependencies(goal_pose_trajectory
    ament_cmake
    rclcpp
    rclcpp_action
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
    moveit_ros_planning
    control_msgs)
ament_target_dependencies(gripper_control
    ament_cmake
    rclcpp
    rclcpp_action
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
    moveit_ros_planning
    control_msgs)
ament_target_dependencies(cartesian_trajectory
    ament_cmake
    rclcpp
    rclcpp_action
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
    moveit_ros_planning
    control_msgs)
ament_target_dependencies(kinematics_plugin_trajectory
    ament_cmake
    rclcpp
    rclcpp_action
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
    moveit_ros_planning
    control_msgs)

# Install the executable
install(TARGETS 
    joint_space_trajectory
	goal_pose_trajectory
	gripper_control
	cartesian_trajectory
	kinematics_plugin_trajectory
    DESTINATION lib/${PROJECT_NAME}
)

# Install the launch file
install(DIRECTORY 
    launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
