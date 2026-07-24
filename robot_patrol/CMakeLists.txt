cmake_minimum_required(VERSION 3.8)
project(robot_patrol)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetDirection.srv"
  "action/GoToPose.action"
  DEPENDENCIES sensor_msgs
)

# Include directories
include_directories(include)

# Get C++ typesupport target
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Create executables
add_executable(patrol src/patrol.cpp)
ament_target_dependencies(patrol rclcpp sensor_msgs geometry_msgs nav_msgs std_srvs cv_bridge OpenCV tf2)

# 2. Direction Service Executable (Server)
add_executable(direction_service src/direction_service.cpp)
ament_target_dependencies(direction_service rclcpp sensor_msgs)
target_link_libraries(direction_service "${cpp_typesupport_target}")
add_dependencies(direction_service ${PROJECT_NAME}__rosidl_generator_cpp)

# 3. Patrol with Service Executable (Client)
add_executable(patrol_with_service src/patrol_with_service.cpp)
ament_target_dependencies(patrol_with_service rclcpp rclcpp_action sensor_msgs geometry_msgs nav_msgs)
target_link_libraries(patrol_with_service "${cpp_typesupport_target}")
add_dependencies(patrol_with_service ${PROJECT_NAME}__rosidl_generator_cpp)

# 4. Test Service Executable
add_executable(test_service src/test_service.cpp)
ament_target_dependencies(test_service rclcpp sensor_msgs)
target_link_libraries(test_service "${cpp_typesupport_target}")
add_dependencies(test_service ${PROJECT_NAME}__rosidl_generator_cpp)

# go_to_pose_action executable
add_executable(go_to_pose_action src/go_to_pose_action.cpp)
ament_target_dependencies(go_to_pose_action rclcpp rclcpp_action sensor_msgs geometry_msgs nav_msgs tf2 tf2_ros)
target_link_libraries(go_to_pose_action "${cpp_typesupport_target}")
add_dependencies(go_to_pose_action ${PROJECT_NAME}__rosidl_generator_cpp)

# Install executables
install(TARGETS
  patrol
  direction_service
  patrol_with_service
  test_service
  go_to_pose_action
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

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
